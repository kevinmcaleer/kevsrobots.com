# frozen_string_literal: true

# Learning pathways (web/_learning_pathways/*.md).
#
# A pathway lists its courses by name, in order:
#
#   courses:
#     - "Robotics 101"
#     - "Python for beginners"
#
# At build time this looks every name up in _data/courses.yml (written by
# build.py) and adds to the pathway page:
#
#   page.course_items   the course records, in pathway order
#   page.course_count   how many courses
#   page.lesson_count   lessons across all of them
#   page.total_minutes  total length in minutes
#   page.duration       the same as text, e.g. "6h 23m"
#   page.levels         the levels it covers, easiest first, e.g. ["beginner", "intermediate"]
#   page.related        up to three other pathways, the ones sharing most courses first
#
# and lists every pathway in site.data.pathways, sorted by its front matter
# `order:` (lowest first, pathways without one last, then by title). The
# Learn homepage and the pathways index show them in that order.
#
# so layouts and cards never loop over every course to find one, and the
# totals are worked out in one place.
#
# A name that matches no course stops the build with a message saying which
# pathway and which name. Before this check a renamed course simply vanished
# from its pathway.
#
# A pathway can instead split its courses into stages (#631):
#
#   stages:
#     start:                       # no experience needed
#       - "How to install MicroPython"
#     build:                       # the core skills
#       - "Raspberry Pi Pico with MicroPython - GPIO Mastery"
#       - soon: "Sensors for robots"   # a course not written yet
#         issue: 639                    # optional: its GitHub issue
#     further:                     # pick what interests you
#       - "Introduction to Rust"
#
# The stages are flattened, in that order, into `courses` (so totals and
# `related` work as before), and the page also gets
#
#   page.stage_items    [{key, label, blurb, items}], items being course
#                       records or {"soon" => title, "issue" => n} placeholders
#
# Placeholders are shown as "Coming soon" and are not counted in the totals.
# Every course that is in no pathway is listed in a build warning.
#
# Every lesson page and course overview page also gets, for the "What next" box on a course's last
# lesson and the "Course complete" dialog (#635):
#
#   page.course_pathways  [{title, url, position, of, next}] for each pathway
#                         the course is in, `next` being the following course
#                         record (nil at the end of the pathway)

module KevsRobots
  class PathwayGenerator < Jekyll::Generator
    safe true
    priority :normal

    LEVELS = %w[beginner intermediate advanced].freeze

    STAGES = [
      ["start", "Start", "No experience needed"],
      ["build", "Build", "The core skills"],
      ["further", "Go further", "Pick what interests you"]
    ].freeze

    # Courses that are deliberately in no pathway.
    UNPATHED_OK = ["How to use the Learning Platform"].freeze

    def generate(site)
      collection = site.collections["learning_pathways"]
      return unless collection

      by_name = {}
      (site.data["courses"] || []).each { |course| by_name[course["name"]] = course }

      problems = []
      collection.docs.each { |doc| expand_stages(doc, by_name, problems) if doc.data["stages"].is_a?(Hash) }

      pathways = collection.docs.select { |doc| doc.data["courses"].is_a?(Array) }

      pathways.each do |doc|
        items = doc.data["courses"].map do |name|
          course = by_name[name]
          problems << "#{doc.relative_path}: no course named #{name.inspect}" unless course
          course
        end.compact

        minutes = items.sum { |course| self.class.minutes(course["duration"]) }
        levels = items.map { |course| course["level"] || "beginner" }.uniq
        doc.data["course_items"] = items
        doc.data["course_count"] = items.size
        doc.data["lesson_count"] = items.sum { |course| course["lessons"].to_i }
        doc.data["total_minutes"] = minutes
        doc.data["duration"] = self.class.duration(minutes)
        doc.data["levels"] = levels.sort_by { |level| LEVELS.index(level) || LEVELS.size }
      end

      unless problems.empty?
        raise Jekyll::Errors::FatalException,
              "Learning pathways list courses that are not in _data/courses.yml " \
              "(names must match a course's `name:` exactly):\n  " + problems.join("\n  ")
      end

      in_a_pathway = pathways.flat_map { |doc| doc.data["courses"] }
      unpathed = by_name.keys - in_a_pathway - UNPATHED_OK
      unless unpathed.empty?
        Jekyll.logger.warn "Pathways:", "#{unpathed.size} course(s) are in no learning pathway:"
        unpathed.sort.each { |name| Jekyll.logger.warn "", "  #{name}" }
      end

      site.data["pathways"] = pathways.sort_by do |doc|
        [doc.data["order"] || 9999, doc.data["title"].to_s]
      end

      pathways.each do |doc|
        names = doc.data["courses"]
        others = pathways.reject { |other| other.equal?(doc) }
        ranked = others.sort_by do |other|
          [-(names & other.data["courses"]).size, -other.data["course_count"], other.data["title"].to_s]
        end
        doc.data["related"] = ranked.first(3)
      end

      add_course_pathways(site)
    end

    def add_course_pathways(site)
      by_slug = Hash.new { |hash, key| hash[key] = [] }
      site.data["pathways"].each do |doc|
        items = doc.data["course_items"]
        items.each_with_index do |course, index|
          slug = course["link"].to_s[%r{\A/learn/([^/]+)/}, 1]
          next unless slug

          by_slug[slug] << {
            "title" => doc.data["title"], "url" => doc.url,
            "position" => index + 1, "of" => items.size, "next" => items[index + 1]
          }
        end
      end

      site.pages.each do |page|
        next unless %w[lesson course_overview].include?(page.data["layout"])

        slug = page.url[%r{\A/learn/([^/]+)/}, 1]
        page.data["course_pathways"] = by_slug[slug] if slug && by_slug.key?(slug)
      end
    end

    # stages: {start: [...], build: [...], further: [...]} -> courses + stage_items
    def expand_stages(doc, by_name, problems)
      stages = doc.data["stages"]
      unknown = stages.keys - STAGES.map(&:first)
      problems << "#{doc.relative_path}: unknown stage(s) #{unknown.join(', ')} (use start, build, further)" unless unknown.empty?

      names = []
      doc.data["stage_items"] = STAGES.filter_map do |key, label, blurb|
        entries = Array(stages[key])
        next if entries.empty?

        items = entries.filter_map do |entry|
          if entry.is_a?(Hash) && entry["soon"]
            { "soon" => entry["soon"].to_s, "issue" => entry["issue"] }
          else
            names << entry.to_s
            by_name[entry.to_s]
          end
        end
        { "key" => key, "label" => label, "blurb" => blurb, "items" => items }
      end
      doc.data["courses"] = names
    end

    # "1h 38m" -> 98
    def self.minutes(text)
      text = text.to_s
      text[/(\d+)\s*h/, 1].to_i * 60 + text[/(\d+)\s*m/, 1].to_i
    end

    # 98 -> "1h 38m", 45 -> "45m", 120 -> "2h"
    def self.duration(minutes)
      hours, mins = minutes.divmod(60)
      [("#{hours}h" if hours.positive?), ("#{mins}m" if mins.positive? || hours.zero?)].compact.join(" ")
    end
  end
end
