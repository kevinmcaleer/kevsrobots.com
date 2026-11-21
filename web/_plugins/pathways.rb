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

module KevsRobots
  class PathwayGenerator < Jekyll::Generator
    safe true
    priority :normal

    LEVELS = %w[beginner intermediate advanced].freeze

    def generate(site)
      collection = site.collections["learning_pathways"]
      return unless collection

      by_name = {}
      (site.data["courses"] || []).each { |course| by_name[course["name"]] = course }

      pathways = collection.docs.select { |doc| doc.data["courses"].is_a?(Array) }
      problems = []

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
