# frozen_string_literal: true

# Course health scorecard (#637): an unlisted page, /learn/course-health/,
# that scores every course in _data/courses.yml on the things that make a
# course good to learn from, so it is clear which ones to fix first.
#
# Each check looks at the course's lesson pages (layout: lesson) as written:
#
#   intro          a first lesson numbered 00 or 01
#   prerequisites  says what you need first ("prerequisite", "what you need",
#                  "you will need", "before you start")
#   whats_next     the last lesson points somewhere ("next", "where to go",
#                  "further", "what now")
#   quiz           at least one quiz question (quiz_total from build.py)
#   try_it         a "Try it yourself" exercise
#   common_issues  a "Common issues" or "Troubleshooting" section
#   tested_with    says what it was tested with ("tested with", "tested on")
#   fresh          published or updated within the last 18 months
#   in_pathway     is in at least one learning pathway
#
# The page also shows each course's "Was this lesson helpful?" votes, which
# assets/js/course-health.js reads from Nibsy.

module KevsRobots
  class CourseHealthGenerator < Jekyll::Generator
    safe true
    priority :lowest

    CHECKS = [
      ["intro", "Intro"],
      ["prerequisites", "Prerequisites"],
      ["whats_next", "What's next"],
      ["quiz", "Quiz"],
      ["try_it", "Try it yourself"],
      ["common_issues", "Common issues"],
      ["tested_with", "Tested with"],
      ["fresh", "Updated in 18 months"],
      ["in_pathway", "In a pathway"]
    ].freeze

    PATTERNS = {
      "prerequisites" => /prerequisite|what you(?:'|’)?ll need|what you need|you will need|before you (?:start|begin)/i,
      "try_it" => /try it yourself/i,
      "common_issues" => /common (?:issues|problems|mistakes)|troubleshoot/i,
      "tested_with" => /tested (?:with|on)/i
    }.freeze

    NEXT_PATTERN = /\bnext\b|where to go|further|what now|keep learning/i

    def generate(site)
      courses = site.data["courses"] || []
      return if courses.empty?

      lessons = Hash.new { |hash, key| hash[key] = [] }
      site.pages.each do |page|
        next unless page.data["layout"] == "lesson"

        slug = page.url[%r{\A/learn/([^/]+)/}, 1]
        name = File.basename(page.url)
        next if slug.nil? || name == "" || name == "index.html"

        lessons[slug] << page
      end

      pathed = (site.data["pathways"] || []).flat_map { |doc| doc.data["courses"] || [] }
      cutoff = Date.today << 18

      rows = courses.filter_map do |course|
        slug = course["link"].to_s[%r{\A/learn/([^/]+)/}, 1]
        next unless slug

        pages = lessons[slug].sort_by(&:url)
        text = pages.map(&:content).join("\n")
        updated = pages.map { |page| to_date(page.data["date_updated"]) }.compact.max ||
                  to_date(course["date_published"])
        checks = {
          "intro" => pages.any? { |page| File.basename(page.url) =~ /\A0[01]_/ },
          "quiz" => pages.any? { |page| page.data["quiz_total"].to_i.positive? },
          "whats_next" => !pages.empty? && NEXT_PATTERN.match?(pages.last.content.to_s),
          "fresh" => !updated.nil? && updated >= cutoff,
          "in_pathway" => pathed.include?(course["name"])
        }
        PATTERNS.each { |key, pattern| checks[key] = pattern.match?(text) }
        quality = (site.data["course_quality"] || {})[slug] || {}

        {
          "name" => course["name"], "link" => course["link"], "key" => "/learn/#{slug}/",
          "lessons" => pages.size, "updated" => updated&.strftime("%Y-%m-%d"),
          "checks" => CHECKS.map { |key, _label| checks[key] },
          "score" => checks.values.count(true),
          # The 100-point checklist score (#655), from scripts/course_quality.py --write.
          "quality" => quality["score"], "band" => quality["band"],
          "gaps" => (quality["gaps"] || []).join("\n")
        }
      end

      page = Jekyll::PageWithoutAFile.new(site, site.source, "learn/course-health", "index.html")
      page.content = ""
      page.data.merge!(
        "layout" => "course_health",
        "title" => "Course health",
        "description" => "How every course scores on the things that make it good to learn from",
        "sitemap" => false,
        "noindex" => true,
        "hide_likes_and_comments" => true,
        "health_checks" => CHECKS.map { |key, label| { "key" => key, "label" => label } },
        "health_rows" => rows.sort_by { |row| [row["quality"] || 0, row["score"], row["name"].to_s] }
      )
      site.pages << page
    end

    private

    def to_date(value)
      case value
      when Date then value
      when Time then value.to_date
      when String then (Date.parse(value) rescue nil)
      end
    end
  end
end
