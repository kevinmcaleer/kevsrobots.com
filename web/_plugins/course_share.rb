# frozen_string_literal: true

# Course social cards and "I completed it" share pages.
#
# For every course in _data/courses.yml (written by build.py) whose cards exist
# in assets/img/learn/cards/ (written by course_builder/social_cards.py):
#
#   * every lesson page under /learn/<slug>/ gets
#       page.og_image      /assets/img/learn/cards/<slug>.jpg (head.html uses it
#                          for og:image / twitter:image instead of the cover)
#       page.course_share  the course record plus slug, share_url and
#                          completed_card, for the "Course complete" prompt
#
#   * a page /learn/<slug>/completed.html is added (layout: course_completed).
#     That is the link learners share when they finish: its title says
#     "I completed <course>", its og:image is the completed card, and it leads
#     with the course video so whoever clicks it lands on something to watch.
#
# A course without cards is left alone, so a new course just needs build.py.

module KevsRobots
  class CourseShareGenerator < Jekyll::Generator
    safe true
    priority :low

    CARDS = "assets/img/learn/cards"

    def generate(site)
      courses = site.data["courses"] || []
      by_slug = {}

      courses.each do |course|
        slug = course["link"].to_s[%r{\A/learn/([^/]+)/}, 1]
        next unless slug

        card = "/#{CARDS}/#{slug}.jpg"
        completed_card = "/#{CARDS}/#{slug}-completed.jpg"
        next unless File.exist?(File.join(site.source, card)) &&
                    File.exist?(File.join(site.source, completed_card))

        share_path = "/learn/#{slug}/completed.html"
        by_slug[slug] = course.merge(
          "slug" => slug,
          "card" => card,
          "completed_card" => completed_card,
          "share_path" => share_path,
          "share_url" => "#{site.config['url']}#{share_path}"
        )
      end

      site.pages.each do |page|
        next unless page.data["layout"] == "lesson"

        slug = page.url[%r{\A/learn/([^/]+)/}, 1]
        info = slug && by_slug[slug]
        next unless info

        page.data["og_image"] ||= info["card"]
        page.data["course_share"] = info
      end

      # Pathways each course is part of, for "Where next" on the completed page
      (site.data["pathways"] || []).each do |pathway|
        (pathway.data["course_items"] || []).each do |course|
          slug = course["link"].to_s[%r{\A/learn/([^/]+)/}, 1]
          next unless by_slug[slug]

          (by_slug[slug]["pathways"] ||= []) << {
            "title" => pathway.data["title"], "url" => pathway.url,
            "course_count" => pathway.data["course_count"]
          }
        end
      end

      by_slug.each_value do |info|
        site.pages << completed_page(site, info)
      end
    end

    private

    def completed_page(site, info)
      page = Jekyll::PageWithoutAFile.new(site, site.source, "learn/#{info['slug']}", "completed.html")
      page.content = ""
      page.data.merge!(
        "layout" => "course_completed",
        "title" => "I completed #{info['name']} on KevsRobots",
        "description" => info["description"],
        "og_image" => info["completed_card"],
        "course_share" => info,
        "sitemap" => false,
        "hide_likes_and_comments" => true
      )
      page
    end
  end
end
