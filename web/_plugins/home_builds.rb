# Data for the homepage "Kev's builds" band (_includes/home/builds.html).
#
# _data/robots.yml and _data/projects.yml only hold a name, link, cover,
# description and date. The YouTube videos for each build already live in the
# front matter of the page the entry links to (`videos:`, a list of video IDs,
# or a single `video:`). This generator follows each link to its page, reads
# those videos, and adds each video's title from _data/youtube.yml, so nothing
# has to be entered twice. An entry whose page isn't in this site (a
# community project, say) can list its own `videos:` in the .yml instead;
# those win over the page's.
#
# It sets site.data["home_builds"] to:
#
#   latest:   robots and projects together, newest first (one entry per page)
#   robots:   robots.yml, newest first
#   projects: projects.yml, newest first
#
# Each entry: name, link, cover, description, date, kind ("Robot" or
# "Project"), videos ([{ "id", "title" }]), video_count.
require "date"

module KevsRobots
  class HomeBuildsGenerator < Jekyll::Generator
    safe true
    priority :low

    PER_LIST = 8

    def generate(site)
      pages = page_index(site)
      titles = video_titles(site)

      robots = entries(site.data["robots"], "Robot", pages, titles)
      projects = entries(site.data["projects"], "Project", pages, titles)

      seen = {}
      latest = (robots + projects).sort_by { |e| e["sort_date"] }.reverse.select do |e|
        key = normalise(e["link"])
        next false if seen[key]
        seen[key] = true
      end

      site.data["home_builds"] = {
        "latest" => latest.first(PER_LIST),
        "robots" => robots.first(PER_LIST),
        "projects" => projects.first(PER_LIST)
      }
    end

    private

    # url -> front matter, for every page, post and collection document.
    def page_index(site)
      index = {}
      docs = site.pages + site.collections.values.flat_map(&:docs)
      docs.each do |doc|
        index[normalise(doc.url)] ||= doc.data
      end
      index
    end

    def video_titles(site)
      titles = {}
      Array(site.data["youtube"]).each do |v|
        next unless v.is_a?(Hash) && v["video_id"]
        titles[v["video_id"].to_s] ||= v["title"].to_s
      end
      titles
    end

    # "/projects/cubie-3/index.html", "/projects/cubie-3/" and
    # "/projects/cubie-3" are the same page; so are "/blog/x.html" and "/blog/x".
    def normalise(url)
      path = url.to_s.sub(%r{\Ahttps?://(www\.)?kevsrobots\.com}, "").split(/[?#]/).first.to_s
      path = path.sub(%r{/index\.html\z}, "/").sub(/\.html\z/, "")
      path = path.chomp("/")
      path.empty? ? "/" : path
    end

    def entries(list, kind, pages, titles)
      Array(list).filter_map do |item|
        next unless item.is_a?(Hash) && item["name"] && item["link"]
        next if item["title"] == "Projects" || item["title"] == "Robots"

        data = pages[normalise(item["link"])] || {}
        ids = Array(item["videos"] || item["video"] || data["videos"] || data["video"]).map(&:to_s).map(&:strip).reject(&:empty?).uniq
        videos = ids.each_with_index.map do |id, i|
          { "id" => id, "title" => titles[id].to_s.empty? ? "Video #{i + 1}" : titles[id] }
        end

        {
          "name" => item["name"].to_s,
          "link" => item["link"].to_s,
          "cover" => item["cover"].to_s,
          "description" => item["description"].to_s,
          "date" => item["date"],
          "sort_date" => to_date(item["date"]),
          "kind" => kind,
          "videos" => videos,
          "video_count" => videos.size
        }
      end.sort_by { |e| e["sort_date"] }.reverse
    end

    def to_date(value)
      case value
      when Date then value
      when Time then value.to_date
      else Date.parse(value.to_s.tr("/", "-"))
      end
    rescue ArgumentError
      Date.new(1970, 1, 1)
    end
  end
end
