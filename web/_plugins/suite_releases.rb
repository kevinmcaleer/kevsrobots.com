# frozen_string_literal: true

# "What's new" across the kevsrobots suite (#581).
#
# At build time this reads the release notes of every app in _data/suite.yml
# that has a `repo:` (Snakie and BetterCAD) and puts them, newest first, in
# site.data["whats_new"] for /whats-new/:
#
#   1. The repo's published GitHub Releases.
#   2. If it has none yet, the released sections of its CHANGELOG.md
#      ("## [0.91.0] - 2026-09-20" or "## v1.0.0 - 2026-12-01").
#   3. If GitHub can't be reached (offline, rate-limited), the copy committed
#      in _data/whats_new.yml, so the page is never empty.
#
# Set KR_OFFLINE=1 (or `suite_releases: false` in _config.yml) to skip the
# network and use the committed copy. GITHUB_TOKEN is used when set, which
# lifts GitHub's limit of 60 unauthenticated requests an hour.
#
# Refresh the committed copy with:
#   ruby web/_plugins/suite_releases.rb --write web/_data/whats_new.yml
#   ruby web/_plugins/suite_releases.rb --write web/_data/whats_new.yml snakie=../snakie/CHANGELOG.md

require "json"
require "net/http"
require "uri"
require "date"
require "yaml"

module KevsRobots
  module SuiteReleases
    PER_APP = 6
    BODY_LIMIT = 6000
    TIMEOUT = 8

    module_function

    # One entry per release, newest first.
    def collect(apps, offline: false, log: nil)
      entries = []
      apps.each do |app|
        repo = app["repo"].to_s
        next if repo.empty?

        found = offline ? [] : from_releases(app, log)
        found = from_changelog(app, fetch_changelog(repo, log), log) if found.empty? && !offline
        entries.concat(found.first(PER_APP))
      end
      sort(entries)
    end

    def sort(entries)
      entries.sort_by { |e| [e["date"].to_s, e["version"].to_s] }.reverse
    end

    def from_releases(app, log)
      body = get("https://api.github.com/repos/#{app['repo']}/releases?per_page=#{PER_APP}", log, json: true)
      return [] unless body

      list = JSON.parse(body)
      return [] unless list.is_a?(Array)

      list.reject { |r| r["draft"] }.map do |r|
        version = r["tag_name"].to_s
        entry(app,
              version: version,
              title: r["name"].to_s.strip.empty? ? version : r["name"].to_s.strip,
              date: r["published_at"].to_s[0, 10],
              url: r["html_url"],
              body: r["body"].to_s)
      end
    rescue JSON::ParserError => e
      log&.call("#{app['repo']}: bad releases JSON (#{e.message})")
      []
    end

    def fetch_changelog(repo, log)
      %w[main master].each do |ref|
        text = get("https://raw.githubusercontent.com/#{repo}/#{ref}/CHANGELOG.md", log)
        return text if text
      end
      nil
    end

    HEADING = /^##\s+\[?v?(\d+\.\d+(?:\.\d+)?(?:[-+][\w.]+)?)\]?(?:\s*[-–—]\s*(\d{4}-\d{2}-\d{2}))?\s*$/

    # Released sections only: "Unreleased" ones are skipped.
    def from_changelog(app, text, _log = nil)
      return [] if text.nil? || text.empty?

      text = text.dup.force_encoding("UTF-8")

      sections = []
      current = nil
      text.each_line do |line|
        if line.start_with?("## ")
          sections << current if current
          m = HEADING.match(line.strip)
          current = m && !line.downcase.include?("unreleased") ? { version: m[1], date: m[2], lines: [] } : nil
        elsif line =~ /^\[[^\]]+\]:\s*http/ # compare-link footer
          next
        elsif current
          current[:lines] << line
        end
      end
      sections << current if current

      sections.first(PER_APP).map do |s|
        version = "v#{s[:version]}"
        entry(app,
              version: version,
              title: "#{app['name']} #{version}",
              date: s[:date].to_s,
              url: "https://github.com/#{app['repo']}/blob/HEAD/CHANGELOG.md",
              body: s[:lines].join.strip)
      end
    end

    def entry(app, version:, title:, date:, url:, body:)
      body = body.gsub("\r\n", "\n").strip
      body = "#{body[0, BODY_LIMIT].sub(/\n[^\n]*\z/, '')}\n\n…" if body.length > BODY_LIMIT
      {
        "app" => app["id"],
        "app_name" => app["name"],
        "version" => version,
        "title" => title,
        "date" => date,
        "url" => url,
        "body" => body
      }
    end

    def get(url, log, json: false, redirects: 3)
      uri = URI(url)
      req = Net::HTTP::Get.new(uri)
      req["User-Agent"] = "kevsrobots-site-build"
      req["Accept"] = "application/vnd.github+json" if json
      token = ENV["GITHUB_TOKEN"].to_s
      req["Authorization"] = "Bearer #{token}" if json && !token.empty?
      res = Net::HTTP.start(uri.host, uri.port, use_ssl: uri.scheme == "https",
                                                open_timeout: TIMEOUT, read_timeout: TIMEOUT) do |http|
        http.request(req)
      end
      case res
      when Net::HTTPSuccess then res.body.to_s.dup.force_encoding("UTF-8")
      when Net::HTTPRedirection
        redirects.positive? ? get(res["location"], log, json: json, redirects: redirects - 1) : nil
      else
        log&.call("#{url}: HTTP #{res.code}") unless res.code == "404"
        nil
      end
    rescue StandardError => e
      log&.call("#{url}: #{e.class}: #{e.message}")
      nil
    end
  end
end

if defined?(Jekyll)
  module KevsRobots
    class SuiteReleasesGenerator < Jekyll::Generator
      safe true
      priority :high

      def generate(site)
        committed = Array(site.data["whats_new"])
        offline = ENV["KR_OFFLINE"] == "1" || site.config["suite_releases"] == false
        log = ->(msg) { Jekyll.logger.warn("What's new:", msg) }
        apps = Array(site.data["suite"])

        fresh = offline ? [] : SuiteReleases.collect(apps, log: log)
        # Per app: the fresh list when we got one, else what's committed.
        got = fresh.map { |e| e["app"] }.uniq
        kept = committed.reject { |e| got.include?(e["app"]) }
        site.data["whats_new"] = SuiteReleases.sort(fresh + kept)
        Jekyll.logger.info("What's new:", "#{fresh.size} from GitHub, #{kept.size} from _data/whats_new.yml")
      end
    end
  end
end

# ruby suite_releases.rb --write OUT.yml [app=CHANGELOG.md ...]
if $PROGRAM_NAME == __FILE__
  out = ARGV[ARGV.index("--write").to_i + 1] if ARGV.include?("--write")
  abort "usage: ruby #{File.basename(__FILE__)} --write OUT.yml [app=path/to/CHANGELOG.md ...]" unless out

  suite = YAML.safe_load(File.read(File.join(__dir__, "..", "_data", "suite.yml"), encoding: "UTF-8"))
  local = ARGV.grep(/=/).to_h { |a| a.split("=", 2) }
  log = ->(msg) { warn msg }
  entries = suite.select { |a| a["repo"] }.flat_map do |app|
    if local[app["id"]]
      KevsRobots::SuiteReleases.from_changelog(app, File.read(local[app["id"]], encoding: "UTF-8")).first(KevsRobots::SuiteReleases::PER_APP)
    else
      KevsRobots::SuiteReleases.collect([app], log: log)
    end
  end
  header = "# What's new fallback (#581): used when the build can't reach GitHub.\n" \
           "# Regenerate with: ruby web/_plugins/suite_releases.rb --write web/_data/whats_new.yml\n"
  File.write(out, header.encode("UTF-8") + KevsRobots::SuiteReleases.sort(entries).to_yaml(line_width: -1))
  puts "#{entries.size} entries written to #{out}"
end
