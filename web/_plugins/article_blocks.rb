# Layout blocks for Markdown pages.
#
# Block tags that wrap ordinary Markdown in a richer layout. They work on any
# page, and are styled by assets/css/article.scss (loaded by the `article`
# layout). See docs/article_layout.md for the cheat sheet.
#
#   {% callout tip "Title" %} ... {% endcallout %}
#   {% split image="/path.jpg" alt="..." side="right" caption="..." %} ... {% endsplit %}
#   {% columns %} {% column %} ... {% endcolumn %} {% column %} ... {% endcolumn %} {% endcolumns %}
#   {% cards %} {% card title="..." image="..." link="..." %} ... {% endcard %} {% endcards %}
#   {% fullwidth style="dark" image="/path.jpg" %} ... {% endfullwidth %}
#
# Inner content stays Markdown: each wrapper is emitted with markdown="1" so
# kramdown converts what's inside it.

module KevsRobots
  module ArticleBlocks
    ARG_PATTERN = /(\w[\w-]*)\s*=\s*(?:"([^"]*)"|'([^']*)'|(\S+))|"([^"]*)"|'([^']*)'|(\S+)/

    # Parses `word "quoted" key="value" key=value` into [positional, named].
    def self.parse_args(markup)
      positional = []
      named = {}
      markup.to_s.scan(ARG_PATTERN) do |key, dq, sq, bare, pdq, psq, pbare|
        if key
          named[key] = dq || sq || bare
        else
          positional << (pdq || psq || pbare)
        end
      end
      [positional, named]
    end

    # Resolves a relative image path against the page's own folder, so images
    # can sit next to the Markdown file. Absolute paths and URLs pass through.
    def self.resolve(path, context)
      return nil if path.nil? || path.empty?
      return path if path.start_with?("/", "http://", "https://", "data:")
      page_url = context.registers[:page] && context.registers[:page]["url"]
      dir = page_url.to_s.end_with?("/") ? page_url.to_s : File.dirname(page_url.to_s)
      File.join(dir, path)
    end

    def self.h(text)
      CGI.escapeHTML(text.to_s)
    end

    # Wraps rendered Markdown so kramdown treats it as Markdown, not raw HTML.
    def self.md(body)
      "\n\n#{body.strip}\n\n"
    end

    class Block < Liquid::Block
      def initialize(tag_name, markup, tokens)
        super
        @positional, @named = ArticleBlocks.parse_args(markup)
      end

      def arg(name, index = nil, default = nil)
        value = @named[name]
        value = @positional[index] if value.nil? && index
        value.nil? || value.empty? ? default : value
      end
    end

    class Callout < Block
      KINDS = {
        "note" => ["Note", "fa-solid fa-circle-info"],
        "tip" => ["Tip", "fa-solid fa-lightbulb"],
        "warning" => ["Warning", "fa-solid fa-triangle-exclamation"],
        "danger" => ["Danger", "fa-solid fa-bolt"],
        "try" => ["Try it", "fa-solid fa-flask"]
      }.freeze

      def render(context)
        kind = arg("type", 0, "note").downcase
        kind = "note" unless KINDS.key?(kind)
        label, icon = KINDS[kind]
        title = arg("title", 1, label)
        "\n<aside class=\"kr-callout kr-callout-#{kind}\" markdown=\"1\">\n" \
          "<p class=\"kr-callout-title\"><i class=\"#{icon}\" aria-hidden=\"true\"></i> #{ArticleBlocks.h(title)}</p>" \
          "#{ArticleBlocks.md(super)}</aside>\n"
      end
    end

    class Split < Block
      def render(context)
        src = ArticleBlocks.resolve(arg("image", 0), context)
        alt = arg("alt", nil, "")
        side = arg("side", nil, "left") == "right" ? "right" : "left"
        caption = arg("caption")
        figure = ""
        if src
          figure = "<figure class=\"kr-split-media\"><img src=\"#{ArticleBlocks.h(src)}\" alt=\"#{ArticleBlocks.h(alt)}\" class=\"kr-zoom\" loading=\"lazy\">"
          figure += "<figcaption>#{ArticleBlocks.h(caption)}</figcaption>" if caption
          figure += "</figure>"
        end
        "\n<div class=\"kr-split kr-split-#{side}\">#{figure}<div class=\"kr-split-text\" markdown=\"1\">" \
          "#{ArticleBlocks.md(super)}</div></div>\n"
      end
    end

    class Columns < Block
      def render(context)
        count = arg("count", 0)
        style = count.to_s =~ /\A[2-4]\z/ ? " style=\"--kr-cols: #{count}\"" : ""
        "\n<div class=\"kr-columns\"#{style}>#{super}</div>\n"
      end
    end

    class Column < Block
      def render(context)
        "<div class=\"kr-column\" markdown=\"1\">#{ArticleBlocks.md(super)}</div>"
      end
    end

    class Cards < Block
      def render(context)
        count = arg("count", 0)
        style = count.to_s =~ /\A[2-4]\z/ ? " style=\"--kr-cols: #{count}\"" : ""
        "\n<div class=\"kr-cards\"#{style}>#{super}</div>\n"
      end
    end

    class Card < Block
      def render(context)
        title = arg("title", 0)
        src = ArticleBlocks.resolve(arg("image"), context)
        link = arg("link")
        label = arg("label")
        body = super
        tag = link ? "a" : "div"
        href = link ? " href=\"#{ArticleBlocks.h(link)}\"" : ""
        html = "<#{tag} class=\"kr-card\"#{href}>"
        html += "<img src=\"#{ArticleBlocks.h(src)}\" alt=\"\" loading=\"lazy\">" if src
        html += "<div class=\"kr-card-body\" markdown=\"1\">"
        html += "<p class=\"kr-card-label\">#{ArticleBlocks.h(label)}</p>" if label
        html += "<p class=\"kr-card-title\">#{ArticleBlocks.h(title)}</p>" if title
        html += "#{ArticleBlocks.md(body)}</div></#{tag}>"
        html
      end
    end

    class FullWidth < Block
      STYLES = %w[dark light red photo].freeze

      def render(context)
        src = ArticleBlocks.resolve(arg("image"), context)
        style = arg("style", 0, src ? "photo" : "dark")
        style = "dark" unless STYLES.include?(style)
        bg = src ? " style=\"--kr-band-image: url('#{ArticleBlocks.h(src)}')\"" : ""
        "\n<section class=\"kr-band kr-band-#{style}\"#{bg}><div class=\"kr-band-inner\" markdown=\"1\">" \
          "#{ArticleBlocks.md(super)}</div></section>\n"
      end
    end
  end
end

require "cgi"

Liquid::Template.register_tag("callout", KevsRobots::ArticleBlocks::Callout)
Liquid::Template.register_tag("split", KevsRobots::ArticleBlocks::Split)
Liquid::Template.register_tag("columns", KevsRobots::ArticleBlocks::Columns)
Liquid::Template.register_tag("column", KevsRobots::ArticleBlocks::Column)
Liquid::Template.register_tag("cards", KevsRobots::ArticleBlocks::Cards)
Liquid::Template.register_tag("card", KevsRobots::ArticleBlocks::Card)
Liquid::Template.register_tag("fullwidth", KevsRobots::ArticleBlocks::FullWidth)
