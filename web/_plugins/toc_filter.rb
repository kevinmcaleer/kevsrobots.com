module Jekyll
    module TOCFilter
        def toc_only(input)
            # require 'nokogiri'

      # Parse the content with Nokogiri
      doc = Nokogiri::HTML.fragment(input)

      # Find all heading tags
      headings = doc.css('h1, h2, h3, h4, h5, h6')

      # Build the TOC
      toc = '<ul>'
      headings.each do |heading|
        toc << "<li><a href='##{heading['id']}'>#{heading.text}</a></li>"
      end
      toc << '</ul>'

      toc
        end
    end
end

Liquid::Template.register_filter(Jekyll::TOCFilter)