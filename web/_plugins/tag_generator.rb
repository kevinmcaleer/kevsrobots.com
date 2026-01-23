# Tag page generator for Jekyll
# Automatically creates a page for each tag used in posts

module Jekyll
  class TagPageGenerator < Generator
    safe true

    def generate(site)
      if site.layouts.key? 'tag_page'
        site.tags.each_key do |tag|
          site.pages << TagPage.new(site, site.source, tag)
        end
      end
    end
  end

  class TagPage < Page
    def initialize(site, base, tag)
      @site = site
      @base = base
      @dir  = File.join('tags', Utils.slugify(tag))
      @name = 'index.html'

      self.process(@name)
      self.read_yaml(File.join(base, '_layouts'), 'tag_page.html')
      self.data['tag'] = tag
      self.data['title'] = "Posts tagged: #{tag}"
      self.data['description'] = "All posts tagged with #{tag}"
    end
  end
end
