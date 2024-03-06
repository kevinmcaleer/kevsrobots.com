module Jekyll
    class MermaidBlock < Liquid::Block
      def initialize(tag_name, text, tokens)
        super
      end
  
      def render(context)
        "<pre class=\"mermaid\">#{super}</pre>"
      end
    end
  end
  
  Liquid::Template.register_tag('mermaid', Jekyll::MermaidBlock)
  