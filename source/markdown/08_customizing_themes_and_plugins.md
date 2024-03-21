---
title: Customizing Themes and Plugins
description: Discover how to personalize your Jekyll site by customizing themes and extending functionality with plugins, making your site uniquely yours.
layout: lesson
type: page
cover: /learn/jekyll/assets/customizing_themes_plugins.jpg
---

![Customizing Themes and Plugins cover image]({{ page.cover }}){:class="cover"}

## Introduction

One of Jekyll's strengths is its flexibility, allowing for extensive customization of themes and the use of plugins to extend site functionality. This lesson explores how to tailor your site's appearance and add new features through customization and plugins.

## Customizing Themes

Themes in Jekyll determine the look and feel of your site. While Jekyll provides a variety of themes, you might want to customize a theme to better suit your personal or brand style.

### Overriding Theme Defaults

1. **Layouts and Includes:** You can override any theme layout or include by creating a file with the same name in your `_layouts` or `_includes` directory.
2. **CSS and JavaScript:** To customize styles or scripts, copy the theme's stylesheets or JavaScript files to your `assets` directory and modify them as needed.

### Adding Custom Styles

For minor changes or additional styles, consider adding a custom stylesheet:

1. Create a new CSS file in your `assets/css` directory.
2. Include the custom stylesheet in your site's head section, after the theme's stylesheets to ensure your rules take precedence.

## Using Plugins

Plugins extend Jekyll's functionality, allowing you to add features such as SEO tags, site search, and more.

### Installing Plugins

1. Add the plugin to your site's `_config.yml` under the `plugins` section:
   ```yaml
   plugins:
     - jekyll-seo-tag
   ```
2. Install the plugin gem if required, and add it to your `Gemfile`:
   ```ruby
   gem 'jekyll-seo-tag'
   ```

### Configuring Plugins

Some plugins require additional configuration in `_config.yml`. Refer to the plugin's documentation for specific setup instructions.

## Practice Exercise

1. Choose a theme for your Jekyll site and customize its layout by overriding the default layout with your version.
2. Add a custom stylesheet to change the theme's color scheme.
3. Install and configure a plugin to add a new feature to your site, such as social media sharing buttons.

## Additional Resources

- [Jekyll Themes Documentation](https://jekyllrb.com/docs/themes/)
- [Jekyll Plugins Directory](https://jekyllrb.com/docs/plugins/)

---
