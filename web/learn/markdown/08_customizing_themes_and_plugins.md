---
layout: lesson
title: Customizing Themes and Plugins
author: Kevin McAleer
type: page
cover: /learn/jekyll/assets/customizing_themes_plugins.jpg
date: 2024-03-21
previous: 07_using_liquid_templating.html
next: 09_building_and_testing_locally.html
description: Discover how to personalize your Jekyll site by customizing themes and
  extending functionality with plugins, making your site uniquely yours.
percent: 80
duration: 2
navigation:
- name: Mastering Markdown for Documentation with Jekyll
- content:
  - section: Introduction to Markdown and Jekyll
    content:
    - name: Overview of Jekyll and Static Sites
      link: 01_overview.html
    - name: Introduction to Markdown
      link: 02_markdown_basics.html
    - name: Setting Up Your Environment
      link: 03_setup_environment.html
  - section: Creating Content with Markdown and Jekyll
    content:
    - name: Writing Content in Markdown
      link: 04_writing_content_in_markdown.html
    - name: Working with Jekyll Layouts and Includes
      link: 05_working_with_layouts_and_includes.html
    - name: Managing Static Assets
      link: 06_managing_static_assets.html
  - section: Enhancing Your Site
    content:
    - name: Using Liquid Templating for Dynamic Content
      link: 07_using_liquid_templating.html
    - name: Customizing Themes and Plugins
      link: 08_customizing_themes_and_plugins.html
  - section: Publishing Your Site
    content:
    - name: Building and Testing Your Site Locally
      link: 09_building_and_testing_locally.html
    - name: Deploying to GitHub Pages
      link: 10_deploying_to_github_pages.html
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
