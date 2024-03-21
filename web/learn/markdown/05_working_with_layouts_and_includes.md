---
layout: lesson
title: Working with Jekyll Layouts and Includes
author: Kevin McAleer
type: page
cover: /learn/jekyll/assets/layouts_includes.jpg
date: 2024-03-21
previous: 04_writing_content_in_markdown.html
next: 06_managing_static_assets.html
description: Learn how to use Jekyll layouts and includes to create reusable website
  components and streamline your site development process.
percent: 50
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


![Jekyll Layouts and Includes cover image]({{ page.cover }}){:class="cover"}

## Introduction

Jekyll's layouts and includes are powerful tools that allow you to abstract common website elements and reuse them across your site. This not only makes your site more consistent but also significantly speeds up development and maintenance.

## Understanding Layouts

Layouts are templates that wrap around your content. They're used to define a common structure for your pages or posts.

### Creating a Layout

1. Layouts are stored in the `_layouts` directory.
2. To create a layout, simply create a new HTML file in this directory. For example, `default.html`.

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>{{ page.title }}</title>
</head>
<body>
    {{ content }}
</body>
</html>
```

In this example, {% raw %}`{{ page.title }}`{% endraw %} dynamically inserts the title of the page or post using it, and `{{ content }}` is where Jekyll inserts the content from your Markdown file.

### Using a Layout

In your Markdown file's front matter, specify the layout you want to use:

{% raw %}
---
layout: default
title: My First Post
---
{% end raw%}

## Working with Includes

Includes allow you to insert snippets of code into your layouts or content. They're perfect for reusable components like headers, footers, and navigation bars.

### Creating an Include

1. Includes are stored in the `_includes` directory.
2. To create an include, add a new file in this directory. For example, `header.html`.

{% raw %}
<header>
    <nav>
        <ul>
            <li><a href="{{ site.baseurl }}/about">About</a></li>
            <li><a href="{{ site.baseurl }}/contact">Contact</a></li>
        </ul>
    </nav>
</header>
{% endraw %}

### Using an Include

To use an include in a layout or another include, use the `include` tag:

{% raw %}
{% include header.html %}
{% endraw %}

## Practice Exercise

1. Create a `default` layout that includes a basic HTML structure.
2. Design a `header.html` and `footer.html` include for navigation and footer content, respectively.
3. Create a new page that uses your `default` layout and includes your `header` and `footer`. Make sure this page contains some basic content to demonstrate the layout and includes in action.

## Additional Resources

- [Jekyll Layouts Documentation](https://jekyllrb.com/docs/layouts/)
- [Jekyll Includes Documentation](https://jekyllrb.com/docs/includes/)

---
