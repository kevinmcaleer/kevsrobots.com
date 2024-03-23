---
title: Using Liquid Templating for Dynamic Content
description: Explore how to use the Liquid templating language to add dynamic content and logic to your Jekyll site, enhancing its interactivity and functionality.
layout: lesson
type: page
cover: assets/7.png
---

![Using Liquid Templating for Dynamic Content cover image]({{ page.cover }}){:class="cover"}

## Introduction

Liquid is a templating language used by Jekyll to process templates. It allows you to add dynamic content, use variables, and implement logic in your site's pages and posts. Understanding Liquid is essential for maximizing the power of Jekyll as a static site generator.

## Basic Syntax

Liquid syntax is simple and consists of objects, tags, and filters:

- **Objects** output content into the template and are denoted by double curly braces: {% raw %}`{{ }}`{% endraw %}.
- **Tags** create logic and control flow in templates and are denoted by curly braces and percent signs: {% raw %}`{% %}`{% endraw %}.
- **Filters** modify the output of objects and are used within an object tag: {% raw %}`{{ 'data' | filter }}`{% endraw %}.

## Displaying Data with Objects

You can use objects to display data from your site's configuration file (`_config.yml`), front matter, or page content:

{% raw %}
{{ site.title }}
{{ page.author }}
{% endraw %}

## Implementing Logic with Tags

Tags allow you to implement control flow statements like `if`, `else`, `elsif`, and loops using `for`.

### Conditional Statements

{% raw %}
{% if page.title %}
  <h1>{{ page.title }}</h1>
{% else %}
  <h1>Untitled Page</h1>
{% endif %}
{% endraw %}

### Loops

To iterate over a list of items, such as posts:

{% raw %}
{% for post in site.posts %}
  <article>
    <h2>{{ post.title }}</h2>
    <p>{{ post.excerpt }}</p>
  </article>
{% endfor %}
{% endraw %}

## Using Filters to Modify Output

Filters allow you to modify the output of objects for formatting or transforming data:

{% raw %}
{{ 'Welcome to Jekyll!' | upcase }}
{% endraw %}

This filter changes the string to uppercase.

## Practice Exercise

1. Create a new page in your Jekyll site that lists the titles of all posts.
2. Use conditional logic to display a message if there are no posts.
3. Apply a filter to the post titles to alter their appearance.

## Additional Resources

- [Liquid Templating Language Documentation](https://shopify.github.io/liquid/)
- [Jekyll Variables and Liquid Tags](https://jekyllrb.com/docs/variables/)

---
