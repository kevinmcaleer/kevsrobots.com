---
layout: lesson
title: Introduction to Markdown
author: Kevin McAleer
type: page
cover: assets/2.png
date: 2024-03-21
previous: 01_overview.html
next: 03_setup_environment.html
description: Learn the basics of Markdown syntax and how to use it to create content
  for your Jekyll site.
percent: 20
duration: 3
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


![Markdown cover image]({{ page.cover }}){:class="cover"}

## What is Markdown?

Markdown is a lightweight markup language with plain-text formatting syntax. Its main goal is to be as readable and understandable as possible. Markdown files have the extension `.md` or `.markdown` and can be converted into HTML by tools like Jekyll, making it an ideal choice for writing content on the web.

---

## Why Use Markdown?

- **Simplicity:** Markdown's syntax is straightforward, making it easy for anyone to learn and use.
- **Flexibility:** It can be converted into HTML, PDFs, and other formats.
- **Compatibility:** Markdown files are compatible with version control systems, facilitating collaboration and changes tracking.

The best feature of Markdown is that its just plain text. You can write Markdown in any text editor and there are no special tags to learn or remember, its *mostly* formatted using punctuation that looks natural.

This entire website is written using Markdown and converted to HTML using Jekyll.

---

## Basic Markdown Syntax

### Headings

Use `#` for headings. The number of `#` symbols before the text corresponds to the level of the heading.

```markdown
# Heading 1
## Heading 2
### Heading 3
#### Heading 4
##### Heading 5
###### Heading 6
```

---

### Emphasis

- Bold: `**bold text**` or `__bold text__`
- Italic: `*italic text*` or `_italic text_`

---

### Lists

- Unordered list: Start a line with `-`, `*`, or `+`.
- Ordered list: Start a line with a number.

```markdown
- Item 1
- Item 2
  - Sub Item 2.1
  - Sub Item 2.2

1. First item
2. Second item
```

---

### Links and Images

- Link: `[Link text](URL)`
- Image: `![Alt text](Image URL)`

---

### Code Blocks and Inline Code

- Inline code: Use single backticks: `` `code` ``.
- Code block: Use triple backticks or indent with four spaces.

{% raw %}
`inline code`

```python
a = 1
```

{% endraw %}

---

## Using Markdown with Jekyll

In Jekyll, Markdown files are processed and converted into HTML, making up the content of your site. You can also use Liquid templating language to incorporate dynamic content into your Markdown files.

---

## Practice Exercise

Create a new Markdown file in your Jekyll site's `_posts` directory. Write a short post that includes a heading, a few paragraphs of text, a list, and a link to Jekyll's documentation.

---
