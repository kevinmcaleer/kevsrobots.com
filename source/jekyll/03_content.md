---
title: Content Management in Jekyll
description: Learn how to effectively manage content in Jekyll, including writing posts, using Markdown, and understanding front matter.
layout: lesson
type: page
---

## Introduction

In this lesson, we'll delve into the heart of managing content in Jekyll. You'll learn how to create and organize posts and pages, use Markdown for easy content creation, and leverage Jekyll's front matter and data files for advanced content management.

---

## Part 1: Writing Posts and Pages

Jekyll simplifies the process of creating posts and pages. Here's how you can start:

---

### Creating a New Post

- Posts are typically blog entries.
- To create a post, add a new file in the `_posts` directory.
- Name the file in the format `YYYY-MM-DD-title.md`.
- Start with front matter, for example:
  
  ```
  ---
  layout: post
  title: "My First Post"
  date: 2024-01-25
  categories: tutorial
  ---
  ```

---

### Creating a New Page

- Pages are more static and not date-based like posts.
- To add a page, create a new markdown file in your root directory or in a subdirectory.
- For example, `about.md` for an About page. Start with front matter too.

---

## Part 2: Markdown Basics

Markdown is a lightweight markup language that allows you to write using an easy-to-read, easy-to-write plain text format.

---

### Basic Markdown Syntax

- **Headings**: Use `#` for headings. More `#`s mean a lower level heading.
- **Paragraphs**: Write plain text in paragraphs.
- **Bold and Italic**: Use `**bold**` for bold text, `*italic*` for italic text.
- **Links**: `[Link text](URL)` to create hyperlinks.
- **Images**: `![Alt text](image URL)` to insert images.

---

## Part 3: Front Matter and Data Files

Front matter allows you to set variables for your page or post, which you can then access using Jekyll's templating system.

---

### Using Front Matter

- It's written in YAML and sits between two triple-dashed lines.
- Example:

  ```
  ---
  layout: page
  title: "About"
  ---
  ```

---

### Data Files

- Jekyll supports data files in YAML, JSON, and CSV formats.
- Store these files in the `_data` directory.
- Access the data in these files using Liquid templating.

> ## Liquid Templating
>
>Liquid is a templating language developed by Shopify. Jekyll uses it to process templates, allowing you to use variables, logic, and filters in your content.
> for example:
>
>```markdown
>  {% raw %}{% for item in site.data.mydata %}
>    {{ item.name }}
>  {% endfor %}{% endraw %}
>```
>
> Note how the logic is wrapped in `{% raw %}{% %}{% endraw %}` and the variables are wrapped in `{% raw %}{{ }}{% endraw %}`.

In this example, `mydata` is the name of the data file, and `name` is a variable in the data file. The `for` loop iterates through the data file and prints the value of the `name` variable.

---

## Part 4: Organizing Content

Organizing your content helps visitors navigate your site easily.

---

### Categories and Tags

- Use categories and tags in your post's front matter for organization.
- Example:

  ```
  categories: tutorial
  tags: jekyll markdown
  ```

---

## Conclusion

You now have the knowledge to manage content in Jekyll effectively. Experiment with writing posts, using Markdown, and organizing your site's content.

---

## Additional Resources

- [Markdown Guide](https://www.markdownguide.org/)
- [Jekyll Docs on Posts](https://jekyllrb.com/docs/posts/)
- [Jekyll Data Files](https://jekyllrb.com/docs/datafiles/)

---

## Assignment

Create a blog post about a topic of your choice using Markdown. Include front matter, and categorize and tag your post.

---
