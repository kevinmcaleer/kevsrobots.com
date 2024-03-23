---
title: Writing Content in Markdown
description: Learn advanced Markdown syntax and best practices for creating engaging content for your Jekyll site.
layout: lesson
type: page
cover: assets/4.png
---

![Writing Content cover image]({{ page.cover }}){:class="cover"}

## Introduction

With a foundational understanding of Markdown and your environment set up, it's time to start creating content for your Jekyll site. This lesson will cover advanced Markdown techniques and how to format your content effectively for the web.

## Advanced Markdown Syntax

### Tables

Create tables by aligning columns with dashes (`-`) and separating them with pipes (`|`):

```markdown
| Header 1 | Header 2 | Header 3 |
| -------- | -------- | -------- |
| Row 1    | Data     | Data     |
| Row 2    | Data     | Data     |
```

### Footnotes

Add footnotes by using `[^1]` for the reference in the text and `[^1]:` for the footnote definition:

```markdown
This is a sentence with a footnote.[^1]

[^1]: Here is the footnote.
```

### Blockquotes

Use `>` for blockquotes, and you can nest them by adding additional `>`:

```markdown
> This is a blockquote.
>
>> Nested blockquote.
```

## Structuring Jekyll Content

### Front Matter

Every Jekyll document begins with front matter, which is written in YAML and enclosed between triple-dashed lines. Here you can set variables like the layout, title, and custom variables:

{% raw %}

---
layout: post
title: "My First Post"
date: 2024-03-20 19:32:00 -0000
categories: jekyll update
---

{% endraw%}

### Posts and Pages

- **Posts** are typically used for blog entries. Jekyll automatically processes files in the `_posts` directory and supports post categories and tags.
- **Pages** are static and can be located anywhere in the site's directory, except for directories that Jekyll processes specially (like `_posts`).

## Using Images and Links

In Markdown, you can embed images and links easily, but with Jekyll, you can also take advantage of site variables:

{% raw %}

![My Image]({{ site.url }}/assets/my_image.jpg)
[Link to a post]({{ site.url }}{% post_url 2024-03-20-my-first-post %})

{% endraw %}

## Practice Exercise

Create a new post for your Jekyll site that includes:
- A title and custom front matter
- Headings, paragraphs, and a blockquote
- A table and a list
- An image and a link to another post or page on your site

Publish this post by placing it in the `_posts` directory and running your site locally to view it.

## Additional Resources

- [Markdown Cheat Sheet](https://www.markdownguide.org/cheat-sheet/)
- [Jekyll Documentation on Posts](https://jekyllrb.com/docs/posts/)

---
