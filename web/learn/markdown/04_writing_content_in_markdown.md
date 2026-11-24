---
layout: lesson
title: Writing Content in Markdown
author: Kevin McAleer
type: page
cover: assets/4.png
date: 2024-03-21
previous: 03_setup_environment.html
next: 05_working_with_layouts_and_includes.html
description: Use tables, footnotes and front matter, and learn which Markdown features
  are extensions that only some tools understand.
percent: 36
duration: 7
quiz_total: 35
date_updated: 2026-11-24
navigation:
- name: Mastering Markdown for Documentation with Jekyll
- content:
  - section: Introduction to Markdown and Jekyll
    content:
    - name: Course Overview
      link: 00_intro.html
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
    - name: Summary and Next Steps
      link: 11_summary.html
---


![Writing Content cover image]({{ page.cover }}){:class="cover"}

By the end of this lesson you will be able to write a Jekyll post with front matter, a table, a footnote and a code block, and say which parts are core Markdown and which are extensions.

## Beyond the basics, with a warning

Lesson 2 covered syntax that works everywhere. This lesson adds **extensions**: features that were invented later and that each Markdown tool supports in its own way. Jekyll uses a converter called **kramdown**. Its default setup, which this site and GitHub Pages share, reads GitHub Flavored Markdown (GFM) with a few kramdown extras on top. That means most extensions behave like they do on GitHub.com, but the extras are specific to kramdown, so we will say which is which.

| Feature | Core Markdown? | In Jekyll's kramdown? |
| --- | --- | --- |
| Headings, lists, links, images, code, quotes | Yes | Yes |
| Fenced code blocks with three backticks | No (a CommonMark and GFM addition) | Yes |
| Tables | No | Yes |
| Footnotes | No | Yes |
| Task lists, `- [ ] item` | No | Yes, as checkboxes, because Jekyll reads GFM by default |
| Attribute lists, `{:class="table"}` | No | Yes, but only in kramdown. GitHub shows them as text |
{:class="table table-single"}

Support for tables, footnotes and task lists is not guaranteed in every Markdown tool, such as a note-taking app or a plain-text previewer, so if a feature matters, test it on the tool that will actually show your page.

## Tables

Separate columns with pipes, and put a row of dashes under the header. Colons in that row set alignment:

```markdown
| Servo | Torque (kg.cm) | Notes |
| :---- | :------------: | ----: |
| SG90  | 1.8            | Plastic gears |
| MG90S | 2.2            | Metal gears |
{:class="table table-single"}
```

The `{:class=...}` line is a kramdown extra, not Markdown. It adds CSS classes to the table above it, and this site uses it to style every table. On GitHub.com it would show up as plain text, so leave it out in READMEs.

The pipes do not have to line up in your source, but aligning them makes the file easier to read. There must be at least three dashes in each divider cell.

Here is the rendered result of the example above:

| Servo | Torque (kg.cm) | Notes |
| :---- | :------------: | ----: |
| SG90  | 1.8            | Plastic gears |
| MG90S | 2.2            | Metal gears |
{:class="table table-single"}

The torque figures are typical datasheet values and vary between manufacturers.

## Footnotes

A footnote puts a reference in the text and the note at the bottom of the page:

```markdown
The MG90S has metal gears.[^gears]

[^gears]: Metal gears survive a stalled arm much better than plastic ones.
```

Here it is working. Many servos use plastic gears.[^plastic] Look for the note at the bottom of this page.

## Blockquotes and callouts

Use `>` to quote or to highlight a warning. Add another `>` to nest:

```markdown
> Check polarity before connecting the battery.
>
>> Reversed batteries can destroy a motor driver.
```

## Front matter

A Jekyll file that starts with two lines of three dashes, with settings between them, is processed by Jekyll. That block is **front matter**, written in YAML:

````markdown
---
layout: post
title: "Day 1: printing the chassis"
date: 2026-11-20 18:30:00 +0000
categories: build-log
---

Your Markdown starts here.
````

Without front matter, Jekyll copies the file untouched. With it, Jekyll converts it to HTML and applies a layout. The `layout` and `title` become variables you can use in the layout, for example as `page.title`. You will use those in lesson 7.

## Posts and pages

- **Posts** live in the `_posts` folder, and the file name must be `YYYY-MM-DD-title.md`, such as `2026-11-20-printing-the-chassis.md`. The date in the name sets the post date and its order.
- **Pages** can sit anywhere outside the special folders, such as `about.md` or `parts/index.md`, and are not dated.

## Links and images between pages

Links inside your site can be written as plain paths, such as `[About](/about/)`. To link to a post by its file name, so the link survives a rename, Jekyll provides a Liquid tag. Liquid tags are run by Jekyll before Markdown is converted, so to show one as an example we have to wrap it in `raw` markers. You will not need those when you use the tag for real:

{% raw %}
```markdown
[Read day one]({% post_url 2026-11-20-printing-the-chassis %})
```
{% endraw %}

Lesson 6 shows the right way to build image paths, which depends on where your site is hosted.

## Try it Yourself

1. Write a post called `2026-11-20-first-test.md` in `_posts/` with front matter and a heading.
2. Add a table of the parts in your robot, with the middle column centre-aligned.
3. Add a footnote that says where you bought one part.
4. Challenge: write a task list (`- [ ] Mount motors` and `- [x] Print chassis`) and preview it in Jekyll and in a different Markdown previewer. Do they look the same?

## Common Issues

- **Problem**: My table appears as lines of pipes and dashes.
- **Solution**: Add a blank line before the table, and check the row of dashes under the header.
- **Why**: The converter only recognises a table when the header row is followed by a divider row, and when it starts its own block.

- **Problem**: My new post does not appear on the site.
- **Solution**: Check the file name is `YYYY-MM-DD-title.md`, the file is in `_posts/`, and the date is not in the future.
- **Why**: Jekyll skips future-dated posts by default, and ignores files that do not match the naming pattern.

- **Problem**: The page shows `---` and my settings as text.
- **Solution**: Make sure the front matter is the very first thing in the file, with no blank line before the first `---`.
- **Why**: Jekyll only treats a file as having front matter when it begins with the opening dashes.

- **Problem**: `{:class="table"}` appears under my table on GitHub.
- **Solution**: Remove it for GitHub, or keep two versions of the file.
- **Why**: It is a kramdown feature, and GitHub does not know it.

## Check your understanding

{% quiz %}
? Which of these is an extension rather than core Markdown?
- [ ] A heading
- [x] A table
- [ ] A bulleted list
> Tables were added by later flavours, so support varies between tools.

? What must a Jekyll post's file name look like?
- [ ] `title.md`
- [x] `YYYY-MM-DD-title.md`
- [ ] `post.html`
> The date in the file name sets the post's date, and Jekyll ignores posts that do not match.

? What is front matter?
- [ ] The first heading on the page
- [x] YAML settings between two `---` lines at the top of a file
- [ ] A footnote
> Front matter tells Jekyll to process the file and gives it variables such as the layout and title.

? Why can a `{:class="..."}` line be a problem?
- [x] It is kramdown-specific and appears as text on GitHub
- [ ] It deletes the table
- [ ] It only works on posts
> The lesson explains it is a kramdown extra that GitHub does not understand.
{% endquiz %}

[^plastic]: Plastic gears are light and cheap, but they can strip if the arm is forced.
