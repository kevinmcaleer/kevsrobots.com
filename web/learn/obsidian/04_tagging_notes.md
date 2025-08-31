---
layout: lesson
title: Tagging Notes
author: Kevin McAleer
type: page
cover: /learn/obsidian/assets/cover.jpg
date: 2025-08-24
previous: 03_creating_notes.html
next: 05_linking_notes.html
description: How to use tags to organize your notes in Obsidian.
percent: 35
duration: 3
navigation:
- name: Obsidian
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Getting Started with Obsidian
    content:
    - name: Installing Obsidian
      link: 01_installing.html
    - name: Creating Your First Vault
      link: 02_first_vault.html
  - section: Taking Notes and Organization
    content:
    - name: Creating Notes
      link: 03_creating_notes.html
    - name: Tagging Notes
      link: 04_tagging_notes.html
    - name: Linking Notes
      link: 05_linking_notes.html
  - section: Advanced Features
    content:
    - name: Canvas in Obsidian
      link: 06_canvas.html
    - name: Tasks in Obsidian
      link: 07_tasks.html
    - name: Dataview and Bases
      link: 08_dataview_and_bases.html
    - name: Properties in Obsidian
      link: 09_properties.html
    - name: Graph View in Obsidian
      link: 12_graph_view.html
  - section: Markdown in Obsidian
    content:
    - name: Markdown in Obsidian
      link: 10_markdown_in_obsidian.html
    - name: 'Learn More: Markdown Course'
      link: 11_markdown_course_link.html
---



## Tagging Notes

Tags are a powerful way to organize, group, and quickly find your notes in Obsidian. You can add tags anywhere in your note using the `#` symbol, and Obsidian will automatically recognize and index them.

---

## How to Add Tags

- Simply type `#` followed by your tag name, e.g., `#project`, `#meeting`, or `#idea`.
- Tags can be placed anywhere in your note—at the top, bottom, or [inline](#inline) with your text.
- Tags can also be placed in the [frontmatter](#in-frontmatter) of your note

---

### Inline

```text
#project #python
This note contains ideas for my next Python project.
```

---

### In FrontMatter

```text
---
tags:
  - project
  - python
---
```

---

> ## What is FrontMatter?
>
> FrontMatter is a section at the beginning of a note where you can define metadata, including tags, aliases, and other properties.
>
> It is enclosed in triple dashes (`---`) and allows for better organization and searchability of your notes.
>
> By default, notes do not contain any front matter, begin adding it by typing the triple dashes on the first line after the note title.
>
> For example:
>
> ```
> ---
> tags:
>   - project
>   - python
> date: 2023-10-01
> author: Kevin McAleer
> topics:
>   - tagging
>   - organization
> ---
> ```

---

## Useful Tag Ideas

- `#todo` — for notes that need action
- `#reference` — for notes with useful information
- `#meeting` — for meeting notes
- `#journal` — for daily or weekly logs
- `#idea` — for brainstorming and creative thoughts
- `#work`, `#personal`, `#study` — for different life areas

---

## Structuring Tags

You can use nested tags for more structure. For example:

- `#project/active`
- `#project/completed`
- `#reading/book`
- `#reading/article`

Obsidian will display these as a hierarchy in the tags pane.

---

## Viewing and Managing Tags

<div class="row">
{% include gallery.html images="assets/tags_pane.jpg" titles="The Tags Pane" noborder=true smalltitle=true cols=2 %}
</div>


Open the **Tags Pane** from the sidebar (click the tag icon). Here you can:

- See a list of all tags in your vault
- View tag hierarchies (for nested tags)
- Click a tag to see all notes containing it

You can also search for tags using the search bar, or combine tags in search queries (e.g., `#project #todo`).

---

## Tips

- Be consistent with your tag names and structure for easier searching.
- Use tags for broad categories and folders for more rigid organization.

---

Tags make it easy to group, filter, and find your notes—experiment with different tag strategies to see what works best for you!

---
