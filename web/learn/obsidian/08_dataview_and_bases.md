---
layout: lesson
title: Dataview and Bases
author: Kevin McAleer
type: page
cover: /learn/obsidian/assets/cover.jpg
date: 2025-08-24
previous: 07_tasks.html
next: 09_properties.html
description: Use the Dataview plugin to create dashboards and databases from your
  notes.
percent: 63
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



## Dataview and Bases

Obsidian's Dataview plugin and the new Bases feature (introduced in version 1.9.10) let you turn your notes into powerful, interactive databases and dashboards.

---

## What is Dataview?

Dataview is a community plugin that allows you to query your notes and display the results as tables, lists, or task lists. You can use it to:
- Create dynamic tables of notes by tag, folder, or property
- Summarize tasks, projects, or reading lists
- Build custom dashboards for your vault

---

**Example: List all notes with the tag #project**

````markdown
```dataview
table file.mtime as "Last Modified"
from #project
sort file.mtime desc
```
````

---

## What are Bases?

Bases are a new core feature in Obsidian 1.9.10 that let you create and manage databases directly in your vault—no plugin required! With Bases, you can:

- Create a database view for any folder or tag
- Add, edit, and filter properties (columns) for your notes
- Sort and group notes visually, like a spreadsheet or Notion database

{% include gallery.html images="assets/bases.jpg" titles="Bases Overview" noborder=true smalltitle=true cols=2 %}

---

**How to Create a Base:**

1. Create a Base:
	- Click the ribbon button: **New base**, or
	- Right‑click empty space in the File Explorer and choose **New base**, or
	- Use the Command Palette: **Bases: New base**.
2. Scope your Base using Filters:
	- Filter by folder (e.g., `file.path startswith "Projects/"`).
	- Filter by tag (e.g., `file has tag "project"`).
3. Add/show properties as columns, sort and group as needed. Edit values directly in Table or Cards views.

Notes:

- Bases are saved as `.base` files and backed by your Markdown files’ YAML properties.
- You can embed Bases and “Pin view…” to lock a specific view when embedding.

---

### Example - pinning a view

````markdown
```base 
views:
  - type: table
    name: table
    order:
      - file.name
      - file.mtime
    sort:
      - property: file.name
        direction: DESC
      - property: file.mtime
        direction: ASC
```
````

---

## Use Cases for Dataview and Bases

- Project management (track status, deadlines, owners)
- Reading lists and book notes
- Habit and goal tracking
- Research and knowledge management

---

## Tips

- Use properties (frontmatter) in your notes for best results (e.g., `status:`, `due:`, `tags:`)
- Combine Dataview queries and Bases for advanced workflows
- Check the [Dataview documentation](https://blacksmithgu.github.io/obsidian-dataview/) for more examples

---

With Dataview and Bases, you can turn your notes into a flexible, powerful database—customize your setup to fit your workflow!

---
