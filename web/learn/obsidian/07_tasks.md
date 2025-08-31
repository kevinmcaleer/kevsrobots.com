---
layout: lesson
title: Tasks in Obsidian
author: Kevin McAleer
type: page
cover: /learn/obsidian/assets/cover.jpg
date: 2025-08-24
previous: 06_canvas.html
next: 08_dataview_and_bases.html
description: Manage your tasks and to-dos in Obsidian using checkboxes and plugins.
percent: 56
duration: 2
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



## Tasks in Obsidian

Obsidian makes it easy to manage tasks and to-dos right inside your notes. You can use simple Markdown checkboxes for basic lists, or install plugins for advanced task management.

---

## Creating Tasks with Checkboxes

- To create a task, use `- [ ]` at the start of a line:

```markdown
- [ ] Buy groceries
- [ ] Write blog post
- [ ] Review project notes
```

- To mark a task as complete, change it to `- [x]`:

```markdown
- [x] Buy groceries
```

---

## Viewing and Organizing Tasks

- You can add tasks anywhere in your notes—daily notes, project pages, or meeting logs.
- Use tags (e.g., `#todo`, `#urgent`) to group and filter tasks.

---

## The Tasks Plugin

For more advanced features, install the [Tasks plugin](https://github.com/obsidian-tasks-group/obsidian-tasks):

- Query tasks across your entire vault
- Filter by tag, due date, or completion status
- Schedule recurring tasks
- View tasks in a dedicated pane

---

**Example query:**

```markdown
```tasks
not done
tag: #urgent
```
```

---

## Tips

- Use daily notes to keep track of what you need to do each day
- Review completed tasks regularly to track your progress
- Combine tasks with links and tags for powerful project management

---

With Obsidian, you can keep all your tasks and notes in one place—customize your workflow to fit your needs!
