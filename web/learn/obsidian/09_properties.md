---
layout: lesson
title: Properties in Obsidian
author: Kevin McAleer
type: page
cover: /learn/obsidian/assets/cover.jpg
date: 2025-08-24
previous: 08_dataview_and_bases.html
next: 12_graph_view.html
description: Learn how to use properties (frontmatter) for metadata and organization.
percent: 70
duration: 5
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


## Properties in Obsidian

Properties (formerly called frontmatter) are key–value metadata stored at the top of your notes. They power search, filters, Dataview, and Bases, and make it easier to organize and query your vault.

---

## What are properties?

Properties live in a YAML block at the very top of a note, surrounded by `---` lines. Many can also be edited via the Properties UI in Obsidian without touching YAML.

---

## Common properties

- `title`: Overrides the display title of the note
- `description`: Short summary shown in listings
- `tags`: A list of tags (must be a list, not a single string)
- `aliases`: Alternative names for the note (list)
- `cssclasses`: Extra CSS classes to style a note (list)
- `created` / `updated`: Dates for tracking when a note was created/modified
- `status`: Your custom workflow state, e.g., draft, in-progress, done
- `due`: A date for tasks or projects
- `cover`: Path to an image used as a cover/thumbnail

Note: As of Obsidian 1.9, the old singular properties `tag`, `alias`, and `cssclass` are deprecated. Use the plural forms above and make them lists.

---

## Property types (with examples)

### Text
Use for free-form strings like titles, authors, statuses, etc.

YAML:
```yaml
author: "Ada Lovelace"
status: "in-progress"
notes: |
  Multi-line text is possible using YAML block scalars.
  Each line is preserved until the indentation ends.
```

Tips:
- Wrap text in quotes when it contains punctuation or colons.
- Use block scalars (|) for multi-line descriptions.

### Number
Use for anything you’ll sort or compute on numerically.

YAML:
```yaml
priority: 1
estimate_hours: 12.5
```

Tips:
- Don’t quote numbers if you want them treated as numbers.

### Checkbox (boolean)
A simple true/false toggle.

YAML:
```yaml
done: false
archived: true
```

Tips:
- YAML recognizes `true/false` (lowercase) as booleans.

### Date and Datetime
Track when things were created, due, or updated.

YAML:
```yaml
created: 2025-08-24
updated: 2025-08-24
meeting_time: 2025-08-24T14:30
```

Dataview example:
```dataview
table created, updated
from "Projects"
where created >= date(2025-01-01)
sort created desc
```

Tips:
- Prefer ISO format (YYYY-MM-DD) and ISO datetime (YYYY-MM-DDTHH:MM) for reliable sorting.

### List (multi‑value)
Use when you need multiple values for the same property.

YAML (block list):
```yaml
stakeholders:
  - Alice
  - Bob
```

YAML (inline list):
```yaml
platforms: [ios, android, web]
```

Tips:
- Use lists for `tags`, `aliases`, and `cssclasses`.
- Inline and block styles are equivalent—pick one for consistency.

### Tags
Special list used across Obsidian.

YAML:
```yaml
tags:
  - project
  - planning
  - projects/mobile
```

Tips:
- Always a list. Prefer nested tags for hierarchy (e.g., `projects/mobile`).

### Links to notes and files
Store relationships to other notes or attach files.

YAML:
```yaml
related:
  - "[[Design Spec]]"
  - "[[Project Charter]]"
cover_file: "[[Assets/cover.png]]"
```

Tips:
- Quote wiki-links in YAML to avoid parsing quirks.
- In the Properties UI, choose the Link/File type so Obsidian treats values as links.

### URL and Email
Clickable external references.

YAML:
```yaml
url: https://example.com/docs/overview
contact: user@example.com
```

Tips:
- Use `url` for external docs, issues, or PRs you want to reference.

### Select-style (constrained values)
Emulate a single-choice field by agreeing on allowed values.

YAML:
```yaml
status: draft # allowed: draft | in-progress | done
```

Tips:
- In Bases, you can group/sort by `status` to create simple workflows.

---

## YAML example

```yaml
---
title: "My Project Plan"
description: "A plan to deliver the new feature by Q4."
tags:
  - project
  - planning
aliases:
  - Project Plan
cssclasses:
  - wide
status: in-progress
created: 2025-08-01
due: 2025-10-15
cover: /assets/img/projects/plan-cover.png
---
```

---

## Editing properties in Obsidian

1. Open a note.
2. At the top, click into the Properties panel (or use the Properties button) to add/edit properties.
3. Choose the appropriate type (text, date, checkbox, list, etc.).

You can also toggle the visibility of properties per note if you prefer a cleaner view.

---

## Using properties with Search and Dataview

Search can filter by properties. For example, search for notes that have the `status` property set to `in-progress`.

With Dataview you can query and display properties as tables or lists:

```dataview
table status, due
from "Projects"
where status = "in-progress"
sort due asc
```

---

## Using properties with Bases (Obsidian 1.9.10+)

Bases can display your notes as a table or cards, with columns backed by properties. Add, edit, sort, group, and filter on properties directly in the Base. Combine this with consistent properties to build dashboards for projects, reading lists, and more.

---

## Practice exercises

Try these quick drills to build muscle memory:

- Add frontmatter to a new note with `tags`, `status`, and `date`.
- Add an inline marker like `#starred` somewhere in the body.
- Create a Dataview table of in-progress notes, sorted by `date`.
- Optional: Build a Base filtered to a folder or tag, add columns for `status` and `date`, then sort/group.

Example Dataview:

```dataview
table file.link as Note, tags, date, authors
where contains(file.content, "#starred")
sort date desc
```

---
