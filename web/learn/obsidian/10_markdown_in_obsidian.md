---
layout: lesson
title: Markdown in Obsidian
author: Kevin McAleer
type: page
cover: /learn/obsidian/assets/cover.jpg
date: 2025-08-24
previous: 12_graph_view.html
next: 11_markdown_course_link.html
description: How to use Markdown syntax and features within Obsidian.
percent: 84
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


## Markdown in Obsidian

Obsidian stores notes as plain Markdown. Here’s a quick, practical reference tailored for Obsidian.

---

## Headings

Use `#` to `######` for H1–H6. Keep one H1 per note (Obsidian can show the filename as title).

```markdown
# Heading 1
## Heading 2
### Heading 3
```

---

## Emphasis

```markdown
*italic*  _italic_
**bold**  __bold__
~~strikethrough~~
```

---

## Lists

```markdown
- Unordered item
  - Nested item
1. Ordered item
2. Another item
```

Tips:

- Leave a blank line before and after lists for best rendering.

---

## Links

- Wiki-links (best for note-to-note): `[[Note Name]]` or `[[Note Name|Custom text]]`
- External links: `[Text](https://example.com)`

```markdown
See [[This is a note with a tag]] or [Obsidian](https://obsidian.md).
```

---

## Images

Use standard Markdown or drag-and-drop. For site images in this repo:

```markdown
![Graph View](assets/graph.png)
```

---

## Callouts (Obsidian-specific)

```markdown
> [!note]
> Helpful note.
>
> [!tip]
> A handy tip.
>
> [!warning]
> Use with care.
```

---

## Tables

```markdown
| Feature  | Supported |
|---------:|:---------:|
| Tables   |    Yes    |
| Callouts |    Yes    |
```

---

## Code blocks and inline code

Inline: ``Use `code` for short snippets``

Fenced blocks:

```markdown
```python
print("Hello from Obsidian")
```
```

---

## Tasks

```markdown
- [ ] Incomplete task
- [x] Completed task
```

Tip: With the Tasks community plugin, you can query tasks across notes.

---

## Footnotes

```markdown
Here’s a claim with a footnote.[^1]

[^1]: The footnote text.
```

---

## Frontmatter (properties)

```yaml
---
title: "Sample Note"
tags:
  - demo
status: in-progress
---
```

Pairs nicely with Dataview and Bases.

---

## Embeds (notes, images, bases)

- Embed a note: `![[This is a note with a tag]]`
- Embed an image: `![[assets/cover.jpg]]`
- Embed a Base view (pin a specific view):


````markdown
```base
views:
  - type: table
    name: Table
    order:
      - file.name
    sort:
      - property: file.name
        direction: ASC
```
````

---

## Math (optional)

Enable MathJax to use LaTeX:

```markdown
Inline: $E=mc^2$

Block:
$$
\int_a^b f(x)\,dx
$$
```

---

## Keyboard shortcuts

Use Cmd+P to open the Command Palette. Toggle Live Preview/Source Mode via the More menu (…) per note.

---

## Try it now

- Create a new note and add a callout and a task list.
- Link it to another note using wiki-links, then inspect backlinks.
- Add a small table and a fenced code block to test formatting.
