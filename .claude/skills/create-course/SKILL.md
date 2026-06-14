---
name: create-course
description: Create a new course for KevsRobots.com in /source, following site conventions — course.yml structure, lesson frontmatter, naming, tone, no-emoji headings, cover image, and build verification. Use when asked to create, scaffold, or build a new course (or add lessons to one).
---

# Creating a KevsRobots Course

Courses are authored as Markdown in `/source/<slug>/` and compiled by `python3 build.py` into `/web/learn/<slug>/`. Never edit `/web/learn/` directly.

## Workflow

1. **Validate the outline first.** Before writing content, sanity-check the proposed structure: 10–20 lessons, progressive difficulty, one concrete project threaded through the whole course (KevsRobots courses always build toward a working thing — name it explicitly). Report fixes to Kev before or while building, don't silently restructure.
2. **Scaffold** `/source/<slug>/` with `course.yml`, lesson files, and an `assets/` folder.
3. **Write lessons** (use the course-architect agent for bulk writing — give it the full per-lesson spec and the conventions below in the prompt; it won't see this file).
4. **Cover image**: every course needs `assets/cover.jpg` in **4:3 aspect ratio** (e.g. 1200×900, <300KB). If no real image exists yet, generate a placeholder with PIL (Pillow is installed) so nothing 404s, and tell Kev to replace it. Verify the rendered placeholder by Reading the .jpg — text bands can hide drawn elements.
5. **Verify**: run `python3 build.py` from the repo root; confirm the course appears in the output, check `/web/learn/<slug>/` was generated, and check the entry in `/web/_data/courses.yml` (it gains a computed `duration` and `link`).

## course.yml format

A YAML **list with one entry** (this exact shape — `build.py` depends on it):

```yaml
- name: Course Title
  author: Kevin McAleer
  date_created: YYYY-MM-DD
  date_published: YYYY-MM-DD
  layout: course
  cover: assets/cover.jpg
  groups:
      - robotics        # must exist as a file in /web/_groups/
      - micropython
  description: >-
    2-3 sentences. What you'll learn and what you'll build.
  content:
    - section:
        name: Introduction
        content:
          - 00_intro.md
    - section:
        name: Section Name
        content:
          - 01_first_lesson.md
          - 02_second_lesson.md
```

Check `/web/_groups/` for valid group names before assigning them.

## Lesson files

- Naming: `00_intro.md`, then sequential `01_`, `02_`… with descriptive snake_case names (`09_coding_the_learning_loop.md`, not `09_lesson.md`). End with `NN_summary.md`.
- Frontmatter (all keys required):

```yaml
---
title: Lesson Title
description: Brief description for SEO and previews
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: YYYY-MM-DD
---
```

- Start the body with `![Cover](assets/cover.jpg){:class="cover"}` followed by `---`.
- Do **not** add manual previous/next breadcrumb links — the build system generates navigation.
- **Every markdown table must be followed by `{:class="table table-single"}`** on the line immediately after the table's last row — no blank line between. This is a Kramdown IAL that applies the site's table styling; without it tables render unstyled. Example:

  ```markdown
  | Hardcoded approach | Learned approach |
  |---|---|
  | You write the rules | The robot discovers the rules |
  {:class="table table-single"}
  ```

  (A `table table-single table-narrow` variant exists for narrow tables, but default to `table table-single`.) After writing, sanity-check that the count of table separator rows equals the count of `table-single` tags.

## Content rules

- **No emojis in titles or headings.** Not in frontmatter `title:`, not in `##` section headers (`## What You'll Learn`, never `## 🤖 What You'll Learn`). Emoji headers are an instant tell that content is AI-generated. Some older courses use them — don't copy that pattern. Emojis are acceptable nowhere in course content unless Kev explicitly asks.
- **Tone**: friendly, conversational, maker-focused — "Let's build…" not "You must…". Teaching a friend, not lecturing.
- **No LaTeX.** Lesson markdown doesn't reliably render MathJax. Express maths as commented Python with a plain-English walkthrough of each term. Greek letters as words (alpha, gamma) or unicode (α, γ) in prose only.
- Every concept lesson needs: real-world robot-flavoured code examples (no bare reference tables), a "Try it Yourself" section, and a "Common Issues" section (Problem / Solution / Why) where the topic warrants it.
- All code must be complete and runnable. Prefer standard-library-only Python for simulations; actually execute project-lesson code to confirm it runs. MicroPython examples should target the Pico with pin assignments consistent with the existing `micropython_robotics` course.
- Word counts: intro 300–500 (may run over to fit required sections), concept lessons 400–800, project lessons 800–1500 (code listings may push this over — that's fine).
- `00_intro.md` must contain: `## Overview`, `## Course Content` (bullets), `## Key Results`, `## What you'll need`, `## How the course works`, plus prerequisite links to related `/learn/...` courses. The summary lesson links onward ("What's Next") to related courses.

## After building

- Run `python3 optimize_images.py` if any real images were added.
- Remind Kev to replace any placeholder cover before deploying.
