---
name: create-course
description: Create a new course for KevsRobots.com in /source, following site conventions — course.yml structure, lesson frontmatter, naming, tone, no-emoji headings, cover image, generated per-lesson header banners, and build verification. Use when asked to create, scaffold, or build a new course (or add lessons to one).
---

# Creating a KevsRobots Course

Courses are authored as Markdown in `/source/<slug>/` and compiled by `python3 build.py` into `/web/learn/<slug>/`. Never edit `/web/learn/` directly.

## Workflow

1. **Validate the outline first.** Before writing content, sanity-check the proposed structure: 10–20 lessons, progressive difficulty, one concrete project threaded through the whole course (KevsRobots courses always build toward a working thing — name it explicitly). Report fixes to Kev before or while building, don't silently restructure.
2. **Scaffold** `/source/<slug>/` with `course.yml`, lesson files, and an `assets/` folder.
3. **Write lessons** (use the course-architect agent for bulk writing — give it the full per-lesson spec and the conventions below in the prompt; it won't see this file).
4. **Course cover**: every course needs `assets/cover.jpg` in **4:3 aspect ratio** (e.g. 1200×900, <300KB) — this is the *card thumbnail* in course listings, not the page header. If no real image exists yet, generate a placeholder with PIL (Pillow is installed) so nothing 404s, and tell Kev to replace it. Verify the rendered placeholder by Reading the .jpg — text bands can hide drawn elements.
5. **Lesson header banners**: run `python3 generate_banners.py <slug>` — see the section below. Do this *after* the lessons exist and `course.yml` lists them. Not optional: a page with no header is a wall of grey text.
6. **Verify**: run `python3 build.py` from the repo root; confirm the course appears in the output, check `/web/learn/<slug>/` was generated, and check the entry in `/web/_data/courses.yml` (it gains a computed `duration` and `link`).

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
cover: /learn/<slug>/assets/banners/NN_lesson_name.jpg
date_updated: YYYY-MM-DD
---
```

- Start the body with the cover image, a blank line, then a rule:

  ```markdown
  ![Course Cover Image]({{page.cover}}){:class="cover"}

  ---
  ```

  Use the Liquid `{{page.cover}}` rather than repeating the path, so the two can never drift apart.
- You do not have to fill in the `cover:` path or the image line by hand — `generate_banners.py` writes both. Put any placeholder in while drafting; the script normalises it.
- Do **not** add manual previous/next breadcrumb links — the build system generates navigation.
- **Every markdown table must be followed by `{:class="table table-single"}`** on the line immediately after the table's last row — no blank line between. This is a Kramdown IAL that applies the site's table styling; without it tables render unstyled. Example:

  ```markdown
  | Hardcoded approach | Learned approach |
  |---|---|
  | You write the rules | The robot discovers the rules |
  {:class="table table-single"}
  ```

  (A `table table-single table-narrow` variant exists for narrow tables, but default to `table table-single`.) After writing, sanity-check that the count of table separator rows equals the count of `table-single` tags.

## Lesson header banners

Every lesson gets its **own** generated header banner, so pages feel alive instead of reading as unbroken formatted text. One repeated cover across twenty lessons makes them all look like the same page.

```bash
python3 generate_banners.py <slug>              # generate + wire up frontmatter
python3 generate_banners.py <slug> --force      # regenerate existing images
python3 generate_banners.py <slug> --images-only  # leave the markdown alone
python3 generate_banners.py --contact-sheet /tmp/sheet.jpg   # preview all styles
```

The script reads `course.yml`, writes `source/<slug>/assets/banners/<lesson-stem>.jpg` for each lesson, sets each lesson's `cover:` to the absolute `/learn/<slug>/assets/banners/…` path, and normalises the cover image line at the top of the body. It is **idempotent** — safe to re-run after adding lessons, and it only regenerates missing images unless you pass `--force`.

**What the banners look like and why:**

- **Subtle, colourful, geometric.** The *colour* is loud, the *geometry* is quiet: a saturated ground with tone-on-tone shapes over it, matching the house style (the BurgerBot red, the DuckDB yellow, the MeshCore navy). Desaturated mid-tones just look muddy and the script never generates them.
- **One family per course.** The course name seeds a base hue; each lesson rotates within ±36° of it. Patterns are dealt from a per-course shuffled deck so all eleven appear before any repeats.
- **Deterministic.** Same course name and lesson filename always give the same image, so re-running never churns the diff. Renaming a lesson changes its banner — that is intended.

**Geometry that matters** (do not change these without checking the CSS):

| Fact | Consequence |
|---|---|
| `.cover` is `height: 200px; width: 100%; object-fit: cover` | The visible box is 5:1 to 7:1, so banners are authored at 1600×400 and the top and bottom get cropped |
| Lesson `cover:` also feeds `og:image` / `twitter:image` | Social previews crop it differently again |
| Both crops are centred | Patterns must be **full-bleed and vertically uniform** — never put text, a logo or a focal point in a banner |
{:class="table table-single"}

That last row is the important one. These are textures, not illustrations. The lesson `title` already renders as a heading directly beneath the banner, so text in the image would duplicate it *and* get sliced in half.

Adding a pattern means adding a function to `PATTERNS` in `generate_banners.py`. Sparse patterns must also be kept out of `INK_SAFE` — on a near-black ground they vanish.

## Content rules

- **No emojis in titles or headings.** Not in frontmatter `title:`, not in `##` section headers (`## What You'll Learn`, never `## 🤖 What You'll Learn`). Emoji headers are an instant tell that content is AI-generated. Some older courses use them — don't copy that pattern. Emojis are acceptable nowhere in course content unless Kev explicitly asks.
- **Tone**: friendly, conversational, maker-focused — "Let's build…" not "You must…". Teaching a friend, not lecturing.
- **No LaTeX.** Lesson markdown doesn't reliably render MathJax. Express maths as commented Python with a plain-English walkthrough of each term. Greek letters as words (alpha, gamma) or unicode (α, γ) in prose only.
- Every concept lesson needs: real-world robot-flavoured code examples (no bare reference tables), a "Try it Yourself" section, and a "Common Issues" section (Problem / Solution / Why) where the topic warrants it.
- All code must be complete and runnable. Prefer standard-library-only Python for simulations; actually execute project-lesson code to confirm it runs. MicroPython examples should target the Pico with pin assignments consistent with the existing `micropython_robotics` course.
- Word counts: intro 300–500 (may run over to fit required sections), concept lessons 400–800, project lessons 800–1500 (code listings may push this over — that's fine).
- `00_intro.md` must contain: `## Overview`, `## Course Content` (bullets), `## Key Results`, `## What you'll need`, `## How the course works`, plus prerequisite links to related `/learn/...` courses. The summary lesson links onward ("What's Next") to related courses.

## After building

- Run `python3 optimize_images.py <dir>` if any real photos were added (it takes a directory argument). Generated banners are already compressed — around 40KB each — so skip them. Note that the optimiser drops a `.optimization_cache.json` in whatever directory you point it at, and `build.py` copies the whole `assets/` folder into `web/learn/` — so delete that cache file afterwards rather than shipping it.
- Check the banners actually look right: Read two or three of the generated `.jpg` files rather than assuming. If a course lands on an unlucky hue, re-roll by adjusting the course `name`, or force a single style with `--style`.
- Remind Kev to replace the placeholder **course cover** (the 4:3 card thumbnail) before deploying. The lesson banners are finished artwork and need no replacing.

## Known gotcha

`build_search.py` currently crashes on `web/_data/courses.yml` (it reads `item['title']`, but course entries use `name`). Unrelated to course creation — do not try to "fix" it as part of building a course.
