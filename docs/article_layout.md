# Article layout cheat sheet

A photo-led layout for blog posts and project write-ups. Everything stays in Markdown: switch a post over by changing its `layout`, then use the blocks below wherever they help.

Preview: `/demo/article-layout.html` (source in `web/demo/article-layout.md`).

## Front matter

```yaml
---
layout: article
title: Bubo-2T2
description: Build your own steampunk companion owl.   # shown under the title
author: Kevin McAleer
date: 2023-05-01
cover: /assets/img/blog/bubo/bubo_cover.jpg           # used for cards and social previews
hero: /assets/img/blog/bubo/bubo-2t.jpg               # optional, defaults to cover
hero_style: overlay        # overlay (default), split or plain
difficulty: intermediate   # optional: beginner, intermediate, advanced
specs:                     # optional strip of key facts under the hero
  Brains: Pico + Pi 4
  Servos: 3 × DS-929MG
  Parts cost: £151.59
toc: false                 # optional: hide the "On this page" list
videos:                    # YouTube IDs; the first is featured right under the hero
  - eDvpRmfa6nU
code: https://github.com/kevinmcaleer/bubo-2t   # or a list of links
stl:                       # optional downloads, listed under "Build files"
  - name: Face
    link: /assets/stl/bubo/face.stl
    description: Front of the head
dxf:
  - name: Base plate
    link: /assets/dxf/example.dxf
tags:
  - robot
---
```

| `hero_style` | What you get |
|---|---|
| `overlay` | Full-width photo with the title on top of it |
| `split` | Title on the left, photo on the right, on a dark background |
| `plain` | No photo, light background (used automatically when there's no `hero` or `cover`) |

When a post has `videos`, a red **Watch the video** button sits in the hero and the first video is featured in a band straight underneath, with Watch on YouTube and Subscribe buttons. Any other videos show as smaller thumbnails beside it. Every thumbnail opens the video on YouTube.

`code`, `stl` and `dxf` are listed in a **Build files** section after the article.

The "On this page" list is built from your `##` headings and appears on wide screens once the reader scrolls into the article.

All photos in the article open in the full-screen viewer when clicked, unless they're inside a link or a card.

## Blocks

Blocks wrap normal Markdown. Leave a blank line or a line break after the opening tag, and put the closing tag on its own line.

### Callout

```liquid
{% callout tip "Why 2T?" %}
Strike a pose and Bubo takes your photo and **toots** it.
{% endcallout %}
```

Types: `note` (blue), `tip` (green), `warning` (yellow), `danger` (red), `try` (purple, for "try it yourself"). The title is optional; each type has a default.

### Photo beside text

```liquid
{% split image="/assets/img/blog/bubo/bubo02.jpg" alt="The eye mechanism" caption="On its test stand" %}
**Two big eyes that blink.** Each eye has its own servo.
{% endsplit %}
```

Add `side="right"` to put the photo on the right. On phones the photo stacks above the text.

### Columns

```liquid
{% columns %}
{% column %}
### Films
Clash of the Titans, Return to Oz.
{% endcolumn %}
{% column %}
### Makers
Alex Glow, Jorvon Moss.
{% endcolumn %}
{% endcolumns %}
```

Columns share the width evenly and stack on phones. `{% columns 3 %}` forces three across.

### Cards

```liquid
{% cards %}
{% card title="PicoCat v2" image="/assets/img/picocat01.jpg" link="/blog/picocat-v2.html" label="Quadruped" %}
New upgrades to the Pico-powered robot cat.
{% endcard %}
{% card title="Another build" image="/path/to/photo.jpg" link="/blog/another.html" %}
One line about it.
{% endcard %}
{% endcards %}
```

`image`, `link` and `label` are optional. Cards fill the row and wrap; `{% cards 3 %}` forces three across.

### Full-width band

```liquid
{% fullwidth image="/assets/img/blog/bubo/bubo01.jpg" %}
## Download the STLs and print today
Sixteen parts, ready to slice.
{% endfullwidth %}
```

Runs edge to edge across the page. With an `image` it's a darkened photo; without one, pick `style="dark"` (default), `style="light"` or `style="red"`.

## Image paths

Use the usual `/assets/img/...` paths for now. A path without a leading `/` is treated as relative to the page's own folder, which is what the photo work in step 2 will build on.
