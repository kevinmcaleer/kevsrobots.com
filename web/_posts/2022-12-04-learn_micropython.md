---
layout: blog
title: Learn now live!
short_title: Learn now live!
short_description: New MicroPython course
description: New MicroPython course
date: 2022-12-04
author: Kevin McAleer
excerpt: Learn at MicroPython, from scratch, and your own pace
cover: /assets/img/blog/learn/micropython.jpg
tags:
 - Raspberry Pi Pico
 - Learn
 - MicroPython
 - Course
 - Tutorial
groups:
 - micropython
---

## The new Learning Platform is now Live

You'll notice a new [Learn](/learn/) item on the menu navigation. This is the gateway to the brand new ***Learning Platform***.

The Learning Platform enables courses to be built and published using a simple folder structure and accompanying course content.

More about this is below.

## Learn MicroPython

`Learn MicroPython` starts with the basics of programming in MicroPython, explaining each concept as they are introduced and building upon previous knowledge with each new lesson. It's designed for people new to programming or just new to MicroPython.

This new course is the reason I built the Learning Platform.

The course is broken into individual lessons, each with a singular focus. The course is easy to navigate, with a clear structure always visible on the left-hand side (when viewed on a desktop browser).

![Learning Platform Screen layout](/learn/learning_platform/assets/course_layout.jpg){:class="img-fluid w-100"}

This course is accompanied by a series of bite-sized videos to help bring the content to life.

---

## More to come

I'll be using the new Learning Platform to provide instructions for more complicated builds that are too big for a typical blog post.

---

## Feedback is always welcome

I'd love to hear what you think about the course content, the Learning Platform itself, and any future features you'd like to see added. The best way to provide this feedback is via our [Discord server](/discord).

---

## About the Learning Platform

The Learning Platform combines course content folders and a python program that transforms the raw content into a polished interactive website.

Each course is contained within a parent folder, and each course folder contains:

* A `course.yml` file - that outlines the course details and order the lessons should appear.
* An `assets` folder - that contains any images, `PDF`s for `STL` files related to the project.

Here is an example `course.yml`:

```YAML
- name: Learn MicroPython - The basics
  author: Kevin McAleer
  date_created: 28-10-2022
  date_published: 28-10-2022
  layout: course
  cover: assets/micropython.jpg
  description: >- 
     Get started with MicroPython, What Python is, where to download it and which software to use to develop MicroPython code
  content:
  - section:
      name: Overview
      content:
      - 00_intro.md 
      - 00_videos.md
```

You can see that there are some basic details about the course, and following that there is a content section. For example, a content section can contain multiple `section` blocks, and each section can have multiple files, one per lesson.

The lessons need a title and description, and the `course_builder.py` will use the structure from the `course.yml` to add `next` and `previous` buttons to each page.

`course_builder.py` also counts the words per page and estimates how long each lesson will take to read, with the collective duration being added to the final `courses.yml` file.

---

## Course Content as Markdown

The lesson files that make up each course are written in [Markdown](https://github.github.com/gfm/), which enables a clear focus on the content being written rather than tweaking the formatting or style.

The Markdown files are converted to HTML using [Jekyll](https://www.jekyllrb.com), which is used to build all KevsRobots websites.

Markdown was chosen for several reasons. First, I wanted to store the draft content in a form that's easy to repurpose (and Markdown is just basic text, so it doesn't get better than this).

I also wanted the final website to be a collection of simple static web pages. Dynamic websites hosted on WordPress etc., require constant security patching to prevent attacks and spam.

---
