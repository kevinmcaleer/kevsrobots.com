---
title: Tags
description: Browse posts by tag
layout: content
date: 2024-01-24
---

{% include nav_blog.html %}

{% include breadcrumbs.html %}

# {{ page.title }}

## {{ page.description }}

Click a tag to see all related posts.

---

<div class="tag-cloud my-4">
  {% assign sorted_tags = site.tags | sort %}
  {% for tag in sorted_tags %}
    {% assign tag_name = tag[0] %}
    {% assign tag_count = tag[1] | size %}
    {% assign tag_slug = tag_name | slugify %}
    <a href="/tags/{{ tag_slug }}/" class="btn btn-primary btn-sm m-1">
      {{ tag_name }} <span class="badge bg-light text-primary">{{ tag_count }}</span>
    </a>
  {% endfor %}
</div>
