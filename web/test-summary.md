---
layout: content
title: Test Summary Component
description: Test page for the like-comment-summary component
---

# Test Like/Comment Summary Component

This page demonstrates how to show like and comment counts in cards or galleries.

## Example Cards

<div class="row row-cols-1 row-cols-md-2 g-4">
  <div class="col">
    <div class="card">
      <div class="card-body">
        <h5 class="card-title">Test Likes Page</h5>
        <p class="card-text">A test page with likes and comments.</p>
        <a href="/test-likes.html" class="btn btn-primary">Read More</a>
        <div class="mt-2">
          {% include like-comment-summary.html url="test-likes.html" %}
        </div>
      </div>
    </div>
  </div>

  <div class="col">
    <div class="card">
      <div class="card-body">
        <h5 class="card-title">SMARS Q Robot</h5>
        <p class="card-text">An Arduino Uno Q Based Robot.</p>
        <a href="/blog/smars-q.html" class="btn btn-primary">Read More</a>
        <div class="mt-2">
          {% include like-comment-summary.html url="blog/smars-q.html" %}
        </div>
      </div>
    </div>
  </div>
</div>

## How to Use

In your card templates, add:

```liquid
{% include like-comment-summary.html url="path/to/page.html" %}
```

The component will:
- Show heart icon with like count
- Show comment icon with comment count
- Hide itself if there are no likes and no comments
- Load data asynchronously without blocking page render
