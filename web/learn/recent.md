---
layout: content
title: Recent Courses
description: Recent Courses
---

{% include breadcrumbs.html %}

<div class="row">
    <div class="col-md-3 col-lg-2">
        {% include learn_sidebar.html %}
    </div>
    <div class="col-md-9 col-lg-10">
        <h1>{{page.title}}</h1>
        <h2>{{page.description}}</h2>
        <hr>
        {% include recent_courses.html %}
    </div>
</div>
