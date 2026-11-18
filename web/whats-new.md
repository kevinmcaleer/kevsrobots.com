---
layout: content
title: What's new in the kevsrobots suite
description: The latest release notes from Snakie by kevsrobots and BetterCAD by kevsrobots, in one place.
permalink: /whats-new/
---

{%- comment -%}
  One "What's new" feed for the suite (#581). The entries come from
  _plugins/suite_releases.rb: GitHub Releases, else CHANGELOG.md, else the
  committed copy in _data/whats_new.yml.
{%- endcomment -%}

<div class="container py-4" style="max-width: 52rem">

<h1>What&rsquo;s new</h1>
<p class="lead">Release notes from the apps in the kevsrobots suite, newest first.</p>

{% include suite_links.html tone="light" %}

{% assign entries = site.data.whats_new %}
{% assign apps_with_notes = entries | map: "app" | uniq %}

<ul class="kr-whats-new mt-4">
{%- for e in entries %}
  <li class="kr-wn-{{ e.app }}">
    <div class="d-flex justify-content-between flex-wrap gap-2">
      <span class="kr-wn-app">{{ e.app_name }}{% if e.app != "projects" %} by kevsrobots{% endif %}</span>
      {% if e.date and e.date != "" %}<time class="kr-wn-date" datetime="{{ e.date }}">{{ e.date | date: "%-d %B %Y" }}</time>{% endif %}
    </div>
    <h3><a href="{{ e.url }}" target="_blank" rel="noopener">{{ e.title }}</a></h3>
    <div class="kr-wn-body">{{ e.body | markdownify }}</div>
  </li>
{%- endfor %}
</ul>

{%- for app in site.data.suite %}
  {%- if app.repo and apps_with_notes contains app.id %}{% elsif app.repo %}
<p class="mt-3 text-body-secondary">{{ app.name }} {{ app.by }} hasn&rsquo;t published a release yet. Follow it on <a href="https://github.com/{{ app.repo }}" target="_blank" rel="noopener">GitHub</a>.</p>
  {%- endif %}
{%- endfor %}

</div>
