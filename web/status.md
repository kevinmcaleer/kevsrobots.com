---
layout: default
description: Status of the hosts
title: Status
date_published: 2024-01-29
---

# Status

This page is hosted on node {{ site.hostname }}

---

{% for item in hostname %}
{{ item.hostname }}
{% endfor %}