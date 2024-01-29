---
layout: content
description: Status of the hosts
title: Status
date_published: 2024-01-29
---

# Status
## {{page.description}}

---

This page is hosted on node {{ site.hostname }}

---

{% for item in hostname %}
{{ item.hostname }}
{% endfor %}