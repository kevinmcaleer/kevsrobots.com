---
layout: content
title: About KevsRobots
description: About Kev's Robots
date: 2024-01-01
cover: /assets/img/kevsrobots.jpg
---

{% include breadcrumbs.html %}

<div class="row">
<div class="col-12 col-md-3 col-lg-2">
{% include about_sidebar.html %}
</div>
<div class="col-12 col-md-9 col-lg-10">

{% assign profile = site.data.profile %}

{% capture content %}

# About Kev's Robots

## Learn about this website

---

Hi - I'm Kevin McAleer and I run this website using a bunch of [Raspberry Pi's](https://www.clustered-pi.com), if you want to know more about me, head over to the [bio](/about/bio) page.

This is a website **dedicated** to robotics, and *other* things I've made. There is a list of all the [YouTube](/videos) videos I've made as well as [Blog articles](/blog/) I've written to support the projects.

My aim is to inspire, educate and entertain you, and hopefully you'll find something useful on this site. If you want to ask questions or meet the community we have a dedicated [Discord Server](/discord) where we can take the conversation deeper.

If you're a business and you'd be interesting in working with me sending samples for review, you can always get in touch via [Twitter](/twitter) and we can take it from there.

Please note this is a hobby project of mine (though I do take it very seriously), but that means I'm unlikely to have capacity to work on builds for anyone for the forseeable.

{% endcapture %}
{{ content | markdownify }}
</div>
</div>