---
layout: article
title: Bubo-2T2
description: Build your own steampunk companion owl with blinking NeoPixel eyes, a talking beak and hand-gesture photos.
author: Kevin McAleer
date: 2023-05-01
hero: /assets/img/blog/bubo/bubo-2t.jpg
hero_style: overlay
difficulty: intermediate
specs:
  Brains: Pico + Pi 4
  Servos: 3 × DS-929MG
  Camera: Pi Camera 3 Wide
  Parts cost: £151.59
tags:
  - robot
  - steampunk
  - companion_bot
sitemap: false
hide_likes_and_comments: true
---

This page is a preview of the new `article` layout, rebuilt from the Bubo-2T2 post. Every block on it is written in plain Markdown; see [the cheat sheet on GitHub](https://github.com/kevinmcaleer/kevsrobots.com/blob/main/docs/article_layout.md) for the syntax.

## What is Bubo-2T?

Bubo-2T is a robotic owl made in the steampunk style. It's named after Bubo, the clockwork owl from the 1981 film *Clash of the Titans*, animated by Ray Harryhausen.

{% split image="/assets/img/blog/bubo/bubo02.jpg" alt="Bubo's printed eye mechanism with its NeoPixel ring" caption="The eye mechanism on its test stand" %}
**Two big eyes that blink.** Each eye opens and closes with its own servo, and a NeoPixel ring behind it is animated by the Raspberry Pi Pico.

One eye also hides a Raspberry Pi Camera 3 Wide, which watches for hand gestures.
{% endsplit %}

{% callout tip "Why 2T?" %}
Strike a pose and Bubo takes your photo and toots it to Mastodon. Two T's: **tooty**.
{% endcallout %}

## Inspiration

{% columns %}
{% column %}
### Films

Bubo from *Clash of the Titans*, the Nautilus from *20,000 Leagues Under the Sea*, and Tik-Tok, the clockwork man from *Return to Oz*.
{% endcolumn %}
{% column %}
### Makers

Companion robot builders like Alex Glow and her robot Archie, and Jorvon Moss, plus wearable-tech makers Naomi Wu and Sophy Wong.
{% endcolumn %}
{% endcolumns %}

## Features

* Two large eyes that open and close, driven by two servos
* An opening and closing beak
* A NeoPixel ring in each eye, animated by a Raspberry Pi Pico
* The Pico talks to a Raspberry Pi 4 2GB over UART
* A Raspberry Pi Camera 3 Wide in one eye for gesture detection

{% split image="/assets/img/blog/bubo/bubo05.jpg" alt="Bubo's body printing in yellow PLA" side="right" %}
### Print your own

All the 3D printed parts are free to download, from the face and body down to the eyelids and the beak arm.
{% endsplit %}

{% callout note "What you'll need" %}
A Raspberry Pi 4 running the latest 64-bit Raspberry Pi OS, plus the parts below.
{% endcallout %}

## Bill of materials

Item                   | Description                             | Qty |   Cost
-----------------------|-----------------------------------------|:---:|------:
Pico                   | Raspberry Pi Pico                       |  1  |  £6.30
RPi 4                  | Raspberry Pi 4 2GB                      |  1  | £79.50
NeoPixel rings         | Adafruit 12-pixel NeoPixel ring         |  2  |  £9.30
Servos                 | DS-929MG                                |  3  |  £9.60
M2 and M3 nuts, bolts  | Box of screws and nuts, various lengths |  1  |  £9.99
Camera                 | Raspberry Pi Camera Module 3 Wide       |  1  | £36.90

{% fullwidth image="/assets/img/blog/bubo/bubo01.jpg" %}
## Download the STLs and print today

Sixteen parts, from the face and body to the eyelids and beak arm. Every file is in the `/assets/stl/bubo/` folder, ready to slice.
{% endfullwidth %}

## Code

The code is on GitHub at <https://github.com/kevinmcaleer/bubo-2t>. There are three parts:

* `hands.py` demos hand gesture recognition with the Raspberry Pi Camera Module 3
* `toot_randomiser.py` picks a random message so repeated posts aren't flagged
* `toot.py` posts the photo and message

```bash
libcamerify python hands.py
```

{% callout note "Camera workaround" %}
There's a clash between libcamera, OpenCV and CVZone. Running the script through `libcamerify` instead of plain `python` fixes it.
{% endcallout %}

## More companion robots

{% cards %}
{% card title="PicoCat v2" image="/assets/img/picocat01.jpg" link="/blog/picocat-v2.html" label="Quadruped" %}
New upgrades and improvements to the Pico-powered robot cat.
{% endcard %}
{% card title="My first robot just got five new senses" image="/assets/img/blog/modulino_smars/cover.jpg" link="/blog/my-first-robot-just-got-five-new-senses.html" label="SMARS" %}
Five Arduino Modulino modules give SMARS sight, sound, movement, expression and balance.
{% endcard %}
{% card title="Chess with the Arduino UNO Q" image="/assets/img/blog/uno_q_chess/cover.jpg" link="/blog/uno-q-chess-camera.html" label="Arduino" %}
A webcam watches a real board and Stockfish plays back.
{% endcard %}
{% endcards %}
