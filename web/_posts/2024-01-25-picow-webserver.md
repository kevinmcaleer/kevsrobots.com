---
title: Build a Web Server Using a Raspberry Pi Pico W and Phew!
description: >-
    Explore Phew! today and revolutionize your web server building experience with a little help from Raspberry Pi Pico W.
layout: project
date: 2024-01-25
author: Kevin McAleer
difficulty: beginner
excerpt: >-
    Learn how to build a web server and templating engine using a Raspberry Pi Pico W.
cover: /assets/img/blog/phew/phew.jpg
tags:
  - micropython
  - web_server
  - phew!
  - pimoroni
groups:
  - micropython
  - pico
videos:
  - 0sPPxIq4hg8
---

![Cover photo](/assets/img/blog/phew/phew.jpg){:class="cover"}

Hello fellow robot makers! Have you ever given thought to building your own website using a Raspberry Pi Pico W, only to pause at the complicated steps or unsatisfactory results? If yes, let's put a halt to that and present a superior alternative, a brilliant tool called Phew! In this blog post, we'll guide on you on using this wonderful web server and templating engine ideally designed for the Raspberry Pi Pico W. So, let's dive right into it!

---

## The Phew! Web Server

Developed by [Pimoroni](https://shop.pimoroni.com) for their `Enviro` products, Phew! was conceived for creating captive Wi-Fi's hotspots and to provide an easy to use web interface when setting up the devices. This vital tool runs impeccably on Pico W's and practically any other MicroPython board, however, it's customised specifically for the Pico W. Besides offering a sturdy built-in web server, Phew! also provides incredible features such as logging and an intuitive templating system, aiming to make interactive websites building an effortless process.

---

## Grab the Next Cube 3D STL files

You can grab the files for the Next Cube build from here: <https://www.kevsrobots.com/blog/next-cube.html>

---

## The Magic of Templates

With Phew!, templates become a programmer's best friend. They allow you to integrate data from your program dynamically into the web page. A straightforward concept, you merely embed the variable data within 'squiggly brackets' in the template and pass the variables while rendering the template. As a result, you can include the page title, content, or virtually any program data using templates making your development process a breeze!

E.g.

{% raw %}
    <html>
        <head>
            <title>{{ title }}</title>
        </head>
        <body>
            <h1>{{ title }}</h1>
            <p>{{ content }}</p>
        </body>
    </html>
{% endraw %}

In this example, the title and content are variables that are passed to the template when rendering it. The template engine replaces the variables with the actual values, resulting in a complete web page.

---

## Multi-Part Web Pages

One of the most influential features within Phew! is its ability to handle multipart web pages. You can essentially construct different portions of your pages while leaving it to Phew! to compile the complete page drawing from these fragments. This convenient feature facilitates the reusability of common parts like the heading, footer, or navigation across different pages leading to consistent and easy web page development.

---

## Effective Logging

Logging is an integral aspect of any program, more so for embedded systems. Phew! provides a detailed logging feature that stamps the type, time, and sequence of the messages, offering a fundamental debugging feature. With four varied levels, debug, info, warn, and error, logging caters to different programming and diagnostic needs, saving the contents of your messages to your system, especially beneficial for autonomous functioning systems.

---

## Creating Forms

A pivotal feature, forms, enable the capturing of data from the user. Phew! provides a simplistic method to create forms and capture this data through the request object. Once captured, the request object allows you to modify and manipulate data to fit your control requirements. Need it to control flashing lights or send data off to an MQTT server? Phew! has you covered.

Phew! represents a significant breakthrough for Raspberry Pi Pico W users, offering an extensive set of useful features crafted to cater to all your web server needs. So, the next time you find yourself struggling with complicated steps or unsatisfactory results, remember there's a simple and effective tool right at your fingertips.

Explore Phew! today and revolutionize your web server building experience with a little help from Raspberry Pi Pico W.

Visit the Phew! GitHub repository <https://www.github.com/pimoroni/phew>

---
