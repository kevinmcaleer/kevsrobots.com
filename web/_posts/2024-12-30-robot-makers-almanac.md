---
title: "Robot Makers Almanac"
description: >-
    The ultimate pocket-sized companion for building, designing, and repairing robots with ease
excerpt: >-
   
layout: showcase
date: 2024-12-30
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/robot_makers_almanac/cover.jpg
# hero:  /assets/img/blog/high_five_bot/hero.png
mode: light
tags:
 - robot_makers_almanac
groups:
 - robots
videos:
#  - f4p6wBqGEJI
---

**🔥 Announcing The Robot Makers Almanac 🔥**
{:class="text-center"}

## What is the Robot Makers Almanac?

Introducing the `Robot Makers Almanac` – your ultimate pocket-sized companion for building, designing, and repairing robots with ease!

Whether you’re a **hobbyist**, **maker**, or **student**, the Robot Makers Almanac is here to streamline your creative process. Packed with vital reference material, it’s the must-have guide that keeps all your essential robot-building info right at your fingertips. From ***pinouts*** to ***dimensions*** and everything in between, this handy almanac has got you covered.

No more scrambling online for that elusive pinout or last-minute measurement—this book has all the essential details you need for popular components like ***Raspberry Pi***, ***Arduino***, ***M5Stamp***, and ***Micro:Bit***. Plus, it’s equipped with ***electrical symbols*** and ***resistor values**, making it an indispensable resource for anyone passionate about robotics!

{% include gallery.html images="/assets/img/robot_makers_almanac/cover.jpg,/assets/img/robot_makers_almanac/electronics.jpg,/assets/img/robot_makers_almanac/pico.jpg" cols=3 %}

---

## Why I created the Robot Makers Almanac

I've been building robots for about 10 years now and one of the frustrations I've had is finding the information I need quickly. I wanted to create a resource that would help me and others like me to quickly find the information they need to build robots. This information is typically the physical dimensions of common components such as Motors, Servos, cameras and of course Microcontrollers.

Next, when I'm wiring up the robot I need to remember which GPIO pins on the microcontroller are connected to which components. This is where the pinout diagrams come in handy, and each microcontroller has a different pinout layout. All this information is avaialble online, but I wanted to have a small pocket sized reference guide that I could quickly refer to.

Finally, I wanted to include some basic electrical information such as resistor values and electrical symbols. This is something that I always forget and have to look up online. So I included this information in the Robot Makers Almanac, too.

I planned to launch this at the beginning 2024, however, there was quite a bit more to this than I first thought, so it took a bit longer to get it ready for launch. However, I still achieved the goal of launching this in 2024, just!

---

## Designing the Robot Makers Alamanc

I'd decided roughly what I wanted to include in the Robot Makers Almanac, so I started to design the layout of the book. I wanted to make sure that the book was small enough to fit in my pocket, but also large enough to be able to read the text and diagrams. I decided on a size of 5.5" x 8.5" which is half the size of a standard A4 sheet of paper. This size would also mean I could print two pages per sheet of paper and then cut  paper in half to create two book.

---

## Two-up printing

It turns out its quite tricky to create a PDF file that is half the size of standard paper, but contains two copies of each page, when the pages are doubled-sided and ordered in a printable format. I used Adobe InDesign to create the layout of the book and then exported the PDF file. I then created a Python program to take the PDF file and create a new PDF file that was half the size of the original and contained two copies of each page. This took quite a bit of devopment time to get right, but I'm happy with the final result. 

I'll release this code as open source so others can use it to create their own pocket sized books, too.

---

## Manufacturing the Robot Makers Almanac

To keep the costs of manufacting down, I decided to print, cut and assemble the books myself. I've an industrial guillotine to cut pages, an industrial corner cutter for getting the nice rounded edges, and an industrial stapler that can staple the pages accurately (and many pages at once).

I also wanted make sure the book was sourced from sustainable materials, so I used FSC certified paper and card for the book.

To purchase the Robot Makers Almanac, see the links below.

{% include store/button_robot_makers_almanac.html %}

---

## Maker Notes

![Maker Notes](/assets/img/maker_notes/cover.jpg){:class="img-fluid w-100 rounded-3"}

Whilst creating the Robot Makers Almanac I also thought, why not use the same process to create blank notebooks, but ones specifically designed for makers. So I created Maker Notes - small, pocket sized blank notebooks that are perfect for jotting down ideas, sketches, and notes. I've included a subtle dot grid on each page to help with sketching circuits, but unobtrustive enough to also allow freehand sketching too.

{% include store/button_maker_notes.html %}

---

## PCBs

![PCBs](/assets/img/pcbs/cover.jpg){:class="img-fluid w-100 rounded-3"}

Now that I have a store up and running, I've also included the PCBs that I've designed, which at launch include the Picotamachibi virtual pet PCB, Gamepad 2.0 PCB and Servo Helper PCB. I'll be adding more PCBs in the future, so keep an eye out for those.

{% include store/pcbs.html %}

---
