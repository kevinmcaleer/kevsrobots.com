---
layout: project
title: Bugs the Robo-Bunny
short_title: Bugs the Robo-Bunny
short_description: An Easter Robot
difficulty: Intermediate
description: An Easter Robot
date: 2022-04-10
author: Kevin McAleer
excerpt: Robo-Bunny build blog
cover: /assets/img/blog/bugs.jpg
tags:
 - picocat
 - robobunny
 - bugs
groups:
 - pets
 - pico
 - micropython
 - servos 
 - seasonal
videos:
 - mG7stDcin3c
code: 
 - https://www.github.com/kevinmcaleer/bugs_robobunny
---

## Design and Inspiration

RoboBunny is based on [PicoCat version 2](/blog/picocat-v2.html), which was originally based on OpenCat. I uses the [pimoroni](https://www.pimoroni.com/servo2040) Servo 2040 all-in-one 18 servo microcontroller, powered by the Raspberry Pi RP2040.

![Servo2040](/assets/img/blog/servo2040.jpg){:class="img-fluid w-50"}

RoboBunny features a new head & ear profile, modified rear legs as well as a pom-pom floofy tail.

---

## 3D Printable Parts

Here is a list of the parts:

Part | stl | Description | Qty to print
---|---|---|:-:
body | [body.stl](/assets/stl/bugs/body.stl) | The main body section | 1
tibia | [tibia.stl](/assets/stl/bugs/tibia.stl) | The upper leg section | 4
collar | [collar.stl](/assets/stl/bugs/collar.stl) | Connects the legs to the body | 2
foot | [foot.stl](/assets/stl/bugs/foot.stl) | The front feet - you'll need to mirror one of these in your slicing software | 2
leg | [leg.stl](/assets/stl/bugs/leg.stl) | The rear leg section - you'll need to mirror one of these in your slicing software | 2
rear foot | [rear_foot.stl](/assets/stl/bugs/rear_foot.stl) | The rear foot - you'll need to mirror one of these in your slicing software | 2
eye mask | [eye_mask.stl](/assets/stl/bugs/eye_mask.stl) | The Eye surround | 1
profile | [profile.stl](/assets/stl/bugs/profile.stl) | the head profile | 1
nose | [nose.stl](/assets/stl/bugs/nose.stl) | the nose piece | 1
ear profile | [ear_profile.stl](/assets/stl/bugs/ear_profile.stl) | the ear profile | 1
chin | [chin.stl](/assets/stl/bugs/chin.stl) | the chin section | 1
neck | [neck.stl](/assets/stl/bugs/neck.stl) | the neck servo connector | 1
{:class="table table-striped"}

---

## Code Repository

The Robobunny code is written in MicroPython using the Pimoroni Servo 2040 (batteries included) build.
The code is available on github: <https://www.github.com/kevinmcaleer/bugs_robobunny>

---

## Gallery

[![picture](/assets/img/blog/bugs/bunny.png){:class="img-fluid w-25"}](/assets/img/blog/bugs/bunny.png)
[![picture](/assets/img/blog/bugs/foot.png){:class="img-fluid w-25"}](/assets/img/blog/bugs/foot.png)
[![picture](/assets/img/blog/bugs/tibia.png){:class="img-fluid w-25"}](/assets/img/blog/bugs/tibia.png)
[![picture](/assets/img/blog/bugs/head_assembly.png){:class="img-fluid w-25"}](/assets/img/blog/bugs/head_assembly.png)
