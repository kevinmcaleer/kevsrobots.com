---
layout: blog
title: Servo Easing & Pancake-Bot
description: Learn Servo easing in MicroPython with Pancake-Bot
short_title: Servo Easing
short_description: Learn Servo easing in MicroPython with Pancake-Bot
date: 2023-02-18
author: Kevin McAleer
excerpt: Learn how to use Easing algorithms for servos using MicroPython, with Pancake-Bot
cover: /assets/img/blog/easing/pancakebot.jpg
tags: 
 - robot
 - servo
 - easing

---

## Contents

{:toc}
* toc

---

### Video

Click here to watch the videos:

{% include youtubeplayer.html id="kXs2nYP-aws" %}

---

## What is Servo Easing?

`Servo easing` is a technique used to improve the smoothness of the movement of servo motors. It is achieved by using a mathematical algorithm to adjust the acceleration and deceleration of the servo motor as it moves between two positions. This helps create a smoother, more natural motion and can help reduce motor noise and wear.

---

> ### Get the Servo Easing cheatsheet PDF and support my work
>
> You can download a Servo Easing cheatsheet PDF and support my work at the same time, its available on my [Buy Me A Coffee](https://www.buymeacoffee.com/kevinmcaleer) page. It only costs £5 and will help support my YouTube channel and keep me making more robots!
>
> [Click Here](https://www.buymeacoffee.com/kevinmcaleer/e/119219) to get the PDF
>
> [![PDF Thumbnail](/assets/img/easing_pdf.png){:class="img-fluid w-25"}](https://www.buymeacoffee.com/kevinmcaleer/e/119219)
> [![PDF Thumbnail](/assets/img/easing_pdf2.jpg){:class="img-fluid w-25"}](https://www.buymeacoffee.com/kevinmcaleer/e/119219)
>
{:class="bg-ocre"}

### Types of Servo easing algorithms

There are quite a few common servo easing algorithms, we'll look at the most common here.

There are 3 types of algorithm:

1. **Ease In** - starts with a slow changing value and builds up towards the end position
1. **Ease Out** - starts with a fast changing value and slows towards the end position
1. **Ease In-Out** - Starts and ends with a slow changing value, with a burst of speed half way through

![Easing graphs](/assets/img/blog/easing/ease01.png){:class="img-fluid w-100 shadow-lg"}

For each of the Ease-in, Ease-out and Ease-in-out, there are also many different algorithms which change the speed of increase and decrease of the easing:

* linear - (no easing applied)
* Sine
* Circular
* Quad
* Cubic
* Quart
* Quint
* Exponential

![A chart of all the different easing algorithms](/assets/img/blog/easing/ease02.png){:class="img-fluid w-100 shadow-lg"}

---

### How easing works

Easing algorithms take 3 initial values:

* Start value
* End Value
* Duration

The `ease` function will calculate the position of the servo, given a `time` value. 

A `time` value of `0` will give the the `start` value, a time value of the `duration` with give the `end` value, and any time value inbetween `0` and `duration` will give the `servo value`, based on which easing algorithm is used.

Here is an example snippet of code, for the `ease in quad` algorithm.

```python
def ease_in_quad(t: float) -> float:
    """ t is the time value, returns the servo position"""
        return t * t
```

---

## Code

Here is the link to the Github repository that contains the `easing.py` code: <https://github.com/kevinmcaleer/bubo-2t>

---

## Pancake-Bot

As of the writing of this article, its nearly Pancake day (also known as Shrove Tuesday), so I thought it would be fun to create a robot that can flip pancakes. This would also be useful for experimenting with servo easing techniques, to find the best one for flipping a tiny 3d printed pancake.

---

### Features

This small robot has two servo controlled arms, with cooking implements attached:

* a cooking spatula
* a frying pan

The robot also has a small pancacke, and an optional chef's hat.

---

### Bill of Materials

Item        | Description              | Qty |   Cost
------------|--------------------------|:---:|------:
Servo 2040  | Pimoroni Servo 2040      |  1  | £24.00
2x Servos   | DS 929 MG servos         |  2  |  £9.60
1x m2 screw | Screw to attach the head |  1  |  £0.10
{:class="table table-striped"}

---

### Download the STLS and print today

There are quite a few parts, some of which are very delicate so becareful when removing them from the build surface and when removing suport structures.

* [`chef.stl`](/assets/stl/pancake-bot/chef.stl)
* [`hat.stl`](/assets/stl/pancake-bot/hat.stl)
* [`head.stl`](/assets/stl/pancake-bot/head.stl)
* [`left_arm.stl`](/assets/stl/pancake-bot/left_arm.stl)
* [`pan.stl`](/assets/stl/pancake-bot/pan.stl)
* [`pancake.stl`](/assets/stl/pancake-bot/pancake.stl)
* [`right_arm.stl`](/assets/stl/pancake-bot/right_arm.stl)
* [`spatula.stl`](/assets/stl/pancake-bot/spatula.stl)

![Pancake-Bot 3d printed files on a 3d printer](/assets/img/blog/easing/pancakebot01.jpg){:class="img-fluid w-100 shadow-lg"}

![Pancake-Bot 3d looking at the camera](/assets/img/blog/easing/pancakebot02.jpg){:class="img-fluid w-100 shadow-lg"}

---

### Pancake-Bot code

The code for Pancake-Bot is bundled with the [Bubo-2t](https://github.com/kevinmcaleer/bubo-2t) Repository; you'll need to upload the `bubo` folder to the Pico or Servo 2040, along with the `chef.py` Python file.

There is also a Jupyter Notebook named `notebook.ipynb` which contains code for you to explore and experiment with.

---

### Wiring

The left_arm (the one with the frying pan) is connected to servo socket 1 on the Servo 2040, the right_arm (the one with the spatula) is connected to servo socket 3 on the Servo 2040.

---
