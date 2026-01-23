---
title: "Robots and Lasers"
description: >-
    Getting up to speed with the Creality Falcon 5W and Lightburn software.
excerpt: >-
layout: showcase
date: 2024-09-25
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/falcon5w/cover.jpg
hero:  /assets/img/blog/falcon5w/hero.png
mode: light
tags: 
 - laser_cutting
groups:
 - lasercutting
# videos:

---

## Creality Falcon 5W - Unboxing & Setup

This week I introduced a new tool to my RobotLab - the [Creality Falcon 5W](https://uk.crealityfalcon.com/products/cr-laser-falcon-5w-laser-engraver) laser cutter. It's a small(ish), desktop machine that can cut and engrave a variety of materials. I paid £139 for the machine, which is a great price for a 5W laser cutter. Ordered it on Saturday and it arrived the following Tuesday. The cutting area is 400x415

Setting up the Lasercutter took around an 30 minutes to complete, its pretty straightforward, and there is an assemnbly instruciton video on the included SD Card, along with a trial version of Lightburn software (for Windows).

The website mentions optional rises for the machine, however this is not included in the box, so if you need extra height (to cut objects that don't lie flat on the bed) you'll need to figure that out.

The kit also dosen't include a steel base plate, or the honeycome bed which can help disipate heat from the cutting surface. I'll need to source these seperately.

---

## How to connect the Falcon 5W to Lightburn on a Silcon Mac using USB-C

I spent around 2-3 hours trying to get the Falcon 5W to connect to my Mac. The issue was that the Falcon 5W uses a USB C to USB C cable and for whatever reason the Mac coulnd't detect the Laser cutter. I tried a few different cables, but none of them worked, I tried flashing the firmware, that didn't work either.

What did work was to use a USB 2.0 hub between the Mac and the Falcon 5W. This worked first time and I was able to connect to the Falcon 5W using Lightburn.

---

## First test

I ran a test cut on the included 2mm plywood. The cut was clean and the edges were smooth, the etched text was sharp and clear. I was impressed with the results, given this is one of the cheapest laser cutters on the market.

---

### Settings - Speed & Power

As I'm new to Laser Cutting I thought it would be useful to capture the things I'm learning as I go as you'll probably go through a similar learning curve.

First up is the ***speed*** and ***power*** settings.
The laser cutter has a `5W` laser, which is powerful enough to cut through 2mm plywood in a single pass. The speed and power settings are important to get right, as they can affect the quality of the cut. Too little power and it won't cut through the material, too much power and it can burn the material.

There are two types of action that can be performed with the laser cutter - ***cutting*** and ***etching***. `Cutting` is where the laser cuts through the material, `etching` is where the laser burns the surface of the material to create a design or pattern.

The speed and power settings are different for cutting and etching, so it's important to get them right for each type of action. The laser cutter comes with a set of recommended settings for different materials, which can be used as a starting point.

---

### What is Lightburn?

I'm using [Lightburn](https://lightburnsoftware.com/) software to control the laser cutter, which makes it easy to adjust the speed and power settings for each type of action. Lightburn also has a preview mode, which shows you what the design will look like before you start the cut, so you can make any adjustments if needed. Lightburn also lets you do a material test, which is where the laser cutter does a test cut on a small piece of material to check the speed and power settings are correct.

For this cut I used the recommended settings for 2mm plywood:

Material    | Type    | Power | Speed    | Passes
------------|---------|-------|----------|:-----:
2mm Plywood | Cutting | 100%  | 4.67mm/s |   1
2mm Plywood | Etching | 100%  | 100mm/s  |   1
{:class="table tables-striped"}

---

Hello World {:class="caption"}

![Hello World](/assets/img/blog/falcon5w/hello.jpg){:class="img-fluid w-100 rounded card-shadow"}

---

## Lightburn Material Testing

Material      | Type    | Power | Speed   | Passes
--------------|---------|-------|---------|:-----:
3.6mm Plywood | Cutting | 100%  | 1.3mm/s |   2
{:class="table tables-striped"}

---

## Honeycomb Bed

I ordered a honeycome bed from [Amazon](https://www.amazon.co.uk/dp/B0CYZBNPHD?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1) for £46.99. It's a 500x500mm bed and fits perfectly underneath the Creality Falcon 5W. It's made from steel and is designed to help disipate heat from the cutting surface. It's a great addition to the Falcon 5W and I'd recommend it to anyone who is looking to get the most out of their laser cutter. It comes with some small plastic plugs that help secure material to the bed. It also has a solid sheet of steel that can be used as a base plate, preventing the laser from cutting through to the surface below.

---

## Fire Extinguisher

I also bought a fire extinguisher just in case there are any small fires whilst the laser is cutting. This is usually caused by previous cuts that have left a small amount of material on the bed. The fire extinguisher is a small 1kg powder extinguisher, which is suitable for small fires. I hope I never have to use it, but it's good to have it just in case.

---
