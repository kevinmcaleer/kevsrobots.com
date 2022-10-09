---
layout: blog
title: Spooky Scary Skeleton 
short_title: Spooky Scary Skeleton
short_description: A Halloween Robot 
date: 2022-10-09
author: Kevin McAleer
excerpt: Build your own cute Spooky Scary Skeleton Robot for Halloween - Using a Raspberry Pi Pico/Pico W, HC-SR04 Ultrasonic Range finder, and an SG90 Servo
cover: /assets/img/blog/spookyscaryskeleton/spookyscaryskeleton.jpg
tags:
 - Raspberry Pi Pico W
 - Pico
 - Pico W
 - Robot
 - MicroPython
 - Halloween
 
---

## Table of Contents

{:toc}
* toc

---

## Videos
There are a couple of videos covering the features, demo and build process for HeyBot.

{% include youtubeplayer.html id="/" %}
{% include youtubeplayer.html id="/gboF8HgULg0" %}

---

## Spooky Scary Skeleton Robot

This is a rather bare bones robot for Halloween, but its loads of fun! It uses an Ultrasonic range finder to detect people in front of it to activate its scared stiff expression. The Servo moves a mechanism inside to raise the eye brows and lower the jaw.

---

## Bill of Materials

Part                                                                         | Description                                            | Qty | Cost
-----------------------------------------------------------------------------|--------------------------------------------------------|-----|-------
Raspberry Pi Pico                                                           | The $4 microcontroller from Raspberry Pi               | 1   | £4.00
Servo | A low cost SG90 Servo | 1| £3.00
Ultrasonic Range Finder | HC-SR04 3.3v version | 1 | £2.00
M2 Bolts | Securely attach the Pico W and Ultrasonic Rangefinder using M2 Bolts | 8 | £0.80
M3 Bolts and nuts | Securely attach the top section to the bottom | 3 | £0.60
{:class="table table-striped"}

> Prices and availability may vary.

---

## MicroPython Code

Download or clone the code here: <https://www.github.com/kevinmcaleer/bare_bones>

---

## Assembly

### Bottom section 

[![The Base](/assets/img/blog/spookyscaryskeleton/part01.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part01.png)

The first part to start with is the `bottom` section.

### Servo

[![The Servo](/assets/img/blog/spookyscaryskeleton/part02.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part02.png)

Screw in the servo using 2 M2 bolts. The servo spindle should be towards the middle of the robot.

### Cog

[![The Cog](/assets/img/blog/spookyscaryskeleton/part03.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part03.png)

1. Cut the `servo horn` with a pair of wire cutters so that it fits into the `cog`
1. Push the `servo Horn` onto the `cog`
1. Make sure the servo is at the minimum rotation position (turn it clockwise until it doesn't turn any more)
1. Push the `cog` onto the `servo spindle`.
1. Ensure the cog can turn correctly and doesnt catch on any rough 3d printed parts

### Jaw

[![The Jaw](/assets/img/blog/spookyscaryskeleton/part04.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part04.png)

1. Slide the jaw into place under the cog - the jaw should be in the closed position

### The Eye Brows

[![The Eye brows](/assets/img/blog/spookyscaryskeleton/part05.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part05.png)

1. Slide the Eyebrows into place under the cog - the eye brows should be in the closed position

### The Gasket

[![The Gasket](/assets/img/blog/spookyscaryskeleton/part06.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part06.png)

1. The gasket adds a little extra room for the cog without affecting the flat underneath of the top section
1. It means we can print out other sizes if this doesn't work correctly

### Top Section

[![The Top section](/assets/img/blog/spookyscaryskeleton/part07.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part07.png)

1. Push the captive M3 nuts into the nut-holes on the bottom section. Use plyers to gently push them into place
1. Screw the 3 M3 bolts into place


### The Range Finder

[![The Range Finder](/assets/img/blog/spookyscaryskeleton/part08.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part08.png)

1. Screw the Range finder into place using the M2 bolts - they wont go all the way in, which is expected

### The Skull

[![The Skull](/assets/img/blog/spookyscaryskeleton/part10.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part10.png)

1. Glue the skull into place but applying superglue onto the raised box section - (adult supervison required!)
1. Wait for the glue to set before moving onto the next step (30 minutes for fast setting glue).

### The Pico

[![The Pico](/assets/img/blog/spookyscaryskeleton/part11.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/part11.png)

1. Flip the robot over and attach the Pico using 4 M2 bolts

**Well done** - you've assembled the robot, next to wire it up

---

## Wiring up the Robot - plug and play

[![Wiring Diagram](/assets/img/blog/spookyscaryskeleton/wiring.png){:class="img-fluid w-50"}](/assets/img/blog/spookyscaryskeleton/wiring.png)

### Connect the servo

1. Connect 3 male to femail Dupont cables from the `servo` to the `5v`, `GND` and the signal line to `GPIO15 pin` on the Pico
1. The middle wire is the `5v` line
1. The brown wire is the `GND` line
1. The orange wire is the `signal` line

### Connecting the HC-SR04 Range Finder

1. Connect 4 male to male Dupont cables from the `range finder` to:
* the `VCC` to the `3v` on the Pico
* the `GND` to a `GND` on the Pico (there are a few to choose from)
* The `Trigger` to `GPIO00 pin` on the pico
* The `Echo` to `GPIO01 pin` on the pico

---

## The STL files

There are a few parts to download and print:

* [`top.stl`](/assets/stl/skeleton/top.stl) - the top section
* [`bottom.stl`](/assets/stl/skeleton/bottom.stl) - the bottom section
* [`skull.stl`](/assets/stl/skeleton/skull.stl) - the front skull
* [`cog.stl`](/assets/stl/skeleton/cog.stl) - the servo cog
* [`eyebrows.stl`](/assets/stl/skeleton/eyebrows.stl) - the eye brows
* [`jaw.stl`](/assets/stl/skeleton/jaw.stl) - the jaw
* [`gasket.stl`](/assets/stl/skeleton/gasket.stl) - the gasket

If you enjoy these files, please consider [buying me a coffee](/coffee) (it took a while to design these!)