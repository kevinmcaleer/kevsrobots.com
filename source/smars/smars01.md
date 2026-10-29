---
title: Print the Parts
description: Build Instructions for standard SMARS
layout: lesson
cover: /learn/smars/assets/how_to_build_a_smars.jpg
---

## Watch the video

{% include youtubeplayer.html id="JpG0nKY4wM4" %}

---

### 1 Print the Parts

Download the build files from Thingiverse. [Click here for the latest version:](https://www.thingiverse.com/thing:2662828)

There are a couple of basic parts to build your SMARS:

### 3D Printed Parts

It makes sense to print the parts in roughly the order you are going to construct it in:

1. [The chassis](/assets/stl/smars/chassis.stl)
1. [The powered wheels x2](/assets/stl/smars/powered_wheel.stl)
1. [The unpowered wheels x2](/assets/stl/smars/unpowered_wheel.stl)
1. [The mechanical tracks x32](/assets/stl/smars/tracks.stl)
1. [The holding board (for the 9v battery)](/assets/stl/smars/motor_holder.stl)
1. [The range finder holder](/assets/stl/smars/range_finder_holder.stl)
1. [The range finder cover](/assets/stl/smars/range_finder_cover.stl)

### Electronics Bill of Materials

Item                    | Description            | Qty | Price
------------------------|------------------------|:---:|-----:
Ultrasonic Range finder | HC-SR04 Range Finder  |  1  | £2.50
Arduino                 | Arduino Uno V3         |  1  | £14.99
Motor Shield            | Fundumoto motor shield |  1  | £15.00
Motors                  | 6v N20 150RPM motors   |  2  | £6.99
{:class="table table-striped"}

You will also need:

- some wire (preferably black and red)
- some solder
- some flux to make the solder adhere properly.

### Tools

Tools required are:

- a 3D printer
- filament (PLA is fine)
- a soldering iron

### 3D Printer settings

- The printer can be comfortably set to 50% infill
- Use different coloured filament to give your SMARS some style
- Prices are a guide from when the course was written; they change a lot

## Check your understanding

{% quiz %}
? Which board is the brain of the original SMARS?
- [ ] Raspberry Pi Pico
- [x] Arduino Uno
- [ ] BBC micro:bit
> The original SMARS uses an Arduino Uno with a Fundumoto motor shield on top. (There's a Pico version too, but this course builds the original.)

? What does the ultrasonic range finder do?
- [ ] Plays sounds
- [x] Measures the distance to objects in front of the robot
- [ ] Charges the battery
> It sends out a pulse of sound and times the echo, so the robot can tell how far away an obstacle is.
{% endquiz %}
