---
layout: blog
title: Radar robot
short_title: Radar Robot
short_description: Scan surroundings using Ultrasound
date: 2022-11-06
author: Kevin McAleer
excerpt: Build a robot that can scan its surroundings with sound
cover: /assets/img/blog/radar/radar.jpg
tags:
 - Raspberry Pi Pico
 - 3d Printing
 - Explora
 - Radar
 - Ultrasonic
 - Servo
---

## Contents

{:toc}
* toc

---

# YouTube Video
Click the thumbnail below to watch the show all about this build.
{% include youtubeplayer.html id="2f0fhte4vno" %}

---

## Ultrasonic Radar - how it works

![slide 03](/assets/img/blog/radar/slide003.jpg){:class="img-fluid w-100"}

We can build a simple, radar like scanning system by attaching an Ultrasonic Range Finder a Servo, and rotate the servo about whilst taking readings.

Specifically, we will rotate the servo 1 degree at a time, take a distance reading, output the reading to the radar display, and then move to the next angle until the entire sweep is complete.

Later, in another part of this series we’ll send the set of readings to a trained ML model and see if it can recognise any objects within the scan.

---
## Radar display

### Drawing the Radar

![slide 06](/assets/img/blog/radar/slide006.jpg){:class="img-fluid w-100"}

SOHCAHTOA - It’s all about triangles!

We want to create a radar-like display. The scan will sweep round a 180° arc, and any objects in front of the range finder will display on the scan, proportionate to the display.

The display will be housed on the back of the robot (we'll add this in a later part).

---

### PicoGraphics

![slide 07](/assets/img/blog/radar/slide007.jpg){:class="img-fluid w-100"}

We'll use the Pimoroni MicroPython as it includes their PicoGraphics library, which is great for drawing vector graphics.

PicoGraphics has a `line` primitive takes X1, Y1, X2, Y2 coordinates. We can use this to draw our radar sweep.

---

### The Display

![slide 08](/assets/img/blog/radar/slide008.jpg){:class="img-fluid w-100"}

The display I've chosen for this project is a 240x240 colour display - you can grab one from here: <https://shop.pimoroni.com/products/1-3-spi-colour-lcd-240x240-breakout>.

The display coordinates `X, Y` `0, 0` are at the top left of the display.

This display uses an ST7789V display driver which also happens to be built into the Pimoroni Pico Explorer Base, which I used to prototype this project.

Other specifications for this display:

* It has 240 x 240 pixels
* Square 1.3” IPS LCD display
* Uses the SPI bus

I’m looking at putting the breakout version of this display on the robot, in a later part of the series.

---

### Drawing the sweep

![slide 09](/assets/img/blog/radar/slide009.jpg){:class="img-fluid w-100"}

We will draw a series of lines, one for each of the 180° angles of the sweep. 

To draw the line we need to solve a triangle to find the `x1` and `y1` start positions of the line
We can then use PicoGraphics function:

``` python
display.line(x1, y1, x2, y2)
```

![slide 10](/assets/img/blog/radar/slide010.jpg){:class="img-fluid w-100"}

We need to solve the triangle to find the position of `x1, y1`.

We know what `x2, y2`is:

* `y2` is the bottom of the screen (`height`) 
* `x2` = its the middle of the screen (`width /2`)
* We know the length of side c of the triangle, angle `A` as well as angle `C`
* We need to find the length of side a (`y1`), and length of side b (`x1`, or more accurately `middle - b`)

---

### AAS Triangle

![slide 11](/assets/img/blog/radar/slide011.jpg){:class="img-fluid w-100"}

***Angle, Angle, Side***

* We can solve Angle B by subtracting 180 from A+C (which we already know)
* We can solve sides a and b using the AAS formula:
  * side a = `a/sin A = c/sin C`
  * side b = `b/sin B = c/sin C`
￼
---

## 3D Design

![slide 12](/assets/img/blog/radar/slide012.jpg){:class="img-fluid w-100"}

### Chassis

![slide 13](/assets/img/blog/radar/slide013.jpg){:class="img-fluid w-100"}

This robot uses the Explora base.

The Explora base is a simple, quick to print and easy to reproduce Chassis for building robots.
It's 3mm thick, very quick to print, Solid, doesn’t bend, and easy to attach motors and wheels.

### Explora Blueprint

![slide 14](/assets/img/blog/radar/slide014.jpg){:class="img-fluid w-100"}

The Explora base starts with a 90 x 70mm rectangle, has four 'tabs'; one for each the wheel.
There are also front and rear sections. 

You will want to add the holes and mounting points depending on your own design.

---

### Servo holder

![slide 15](/assets/img/blog/radar/slide015.jpg){:class="img-fluid w-100"}

The Servo holder sits on top of the chassis and is held in place by 3x `M3` captive nut and screws.

---

### Servo

![slide 16](/assets/img/blog/radar/slide016.jpg){:class="img-fluid w-100"}

Servo screws in from underneath. You can use any commonly available servo, including:

* SG90
* MG90
* DS929MG
* TowerPro MG92B

Use the two larger screws included with the Servo to secure the servo to the servo holder. 

---

### Range Finder Holder

![slide 17](/assets/img/blog/radar/slide017.jpg){:class="img-fluid w-100"}

The Range Finder holder attaches the Servo Horn to the Servo.

Ensure you center the Servo and face range finder straight ahead before screwing it in.

Secure the servo horn to the servo spindle using the small screw included with the servo.

---

## Ultrasonic Range Finder

![slide 18](/assets/img/blog/radar/slide018.jpg){:class="img-fluid w-100"}

Add Ultrasonic Range Finder to the back of the Range Finder holder; it should just push-fit; no glue or screws required.

Connect 4 Dupont cables to:

* GND
* VCC
* Trigger
* Echo

---

## MicroPython code

Download the latest version of the code from GitHub: <https://github.com/kevinmcaleer/radar_robot>

### Radar.py
Radar.py will scan the area in front of the robot by rotating the range finder. Each of the readings will be written to a `readings.csv` file on the Pico.

``` python
# radar.py
# Kevin McAleer
# Nov 2022

from servo import Servo
from time import sleep
from range_finder import RangeFinder

from machine import Pin

trigger_pin = 2
echo_pin = 3

DATA_FILE = 'readings.csv'

s = Servo(0)
r = RangeFinder(trigger_pin=trigger_pin, echo_pin=echo_pin)

def take_readings(count):
    readings = []
    with open(DATA_FILE, 'ab') as file:
        for i in range(0, 90):
            s.value(i)
            value = r.distance
            print(f'distance: {value}, angle {i} degrees, count {count}')
            sleep(0.01)
        for i in range(90,-90, -1):
            s.value(i)
            value = r.distance
            readings.append(value)
            print(f'distance: {value}, angle {i} degrees, count {count}')
            sleep(0.01)
        for item in readings:
            file.write(f'{item}, ')
        file.write(f'{count} \n')
    
    print('wrote datafile')
    for i in range(-90,0,1):
        s.value(i)
        value = r.distance
        print(f'distance: {value}, angle {i} degrees, count {count}')
        sleep(0.05)

def demo():
    for i in range(-90, 90):
        s.value(i)
        print(f's: {s.value()}')
        sleep(0.01)
    for i in range(90,-90, -1):
        s.value(i)
        print(f's: {s.value()}')
        sleep(0.01)

def sweep(s,r):
    """ Returns a list of readings from a 180 degree sweep """
    
    readings = []
    
    for i in range(-90,90):
        s.value(i)
        sleep(0.01)
        readings.append(r.distance)
    return readings

for count in range(1,2):
    take_readings(count)
    sleep(0.25)
```

---

### Radar_Display.py


``` python
from picographics import PicoGraphics, DISPLAY_PICO_EXPLORER
import gc
from math import sin, radians
gc.collect()
from time import sleep
from range_finder import RangeFinder
from machine import Pin
from servo import Servo
from motor import Motor

m1 = Motor((4, 5))
m1.enable()

# run the motor full speed in one direction for 2 seconds
m1.to_percent(100)

trigger_pin = 2
echo_pin = 3

s = Servo(0)
r = RangeFinder(trigger_pin=trigger_pin, echo_pin=echo_pin)

display = PicoGraphics(DISPLAY_PICO_EXPLORER, rotate=0)
WIDTH, HEIGHT = display.get_bounds()

REALLY_DARK_GREEN = {'red':0, 'green':64, 'blue':0}
DARK_GREEN = {'red':0, 'green':128, 'blue':0}
GREEN = {'red':0, 'green':255, 'blue':0}
LIGHT_GREEN = {'red':255, 'green':255, 'blue':255}
BLACK = {'red':0, 'green':0, 'blue':0}

def create_pen(display, color):
    return display.create_pen(color['red'], color['green'], color['blue'])

black = create_pen(display, BLACK)
green = create_pen(display, GREEN)
dark_green = create_pen(display, DARK_GREEN)
really_dark_green = create_pen(display, REALLY_DARK_GREEN)
light_green = create_pen(display, LIGHT_GREEN)

length = HEIGHT //2
middle = WIDTH // 2

angle = 0

def calc_vectors(angle, length):
    # Solve and AAS triangle
    # angle of c is
    #
    #  B        x1, y1
    #  |\        \ 
    #  | \        \
    # a|_ \c       \
    #  |_|_\        \
    # C  b  A       x2,y2
    
    A = angle
    C = 90
    B = (180 - C) - angle
    c = length
    a = int((c * sin(radians(A))) / sin(radians(C))) # a/sin A = c/sin C
    b = int((c * sin(radians(B))) / sin(radians(C)))  # b/sin B = c/sin C
    x1 = middle - b
    y1 = (HEIGHT -1) - a
    x2 = middle
    y2 = HEIGHT -1
    
#     print(f'a:{a}, b:{b}, c:{c}, A:{A}, B:{B}, C:{C}, angle: {angle}, length {length}, x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}')
    return x1, y1, x2, y2
    
a = 1
while True:
    
#     print(f'x1:{x1}, y1:{y1}, x2:{x2}, y2:{y2}')
    s.value(a)
    distance = r.distance
    if a > 1:
        x1, y1, x2, y2 = calc_vectors(a-1, 100)
        display.set_pen(really_dark_green)
        
        display.line(x1, y1, x2, y2)
    
    if a > 2:
        x1, y1, x2, y2 = calc_vectors(a-2, 100)
        display.set_pen(dark_green)
        display.line(x1, y1, x2, y2)
    
#     if a > 3:
#         x1, y1, x2, y2 = calc_vectors(a-3, 100)
#         display.set_pen(black)
#         display.line(x1, y1, x2, y2)
       
    # Draw the full length
    x1, y1, x2, y2 = calc_vectors(a, 100)    
    display.set_pen(light_green)
    display.line(x1, y1, x2, y2)
    
    # Draw lenth as a % of full scan range (1200mm)
    scan_length = int(distance * 3)
    if scan_length > 100: scan_length = 100
    print(f'Scan length is {scan_length}, distance is: {distance}')
    x1, y1, x2, y2 = calc_vectors(a, scan_length)    
    display.set_pen(green)
    display.line(x1, y1, x2, y2)
    
    display.update()

    a += 1
    if a > 180:
        a = 1
        display.set_pen(black)
        display.clear()
        display.update()
```

---

## STL files

Download the STL files for this project here:

* [`chassis.stl`](/assets/stl/radar/chassis.stl) - Chassis
* [`motor_holder_v4.stl`](/assets/stl/radar/motor_holder_v4.stl) - 4x Motor holders
* [`servo_holder_v2.stl`](/assets/stl/radar/servo_holder_v2.stl) - Servo Holder
* [`range_finder_holder.stl`](/assets/stl/radar/range_finder_holder.stl) - Range Finder Holder

