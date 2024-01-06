---
layout: how_it_works
title: RGB LED Strips
short_title: RGB LED Strips
short_description: Learn about how RGB LED Strips work
description: Learn about how RGB LED Strips work
date: 2023-04-05
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/rgb-led_strip.png
tags:
 - RGB LED
 - Lighting
 - WS2812
 - APA102
 - How it works
---

`RGB LED Strips` are a popular choice for adding dynamic lighting effects to projects.

These strips typically consist of a flexible printed circuit board (PCB) with individual light emitting diodes (LEDs) mounted on them. These LEDs are capable of producing red, green, and blue light, which can be combined to create any color in the visible spectrum.

There are two main types of RGB LED strips: non-addressable and addressable. Non-addressable strips are wired in parallel, meaning that all the LEDs on the strip will display the same color at the same time. Addressable strips, on the other hand, have individually controllable LEDs, allowing for more complex lighting patterns and effects.

---

One of the most popular types of addressable RGB LED strips is the [`WS2812`](/resources/glossary#ws2812). These strips have an integrated controller chip, which allows each LED to be addressed individually using a single data wire. This makes them relatively easy to use with microcontrollers like the Arduino or Raspberry Pi.

---

Another popular type of addressable RGB LED strip is the [`APA102`](/resources/glossary#apa102). These strips use a different protocol than the WS2812, but also allow for individual control of each LED. They have a higher refresh rate than WS2812 strips, making them better suited for applications where fast-changing colors are important.

---

> ## Pimoroni Plasma Stick 2040 W
>
> [![Pimoroni Plasma Stick 2040 W](/assets/img/how_it_works/rgb-led_strip02.png){:class="img-fluid w-100"}](https://collabs.shop/fm9pd7)
>
> To make working with RGB LED Strips a lot easier, you can bag yourself a [Pimoroni Plasma Stick 2040 W](https://collabs.shop/fm9pd7) for £12 (plus shipping).
>
> It has a [Raspberry Pi Pico W] aboard, 3 terminal screw blocks and a Qwic connector for attaching additional sensors, and there are some great
> MicroPython tutorials on how to use this so you'll be up and running in no time.
>

## RGB & RGBW

In addition to standard RGB LEDs, there are also RGBW LEDs that have an additional white LED element. This white LED can be used to produce a wider range of colors, including pastels and more natural-looking hues.

The main difference between RGB and RGBW LEDs is the number of LED elements on the strip. RGB LEDs have three elements - red, green, and blue - while RGBW LEDs have four elements - red, green, blue, and white.

The white LED in an RGBW LED strip can be controlled separately from the RGB LEDs, allowing for more precise color mixing. This means that an RGBW strip can produce a wider range of colors than an RGB strip.

However, it's important to note that not all RGBW strips are created equal. Some RGBW strips have a separate white LED element, while others use a combination of the RGB LEDs to produce white light. The latter type is known as RGBWW, with the extra "W" indicating the use of a separate white LED.

When using RGBW or RGBWW LED strips, it's important to ensure that the software you're using is compatible with the extra LED element. For example, some libraries that are designed for standard RGB strips may not work with RGBW or RGBWW strips.

Overall, RGBW and RGBWW LED strips are a great choice if you need to produce a wider range of colors or more natural-looking hues in your project. Just make sure to choose the right type of strip and use compatible software to get the best results.

---

## Common types of addressable RGB LED

Here is a list of some of the most common types of addressable RGB LEDs:

`WS2812` / `WS2812B` - These LEDs are popular due to their ease of use with microcontrollers, as each LED has an integrated driver IC that allows for individual control.

`SK6812` / `SK6812RGBW` - Similar to the `WS2812`, but with some improvements such as higher PWM frequency and compatibility with 400kHz I2C bus.

`APA102` / `APA102C` / `APA102-2020` - These LEDs use a different protocol than `WS2812` LEDs and have a higher refresh rate, making them well-suited for applications where fast-changing colors are important.

* `LPD8806` - These LEDs use a unique protocol that requires two data lines, but can be controlled with a single clock line.
* `P9813` - These LEDs also require two data lines, but use a simpler protocol than `LPD8806` LEDs.
* `SK9822` - These LEDs are similar to `APA102` LEDs but use a different protocol that can be more reliable over longer distances.
* `TM1814` - These LEDs use a protocol similar to the `WS2812`, but with a higher PWM frequency.
* `UCS1903` - These LEDs are similar to the `WS2812`, but with a lower PWM frequency.
* `GS8208` - These LEDs are similar to the `WS2812`, but with a larger color gamut and improved white balance.
* `SM16703` - These LEDs are similar to the `LPD8806`, but with a simpler protocol and improved color accuracy.

Each type of addressable RGB LED has its own unique features and benefits, and the choice of which to use depends on the specific requirements of your project.

---

## Controlling addressable RGB LEDs

To control an addressable RGB LED strip like the `WS2812` or `APA102`, you'll need to send data to each LED over the data wire. In MicroPython, you can use a library like the neopixel library to control these strips. Here's an example MicroPython program that will cycle through different colors on a `WS2812` strip:

```python
import neopixel
import time

# Configure the LED strip
num_pixels = 10 # The total number of RGB LEDs in the strip
pin = 0         # the GPIO pin for the data line on the RGB LED Strip
strip = neopixel.NeoPixel(pin, num_pixels)

# Define some colors
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

# Cycle through the colors
while True:
    for color in colors:
        strip.fill(color)
        strip.write()
        time.sleep(1)
```

In this program, we first configure the LED strip by specifying the number of pixels and the pin that it's connected to. We then define a list of three colors (red, green, and blue) and loop through them, setting the color of the entire strip to each color in turn. The strip.write() method sends the color data to the LEDs, and the time.sleep(1) method pauses for one second between each color change.

---

Overall, RGB LED strips are a versatile and fun way to add lighting effects to your projects. Whether you're using a non-addressable or addressable strip, there are plenty of programming libraries available to make controlling them easy.

---
