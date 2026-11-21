---
layout: lesson
title: Drawing on the LED matrix
author: Kevin McAleer
type: page
cover: /learn/arduino_uno_q/assets/banners/04_drawing_on_the_led_matrix.jpg
date: 2026-11-21
previous: 03_your_first_app.html
next: 05_pins_sensors_and_servos.html
description: Draw pictures, set brightness levels and make a beating heart on the
  Uno Q's 8 by 13 LED matrix
percent: 50
duration: 8
quiz_total: 27
date_updated: 2026-11-21
navigation:
- name: Arduino Uno Q - Two Brains, One Board
- content:
  - section: Meet the board
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Two brains, one board
      link: 01_two_brains_one_board.html
    - name: First boot, App Lab and Wi-Fi
      link: 02_first_boot_app_lab_and_wifi.html
    - name: Your first App
      link: 03_your_first_app.html
  - section: The microcontroller side
    content:
    - name: Drawing on the LED matrix
      link: 04_drawing_on_the_led_matrix.html
    - name: Pins, sensors and servos
      link: 05_pins_sensors_and_servos.html
  - section: The Bridge
    content:
    - name: How the Bridge works
      link: 06_how_the_bridge_works.html
    - name: Python calls the sketch
      link: 07_python_calls_the_sketch.html
    - name: The sketch calls Python
      link: 08_the_sketch_calls_python.html
    - name: When the Bridge goes wrong
      link: 09_when_the_bridge_goes_wrong.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

In lesson 3 you blinked a single LED. This time we have 104 of them to play with. Let's draw a heart, fade it in and out, and give the Q Board its first animation.

---

## One array, 104 LEDs

The blue LED matrix has **8 rows** and **13 columns**. That is 8 x 13 = 104 LEDs. It belongs to the microcontroller, so everything in this lesson lives in `sketch/sketch.ino`. The Python side does not change.

To draw on it, you hand the matrix a flat list of 104 numbers. The list is read **row by row**, starting at the top left. The first 13 numbers are the top row, the next 13 are the second row, and so on.

So how do you find the number for a given LED? Count rows and columns from zero:

```
index = row * 13 + col
```

A worked example. You want the LED in row 2, column 5 (the third row down, the sixth LED along):

- 2 rows of 13 are already used up: 2 * 13 = 26
- add the column: 26 + 5 = **31**

So `frame[31]` is that LED. The top left LED is `frame[0]` and the bottom right is `frame[103]`.

**Why this matters:** every drawing, bar graph and animation in this course is built from that one sum.

---

## Draw a heart (1 bit)

Here is a complete sketch. Each LED is either on (1) or off (0), so we ask for **1 bit** of brightness.

**sketch/sketch.ino**
```cpp
#include <Arduino_LED_Matrix.h>

// 8 rows of 13 LEDs = 104 values. 1 is on, 0 is off.
uint8_t heart[104] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,1,1,0,0,0,1,1,0,0,0,
    0,0,1,1,1,1,0,1,1,1,1,0,0,
    0,0,1,1,1,1,1,1,1,1,1,0,0,
    0,0,0,1,1,1,1,1,1,1,0,0,0,
    0,0,0,0,1,1,1,1,1,0,0,0,0,
    0,0,0,0,0,1,1,1,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,0,0,0
};

Arduino_LED_Matrix matrix;

void setup() {
  matrix.begin();
  matrix.setGrayscaleBits(1);  // 1 bit: each LED is on or off
  matrix.draw(heart);
}

void loop() {
}
```

Walking through it:

- `uint8_t heart[104]` is the picture. Each line of the array is one row of the matrix, so you can see the heart in the numbers.
- `Arduino_LED_Matrix matrix;` creates the matrix object, and `matrix.begin()` switches it on.
- `matrix.setGrayscaleBits(1)` says each value is 1 bit: on or off.
- `matrix.draw(heart)` sends all 104 values to the LEDs. The picture stays there, so `loop()` has nothing to do.

Press **Run** and the heart appears. The Python file stays exactly as it was in lesson 3, so you do not need to touch it.

---

## Paint your own

Typing 104 numbers by hand is no fun. Use the painter below instead.

{% include matrix_painter.html preset="Heart" name="heart" %}

Paint your own picture, then press **Copy sketch** and paste the result into `sketch/sketch.ino`. Press **Run** and your picture is on the board.

---

## Brightness (3 bits)

With 1 bit every LED is on or off. With **3 bits** each LED gets a level from 0 (off) to 7 (brightest). 3 bits is the default. The sketch below lights each row a little brighter than the one above it.

**sketch/sketch.ino**
```cpp
#include <Arduino_LED_Matrix.h>

uint8_t fade[104] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,1,1,1,1,1,1,1,1,1,1,1,
    2,2,2,2,2,2,2,2,2,2,2,2,2,
    3,3,3,3,3,3,3,3,3,3,3,3,3,
    4,4,4,4,4,4,4,4,4,4,4,4,4,
    5,5,5,5,5,5,5,5,5,5,5,5,5,
    6,6,6,6,6,6,6,6,6,6,6,6,6,
    7,7,7,7,7,7,7,7,7,7,7,7,7
};

Arduino_LED_Matrix matrix;

void setup() {
  matrix.begin();
  matrix.setGrayscaleBits(3);  // 3 bits: brightness 0 to 7
  matrix.draw(fade);
}

void loop() {
}
```

The only real change is `setGrayscaleBits(3)`. The numbers in the array now mean brightness instead of on and off. There is also an 8-bit mode, `setGrayscaleBits(8)`, which takes 0 to 255 and maps it down to the matrix's levels. We stay with 3 bits in this course.

Try the painter with levels switched on:

{% include matrix_painter.html preset="Fade" name="fade" %}

Paint a few LEDs at different brightness levels, then copy the sketch and see how it looks on the real matrix.

---

## Q Board, stage 2: the beating heart

The Q Board now gets its first animation. We want the heart to beat: dim, bright, dim, bright.

The obvious way is `delay(400)` between each change. It works, but `delay()` freezes the whole sketch while it waits. Nothing else can run. That is fine today, and a real problem later: in the coming lessons `loop()` will read a sensor, listen for messages from Python and answer them. A sketch that is asleep cannot do any of that.

The trick is to **check the clock instead of waiting**. `millis()` gives the number of milliseconds since the sketch started. We remember when the last beat happened, and on every pass of `loop()` we ask "has 400 ms gone by yet?". If not, we carry on straight away.

**sketch/sketch.ino**
```cpp
#include <Arduino_LED_Matrix.h>

// Q Board, stage 2: a beating heart on the LED matrix

const uint8_t heart[104] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,1,1,0,0,0,1,1,0,0,0,
    0,0,1,1,1,1,0,1,1,1,1,0,0,
    0,0,1,1,1,1,1,1,1,1,1,0,0,
    0,0,0,1,1,1,1,1,1,1,0,0,0,
    0,0,0,0,1,1,1,1,1,0,0,0,0,
    0,0,0,0,0,1,1,1,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,0,0,0
};

uint8_t frame[104];            // what we actually send to the matrix
Arduino_LED_Matrix matrix;

unsigned long lastBeat = 0;
bool bright = false;

// Copy the heart into the frame at a chosen brightness (0 to 7)
void drawHeart(uint8_t level) {
  for (int i = 0; i < 104; i++) {
    frame[i] = heart[i] * level;
  }
  matrix.draw(frame);
}

void setup() {
  matrix.begin();
  matrix.setGrayscaleBits(3);
  drawHeart(2);
}

void loop() {
  // millis() counts the milliseconds since the sketch started.
  // Checking it, instead of calling delay(), leaves loop() free to do other jobs.
  if (millis() - lastBeat >= 400) {
    lastBeat = millis();
    bright = !bright;
    drawHeart(bright ? 7 : 2);
  }
}
```

How it works:

- `heart` is our 1 and 0 picture, kept unchanged. `frame` is the copy we send to the matrix.
- `drawHeart(level)` multiplies every value by the brightness. A 1 becomes the level, a 0 stays 0. That gives a heart at any brightness from one array.
- `lastBeat` remembers when we last changed. `bright` remembers which state we are in.
- In `loop()`, `millis() - lastBeat >= 400` is true once every 400 ms. Then we note the time, flip `bright` and redraw at level 7 or level 2.

**Why this matters:** between beats, `loop()` is spinning round hundreds of times doing nothing. Later, that spare time is where the sensor reading and the Bridge messages will go. The heart will keep beating while they happen.

Python is unchanged again: `python/main.py` is still the file from lesson 3. **Q Board stage 2 is done.**

---

## Try it Yourself

1. Paint your own initials in the painter, copy the sketch and run it on the board.
2. Change the 400 in the beating heart to 200 for a faster heartbeat. Then try 1000.
3. Challenge: make a two-frame animation. Make two arrays (for example a closed and an open mouth), and use `bright` to choose which one to draw each time the timer fires.

## Common Issues

**Problem**: The picture is shifted, scrambled or the sketch will not compile.
**Solution**: Count the values. The array needs exactly 104, in 8 rows of 13.
**Why**: `draw()` reads 104 values from the start of the array. Too few or too many and every LED after the mistake is in the wrong place.

**Problem**: With levels in the array, the picture looks too dim, or the levels all look the same.
**Solution**: Call `matrix.setGrayscaleBits(3)` after `matrix.begin()`.
**Why**: The number of bits tells the matrix how to read your values. 1 bit treats everything as on or off, so levels are lost.

**Problem**: A value of 8 or more in a 3-bit picture gives odd brightness.
**Solution**: Keep every value between 0 and 7 when using 3 bits.
**Why**: 3 bits can only hold 8 levels, 0 to 7.

---

## Check your understanding

{% quiz %}
? Which array index is the LED in row 3, column 4 (counting from zero)?
- [ ] 7
- [ ] 34
- [x] 43
- [ ] 52
> Index = row * 13 + col, so 3 * 13 + 4 = 43.

? Why does the beating heart use millis() instead of delay()?
- [ ] delay() does not work on the Uno Q
- [x] delay() freezes loop(), and later loop() has other jobs to do
- [ ] millis() makes the LEDs brighter
> Checking the clock keeps loop() running, so it stays free for sensors and Bridge messages.

? What range of brightness values does setGrayscaleBits(3) give?
- [ ] 0 to 1
- [x] 0 to 7
- [ ] 0 to 255
> 3 bits hold 8 levels. 1 bit is on or off, and 8 bits is 0 to 255.
{% endquiz %}
