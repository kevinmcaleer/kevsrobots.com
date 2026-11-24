---
title: Pins, sensors and servos
description: Read a potentiometer, draw a bar graph on the matrix and optionally add a Modulino sensor or a servo
layout: lesson
type: page
cover: /learn/arduino_uno_q/assets/banners/05_pins_sensors_and_servos.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

A robot that cannot sense or move is just a light show. In this lesson the microcontroller reads a knob, draws it as a bar graph on the matrix, and (if you have the parts) talks to a sensor and turns a servo.

---

## The headers and their voltage

The Uno Q has the familiar Uno-shaped headers. They are the microcontroller's pins, so sketches control them directly.

> **Warning: the pins are 3.3 V.**
> Treat every pin as 3.3 V. The analogue inputs A0 to A5 are **not** 5 V tolerant, and neither is D3. Never connect more than 3.3 V to them. A 5 V sensor output on an analogue pin can damage the board. If a part runs at 5 V, check what voltage its signal wire outputs before you connect it.

Here are the pin groups you will use:

| Pin group | What it does |
| --- | --- |
| A0 to A5 | Analogue inputs. 14-bit readings, 0 to 16383, with a 3.3 V reference |
| D3, D5, D6, D9, D10, D11 | PWM outputs. `analogWrite` takes 0 to 255 by default, at 500 Hz |
| D20 (SDA), D21 (SCL) | The `Wire` I2C bus on the headers |
| Qwiic connector | A second I2C bus called `Wire1`. 3.3 V only |
{:class="table table-single"}

**Why this matters:** I2C sensors on the Qwiic connector use `Wire1`, not `Wire`. Mixing them up is one of the most common mistakes, and we will meet it again below.

The board is powered by USB-C (5 V, 3 A), or 5 V on the 5V pin, or 7 to 24 V on VIN.

---

## Read a potentiometer

A potentiometer (pot) is a knob with three legs. Turning it slides the middle leg between the two outer legs, so the middle leg gives a voltage anywhere from 0 V to 3.3 V.

Wire it like this, with the board unpowered:

1. Put the pot on your breadboard.
2. Connect one outer leg to **3.3V**.
3. Connect the other outer leg to **GND**.
4. Connect the middle leg to **A0**.

| Pot leg | Board pin |
| --- | --- |
| Outer leg 1 | 3.3V |
| Middle leg (wiper) | A0 |
| Outer leg 2 | GND |
{:class="table table-single"}

Double-check that it is 3.3V and not 5V on the first leg. Now the sketch:

**sketch/sketch.ino**
```cpp
const int potPin = A0;

void setup() {
  Serial.begin(9600);
  analogReadResolution(14);  // 14 bits: readings from 0 to 16383
}

void loop() {
  int reading = analogRead(potPin);
  float volts = reading * 3.3 / 16383.0;

  Serial.print(reading);
  Serial.print("  ");
  Serial.print(volts);
  Serial.println(" V");
  delay(200);
}
```

Press **Run**, open the **Serial Monitor** tab in the console and turn the knob.

- `analogReadResolution(14)` asks for 14-bit readings. 14 bits gives 2 to the power 14 = 16384 steps, numbered 0 to 16383.
- `analogRead(potPin)` returns the number for the voltage on A0. 0 means 0 V and 16383 means 3.3 V.
- `reading * 3.3 / 16383.0` turns that number back into volts. We use `16383.0` with a decimal point so the sum is done with decimals.
- Each `Serial.print` adds to the line, and `println` ends it.

Python stays as it was in lesson 3 throughout this lesson.

---

## Q Board, stage 3: a bar graph

Now let's put the knob on the matrix. We need to turn a reading from 0 to 16383 into a bar length from 0 to 13 columns. That is a job for `map()`:

```
map(value, fromLow, fromHigh, toLow, toHigh)
```

It rescales a number from one range to another. `map(reading, 0, 16383, 0, 13)` gives 0 at one end of the knob and 13 at the other. It works with whole numbers, so any fraction is dropped.

**sketch/sketch.ino**
```cpp
#include <Arduino_LED_Matrix.h>

// Q Board, stage 3: a bar graph of the reading on A0

const int potPin = A0;

uint8_t frame[104];
Arduino_LED_Matrix matrix;

// Light the first `columns` columns of the matrix (0 to 13)
void drawBar(int columns) {
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 13; col++) {
      frame[row * 13 + col] = (col < columns) ? 7 : 0;
    }
  }
  matrix.draw(frame);
}

void setup() {
  analogReadResolution(14);
  matrix.begin();
  matrix.setGrayscaleBits(3);
}

void loop() {
  int reading = analogRead(potPin);           // 0 to 16383
  int columns = map(reading, 0, 16383, 0, 13);
  drawBar(columns);
  delay(50);
}
```

`drawBar()` uses the `row * 13 + col` sum from lesson 4. Every LED in a column to the left of `columns` is set to brightness 7, and everything else is 0. Turn the knob and the bar grows and shrinks. **Q Board stage 3 is done.**

---

## Optional add-on: a Modulino Thermo on Qwiic

*Optional. You need a Modulino Thermo and a Qwiic cable. If you do not have one, skip to the next add-on.*

A Modulino is a small sensor board that plugs into the Qwiic connector, so there is no wiring to get wrong. First, in App Lab use **Add sketch library** and add `Arduino_Modulino`.

**sketch/sketch.ino**
```cpp
#include <Arduino_Modulino.h>

ModulinoThermo thermo;

void setup() {
  Serial.begin(9600);
  Modulino.begin(Wire1);  // the Qwiic connector is the second I2C bus, Wire1
  thermo.begin();
}

void loop() {
  float celsius = thermo.getTemperature();
  float humidity = thermo.getHumidity();

  Serial.print(celsius);
  Serial.print(" C  ");
  Serial.print(humidity);
  Serial.println(" %");
  delay(1000);
}
```

The line that matters is `Modulino.begin(Wire1)`. The Qwiic connector is on `Wire1`, so we tell the library to use it. Then `getTemperature()` and `getHumidity()` return the readings, which appear in the Serial Monitor.

---

## Optional add-on: a servo that follows the pot

*Optional. You need a hobby servo. If you do not have one, the bar graph above is the full lesson.*

Servos need the **Arduino_HardwareServo** library, not the generic `Servo` library. Add it with **Add sketch library** in App Lab.

Wire the servo's three leads like this:

| Servo lead | Board pin |
| --- | --- |
| Orange (signal) | D9 |
| Red (power) | 5V |
| Brown (ground) | GND |
{:class="table table-single"}

Keep the pot on A0 as before.

**sketch/sketch.ino**
```cpp
#include <Arduino_HardwareServo.h>

const int potPin = A0;
HardwareServo pointer;

void setup() {
  analogReadResolution(14);
  pointer.attach(D9);
}

void loop() {
  int reading = analogRead(potPin);
  int angle = map(reading, 0, 16383, 0, 180);
  pointer.write(angle);
  delay(20);
}
```

Here `map()` turns the 14-bit reading into an angle from 0 to 180 degrees. `pointer.write(angle)` moves the servo there.

**Timers.** The PWM pins share hardware timers, and a timer is either driving servos (50 Hz) or `analogWrite` (500 Hz), never both:

| Timer | Pins |
| --- | --- |
| TIM1 | D5, D11 |
| TIM3 | D3, D6 |
| TIM4 | D9, D10 |
{:class="table table-single"}

So a servo on D9 will misbehave if you use `analogWrite` on D10. Keep servos and `analogWrite` on different timers.

**Power.** A servo draws its power from the 5V pin, so use a 5 V / 3 A USB-C supply. For big servos, or several at once, power them from an external supply and join its ground to the board's GND.

---

> **Teachers**: Allow about 60 minutes (our estimate). The pins are 3.3 V and the analogue inputs A0 to A5 and D3 are not 5 V tolerant, so check parts before learners wire them, as a 5 V signal can damage the board. Shared kit needs a potentiometer and three jumper wires per learner or pair. The Modulino Thermo and servo are optional, and each has a no-part route.

## Try it Yourself

1. In the bar graph sketch, change the brightness 7 to 3. What happens to the bar?
2. Change `map(reading, 0, 16383, 0, 13)` so the bar is only ever 0 to 6 columns long.
3. Challenge: make the bar graph show the Modulino temperature instead of the pot. Work out a sensible `map()` range for your room.

## Common Issues

**Problem**: The reading is stuck at 0 or at 16383.
**Solution**: Check the wiring. The middle leg of the pot must go to A0, with the outer legs on 3.3V and GND.
**Why**: 0 means A0 is seeing GND and 16383 means it is seeing 3.3 V, so the middle leg is not reaching the pin properly or an outer leg is missing.

**Problem**: The readings jump around a little even when the knob is still.
**Solution**: Small jitter is normal. Average a few readings, or ignore tiny changes.
**Why**: With 16384 steps, tiny amounts of electrical noise show up as changes in the last digits.

**Problem**: The compiler complains about `Servo.h`.
**Solution**: Use `#include <Arduino_HardwareServo.h>` and the `HardwareServo` class, and add the library with **Add sketch library**.
**Why**: The Uno Q needs its own servo library, not the generic `Servo` one.

**Problem**: The Modulino is not found.
**Solution**: Call `Modulino.begin(Wire1)`, and make sure the `Arduino_Modulino` library has been added.
**Why**: The Qwiic connector is `Wire1`. `Wire` is the header pins.

**Problem**: The servo twitches, or the board keeps resetting.
**Solution**: Use a 5 V / 3 A USB-C supply, or power the servo externally with grounds joined.
**Why**: A servo draws a lot of current from the 5V pin, and a weak supply makes the board brown out.

---

## Check your understanding

{% quiz %}
? What is the highest reading analogRead gives with analogReadResolution(14)?
- [ ] 1023
- [ ] 4095
- [x] 16383
- [ ] 65535
> 14 bits gives 16384 steps, numbered 0 to 16383.

? Which library do you add to drive a servo on the Uno Q?
- [ ] Servo
- [x] Arduino_HardwareServo
- [ ] Arduino_Modulino
> Servos use the HardwareServo class from Arduino_HardwareServo, not the generic Servo library.

? A Modulino on the Qwiic connector is not found. What should you check first?
- [ ] That you used analogWrite
- [x] That you called Modulino.begin(Wire1) and added the Arduino_Modulino library
- [ ] That the pot is on A0
> The Qwiic connector is the second I2C bus, Wire1.
{% endquiz %}
