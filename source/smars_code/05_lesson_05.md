---
title: Lesson 05 - Bluetooth
description: Learn how to connect a Bluetooth module to your Arduino and then use that to pair to an iOS or Android phone
layout: lesson
---

## Watch the livestream

{% include youtubeplayer.html id="TScmQV3fcjw" %}

---

## What you'll learn

In this lesson you will learn how to connect a Bluetooth module to your Arduino and use it to pair with an iOS or Android phone. We will then modify the code written in previous [lessons](02_lesson_02.html) to remotely control our SMARS.

---

## What you'll need

Before you begin, you'll need to make sure you have a couple of things before you start this lesson:

* A [SMARS robot](/learn/smars/)
* A USB Cable to connect to your computer
* A Computer that can run the [Arduino IDE](https://www.arduino.cc/en/software)
* A Bluetooth module: an HM-10 or HC-08 (Bluetooth Low Energy, works with iPhone and Android) or an HC-05 / HC-06 (Android only)
* The free [Dabble](https://ai.thestempedia.com/docs/dabble-app/getting-started-with-dabble/) app on your phone

---

## Preparation

1. Connect the SMARS to the USB cable, and the USB Cable to your computer
1. Launch the [Arduino IDE](https://www.arduino.cc/en/software) on your computer
1. Create a new Sketch, by clicking the `New Sketch` button

---

## Let's code

You can download the code below from [GitHub](https://www.github.com/kevinmcaleer/lesson_05_bluetooth), but it's better to type it in yourself line by line, as you will get to understand what each line means.

Make sure to download the `pitches.h` and `melody.h`, from the github link above, to the same folder where you create the code below.

---

```cpp
// These two lines must come BEFORE #include <Dabble.h>
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>
#include "pitches.h"

// set Motor A to Arduino Pins
int motor_A = 12;
int motor_B = 13;
int buzzer = 4;

// set the Motor Speed using the Arduino Pins
int motor_A_speed = 10;
int motor_B_speed = 11;

// set the time between motor on and motor off
int wait_in_milliseconds = 100;

// notes in the melody:
int melody[] = {
  F3, D3, AS2, AS2, AS2, C3, D3, DS3, F3, F3, F3, D3
};

int noteDurations[] = {
  8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 4, 8
};

void setup() {
  Dabble.begin(9600);   // Bluetooth module on pins 2 (RX) and 3 (TX)
  Serial.begin(9600);

  // set the Arduino pins to OUTPUT mode
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(buzzer, OUTPUT);
}

// set both motor directions, run for a short time, then stop
void drive(int dirA, int dirB) {
  digitalWrite(motor_A, dirA);
  digitalWrite(motor_B, dirB);
  analogWrite(motor_A_speed, 255);
  analogWrite(motor_B_speed, 255);
  delay(wait_in_milliseconds);
  analogWrite(motor_A_speed, 0);
  analogWrite(motor_B_speed, 0);
}

void forward()   { drive(LOW, HIGH); }
void backward()  { drive(HIGH, LOW); }
void turnLeft()  { drive(LOW, LOW); }
void turnRight() { drive(HIGH, HIGH); }

void beep() {
  tone(buzzer, 1000, 200);
}

void playMelody() {
  for (int thisNote = 0; thisNote < 12; thisNote++) {
    // note duration: one second divided by the note type
    // e.g. quarter note = 1000 / 4, eighth note = 1000 / 8
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(buzzer, melody[thisNote], noteDuration);

    // leave a short gap between notes: the note's duration + 30% works well
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(buzzer);
  }
}

void loop() {
  Dabble.processInput();   // read the latest button presses from the app

  if (GamePad.isUpPressed())       forward();
  if (GamePad.isDownPressed())     backward();
  if (GamePad.isLeftPressed())     turnLeft();
  if (GamePad.isRightPressed())    turnRight();
  if (GamePad.isTrianglePressed()) beep();
  if (GamePad.isSquarePressed())   playMelody();
}
```

---

## What the code means

- **Dabble** is a free phone app (iOS and Android) with a gamepad screen. The Dabble Arduino library receives its button presses over Bluetooth.
- The `#define` lines tell the Dabble library which of its modules we want. They must come *before* `#include <Dabble.h>`, or the gamepad won't work.
- `Dabble.begin(9600)` talks to the Bluetooth module on pins 2 and 3 using software serial, so the USB serial port stays free for uploading and debugging.
- `drive()` is the movement code from lessons 1 and 2, squashed into one function that takes the two motor directions.
- In `loop()`, `Dabble.processInput()` reads the app, then each `if` checks one button.

## Connect your phone

1. Install the **Dabble** app and the **Dabble** library (**Tools > Manage Libraries**).
2. Wire the Bluetooth module: VCC to 5V, GND to GND, module TX to Arduino pin 2, module RX to Arduino pin 3.
3. Upload the sketch, open Dabble, tap the connect icon and choose your module, then open the **Gamepad**.

---

## Check your understanding

{% quiz %}
? Why must `#define CUSTOM_SETTINGS` come before `#include <Dabble.h>`?
- [x] The library reads those settings while it's being included
- [ ] The Arduino IDE sorts the lines alphabetically
- [ ] It doesn't matter where it goes
> `#define` and `#include` are handled in order, top to bottom, before the code is compiled.

? Which Bluetooth module would you choose to control SMARS from an iPhone?
- [ ] HC-05
- [x] HM-10 (Bluetooth Low Energy)
- [ ] Any module works with iPhone
> iPhones only talk to Bluetooth Low Energy modules like the HM-10 or HC-08.
{% endquiz %}
