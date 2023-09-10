---
layout: lesson
title: Lesson 05 - Bluetooth
author: Kevin McAleer
type: page
cover: /learn/smars_code/assets/program_smars.png
previous: 04_lesson_04.html
next: 06_lesson_06.html
description: Learn how to connect a bluetooth module to your Arduino and then use
  that to pair to an iOS or Android phone
percent: 70
duration: 3
navigation:
- name: Learn how to program SMARS with Arduino
- content:
  - section: Movement
    content:
    - name: Lesson 01 - Movement
      link: 01_lesson_01.html
    - name: Lesson 02 - Turning
      link: 02_lesson_02.html
  - section: Sensing
    content:
    - name: Lesson 03 - Ultrasonic distance sensor
      link: 03_lesson_03.html
    - name: Lesson 04 - Positioning with the MPU-6050
      link: 04_lesson_04.html
  - section: Control
    content:
    - name: Lesson 05 - Bluetooth
      link: 05_lesson_05.html
    - name: Lesson 06 - How to control servos
      link: 06_lesson_06.html
    - name: Lesson 07 - Line Following
      link: 07_lesson_07.html
---


## LiveStream video

{% include youtubeplayer.html id="TScmQV3fcjw" %}

---

## What you'll learn

In this lesson you will learn how to connect a HC-08 (or HC-09 / HC-10) bluetooth module to your Arduino and then use that to pair to an iOS or Android phone. We will then modify the code written in previous [lessons](02_lesson_02) to remotely control our SMARS.

---

## Things's you'll needs

Before you begin, you'll need to make sure you have a couple of things before you start this lesson:

* A [SMARS robot](/learn/smars/)
* A USB Cable to connect to your computer
* A Computer that can run the [Arduino IDE](https://create.arduino.cc/editor)
* An HC-08 or higher Bluetooth module

---

## Preparation

1. Connect the SMARS to the USB cable, and the USB Cable to your computer
1. Launch the [Arduino IDE](https://create.arduino.cc/editor) on your computer
1. Create a new Sketch, by clicking the `New Sketch` button

---

## Lets code

You can download the code below from [GitHub](https://www.github.com/kevinmcaleer/lesson_05_bluetooth), but its better to type it in yourself line by line, as you will get to understand what each line means.

Make sure to download the `pitches.h` and `melody.h`, from the github link above, to the same folder where you create the code below.

---

``` c
#include <Dabble.h>
#include <SoftwareSerial.h>
#include "pitches.h"

#define CUSTOM_SETTINGS

#define INCLUDE_GAMEPAD_MODULE

// set Motor A to Arduino Pins
int motor_A = 12; // official Arduino Motor Shield uses D12
int motor_B = 13; // official Arduino Motor Shield uses D13
int buzzer = 4;

// set the Motor Speed using the Arduino Pins
int motor_A_speed = 10; // official Arduino Motor Shield uses D3
int motor_B_speed = 11; // official Arduino Motor Shield uses D11

// set the time between motor on and motor off
int wait_in_milliseconds = 100;

void setup() {
  // put your setup code here, to run once:
  Dabble.begin(9600);
  Serial.begin(9600);

    // set the Arduino pin to OUTPUT mode
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(buzzer, OUTPUT);
}

// notes in the melody:
int melody[] = {
  F3, D3, AS2, AS2, AS2, C3, D3, DS3, F3, F3, F3 ,D3 
};

int noteDurations[] = {
  8,  8,   4,   4,   8,  8,  8,  8,  4,  4,   4,  8 
};

struct Note {
 int key;
 int duraiton;
};

void playMelody() {
for (int thisNote = 0; thisNote < 12 ; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(4, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Dabble.processInput();
  if (GamePad.isUpPressed()) 
    forward();
  if (GamePad.isDownPressed()) 
    backward();
  
  if (GamePad.isLeftPressed()) turnLeft();
  if (GamePad.isRightPressed()) turnRight();
  if (GamePad.isTrianglePressed()) beep();
  if (GamePad.isSquarePressed()) playMelody();
}
```

---
