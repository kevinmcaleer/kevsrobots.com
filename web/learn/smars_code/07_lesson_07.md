---
layout: lesson
title: Lesson 07 - Line Following
author: Kevin McAleer
type: page
cover: /learn/smars_code/assets/program_smars.png
previous: 06_lesson_06.html
description: Learn how to use the line follow module
percent: 100
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

{% include youtubeplayer.html id="u9VT32q7ero" %}

---

## What you'll learn

In this lesson you will learn how to use the [line follow](/resources/glossary#tcrt5000) module to make the robot follow along a line, or stay within a marked area.

---

## Downloadable Track PDFs

Download the PDFs here:

* [smars_line_follow_track_01_loop.pdf](assets/smars_line_follow_track_01_loop.pdf)
* [smars_line_follow_track_02_figure_eight.pdf](assets/smars_line_follow_track_02_figure_eight.pdf)
* [smars_line_follow_track_03_tile_corner.pdf](assets/smars_line_follow_track_03_tile_corner.pdf)
* [smars_line_follow_track_04_straight.pdf](assets/smars_line_follow_track_04_straight.pdf)
* [smars_line_follow_track_05_wiggley.pdf](assets/smars_line_follow_track_05_wiggley.pdf)
* [smars_line_follow_track_06_round_corner.pdf](assets/smars_line_follow_track_06_round_corner.pdf)

---

## Things's you'll needs

* A [SMARS robot](/learn/smars/)
* A USB Cable to connect to your computer
* A Computer that can run the [Arduino IDE](https://create.arduino.cc/editor)
* A [line follow module](/resources/glossary#tcrt5000) - TCRT5000 or equivalent

---

## Preparation

1. Connect the SMARS to the USB cable, and the USB Cable to your computer
1. Launch the [Arduino IDE](https://create.arduino.cc/editor) on your computer
1. Create a new Sketch, by clicking the `New Sketch` button

---

## Lets code

You can download the code below from [GitHub](https://www.github.com/kevinmcaleer/lesson_07_linefollowing), but its better to type it in yourself line by line, as you will get to understand what each line means.

```c++
/*
 * Kevin McAleer
 * 24 July 2020 
 * 
 * Line Following Code
 * See Lesson 07 for the full tutorial
 * https://www.smarsfan.com/play/lessons/lesson_07_linefollow
 * Watch the livestream video:
 * https://youtu.be/u9VT32q7ero 
 * 
 *
 */

// set the line sensor thresholds
int light_threshold = 650;
int dark_threshold = 300;
int lineNumber; // stores the line sensor value
int lineSensorPin = 4;


// set Motor A to Arduino Pins
int motor_A = 12; // official Arduino Motor Shield uses D12
int motor_B = 13; // official Arduino Motor Shield uses D13
int buzzer = 4;

// set the Motor Speed using the Arduino Pins
int motor_A_speed = 10; // official Arduino Motor Shield uses D3
int motor_B_speed = 11; // official Arduino Motor Shield uses D11

// set the time between motor on and motor off
int wait_in_milliseconds = 10;

/////////////////////////////////////
/*
 * Movement block of code, from the movement lessons
 * https://www.smarsfan.com/play/lessons/lesson_01_movement
 * https://www.smarsfan.com/play/lessons/lesson_02_turning
 * 
 */

// move forward
void forward() {

  // set the direction to forward
  digitalWrite(motor_A, LOW);  
  digitalWrite(motor_B, HIGH);

  // set to full speed
  analogWrite(motor_A_speed, 255);
  analogWrite(motor_B_speed, 255);

  // wait
  delay(wait_in_milliseconds);

  // stop
  analogWrite(motor_A_speed, 0);
  analogWrite(motor_B_speed, 0);
}

void backward(){
  // set the direction to forward
  digitalWrite(motor_A, HIGH);  
  digitalWrite(motor_B, LOW);

  // set to full speed
  analogWrite(motor_A_speed, 255);
  analogWrite(motor_B_speed, 255);

  // wait
  delay(wait_in_milliseconds);

  // stop
  analogWrite(motor_A_speed, 0);
  analogWrite(motor_B_speed, 0);
}

void turnRight(){
  // set the direction to forward
  digitalWrite(motor_A, HIGH);  
  digitalWrite(motor_B, HIGH);

  // set to full speed
  analogWrite(motor_A_speed, 255);
  analogWrite(motor_B_speed, 255);

  // wait
  delay(wait_in_milliseconds);

  // stop
  analogWrite(motor_A_speed, 0);
  analogWrite(motor_B_speed, 0);
}

void turnLeft(){
  // set the direction to forward
  digitalWrite(motor_A, LOW);  
  digitalWrite(motor_B, LOW);

  // set to full speed
  analogWrite(motor_A_speed, 255);
  analogWrite(motor_B_speed, 255);

  // wait
  delay(wait_in_milliseconds);

  // stop
  analogWrite(motor_A_speed, 0);
  analogWrite(motor_B_speed, 0);
}

/////////////////////////////////////

int readLineSensor() {
  return analogRead(lineSensorPin);
}

void setup() {
  // set the Serial to 9600 baud and open it for comms
  Serial.begin(9600);

  // set the Arduino pin to OUTPUT mode
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(buzzer, OUTPUT);
}

void loop() {
  // Read the value from the line sensor (connected to the line sensor pin)
  lineNumber = readLineSensor();

  // move forward while the line sensor is white
  while(lineNumber > light_threshold)
  {
       Serial.println("move forward"); 
    // move forward
    forward();
     Serial.println(lineNumber); 
    lineNumber = readLineSensor();
  }
   Serial.println(lineNumber); 
  if(lineNumber < light_threshold){
    turnLeft(); 
  } 
  else {
     Serial.println(lineNumber); 
    backward();
    backward();
    turnLeft();
  }
  Serial.println(lineNumber); 
}
```