---
layout: lesson
title: Arduino Code
author: Kevin McAleer
type: page
previous: smars12.html
next: summary.html
description: Control your SMARS with Bluetooth
percent: 84
duration: 4
navigation:
- name: SMARS
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Print Parts
    content:
    - name: Print the Parts
      link: smars01.html
  - section: Assemble Robot
    content:
    - name: Solder wires to motors
      link: smars02.html
    - name: Fit the Motors
      link: smars03.html
    - name: Fit the Battery
      link: smars04.html
    - name: Fit Motors Holders
      link: smars05.html
    - name: Attach Wheels
      link: smars06.html
    - name: Test Motors
      link: smars07.html
  - section: Wiring up the Robot
    content:
    - name: Add the Arduino
      link: smars08.html
    - name: Add Motor Shield
      link: smars09.html
    - name: Insert Wires
      link: smars10.html
    - name: Attach Tracks
      link: smars11.html
  - section: Code
    content:
    - name: Load the code
      link: smars12.html
    - name: Arduino Code
      link: smars13.html
  - section: Summary
    content:
    - name: Summary and Review
      link: summary.html
---


Below is a sample program to control your SMARS with an additional Bluetooth module.

---

```c++
// SMARS Bluetooth remote control
// Kevin McAleer
// April 2019
// May 2019 - added buzzer feature
// Requires a Fundomoto sheild


//int ch_A_Brake = 9;
//int ch_B_Brake = 8;
int ch_A_Direction = 12;
int ch_B_Direction = 13;
int ch_A_speed = 10;
int ch_B_speed = 11;
char state = 0;
int delaylength = 1000;
int buzzerPin = 4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600, SERIAL_8N1);
  Serial.println("SMARSFan OS 1.1");
  Serial.println("---------------");

  // establish motor direction toggle pins
  pinMode(ch_A_Direction, OUTPUT);
  pinMode(ch_B_Direction, OUTPUT);

  // establish motor brake pins
  //  pinMode(ch_A_Brake, OUTPUT);
  //  pinMode(ch_B_Brake, OUTPUT);
}

void buzz(){
  digitalWrite(buzzerPin, HIGH);
  delay(delaylength / 2);
  digitalWrite(buzzerPin, LOW);
}

void forward() {
  // Move Forward
  digitalWrite(ch_A_Direction, LOW); // set direction to forward
  digitalWrite(ch_B_Direction, HIGH); // set direction to forward

  analogWrite(ch_A_speed, 255); // full speed ahead
  analogWrite(ch_B_speed, 255); // full speed ahead

  delay(delaylength);

  analogWrite(ch_A_speed, 0); // stop
  analogWrite(ch_B_speed, 0); // stop


  //  Serial.flush();
}

void backward() {
  // Move Backward

  digitalWrite(ch_A_Direction, HIGH); // set direction to backward
  digitalWrite(ch_B_Direction, LOW); // set direction to backward

  analogWrite(ch_A_speed, 255); // full speed ahead
  analogWrite(ch_B_speed, 255); // full speed ahead

  delay(delaylength);

  analogWrite(ch_A_speed, 0); // stop
  analogWrite(ch_B_speed, 0); // stop

  //  Serial.flush();
}

void left() {
  // Move left

  digitalWrite(ch_A_Direction, HIGH); // set direction to left
  digitalWrite(ch_B_Direction, HIGH); // set direction to left

  analogWrite(ch_A_speed, 255); // full speed ahead
  analogWrite(ch_B_speed, 255); // full speed ahead

  delay(delaylength);

  analogWrite(ch_A_speed, 0); // stop
  analogWrite(ch_B_speed, 0); // stop

  //  Serial.flush();
}

void right() {
  // Move right

  digitalWrite(ch_A_Direction, LOW); // set direction to right
  digitalWrite(ch_B_Direction, LOW); // set direction to right

  analogWrite(ch_A_speed, 255); // full speed ahead
  analogWrite(ch_B_speed, 255); // full speed ahead

  delay(delaylength);

  analogWrite(ch_A_speed, 0); // stop
  analogWrite(ch_B_speed, 0); // stop

  //  Serial.flush();
}

void fullstop() {
  // stop!

  digitalWrite(ch_A_Direction, HIGH); // set direction to right
  digitalWrite(ch_B_Direction, HIGH); // set direction to right

  analogWrite(ch_A_speed, 0); // full speed ahead
  analogWrite(ch_B_speed, 0); // full speed ahead

  delay(delaylength);

  //  Serial.flush();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0 ) {
    state = Serial.read();
    Serial.println(state);

    if (state == 'u') {
      Serial.println("MOTORS: UP");
      forward(); // move forward
      state = 0;
    }

    else if (state == 'd') {
      Serial.println("MOTORS: DOWN");
      backward();
      state = 0;
    }

    else if (state == 'l') {
      Serial.println("MOTORS: LEFT");
      left();
      state = 0;
    }

    else if (state == 'r') {
      Serial.println("MOTORS: RIGHT");
      right();
      state = 0;
    }

    else if (state == "s") {
      Serial.println("MOTORS: STOP");
      fullstop();
      state = 0;
    else if (state == 'b') {
      Serial.println("Buzzer: Sound")
      buzz()
      state = 0;
    }
    }
  }
}
```