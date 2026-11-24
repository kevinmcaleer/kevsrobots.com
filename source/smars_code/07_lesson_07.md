---
title: Lesson 07 - Line Following
description: Learn how to use the line follow module
layout: lesson
type: page
cover: /learn/smars_code/assets/banners/07_lesson_07.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to read a TCRT5000 line sensor with `analogRead()` and tune a threshold so SMARS follows the edge of a black line.

## Watch the livestream

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

Here is the first track, a simple loop, to give you an idea of what to print:

![Track 01, a simple loop](assets/smars_line_follow_track_01_loop.jpg){:class="img-fluid w-50" loading="lazy"}

---

## What you'll need

* A [SMARS robot](/learn/smars/)
* A USB Cable to connect to your computer
* A Computer that can run the [Arduino IDE](https://www.arduino.cc/en/software)
* A [line follow module](/resources/glossary#tcrt5000) - TCRT5000 or equivalent

* A black line on white paper (print a track above, or use black tape on white card)

---

## Wiring

| Line sensor pin | Arduino pin |
| --- | --- |
| VCC | 5V |
| GND | GND |
| AO (analog output) | A4 |
{:class="table table-striped"}

The sketch reads the sensor on `A4`, the same pin lesson 4 used for the MPU-6050's SDA wire. Unplug the MPU-6050 before you wire the line sensor. If your module's pins are labelled differently (some only have a digital `DO` output), use the analogue output.

---

## Preparation

1. Connect the SMARS to the USB cable, and the USB Cable to your computer
1. Launch the [Arduino IDE](https://www.arduino.cc/en/software) on your computer
1. Create a new Sketch, by clicking the `New Sketch` button

---

## Let's code

You can download the code below from [GitHub](https://www.github.com/kevinmcaleer/lesson_07_linefollowing), but it's better to type it in yourself line by line, as you will get to understand what each line means.

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
int lineSensorPin = A4; // line sensor analog output


// set Motor A to Arduino Pins
int motor_A = 12; // official Arduino Motor Shield uses D12
int motor_B = 13; // official Arduino Motor Shield uses D13
int buzzer = 4;          // digital pin D4 (not the same as A4)

// set the Motor Speed using the Arduino Pins
int motor_A_speed = 10; // Fundumoto shield uses D10 (the official Arduino Motor Shield uses D3)
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
  // set the direction to backward
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
  // set both motors the same direction to spin on the spot
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
  // set both motors the same direction to spin on the spot
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

---

## What the code means

- The TCRT5000 shines infrared light at the floor and measures how much bounces back. White paper reflects a lot (a high reading) and a black line reflects very little (a low reading).
- `readLineSensor()` uses `analogRead()` to get a number from 0 to 1023.
- While the reading is above `light_threshold` (650) the robot is on white paper, so it keeps driving forward in short 10 millisecond bursts.
- When the reading drops, the sensor has reached the line, so the robot turns left until it finds white again. This keeps it running along the edge of the line.

Every sensor and every floor is different, so watch the numbers in the Serial Monitor and adjust `light_threshold` to sit halfway between your white and black readings.

## Try it Yourself

1. Print one of the track PDFs and see how fast SMARS can get round it.
2. Change `wait_in_milliseconds` from `10` to `20`. Is it faster or does it lose the line?
3. Challenge: use the `dark_threshold` variable so the robot only turns when the reading is really dark.

> **Teachers**: Line following depends on light, so results vary between classrooms: bright windows and shiny desks change the readings. Run the Serial Monitor first, read the numbers for white and for black, and pick the threshold from those, rather than trusting 650. Stand a wall of card around the track so the robot cannot wander off the table, and print the tracks on matt paper because glossy paper reflects too much light.

## Common Issues

- **Problem**: SMARS drives straight off the line, or spins on the spot and never moves forwards.
- **Solution**: Open the Serial Monitor at `9600` baud, hold the sensor over white paper and then over the line, and set `light_threshold` roughly halfway between the two readings.
- **Why**: Light changes the numbers the sensor gives, so a threshold that worked on one table may be wrong on another.

- **Problem**: The reading is always about the same, whatever is under the sensor.
- **Solution**: Check the sensor sits a few millimetres above the paper, that its signal wire is on `A4`, and that you are using the analogue output pin.
- **Why**: The TCRT5000 only works at a short range, and a digital output can only say "light" or "dark".

- **Problem**: SMARS keeps losing the line on tight corners.
- **Solution**: Try a shorter `wait_in_milliseconds`, or a wider, more gentle track.
- **Why**: Each forward burst is a fixed time, so it can overshoot a bend before the sensor can react.

- **Problem**: Nothing appears in the Serial Monitor.
- **Solution**: Make sure the baud rate box says `9600`, matching `Serial.begin(9600);`.
- **Why**: The sketch and the Serial Monitor must agree on the speed.

---

## Check your understanding

{% quiz %}
? Why does the line sensor give a low reading over a black line?
- [ ] Black lines are warmer
- [x] Black reflects much less infrared light back to the sensor
- [ ] The sensor is switched off over black
> White surfaces reflect most of the light, black surfaces absorb it.

? Your SMARS keeps losing the line. What should you try first?
- [x] Check the readings in the Serial Monitor and adjust `light_threshold`
- [ ] Remove the tracks
- [ ] Change the baud rate
> Lighting and paper change the readings, so the threshold needs tuning for your track.

? In this sketch, what does the robot do while the sensor reading is above `light_threshold`?
- [ ] Turns left
- [x] Drives forward in short bursts
- [ ] Goes backwards
> A high reading means white paper, so the `while` loop keeps calling `forward()`.
{% endquiz %}

## What's next

Congratulations on finishing the course! Try these next:

- [Build a SMARS Quad](/learn/smars_quad/): a four-legged walking SMARS
- [Design a SMARS Robot in Fusion 360](/learn/smars_fusion360/)
- [MicroPython Robotics Projects](/learn/micropython_robotics/): program robots in Python instead
