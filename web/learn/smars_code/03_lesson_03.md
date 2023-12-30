---
layout: lesson
title: Lesson 03 - Ultrasonic distance sensor
author: Kevin McAleer
type: page
cover: /learn/smars_code/assets/program_smars.png
date: 2023-09-10
previous: 02_lesson_02.html
next: 04_lesson_04.html
description: Learn how to detect distance using an ultrasonic sensor
percent: 42
duration: 8
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

{% include youtubeplayer.html id="ZinPFHsna2c" %}

---

## What you'll learn

In this lesson you will learn how to detect distances using a HC-SR04 ultrasonic distance sensor. In the second part of the tutorial we will use this sensing code, and code from [lesson 02](02_lessons_02) to avoid obstacles.

---

## Things's you'll needs

Before you begin, you'll need to make sure you have a couple of things before you start this lesson:

* A [SMARS robot](/learn/smars/)
* A USB Cable to connect to your computer
* A Computer that can run the [Arduino IDE](https://create.arduino.cc/editor)
* A HC-SR04 [ultrasonic sensor](/resources/glossary#ultrasonic)

---

## Preparation

1. Connect the SMARS to the USB cable, and the USB Cable to your computer
1. Launch the [Arduino IDE](https://create.arduino.cc/editor) on your computer
1. Create a new Sketch, by clicking the `New Sketch` button

---

## Lets code

You can download the code below from [GitHub](https://www.github.com/kevinmcaleer/lesson_03_sensors), but its better to type it in yourself line by line, as you will get to understand what each line means.

## The Range Finder program

**Type the following lines:**

``` c
// Lesson 03 Sensors
// www.smarsfan.com/play/lessons/lesson_03_sensors
#define echoPin 8 
#define trigPin 7 

long duration;
int distance;

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor HC-SR07");
  Serial.println("with Arduino UNO R3");
}

void loop() {
  // put your main code here, to run repeatedly:

  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // calculate the distance
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
```

---

## What the code means

The next section explains what each piece of code means.

After the first couple of comments is our first real line of code:

``` c
#define echoPin 8
#define trigPin 7  
```

These two lines define the constants `echoPin` and `trigPin` which correspond to the Echo and Trigger pins on the [HC-SR07](/resources/glossary#ultrasonic) range finder.

The Range finder has an ultrasonic speaker and an ultrasonic microphone. The Trigger is the speaker or output, and the echo is the microphone or input. We can output to the Triger by making the pin `HIGH` and stopping it sending out a signal by sending a `LOW` to the pin.

We can then read in the time taken for the echo pin to receive back the sound of the trigger pin by setting the pin `HIGH` on the echo pin and then reading in the value using the `pulseIn` function. This will return the time taken for the ping to bounce of any objects within range.

Next is:

``` c
long duration;
int distance;
```

These two statements create new variables that we will use and reuse; the duration is of type `long` which can store values from -2,147,483,648 to 2,147,483,647. The `distance` variable is of type `int` or integer which can store numbers in a range of -32,768 to 32,767.

In the `void setup()` function we have:

``` c
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
```

These statements tell the Arduino to set the `trigPin` to type `OUTPUT`, and `echoPin` to type `INPUT`.

The next 3 lines setup the serial and write out a friendly message to the serial port; we can view this using the serial monitor.

We now get to the main section of code:

``` c
void loop() {
  // put your main code here, to run repeatedly:

  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // calculate the distance
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
```

---

The important lines here are the the `digitalWrite(trigPin, LOW)` - which resets the range finder, and the `delayMicroseconds(2)` which allows the reset to occur.

Next the trigger pin is set high for 10 seconds using the three lines: `digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);`

We then read in the time take for the ping signal to bounce of any objects using the `duration = pulseIn(echoPin, HIGH)` statement. The distance is then worked out by multiplying this distance by 0.034 and dividing that by 2 - `distance = duration * 0.034 / 2;`. The speed of sound is 340 m/s or 0.034 cm/µ and we have to divide it by two as the sound wave has to travel to the object, and then back again.

Finally we print out the distance to the serial monitor using the last three lines.

---

## Verify & upload the code

Once you have typed in the code, click the Verify button (the Tick at the top of the screen). This will check that you've not made any typing mistakes.  

If it says 'Success: Done verifying' then you have code that is ready to be uploaded.

If you get an error message, read the message and look at the area of code it is having a problem with and compare it to the code at the top of this page.

Click the upload button to upload the code to the SMARS robot.

---

## Roving Robot

We can now combine the code from [lesson 02](02_lesson_02) with the range finder code above to form an obstacle avoidance program.

The code essensially works like this:

### Loop

* ping the ultrasonic sensor & measure the distance to any objects
* if its all clear then move forward
* if there is an object less than 10 cmm away then go backwards then turn right

Here is the code for the avoid program:

``` c
// Lesson 03 Sensors - Avoid Obstacles
// www.smarsfan.com/play/lessons/lesson_03_sensors

// set Motor A to Arduino Pins
int motor_A = 12;
int motor_B = 13;

// set the Motor Speed using the Arduino Pins
int motor_A_speed = 10;
int motor_B_speed = 11;

// set the time between motor on and motor off
int wait_in_milliseconds = 1000;

// set the Rangefinder pins
#define echoPin 8 
#define trigPin 7  

// set the variables for ping duration and measured distance
long duration;
int distance;

// this code runs once at the start
void setup() {

  // setup the Pin modes for the range finder
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // this sets the speed of communication between the computer and Arduino,
  // used when uploading your code
  Serial.begin(9600);

  Serial.println("SMARS OS v1.0");
  // set the Arduino pin to OUTPUT mode
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
}

// Sends out a ping and measures the distance, and returns it
int ping(){

  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // calculate the distance
  distance = duration * 0.034 / 2;
  return distance;
}

// move forward
void forward() {

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

// move backward
void backward() {

  // set the direction to backward
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

// turn left
void turnLeft() {

  // set the direction to backward
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

// turn right
void turnRight() {

  // set the direction to backward
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

// the main program loop
void loop(){
  while (true) {

   int dist = ping();
   if (dist < 5) {
     turnLeft();
   }
   else {
     forward();
     // wait 
    delay(wait_in_milliseconds);
   }
    
  }
}
```

---

## Congratulations

You now have a real robot, that can take inputs from sensors in the real work and then act upon those readings.

---

## Extra credit

What other types of avoidance steps can you take other than go backwards and turn right? These are called algorithms, and there are many types of obstacle avoidance algorithms to choose from.

---
