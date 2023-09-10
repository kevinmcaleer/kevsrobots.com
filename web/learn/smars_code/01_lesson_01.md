---
layout: lesson
title: Lesson 01 - Movement
author: Kevin McAleer
type: page
cover: /learn/smars_code/assets/program_smars.png
next: 02_lesson_02.html
description: Learn how to move your robot
percent: 14
duration: 9
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

{% include youtubeplayer.html id="2mmB2TD71g4" %}

---

## What you'll learn

In this lesson you will learn how to move your robot forward by writing code and uploading it to your SMARS robot.

---

## Things's you'll needs

Before you begin, you'll need to make sure you have a couple of things before you start this lesson:

* A [SMARS robot](/learn/smars/)
* A USB Cable to connect to your computer
* A Computer that can run the [Arduino IDE](https://create.arduino.cc/editor)

---

## Preparation

1. Connect the SMARS to the USB cable, and the USB Cable to your computer
1. Launch the [Arduino IDE](https://create.arduino.cc/editor) on your computer
1. Create a new Sketch, by clicking the `New Sketch` button

---

## Lets code

You can download the code below from [GitHub](https://www.github.com/kevinmcaleer/lesson_01_movement), but its better to type it in yourself line by line, as you will get to understand what each line means.

**Type the following lines:**

```arduino
// Lesson 01 Movement
// www.smarsfan.com/play/lessons/lesson_01_movement

// set Motor A to Arduino Pins
int motor_A = 12; // official Arduino Motor Shield uses D12
int motor_B = 13; // official Arduino Motor Shield uses D13

// set the Motor Speed using the Arduino Pins
int motor_A_speed = 10; // official Arduino Motor Shield uses D3
int motor_B_speed = 11; // official Arduino Motor Shield uses D11

// set the time between motor on and motor off
int wait_in_milliseconds = 1000;

// this code runs once at the start
void setup() {

  // set the Arduino pin to OUTPUT mode
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
}

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

// the main program loop
void loop(){

  // move forward
  forward();

  // wait 2 seconds
  delay(2000);
}
```

## What the code means

The next section explains what each piece of code means.

---

### Comments

``` arduino
// Lesson 01 Movement
// www.smarsfan.com/play/lessons/lesson_01_movement
```

Comments are lines in code put there to help the programmer understand what the code next to it does; the computer ignores them.

Putting two `//` forward slashes will tell the computer to ignore the rest of the line.

---

### Variables

``` arduino
// set Motor A to Arduino Pins
int motor_A = 12;
int motor_B = 13;

// set the Motor Speed using the Arduino Pins
int motor_A_speed = 10;
int motor_B_speed = 11;

// set the time between motor on and motor off
int wait_in_milliseconds = 1000;
```

Variables are special words that hold a value, such as a number, some text or other value such as `true` or `false`.

`int` means integer, which is a whole number between -32,768 and 32,767.

The Arduino has 32 pins, 20 of which can be used for read from and writing to. We tell the motors to turn on and off by writing a `1` or `0`, also known as `HIGH` or `LOW` values.

`int motor_A = 12;` means tell the Arduino that there is a motor on pin 12, and we will refer to it as motor_A in our program.

`int motor_A_speed = 10;` means tell the Arduino that we will control the speed of Motor A using pin 10.

`int wait_in_milliseconds = 1000;` means set the integer `wait_in_milliseconds` to the value 1000 - the delay command used later on uses a value in milliseconds, (or thousands of a second). This means we can very accurately turn things on and off. A value of 500 would represent half a second. 1000 represents 1 second (a thousand milliseconds).

---

### Setup

``` arduino
// this code runs once at the start
void setup() {

  // set the Arduino pin to OUTPUT mode
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
}
```

Arduino's have two main areas of code:

1. the **setup** section that is used only once at startup
1. and the **loop** section, that is used (executed) over and over until the Arduino is powered off, or the reset button is pressed.

`void setup()`
'void' seems like a strange word, often meaning empty. In this case  it means that there is no value returned by the section `setup` - it is void of value. Sections of code (also call **Functions** can return values or return nothing when they complete running).

`setup()` is the name of the Function.

`{ }` - each function has a section of code that is contained within curly braces. This tells the Arduino to expect code between the two curly braces, and to refer to that section of code by its Function name.

`pinMode(motor_A, OUTPUT);` means set the mode of the pin that `motor_A` is using (pin 12) to `OUTPUT`. Digital pins can either be `INPUT` or `OUTPUT`, which means the Arduino is either reading from or writing to the PIN. Pins can be either `Digital` or `Analog`:

* `Digital` pins use values that are either 1 or 0 (HIGH or LOW). If you were to measure the voltage of a pin that is 1 (or HIGH) it would be 5 volts, where as a pin that is 0 (or LOW) would be 0 volts.
* `Analog` pins use values that are a range of values, from 0 to 1023. If you were to measure the voltage of a pin that is set to an analog value it would be anywhere between 0 volts and 5 volts.

---

### Forward()

``` arduino
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
```

The next section of code is a Function named `forward()`. As you might expect from the name, this function moves the robot forward.

To move the robot forward we need to:

* set the pin to low for Motor_A
* set the pin to high for Motor_B as this is facing in the opposite direction to Motor_A
* set the speed of each motor, using a value between 0 and 255, with 0 being no speed and 255 being the fastest speed.
* wait an amount of time (in milliseconds)
* set the speed of the motors to 0, making them stop

`digitalWrite(motor_A, LOW);` means set the pin for motor_A to low

`digitalWrite(motor_B, HIGH);` means set the pin for motor_B to high, as its facing the opposite direction

`analogWrite(motor_A_speed, 255);` means set the speed of Motor_A to 255 which is the fastest speed.

`analogWrite(motor_B_speed, 255);` means set the speed of Motor_B to 255 which is the fastest speed.

`delay(wait_in_milliseconds);` means wait the amount of time stored in the variable `wait_in_milliseconds` (which we set earlier as 1000 milliseconds, or 1 second).

We need to wait an amount of time othewise the Arduino will continue on to the next instruction which tells the motors to stop. Without the delay the motors wouldn't have any time to turn and the Robot would not move forward.

Changing the amount of time stored in `wait_in_milliseconds` will make the amount of distance travelled either longer or shorter.

`analogWrite(motor_A_speed, 0);` means stop Motor_A from turning by making its speed zero.

`analogWrite(motor_B_speed, 0);` means stop Motor_B from turning by making its speed zero.

---

### The main loop()

``` arduino
// the main program loop
void loop(){

  // move forward
  forward();

  // wait 2 seconds
  delay(2000);
}
```

Every computer program has a main loop which is used to control all other parts of the program.

Arduino's always have a `loop()` function that is executed after the setup() function has completed running. Once the loop() function has completed executing all its instructions, it will return to the beginning and run them all again (which is why its called loop!).

`void loop() {` means this is the function called `loop`, and it doesn't return any values (which is why it starts with `void`).

Again the curly braces `{ }` contain all the code for this function.

`forward();` means run the function called `forward()` that we created earlier on in the code. This function makes the robot move forward for 1 second.

`delay(2000);` means wait for 2 seconds.

After waiting 2 seconds the loop completes, then start back at the very top and executes the code again, calling the `forward();` function again.

---

## Upload

### Verify the code

Once you have typed in the code, click the Verify button (the Tick at the top of the screen). This will check that you've not made any typing mistakes.  

If it says 'Success: Done verifying' then you have code that is ready to be uploaded.

If you get an error message, read the message and look at the area of code it is having a problem with and compare it to the code at the top of this page.

---

### Set the Connection

Select the Arduino connection using the dropdown box next to the upload arrow button. You will see a list of boards to choose from. SMARS robots use the `Arduino/Generic` UNO board type. Select the Port your SMARS is connected to (you should see this in the list).

Click OK to continue.

---

### Upload the code

You are now ready to upload this code to your SMARS robot. Press the upload button (the arrow next to the verify button), to upload the program to your robot.

---

### Roving Robot

Once the code have been uploaded, the robot will start to move forward for 1 second, stop and wait 2 seconds, then move forward again.

---

## Congratulations

If you've made it this far you've successfully created your first SMARS program and have successfully moved the robot forward.

---

### Extra credit

How would you change this program to make the robot go backwards?

Take a minute to appreciate what you've done, and then move on to the next lesson: [Lesson 2 - turning](02_lesson_02).

---
