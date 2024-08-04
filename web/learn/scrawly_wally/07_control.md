---
layout: lesson
title: Writing Control Code
author: Kevin McAleer
type: page
cover: assets/control_code.png
date: 2024-08-02
previous: 06_software.html
next: 08_positioning.html
description: Write Python code to control the wall drawing robot.
percent: 63
duration: 2
date_updated: 2024-08-02
navigation:
- name: Wall Drawing Robot Tutorial
- content:
  - section: Introduction
    content:
    - name: Introduction to the Wall Drawing Robot
      link: 01_intro.html
  - section: Build the Robot
    content:
    - name: Components Needed
      link: 02_components.html
    - name: Assembling the Robot
      link: 03_assembly.html
    - name: Wiring the Components
      link: 04_wiring.html
  - section: Programming
    content:
    - name: Setting Up the Raspberry Pi
      link: 05_setup_pi.html
    - name: Installing the Software
      link: 06_software.html
    - name: Writing Control Code
      link: 07_control.html
    - name: Understanding Positioning with Trigonometry
      link: 08_positioning.html
    - name: Creating Your First Drawing
      link: 09_draw.html
    - name: Vectorizing Images for Drawing
      link: 10_vectorize.html
  - section: Summary
    content:
    - name: Summary and Next Steps
      link: 11_summary.html
---


## Writing Control Code

Create a Python script to control the motors and the servo:

```python
import RPi.GPIO as GPIO
import time

# GPIO pin setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Stepper motor pins
motor_pins = [17, 18, 27, 22]

# Set all pins as output
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# Servo pin
servo_pin = 23
GPIO.setup(servo_pin, GPIO.OUT)

# Set PWM frequency to 50Hz
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_servo_angle(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

# Step sequence for the stepper motor
step_sequence = [
    [1, 0, 0, 1],
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1]
]

def move_motor(steps, direction=1):
    for _ in range(steps):
        for step in step_sequence[::direction]:
            for pin in range(len(motor_pins)):
                GPIO.output(motor_pins[pin], step[pin])
            time.sleep(0.001)

try:
    while True:
        move_motor(512, direction=1)  # Move motor forward
        set_servo_angle(90)           # Lower the pen
        time.sleep(1)
        set_servo_angle(0)            # Raise the pen
        move_motor(512, direction=-1) # Move motor backward
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
    pwm.stop()
```

---
