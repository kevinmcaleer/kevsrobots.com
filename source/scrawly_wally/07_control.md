---
title: Writing Control Code
description: Write Python code to control the wall drawing robot.
layout: lesson
type: page
cover: assets/control_code.png
date_updated: 2024-08-02
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
