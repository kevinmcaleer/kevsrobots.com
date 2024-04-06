---
title: Controlling Servos with Python
description: >-
    Learn how to write Python scripts to control the movements of servos on your robot arm using the Raspberry Pi and PCA9685.
layout: lesson
type: page
cover: assets/8.png
---

![Controlling Servos with Python]({{ page.cover }}){:class="cover"}

## Introduction to Servo Control

With your robot arm assembled, it's time to bring it to life through programming. This lesson focuses on using Python to control the servos connected to the PCA9685 servo driver board, enabling precise movements of your robot arm.

---

## The Basics of Servo Control

Servos move to a position based on the pulse width of a PWM (Pulse Width Modulation) signal. The position is determined by the duration of the pulse, typically between 1ms (minimum) and 2ms (maximum), within a 20ms cycle for standard servos.

---

## Setup and Initialization

Ensure the Adafruit_PCA9685 library is installed and your PCA9685 is wired to the Raspberry Pi as described in previous lessons.

```python
from Adafruit_PCA9685 import PCA9685
import time

# Initialize the PCA9685 using the default address (0x40).
pwm = PCA9685()

# Set the PWM frequency to 60Hz, suitable for servos.
pwm.set_pwm_freq(60)
```

---

## Controlling a Single Servo

Let's start with a simple script to move a servo back and forth.

```python
def set_servo_position(channel, position):
    pulse_length = 4096    # The PCA9685 has 4096 steps
    pulse = int(position * pulse_length / 20)  # Convert position to pulse
    pwm.set_pwm(channel, 0, pulse)

# Example usage
while True:
    set_servo_position(0, 0.5)  # Move servo on channel 0 to middle position
    time.sleep(1)
    set_servo_position(0, 2.5)  # Move servo to maximum position
    time.sleep(1)
```

This example moves a servo connected to channel 0 from its middle position to its maximum position and back. Adjust the `position` parameter to control the servo's angle.

---

## Coordinating Multiple Servos

To control multiple servos, you simply call `set_servo_position` for each servo, specifying the channel and desired position.

```python
# Move multiple servos
set_servo_position(0, 1)  # Base
set_servo_position(1, 1.5)  # Shoulder
set_servo_position(2, 2)  # Elbow
# Add more as needed
```

By coordinating the movements of multiple servos, you can achieve complex motions for your robot arm.

---

## Tips for Smooth Movements

- **Gradual Steps:** Move servos in small increments to achieve smoother motion.
- **Delays:** Include short delays between movements to allow servos time to reach their positions.

---

## Conclusion

You now have the basic knowledge to program your robot arm's movements using Python. Experiment with different positions and sequences to create fluid, coordinated motions for your arm.

---

## Lesson Assignment

Write a Python script that performs a simple task with your robot arm, such as picking up an object. Experiment with the timing and sequence of servo movements to achieve smooth action.

---
