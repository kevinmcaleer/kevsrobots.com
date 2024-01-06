---
layout: how_it_works
title: PID Controlers
short_title: How it works - PID Controllers
short_description: How it works - PID Controllers
description: How it works - PID Controllers
date: 2023-03-13
author: Kevin McAleer & ChatGPT
excerpt: 
cover: /assets/img/how_it_works/pid.jpg
tags:
 - Robot
 - Tips
 - PID Controllers
 - Electronics
 - Control systems
 - PID
 - How it works
---

`PID controllers`, short for Proportional Integral Derivative controllers, are a type of control system that are widely used in robotics, automation, and process control.

In this article, we will discuss how PID controllers work, what they are, and where you would use one.

---

## What are PID Controllers?

PID controllers are a type of feedback control system that uses feedback to control a process or system. They are widely used in industrial applications such as manufacturing, robotics, and process control.

PID controllers work by continuously measuring the difference between the desired setpoint and the actual process variable, then using that information to adjust the control output. The three components of the PID controller, `proportional`, `integral`, and `derivative`, are used to calculate the control output.

---

## How Do PID Controllers Work?

PID controllers use three components to calculate the control output: proportional, integral, and derivative. Each of these components works together to adjust the control output to reach the desired setpoint.

`Proportional Control`: Proportional control is based on the error between the setpoint and the actual process variable. The proportional gain is multiplied by this error to produce the control output. This component provides a linear response, which means that the control output is directly proportional to the error.

`Integral Control`: Integral control is based on the integral of the error between the setpoint and the actual process variable. This component provides a way to eliminate steady-state error in the system. The integral gain is multiplied by the sum of the errors over time to produce the control output.

`Derivative Control`: Derivative control is based on the rate of change of the error between the setpoint and the actual process variable. This component provides a way to respond to changes in the system quickly. The derivative gain is multiplied by the rate of change of the error to produce the control output.

---

## PID Python Code Example

Here is some example Python code to illustrate how this would work in a project:

```python
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def update(self, error, dt):
        derivative = (error - self.last_error) / dt
        self.integral += error * dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

# Example program
pid = PID(Kp=1.0, Ki=0.1, Kd=0.5)
setpoint = 10.0
dt = 0.1
time = 0.0
current_value = 0.0

while time < 10.0:
    error = setpoint - current_value
    control_signal = pid.update(error, dt)
    current_value += control_signal * dt
    print(f"time: {time:.2f}, setpoint: {setpoint:.2f}, current value: {current_value:.2f}, error: {error:.2f}, control signal: {control_signal:.2f}")
    time += dt
```

In this example, the PID controller is used to control a process that has a setpoint of `10.0`. The controller is updated every `dt` seconds, and the current value of the process is adjusted based on the output of the controller. The `pid.update` method calculates the control signal based on the error between the setpoint and the current value of the process, and returns the control signal. The `current_value` is then updated by multiplying the control signal by `dt`. The loop continues until time reaches `10.0`.

---

## Where Would You Use a PID Controller?

PID controllers are used in a wide range of applications, including robotics, manufacturing, and process control. They are used in applications where it is important to maintain a precise setpoint, such as temperature control in a chemical plant, speed control in a motor, or position control in a robot arm.

PID controllers are also commonly used in the control of unmanned aerial vehicles (UAVs). The controllers are used to stabilize the UAV and maintain a desired altitude, orientation, and position.

---

## Conclusion

PID controllers are an essential part of modern control systems. They use feedback to adjust the control output and maintain a precise setpoint. The proportional, integral, and derivative components work together to calculate the control output. PID controllers are widely used in industrial applications where it is important to maintain a precise setpoint. Understanding how PID controllers work can help engineers and technicians design and implement effective control systems for a wide range of applications.

---
