---
layout: lesson
title: Robot Project - Simple Motor Control
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-11-20
previous: 09_iot_project.html
next: 11_next_steps.html
description: Build a basic robot control system with motors, sensors, and autonomous
  behavior using MicroPython
percent: 88
duration: 10
navigation:
- name: Arduino to MicroPython - A Quick Start Guide
- content:
  - section: Getting Started
    content:
    - name: Introduction to Arduino to MicroPython
      link: 00_intro.html
    - name: Why MicroPython?
      link: 01_why_micropython.html
  - section: Language Fundamentals
    content:
    - name: Syntax Basics - No More Semicolons!
      link: 02_syntax_basics.html
    - name: Control Flow - If, Loops, and No Braces!
      link: 03_control_flow.html
    - name: Functions - Goodbye setup and loop!
      link: 04_functions.html
    - name: Pin Control - Goodbye pinMode, Hello Pin Class!
      link: 05_pin_control.html
  - section: Hardware Basics
    content:
    - name: Analog Reading and PWM - ADC and PWM Classes
      link: 06_analog_pwm.html
    - name: Timing and Delays - Seconds vs Milliseconds!
      link: 07_timing.html
    - name: Serial Communication and REPL - No More Serial.begin!
      link: 08_serial_basics.html
  - section: Practical Projects
    content:
    - name: IoT Project - WiFi Temperature Monitor
      link: 09_iot_project.html
    - name: Robot Project - Simple Motor Control
      link: 10_robot_project.html
    - name: Next Steps and Quick Reference
      link: 11_next_steps.html
---


## Building a Simple Robot

Let's build a robot control system that demonstrates practical MicroPython robotics. We'll create a robot that can move, avoid obstacles, and respond to sensors - all the fundamentals of robot programming.

**What you'll build:**
- Motor control with direction and speed
- Ultrasonic distance sensor for obstacle detection
- Autonomous obstacle avoidance behavior
- Manual control mode
- Status LEDs for feedback

**Hardware needed:**
- Arduino Nano ESP32, Pico W, or Pico 2W
- 2x DC motors
- Motor driver (L298N or similar)
- Ultrasonic sensor (HC-SR04)
- 2x LEDs (status indicators)
- Chassis, wheels, and battery pack

## Project Overview

The robot will have two modes:
1. **Manual mode** - Control via buttons or serial commands
2. **Autonomous mode** - Avoid obstacles automatically

Let's build this step by step.

## Step 1: Motor Control Class

First, create a clean motor control abstraction:

**Arduino C++ approach:**
```cpp
class Motor {
private:
    int forwardPin;
    int reversePin;
    int pwmPin;

public:
    Motor(int fwd, int rev, int pwm) {
        forwardPin = fwd;
        reversePin = rev;
        pwmPin = pwm;
        pinMode(forwardPin, OUTPUT);
        pinMode(reversePin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
    }

    void setSpeed(int speed) {
        if (speed > 0) {
            digitalWrite(forwardPin, HIGH);
            digitalWrite(reversePin, LOW);
            analogWrite(pwmPin, speed);
        } else if (speed < 0) {
            digitalWrite(forwardPin, LOW);
            digitalWrite(reversePin, HIGH);
            analogWrite(pwmPin, -speed);
        } else {
            digitalWrite(forwardPin, LOW);
            digitalWrite(reversePin, LOW);
        }
    }
};
```

**MicroPython:**
```python
from machine import Pin, PWM

class Motor:
    def __init__(self, forward_pin, reverse_pin, pwm_pin):
        self.forward = Pin(forward_pin, Pin.OUT)
        self.reverse = Pin(reverse_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)

    def set_speed(self, speed):
        """Set motor speed (-100 to 100)"""
        if speed > 0:
            self.forward.on()
            self.reverse.off()
            duty = int((abs(speed) / 100) * 65535)
            self.pwm.duty_u16(duty)
        elif speed < 0:
            self.forward.off()
            self.reverse.on()
            duty = int((abs(speed) / 100) * 65535)
            self.pwm.duty_u16(duty)
        else:
            self.forward.off()
            self.reverse.off()
            self.pwm.duty_u16(0)

    def stop(self):
        """Stop the motor"""
        self.set_speed(0)
```

Python's cleaner syntax makes this easier to read and maintain!

## Step 2: Robot Control Class

Now create a robot class that controls both motors:

**MicroPython:**
```python
class Robot:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def forward(self, speed=50):
        """Move forward at given speed"""
        self.left.set_speed(speed)
        self.right.set_speed(speed)

    def backward(self, speed=50):
        """Move backward at given speed"""
        self.left.set_speed(-speed)
        self.right.set_speed(-speed)

    def turn_left(self, speed=50):
        """Turn left (spin in place)"""
        self.left.set_speed(-speed)
        self.right.set_speed(speed)

    def turn_right(self, speed=50):
        """Turn right (spin in place)"""
        self.left.set_speed(speed)
        self.right.set_speed(-speed)

    def stop(self):
        """Stop both motors"""
        self.left.stop()
        self.right.stop()

    def arc_left(self, speed=50):
        """Arc left (one motor slower)"""
        self.left.set_speed(speed // 2)
        self.right.set_speed(speed)

    def arc_right(self, speed=50):
        """Arc right (one motor slower)"""
        self.left.set_speed(speed)
        self.right.set_speed(speed // 2)
```

Clean, readable, and easy to extend!

## Step 3: Ultrasonic Sensor

Add distance sensing for obstacle detection:

**Arduino C++:**
```cpp
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

void setup() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

float getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2;
    return distance;
}
```

**MicroPython:**
```python
from machine import Pin
import time

class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)

    def get_distance(self):
        """Get distance in centimeters"""
        # Send trigger pulse
        self.trigger.off()
        time.sleep_us(2)
        self.trigger.on()
        time.sleep_us(10)
        self.trigger.off()

        # Measure echo pulse duration
        timeout = 30000  # 30ms timeout
        start = time.ticks_us()

        # Wait for echo to go high
        while self.echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), start) > timeout:
                return None
            pass

        pulse_start = time.ticks_us()

        # Wait for echo to go low
        while self.echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), pulse_start) > timeout:
                return None
            pass

        pulse_end = time.ticks_us()

        # Calculate distance
        pulse_duration = time.ticks_diff(pulse_end, pulse_start)
        distance = (pulse_duration * 0.0343) / 2  # Speed of sound: 343 m/s

        return distance
```

## Step 4: Complete Robot Program

Now let's put it all together:

**robot_complete.py:**
```python
from machine import Pin, PWM
import time

# Motor class
class Motor:
    def __init__(self, forward_pin, reverse_pin, pwm_pin):
        self.forward = Pin(forward_pin, Pin.OUT)
        self.reverse = Pin(reverse_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)

    def set_speed(self, speed):
        """Set motor speed (-100 to 100)"""
        speed = max(-100, min(100, speed))  # Clamp to range

        if speed > 0:
            self.forward.on()
            self.reverse.off()
            duty = int((abs(speed) / 100) * 65535)
            self.pwm.duty_u16(duty)
        elif speed < 0:
            self.forward.off()
            self.reverse.on()
            duty = int((abs(speed) / 100) * 65535)
            self.pwm.duty_u16(duty)
        else:
            self.forward.off()
            self.reverse.off()
            self.pwm.duty_u16(0)

    def stop(self):
        self.set_speed(0)

# Robot class
class Robot:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def forward(self, speed=50):
        self.left.set_speed(speed)
        self.right.set_speed(speed)

    def backward(self, speed=50):
        self.left.set_speed(-speed)
        self.right.set_speed(-speed)

    def turn_left(self, speed=50):
        self.left.set_speed(-speed)
        self.right.set_speed(speed)

    def turn_right(self, speed=50):
        self.left.set_speed(speed)
        self.right.set_speed(-speed)

    def stop(self):
        self.left.stop()
        self.right.stop()

# Ultrasonic sensor class
class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)

    def get_distance(self):
        self.trigger.off()
        time.sleep_us(2)
        self.trigger.on()
        time.sleep_us(10)
        self.trigger.off()

        timeout = 30000
        start = time.ticks_us()

        while self.echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), start) > timeout:
                return None

        pulse_start = time.ticks_us()

        while self.echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), pulse_start) > timeout:
                return None

        pulse_end = time.ticks_us()
        pulse_duration = time.ticks_diff(pulse_end, pulse_start)
        distance = (pulse_duration * 0.0343) / 2

        return distance

# Configuration
LEFT_FORWARD = 10
LEFT_REVERSE = 11
LEFT_PWM = 12
RIGHT_FORWARD = 13
RIGHT_REVERSE = 14
RIGHT_PWM = 15
TRIG_PIN = 16
ECHO_PIN = 17
STATUS_LED = 25

# Initialize hardware
left_motor = Motor(LEFT_FORWARD, LEFT_REVERSE, LEFT_PWM)
right_motor = Motor(RIGHT_FORWARD, RIGHT_REVERSE, RIGHT_PWM)
robot = Robot(left_motor, right_motor)
sensor = UltrasonicSensor(TRIG_PIN, ECHO_PIN)
status_led = Pin(STATUS_LED, Pin.OUT)

# Robot behavior
def avoid_obstacles():
    """Autonomous obstacle avoidance behavior"""
    SAFE_DISTANCE = 20  # cm
    TURN_TIME = 0.5     # seconds

    while True:
        distance = sensor.get_distance()

        if distance is None:
            # Sensor error - stop and retry
            robot.stop()
            status_led.toggle()
            time.sleep(0.1)
            continue

        print(f"Distance: {distance:.1f} cm")

        if distance < SAFE_DISTANCE:
            # Obstacle detected!
            status_led.on()
            print("Obstacle detected! Avoiding...")

            # Stop
            robot.stop()
            time.sleep(0.3)

            # Back up
            robot.backward(40)
            time.sleep(0.5)

            # Turn (randomly choose left or right)
            import random
            if random.random() > 0.5:
                robot.turn_left(50)
            else:
                robot.turn_right(50)

            time.sleep(TURN_TIME)

        else:
            # Path is clear
            status_led.off()
            robot.forward(60)

        time.sleep(0.1)  # Small delay

def manual_control():
    """Manual control via buttons"""
    button_forward = Pin(5, Pin.IN, Pin.PULL_UP)
    button_back = Pin(6, Pin.IN, Pin.PULL_UP)
    button_left = Pin(7, Pin.IN, Pin.PULL_UP)
    button_right = Pin(8, Pin.IN, Pin.PULL_UP)

    while True:
        # Check buttons (active LOW)
        if button_forward.value() == 0:
            robot.forward(60)
            status_led.on()
        elif button_back.value() == 0:
            robot.backward(60)
            status_led.on()
        elif button_left.value() == 0:
            robot.turn_left(50)
            status_led.on()
        elif button_right.value() == 0:
            robot.turn_right(50)
            status_led.on()
        else:
            robot.stop()
            status_led.off()

        time.sleep(0.05)

# Main program
def main():
    print("=" * 50)
    print("Robot Control System")
    print("=" * 50)

    # Blink LED to show ready
    for i in range(3):
        status_led.on()
        time.sleep(0.2)
        status_led.off()
        time.sleep(0.2)

    print("Starting autonomous mode in 3 seconds...")
    time.sleep(3)

    try:
        avoid_obstacles()
        # Or: manual_control()
    except KeyboardInterrupt:
        print("\nStopping robot...")
        robot.stop()
        status_led.off()

if __name__ == '__main__':
    main()
```

## Step 5: Advanced Features

**Add line following:**
```python
class LineFollower:
    def __init__(self, left_sensor_pin, right_sensor_pin):
        self.left_sensor = Pin(left_sensor_pin, Pin.IN)
        self.right_sensor = Pin(right_sensor_pin, Pin.IN)

    def read_sensors(self):
        """Read line sensors (0 = on line, 1 = off line)"""
        return self.left_sensor.value(), self.right_sensor.value()

def follow_line(robot, line_follower):
    """Follow a line using two IR sensors"""
    BASE_SPEED = 50

    while True:
        left, right = line_follower.read_sensors()

        if left == 0 and right == 0:
            # Both on line - go straight
            robot.forward(BASE_SPEED)
        elif left == 0 and right == 1:
            # Left on line - turn left
            robot.arc_left(BASE_SPEED)
        elif left == 1 and right == 0:
            # Right on line - turn right
            robot.arc_right(BASE_SPEED)
        else:
            # Both off line - stop or search
            robot.stop()

        time.sleep(0.05)
```

**Add speed ramping:**
```python
class Robot:
    # ... previous methods ...

    def ramp_speed(self, target_speed, duration=1.0):
        """Gradually ramp to target speed"""
        steps = 20
        current = 0

        for i in range(steps + 1):
            speed = int(current + (target_speed - current) * (i / steps))
            self.forward(speed)
            time.sleep(duration / steps)
```

**Add battery monitoring:**
```python
from machine import ADC

class BatteryMonitor:
    def __init__(self, adc_pin, voltage_divider_ratio=2.0):
        self.adc = ADC(Pin(adc_pin))
        self.ratio = voltage_divider_ratio

    def get_voltage(self):
        """Get battery voltage"""
        raw = self.adc.read_u16()
        voltage = (raw / 65535) * 3.3 * self.ratio
        return voltage

    def is_low(self, threshold=6.0):
        """Check if battery is low"""
        return self.get_voltage() < threshold

# In main loop
battery = BatteryMonitor(28)

while True:
    if battery.is_low():
        print("Battery low! Stopping...")
        robot.stop()
        break
    # Normal robot behavior
```

## Comparing to Arduino

**Arduino challenges:**
- More verbose class syntax
- Manual memory management for strings
- Harder to debug on the robot
- More complex to add WiFi control

**MicroPython advantages:**
- Clean class syntax
- Easy string handling
- REPL for live debugging
- Simple network integration
- Rapid prototyping

## Try It Yourself

**Exercise 1: Add State Machine**

Create a state machine with three states:
- SEARCHING: Slowly spin and scan for clear path
- MOVING: Move forward when path is clear
- AVOIDING: Back up and turn when obstacle detected

Print the current state and transition between states based on sensor readings.

**Answer:**

```python
class RobotState:
    SEARCHING = 0
    MOVING = 1
    AVOIDING = 2

def state_machine_behavior(robot, sensor):
    state = RobotState.SEARCHING
    SAFE_DISTANCE = 25

    while True:
        distance = sensor.get_distance()

        if state == RobotState.SEARCHING:
            print("State: SEARCHING")
            robot.turn_right(30)  # Slow spin

            if distance and distance > SAFE_DISTANCE:
                state = RobotState.MOVING

        elif state == RobotState.MOVING:
            print("State: MOVING")
            robot.forward(60)

            if distance and distance < SAFE_DISTANCE:
                state = RobotState.AVOIDING

        elif state == RobotState.AVOIDING:
            print("State: AVOIDING")
            robot.stop()
            time.sleep(0.3)
            robot.backward(40)
            time.sleep(0.5)
            robot.turn_left(50)
            time.sleep(0.8)
            state = RobotState.SEARCHING

        time.sleep(0.1)

# Run it
state_machine_behavior(robot, sensor)
```

**Exercise 2: WiFi Remote Control**

Create a web interface to control the robot remotely:
- Forward, backward, left, right buttons
- Stop button
- Display current distance reading
- Show battery voltage

**Answer:**

```python
import network
import socket

def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while not wlan.isconnected():
        time.sleep(0.5)
    print(f"Connected: {wlan.ifconfig()[0]}")
    return wlan

def generate_control_html(distance):
    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ text-align: center; font-family: Arial; padding: 20px; }}
        button {{
            padding: 30px;
            margin: 10px;
            font-size: 24px;
            border-radius: 10px;
            border: none;
            background: #4CAF50;
            color: white;
        }}
        .stop {{ background: #f44336; }}
        .distance {{ font-size: 48px; margin: 20px; }}
    </style>
</head>
<body>
    <h1>Robot Remote Control</h1>
    <div class="distance">Distance: {distance:.1f} cm</div>
    <div>
        <a href="/forward"><button>↑ Forward</button></a>
    </div>
    <div>
        <a href="/left"><button>← Left</button></a>
        <a href="/stop"><button class="stop">■ Stop</button></a>
        <a href="/right"><button>→ Right</button></a>
    </div>
    <div>
        <a href="/backward"><button>↓ Backward</button></a>
    </div>
</body>
</html>"""
    return html

def web_control(robot, sensor):
    wlan = connect_wifi("YourWiFi", "YourPassword")

    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)

    print("Web server running")

    while True:
        client, addr = s.accept()
        request = client.recv(1024).decode()

        # Parse commands
        if "/forward" in request:
            robot.forward(60)
        elif "/backward" in request:
            robot.backward(60)
        elif "/left" in request:
            robot.turn_left(50)
        elif "/right" in request:
            robot.turn_right(50)
        elif "/stop" in request:
            robot.stop()

        # Get sensor reading
        distance = sensor.get_distance() or 0

        # Send response
        html = generate_control_html(distance)
        response = f"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n{html}"
        client.send(response.encode())
        client.close()

# Run it
web_control(robot, sensor)
```

**Exercise 3: Add Emergency Stop**

Add an emergency stop feature:
- If battery voltage drops below threshold, stop immediately
- If ultrasonic sensor reads < 5cm, immediate stop (crash prevention)
- If no sensor reading for 3 seconds, stop (sensor failure)
- Flash LED rapidly when stopped
- Print emergency reason

**Answer:**

```python
import time

class EmergencyStop:
    def __init__(self, robot, sensor, battery, status_led):
        self.robot = robot
        self.sensor = sensor
        self.battery = battery
        self.status_led = status_led
        self.last_reading_time = time.ticks_ms()

    def check(self):
        """Check all emergency conditions. Returns (safe, reason)"""

        # Check battery
        voltage = self.battery.get_voltage()
        if voltage < 6.0:
            return False, f"Low battery: {voltage:.1f}V"

        # Check distance
        distance = self.sensor.get_distance()

        if distance is None:
            # No reading - check timeout
            elapsed = time.ticks_diff(time.ticks_ms(), self.last_reading_time)
            if elapsed > 3000:
                return False, "Sensor timeout"
        else:
            self.last_reading_time = time.ticks_ms()

            if distance < 5:
                return False, f"Collision warning: {distance:.1f}cm"

        return True, "OK"

    def emergency_stop(self, reason):
        """Execute emergency stop"""
        print(f"EMERGENCY STOP: {reason}")
        self.robot.stop()

        # Flash LED rapidly
        for i in range(10):
            self.status_led.toggle()
            time.sleep(0.1)

def safe_autonomous_mode(robot, sensor, battery, status_led):
    """Autonomous mode with emergency stop"""
    emergency = EmergencyStop(robot, sensor, battery, status_led)

    while True:
        safe, reason = emergency.check()

        if not safe:
            emergency.emergency_stop(reason)
            break

        # Normal obstacle avoidance
        distance = sensor.get_distance()
        if distance and distance < 20:
            robot.stop()
            time.sleep(0.3)
            robot.backward(40)
            time.sleep(0.5)
            robot.turn_right(50)
            time.sleep(0.5)
        else:
            robot.forward(60)

        time.sleep(0.1)

# Run it
battery = BatteryMonitor(28)
safe_autonomous_mode(robot, sensor, battery, status_led)
```

---
