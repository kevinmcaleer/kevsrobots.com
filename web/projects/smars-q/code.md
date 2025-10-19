---
title: "SMARS Q"
description: "An Arduino Uno Q Based Robot"
excerpt: >-
    This project showcases the design and construction of the SMARS Q robot, an Arduino Uno-based robot designed for various tasks.
   
layout: showcase
date: 2025-10-19
author: Kevin McAleer
difficulty: beginner
cover: /projects/smars-q/assets/cover.jpg
hero:  /projects/smars-q/assets/hero.png
mode: light

tags:
 - arduino
 - robotics
groups:
 - python
 - robotics
 - arduino
code:
 - https://www.github.com/kevinmcaleer/smars_q
---

{% include projects/nav.html %}

## Code

The code for the SMARS Q robot is available on [GitHub](https://www.github.com/kevinmcaleer/smars_q). 

There are 3 demos included:

- Demo 1: Basic Range Finder - This demo uses the ultrasonic sensor to measure distance to an object and displays the distance the console log


## Demo 1: Basic Range Finder

### Python Code

This could should be saved as `main.py`.

```python
def main():
    print("Hello World!")


if __name__ == "__main__":
    main()
```

### Arduino Sketch

Save this code as `sketch.ino`.

```c++
#include "Arduino_RouterBridge.h"
#define echoPin 2
#define trigPin 3

long duration;
int distance;
int dist;

void setup() {

  Monitor.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  Serial.println("Ultrasnoic Range Finder");
}

int ping(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  //Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Reasd teh echo pin
  long startTime = micros();
  while(digitalRead(echoPin)==LOW);
  startTime = micros();
  while(digitalRead(echoPin)==HIGH);
  long endTime = micros();
  duration = endTime - startTime;

  //calculate the distance
  distance = duration * 0.034 /2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
 }

void loop() {
  dist = ping();
  Monitor.print("Distance is:");
  Monitor.print(dist);
  Monitor.println("cm");
  delay(500);
}
```
---

## Demo 2: Obstacle Avoidance

### Python Code

Save this code as `main.py`.

```python

from arduino.app_utils import *
from time import sleep

distance = 0

logger = Logger("distance")

def update_distance(distance:int):
  output = f"distance is {distance}"
  # print(output)
  logger.info(output)

def main():
  # Provide the function for the MCU sketch
  try:
    Bridge.provide("update_distance", update_distance)
    # print("updated distance")
    output = f"distance is {distance}"
    # print(output)
    logger.info(output)
  except RuntimeError: 
    print("some kind of issue.")

if __name__ == "__main__":
  main()

App.run()
```

### Arduino Sketch

Save this code as `sketch.ino`.

```c++
#include "Arduino_RouterBridge.h"
#define echoPin 2
#define trigPin 3

long duration;
int distance;
int dist;

void setup() {
  Bridge.begin();
  Serial.begin(9600);
  Monitor.begin();

  // Bridge.provide("ping", dist);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.println("Ultrasnoic Range Finder");
}

int ping(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  //Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Reasd teh echo pin
  long startTime = micros();
  while(digitalRead(echoPin)==LOW);
  startTime = micros();
  while(digitalRead(echoPin)==HIGH);
  long endTime = micros();
  duration = endTime - startTime;

  //calculate the distance
  distance = duration * 0.034 /2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
 }

void loop() {
    dist = ping();
    Monitor.print("Distance is:");
    Monitor.print(dist);
    Monitor.println("cm");
    distance = dist;

    Bridge.notify("update_distance",distance);

    Serial.print("Distance is: ");
    Serial.print(distance);
    Serial.println("cm");
    
    delay(500);
}
```

---

## Demo 3: Motor control and web interface

### Python Code

Save this code as `main.py`.

```python
from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
from time import sleep, time
ui = WebUI()
distance = 0
logger = Logger("distance")
last_stop_time = 0
STOP_DEBOUNCE = 0.2  # Don't allow stop commands more than every 200ms
def update_distance(distance:int):
  output = f"distance is {distance}"
  logger.info(output)
def stop_motors():
  global last_stop_time
  current_time = time()
  # Only process stop if enough time has passed
  if current_time - last_stop_time < STOP_DEBOUNCE:
    return "OK"
  
  last_stop_time = current_time
  try:
    logger.info("telling arduino to stop")
    Bridge.call("stopMotors")
  except Exception as e:
    logger.info(f"Error stopping motors: {e}")
  return "OK"

def forward():
  try:
    logger.info("telling arduino to move forward")
    Bridge.call("forward")
  except Exception as e:
    logger.info(f"Error moving forward: {e}")
  return "OK"

def backward():
  try:
    logger.info("telling arduino to move backward")
    Bridge.call("backward")
  except Exception as e:
    logger.info(f"Error moving backward: {e}")
  return "OK"

def left():
  try:
    logger.info("telling arduino to turn left")
    Bridge.call("left")
  except Exception as e:
    logger.info(f"Error turning left: {e}")
  return "OK"

def right():
  try:
    logger.info("telling arduino to turn right")
    Bridge.call("right")
  except Exception as e:
    logger.info(f"Error turning right: {e}")
  return "OK"
def main():
  try:
    Bridge.provide("update_distance", update_distance)
    output = f"distance is {distance}"
    logger.info(output)
  except RuntimeError: 
    print("some kind of issue.")
    logger.info("calling forward")
    Bridge.call("forward")
    sleep(0.5)
    logger.info("calling stop")
    Bridge.call("stopMotors")
    sleep(0.5)
if __name__ == "__main__":
  main()
ui.expose_api("GET", "/forward", forward)
ui.expose_api("GET", "/stop", stop_motors)
ui.expose_api("GET", "/backward", backward)
ui.expose_api("GET", "/left", left)
ui.expose_api("GET", "/right", right)
ui.start()
App.run()
```

---

### Arduino Sketch

Save this code as `sketch.ino`.

```c++
#include "Arduino_RouterBridge.h"
#define echoPin 2
#define trigPin 3
#define leftMotorDirection 13
#define rightMotorDirection 12
#define leftMotorSpeed 11
#define rightMotorSpeed 10

long duration;
int distance;
int dist;
unsigned long lastPingTime = 0;
const unsigned long PING_INTERVAL = 500; // ms
const unsigned long PING_TIMEOUT = 30000; // 30ms timeout for echo

void setup() {
  Bridge.begin();
  Serial.begin(9600);
  Monitor.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(leftMotorDirection, OUTPUT);
  pinMode(rightMotorDirection, OUTPUT);
  
  Serial.println("Ultrasonic Range Finder");
  Bridge.provide("forward", forward);
  Bridge.provide("backward", backward);
  Bridge.provide("left", left);
  Bridge.provide("right", right);
  Bridge.provide("stopMotors", stopMotors);
}

void forward(){
  digitalWrite(leftMotorDirection, HIGH);
  digitalWrite(rightMotorDirection, LOW);
  analogWrite(leftMotorSpeed, 255);
  analogWrite(rightMotorSpeed, 255);
  Monitor.println("Sketch received forward");
}

void backward(){
  digitalWrite(leftMotorDirection, LOW);
  digitalWrite(rightMotorDirection, HIGH);
  analogWrite(leftMotorSpeed, 255);
  analogWrite(rightMotorSpeed, 255);
  Monitor.println("Sketch received backward");
}

void left(){
  digitalWrite(leftMotorDirection, LOW);
  digitalWrite(rightMotorDirection, LOW);
  analogWrite(leftMotorSpeed, 255);
  analogWrite(rightMotorSpeed, 255);
  Monitor.println("Sketch received left");
}

void right(){
  digitalWrite(leftMotorDirection, HIGH);
  digitalWrite(rightMotorDirection, HIGH);
  analogWrite(leftMotorSpeed, 255);
  analogWrite(rightMotorSpeed, 255);
  Monitor.println("Sketch received right");
}

void stopMotors(){
  analogWrite(leftMotorSpeed, 0);
  analogWrite(rightMotorSpeed, 0);
  Monitor.println("Sketch received stop motors");
}

int ping(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Wait for echo to go HIGH with timeout
  unsigned long startWait = micros();
  while(digitalRead(echoPin)==LOW && (micros() - startWait) < PING_TIMEOUT);
  
  if ((micros() - startWait) >= PING_TIMEOUT) {
    Monitor.println("Ping timeout - echo never went HIGH");
    return -1; // Error value
  }
  
  long echoStart = micros();
  
  // Wait for echo to go LOW with timeout
  startWait = micros();
  while(digitalRead(echoPin)==HIGH && (micros() - startWait) < PING_TIMEOUT);
  
  if ((micros() - startWait) >= PING_TIMEOUT) {
    Monitor.println("Ping timeout - echo never went LOW");
    return -1; // Error value
  }
  
  long echoEnd = micros();
  duration = echoEnd - echoStart;
  
  // Calculate distance only if valid
  if (duration > 0) {
    distance = duration * 0.034 / 2;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  return distance;
}

void loop() {
  // Non-blocking ping - only run every PING_INTERVAL ms
  if (millis() - lastPingTime >= PING_INTERVAL) {
    lastPingTime = millis();
    dist = ping();
    
    if (dist > 0) { // Only report valid distances
      Monitor.print("Distance is: ");
      Monitor.print(dist);
      Monitor.println("cm");
      distance = dist;
      Bridge.notify("update_distance", distance);
    }
  }
  // Loop continues immediately - Bridge can respond to commands anytime
}
```

---

## Web page assets

Put the index.html and style.css files into an assets folder.

### index.html

Save this code as `index.html`.

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Control</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }

        .header {
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 20px;
            margin-bottom: 40px;
        }

        .arduino-text {
            color: white;
            font-size: 2.5em;
            font-weight: bold;
        }

        .arduino-logo {
            height: 50px;
            filter: brightness(0) invert(1);
        }

        .container {
            background: white;
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
            max-width: 400px;
            width: 100%;
        }

        .control-title {
            text-align: center;
            color: #333;
            font-size: 1.5em;
            margin-bottom: 30px;
        }

        .directional-pad {
            display: grid;
            grid-template-columns: repeat(3, 100px);
            gap: 10px;
            justify-content: center;
            margin-bottom: 30px;
        }

        .directional-pad button {
            padding: 20px;
            font-size: 1.2em;
            font-weight: bold;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.2s ease;
            color: white;
            box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2);
        }

        .directional-pad button:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(0, 0, 0, 0.3);
        }

        .directional-pad button:active {
            transform: translateY(0);
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.2);
        }

        .btn-up {
            grid-column: 2;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        }

        .btn-left {
            grid-column: 1;
            grid-row: 2;
            background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
        }

        .btn-stop {
            grid-column: 2;
            grid-row: 2;
            background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
            color: #333;
        }

        .btn-right {
            grid-column: 3;
            grid-row: 2;
            background: linear-gradient(135deg, #43e97b 0%, #38f9d7 100%);
            color: #333;
        }

        .btn-down {
            grid-column: 2;
            grid-row: 3;
            background: linear-gradient(135deg, #fa709a 0%, #fee140 100%);
            color: #333;
        }

        .instruction-text {
            text-align: center;
            color: #666;
            font-size: 0.95em;
            margin-bottom: 20px;
        }

        .error-message {
            background-color: #fee;
            color: #c00;
            padding: 12px;
            border-radius: 8px;
            text-align: center;
            border-left: 4px solid #c00;
            font-size: 0.9em;
        }

        .status-indicator {
            text-align: center;
            font-size: 0.85em;
            color: #999;
            margin-top: 20px;
        }

        .status-indicator.active {
            color: #667eea;
            font-weight: bold;
        }

        @media (max-width: 480px) {
            .container {
                padding: 25px;
            }

            .directional-pad {
                grid-template-columns: repeat(3, 80px);
            }

            .directional-pad button {
                padding: 15px;
                font-size: 1em;
            }

            .arduino-text {
                font-size: 2em;
            }
        }
    </style>
</head>
<body>
    <div>
        <div class="header">
            <h1 class="arduino-text">Robot Control</h1>
        </div>
        <div class="container">
            <h2 class="control-title">Directional Controls</h2>
            
            <!-- Directional Pad -->
            <div class="directional-pad">
                <button class="btn-up" data-command="forward">🔼</button>
                <button class="btn-left" data-command="left">🔙</button>
                <button class="btn-stop" data-command="stop">⏹</button>
                <button class="btn-right" data-command="right">🔚</button>
                <button class="btn-down" data-command="backward">🔽</button>
            </div>

            <!-- Instruction Text -->
            <p class="instruction-text">Use directional buttons to control the robot 👆</p>

            <!-- Status Message -->
            <div class="status-indicator" id="status-message">Ready</div>

            <!-- Error message container -->
            <div id="error-container" class="error-message" style="display: none;"></div>
        </div>
    </div>

    <script>
        const ROBOT_IP = 'http://192.168.1.229:7000';
        const commands = {
            forward: '/forward',
            backward: '/backward',
            left: '/left',
            right: '/right',
            stop: '/stop'
        };

        const buttons = document.querySelectorAll('.directional-pad button');
        const errorContainer = document.getElementById('error-container');
        const statusMessage = document.getElementById('status-message');

        buttons.forEach(button => {
            button.addEventListener('mousedown', async (e) => {
                const command = e.target.closest('button').dataset.command;
                await sendCommand(command);
            });

            button.addEventListener('mouseup', async () => {
                await sendCommand('stop');
            });

            // Touch events for mobile
            button.addEventListener('touchstart', async (e) => {
                e.preventDefault();
                const command = e.target.closest('button').dataset.command;
                await sendCommand(command);
            });

            button.addEventListener('touchend', async (e) => {
                e.preventDefault();
                await sendCommand('stop');
            });
        });

        let lastCommandTime = {};
        const DEBOUNCE_DELAY = 150; // ms

        async function sendCommand(command) {
            const now = Date.now();
            const lastTime = lastCommandTime[command] || 0;
            
            // Debounce: skip if same command sent too recently
            if (now - lastTime < DEBOUNCE_DELAY) {
                return;
            }
            lastCommandTime[command] = now;

            const url = ROBOT_IP + commands[command];
            
            try {
                errorContainer.style.display = 'none';
                statusMessage.textContent = `Sending: ${command.toUpperCase()}...`;
                statusMessage.classList.add('active');

                const response = await fetch(url, {
                    method: 'GET',
                    mode: 'no-cors'
                });

                statusMessage.textContent = `✓ ${command.toUpperCase()} sent`;
                setTimeout(() => {
                    statusMessage.textContent = 'Ready';
                    statusMessage.classList.remove('active');
                }, 1500);

            } catch (error) {
                console.error('Error:', error);
                errorContainer.textContent = `Failed to reach robot at ${ROBOT_IP}`;
                errorContainer.style.display = 'block';
                statusMessage.textContent = 'Error - Check connection';
                statusMessage.classList.remove('active');
            }
        }
    </script>
</body>
</html>
```

---

### style.css

Save this code as `style.css`.

```css
/*
 * SPDX-FileCopyrightText: Copyright (C) 2025 ARDUINO SA <http://www.arduino.cc>
 *
 * SPDX-License-Identifier: MPL-2.0
 */

@import url("fonts/roboto-mono.css");

/*
 * This CSS is used to center the various elements on the screen
 */
* {
    margin: 0;
    padding: 0;
}

body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell', 'Fira Sans', 'Droid Sans', 'Helvetica Neue', sans-serif;
    background-color: #ECF1F1;
    color: #2C353A;
    padding: 24px 40px;
}

.header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 32px;
}

.arduino-text {
    color: #008184;
    font-family: "Roboto Mono", monospace;
    font-size: 20px;
    font-weight: 600;
    margin: 0;
    font-style: normal;
    line-height: 170%;
    letter-spacing: 0.28px;
}

.arduino-logo {
    height: 24px;
    width: auto;
}

.container {
    text-align: center;
}

/*
 * LED Button styling
 */
.led-container {
    display: flex;
    justify-content: center;
    margin-bottom: 32px;
    padding-top: 40px;
}

#led-button {
    width: 128px;
    height: 128px;
    border-radius: 50%;
    cursor: pointer;
    transition: all 0.3s ease;
    display: flex;
    align-items: center;
    justify-content: center;
    font-family: inherit;
    font-weight: 600;
    font-size: 14px;
    text-align: center;
    line-height: 1.2;
    outline: none;
    position: relative;
    border: 2px solid #C9D2D2;
}

#led-button.led-off {
    background: #DAE3E3;
    color: #2C353A;
    border-color: #C9D2D2;
}

#led-button.led-on {
    background: #008184;
    color: #ffffff;
    box-shadow: 0 0 20px #008184, 0 0 40px #008184, 0 0 60px #008184;
    border-color: #008184;
}

#led-button:hover {
    transform: scale(1.05);
}

#led-button:active {
    transform: scale(0.95);
}

.instruction-text {
    font-size: 14px;
    font-style: normal;
    font-weight: 400;
    line-height: 160%;
    letter-spacing: 0.12px;
    color: #2C353A;
}

.error-message {
    margin-top: 20px;
    padding: 10px;
    border-radius: 5px;
    background-color: #f8d7da;
    color: #721c24;
    border: 1px solid #f5c6cb;
}

/*
 * Responsive design
 */
@media (max-width: 768px) {
    body {
        padding: 12px 20px;
    }

    .arduino-text {
        font-size: 14px;
    }

    .arduino-logo {
        height: 16px;
        width: auto;
    }
}
```

---
