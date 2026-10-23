---
title: Arduino Code
description: Control your SMARS with Bluetooth 
layout: lesson
---

Below is a sample program to control your SMARS with an additional Bluetooth serial module (such as an HC-05). The module is connected to the Arduino's serial pins, so anything you send from a Bluetooth terminal app on your phone arrives with `Serial.read()`.

| Send | What SMARS does |
| --- | --- |
| `u` | Drive forward for one second |
| `d` | Drive backward |
| `l` | Turn left |
| `r` | Turn right |
| `s` | Stop |
| `b` | Sound the buzzer |
{:class="table table-striped"}

---

```cpp
// SMARS Bluetooth remote control
// Kevin McAleer
// April 2019
// May 2019 - added buzzer feature
// Requires a Fundumoto shield

int ch_A_Direction = 12;
int ch_B_Direction = 13;
int ch_A_speed = 10;
int ch_B_speed = 11;
int buzzerPin = 4;
char state = 0;
int delaylength = 1000;

void setup() {
  Serial.begin(9600, SERIAL_8N1);
  Serial.println("SMARSFan OS 1.1");
  Serial.println("---------------");

  // motor direction pins and the buzzer are outputs
  pinMode(ch_A_Direction, OUTPUT);
  pinMode(ch_B_Direction, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
}

void buzz() {
  digitalWrite(buzzerPin, HIGH);
  delay(delaylength / 2);
  digitalWrite(buzzerPin, LOW);
}

// Set both motor directions, run at full speed for delaylength, then stop
void drive(int dirA, int dirB) {
  digitalWrite(ch_A_Direction, dirA);
  digitalWrite(ch_B_Direction, dirB);

  analogWrite(ch_A_speed, 255); // full speed ahead
  analogWrite(ch_B_speed, 255);

  delay(delaylength);

  analogWrite(ch_A_speed, 0); // stop
  analogWrite(ch_B_speed, 0);
}

void forward()  { drive(LOW, HIGH); }
void backward() { drive(HIGH, LOW); }
void left()     { drive(HIGH, HIGH); }
void right()    { drive(LOW, LOW); }

void fullstop() {
  analogWrite(ch_A_speed, 0);
  analogWrite(ch_B_speed, 0);
}

void loop() {
  if (Serial.available() > 0) {
    state = Serial.read();
    Serial.println(state);

    if (state == 'u') {
      Serial.println("MOTORS: UP");
      forward();
    } else if (state == 'd') {
      Serial.println("MOTORS: DOWN");
      backward();
    } else if (state == 'l') {
      Serial.println("MOTORS: LEFT");
      left();
    } else if (state == 'r') {
      Serial.println("MOTORS: RIGHT");
      right();
    } else if (state == 's') {
      Serial.println("MOTORS: STOP");
      fullstop();
    } else if (state == 'b') {
      Serial.println("BUZZER: SOUND");
      buzz();
    }
    state = 0;
  }
}
```

---

## How it works

- `setup()` starts the serial port at 9600 baud and sets the motor direction pins and buzzer pin as outputs.
- `drive()` sets the direction of each motor, then uses `analogWrite()` to run both at full speed (255) for one second before stopping. Turning the motors the same way spins the robot, because they face opposite directions in the chassis.
- `loop()` waits for a character to arrive and calls the matching function.

> ## Upload tip
>
> The Bluetooth module shares the Arduino's serial pins with the USB connection. Unplug the module while you upload the code, then plug it back in.

## Try it yourself

1. Change `delaylength` to `500` so each command moves the robot a shorter distance.
2. Add a new command, `f`, that drives forward at half speed (`analogWrite(ch_A_speed, 128)`).
3. Challenge: make the robot do a little dance when you send `x`.

## Check your understanding

{% quiz %}
? What does `analogWrite(ch_A_speed, 255)` do?
- [ ] Sets motor A to turn backwards
- [x] Runs motor A at full speed
- [ ] Reads the speed of motor A
> `analogWrite` sends a PWM signal from 0 (stopped) to 255 (full speed).

? Why should you unplug the Bluetooth module while uploading code?
- [x] It shares the serial pins with the USB connection and can block the upload
- [ ] It might get too hot
- [ ] The Arduino IDE can't see it
> Both use the Arduino's TX and RX pins, so they get in each other's way during an upload.
{% endquiz %}

