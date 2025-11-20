---
layout: review
title: Arduino Nano Motor Carrier
author: Kevin McAleer
date: 2025-11-19
excerpt: 
cover: /assets/img/reviews/arduino_nano_motor_carrier/arduino_nano_motor_carrier.jpg
link: https://store.arduino.cc/products/arduino-nano-motor-carrier
rating: 4.1
transparency: 
 
---

## 👍 Recommendation

Expensive, but high quality and well designed motor carrier for Arduino Nano.

---

{:toc}
* toc

---

The Arduino Nano Motor Carrier is a well designed motor driver board for the Arduino Nano R4 or Arduino Nano 33 IoT boards. The Arduino Nano ESP32 is not officially supported (and in fact wont compile with the official library) but I found it did work fine in practice running MicroPython.

---

{% include gallery.html images="/assets/img/reviews/arduino_nano_motor_carrier/graphic.svg" titles="Arduino Nano Motor Carrier" %}

---

The board electronics has an unexpected and unusual design in that there is an onboard SAMD microcontroller that handles the motor driving and control, and the Arduino Nano board communicates with it via I2C. This is an interesting design choice, as it offloads the motor control from the main Arduino Nano board, allowing for more complex motor control without burdening the main microcontroller. However this also limits the compatibility with other boards, as the library is only designed to work with the Arduino Nano R4 and Arduino Nano 33 IoT.

I was able to get the board working with an Arduino Nano ESP32 running MicroPython by porting the official Arduino library to MicroPython (see [below](#micropython-port) for details).

---

The board is quite compact, with connectors for 4 Servos and 4 DC Motors, there are also 4 motor terminal blocks for connecting higher power motors.

---

## Built-in LiPo Charger with XT30 Connector

The board has a built in LiPo charger, with an XT30 connector for connecting a LiPo battery. This is a great feature for mobile robot projects, as it allows you to power both the motors and the Arduino Nano from a single battery.

The battery level and temperature can be monitored via the I2C interface.

---

## MicroPython Port

To get this board working with an Arduino Nano ESP32 running MicroPython, I created a port of the official Arduino library to MicroPython. You can find my ported library on GitHub here: https://github.com/kevinmcaleer/arduino_nano_motor_carrier_mp

---

## Features

| Feature                                                     | Specification                      |
|-------------------------------------------------------------|------------------------------------|
| Microcontroller                                             | ATSAMD11 ( Arm Cortex-M0+ @48 Mhz) |
| Motor Drivers (x4)                                          | MP6522                             |
| Max Input voltage (power terminals)                         | 4V (1S Li-Ion Battery)             |
| Max output current per motor driver                         | 500 mA                             |
| Motor driver output voltage                                 | 12V                                |
| Over Temperature shutdown protection (for DC motor drivers) | Yes                                |
| Battery charging                                            | Yes                                |
| Max battery charging current                                | 500mA (configurable)               |
| Power terminals (connectors)                                | XT-30 and 2POS terminal block      |
| Servo connector                                             | 4 terminals                        |
| Encoder inputs                                              | 2 ports                            |
| DC motor control                                            | 4 ports                            |
| 3V digital/analog sensor input/output                       | 4 ports                            |
| IMU                                                         | BNO055 9axis Acc/Gyr/Mag           |
{:class= "table table-striped"}

---

## Integrated Inertial Measurement Unit (IMU)

The board has an integrated BNO055 9-axis IMU, which can be used for orientation and motion sensing. This is a great feature for robotics projects, as it allows for more advanced motion control and navigation. the Magentometer can also be used for compass heading.

---

## Conclusion

The Arduino Nano Motor Carrier is a well designed and high quality motor driver board for the Arduino Nano R4 and Arduino Nano 33 IoT boards. The built-in LiPo charger and integrated IMU make it a great choice for mobile robotics projects. The only downsides are the price, which is quite high for a motor driver board, and the limited compatibility with the Arduino Nano range. However, if you need the features and quality that this board offers, it is definitely worth the investment.

---

## Review Score

{% include review_scoring.html score="2, 3, 5, 5, 5, 5, 3, 5, 4" %}

---
