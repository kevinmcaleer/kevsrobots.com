---
title: Raspberry Pi Pico
layout: board
manufacturer: Raspberry Pi Trading Ltd
cover: /assets/img/boards/pico.png
date: 2023-06-19
author: Kevin McAleer
tags:
  - pico
  - raspberry pi
  - microcontroller
---

## Overview

The Raspberry Pi Pico is a powerful yet affordable microcontroller board that serves as a gateway into the world of electronics and programming. Released by the Raspberry Pi Foundation in 2021, this compact board offers surprising versatility and functionality that allows users to build a variety of projects, from simple LED blink programs to sophisticated robotics.

---

## About the Manufacturer

The Raspberry Pi Foundation, a UK-based charity, is renowned worldwide for producing the Raspberry Pi series of single-board computers. They aim to boost computer science education by providing low-cost, high-performance computers. The Raspberry Pi Pico, their first microcontroller board, is a new venture that builds on this commitment to accessible technology.

---

## Target Audience

While originally designed for education, the Raspberry Pi Pico is an exciting product for all tech enthusiasts. It's an excellent tool for students and beginners stepping into the realm of electronics and programming. At the same time, its robust features make it suitable for professional developers and makers who want to create complex projects on a budget.

---

## Onboard Features and Specifications

![Pico Pinouts](/assets/img/boards/picopinouts.jpg){:class="img-fluid w-100"}

The Pico is compact but packed with features. It's built around the RP2040 chip designed by the Raspberry Pi team, which boasts a dual-core ARM Cortex-M0+ processor with a flexible clock speed up to 133MHz. The board includes 2MB of onboard Flash memory, and 264KB of SRAM.

The Pico has 26 GPIO pins, 3 of which are analog inputs. The remaining pins offer a range of features such as UART, SPI, and I2C interfaces. Additionally, it provides 3 ground pins and a 3V3 power pin.

The board also includes an on-board temperature sensor and a variety of other features, such as a programmable IO system (PIO) for custom peripheral support.

---

## Programming Languages

The Raspberry Pi Pico can be programmed in MicroPython and C/C++. MicroPython is a lean, efficient version of Python 3, making coding straightforward for beginners, while C/C++ offers more control and efficiency for advanced programmers. Moreover, the Pico is also compatible with the Arduino IDE, opening a vast ecosystem of libraries and examples.

---

## Fun Projects

The Pico opens a world of project possibilities. Beginners might start with a basic project, such as building a thermometer using the onboard temperature sensor or creating a pulsating LED using Pulse Width Modulation (PWM).

Advanced users can explore more complex projects, such as building a weather station by connecting external sensors, creating a MIDI device using the PIO system, or even developing a mini-game console.

---

## GPIO Pinouts

The Raspberry Pi Pico boasts 26 General-Purpose Input/Output (GPIO) pins that let the board interact with external hardware.

Here's a quick overview of the GPIO pinouts:

* **GP0 to GP25**: These are general-purpose IO pins, which can be used for digital input and output. Among these, GP26, GP27, and GP28 can also be used as analog inputs.

* **3V3(OUT)**: This pin provides a 3.3V output.

* **GND**: There are three ground pins that are used to complete the electrical circuit.

* **ADC_VREF**: This pin is a reference voltage for the onboard Analog to Digital Converter (ADC).

* **RUN**: A reset pin that restarts the board when grounded.

The versatile GPIO pins on the Pico give users the ability to connect a wide range of sensors, actuators, and other components, thus enabling a myriad of exciting and complex projects.

---

In conclusion, the Raspberry Pi Pico is a mighty tool that makes embedded systems programming accessible to all, whether you're a novice maker or a seasoned developer. It's a testament to the Raspberry Pi Foundation's commitment to bringing digital making to people worldwide.

---
