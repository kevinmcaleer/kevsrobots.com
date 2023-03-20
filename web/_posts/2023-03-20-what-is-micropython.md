---
layout: blog
title: What is MicroPython?
description: "MicroPython: A Lighter, Faster Python for Microcontrollers"
short_title: What is MicroPython?
short_description: "MicroPython: A Lighter, Faster Python for Microcontrollers"
date: 2023-03-20
author: Kevin McAleer
excerpt: >-
   In this blog post, we will explore what MicroPython is, how it differs from Python, and some of its unique features.
cover: /assets/img/blog/micropython/micropython.jpg
tags: 
 - Micropython
 - What is MicroPython

---

## Introduction

`Python` has become an incredibly popular programming language, thanks to its simplicity, readability, and versatility. However, when it comes to programming microcontrollers and other resource-constrained devices, Python may seem like a heavyweight contender. This is where `MicroPython` comes into play. In this blog post, we will explore what MicroPython is, how it differs from Python, and some of its unique features.

---

{:toc}
* toc

---

## What is MicroPython?

MicroPython (<https://www.micropython.org>) is a lean and efficient implementation of the Python 3 programming language. It is specifically designed to run on microcontrollers and other resource-constrained environments, such as low-powered Internet of Things (IoT) devices. MicroPython allows developers to write and execute Python code on these devices without the need for a full-blown Python interpreter. It brings the simplicity and power of Python to the world of embedded systems, making it more accessible to developers who want to create small-scale, lightweight projects.

---

## How is MicroPython different from Python?

### Memory Footprint and Performance

MicroPython is designed to be lightweight and efficient, which makes it suitable for devices with limited resources. It has a significantly smaller memory footprint compared to the standard Python implementation, CPython. This allows MicroPython to run on devices with as little as 16KB of RAM, whereas CPython typically requires at least a few megabytes of memory to run.

---

### Libraries and Modules

MicroPython includes a subset of the Python standard library, tailored to meet the requirements of resource-constrained environments. It only contains the most essential modules, and some of these modules have been modified or stripped down to save memory and improve performance. Additionally, MicroPython has its own set of libraries and modules specifically designed for microcontroller functionality, such as [GPIO](/resoures/glossary#gpio) control and communication with sensors.

---

### Hardware Access and Control

MicroPython offers direct access to microcontroller hardware, making it easy to interface with sensors, actuators, and other electronic components. It provides built-in libraries and modules for [GPIO](/resoures/glossary#gpio) control, [ADC](/resources/glossary#adc), [PWM](/resources/glossary#pwm), [I2C](/resources/glossary#i2c), [SPI](/resources/glossary#spi), [UART](/resources/glossary#uart), and more, allowing developers to work with hardware at a high level of abstraction without the need for complex low-level programming.

---

### REPL (Read-Eval-Print Loop)

MicroPython comes with a built-in `REPL`, which allows developers to interactively write and execute code on the microcontroller. This makes it easy to test and debug code without having to constantly recompile and upload the program to the device. The REPL can be accessed over a serial connection or through other communication interfaces, depending on the microcontroller being used.

---

### Target Platforms

While Python is designed to run on a wide range of platforms, including desktop computers, servers, and mobile devices, MicroPython is specifically targeted at microcontrollers and resource-constrained environments. Popular microcontrollers supported by MicroPython include the [ESP8266](/resources/glossary#esp8266), [ESP32](/resources/glossary#esp32), and [STM32](/resources/glossary#stm32) series, among others.

---

## Conclusion

MicroPython brings the power and simplicity of Python to the world of microcontrollers, making it easier for developers to create lightweight, resource-efficient projects. Although it lacks some features and libraries found in standard Python, it compensates with its unique advantages, such as its small memory footprint, performance optimization, and built-in hardware control capabilities. If you're interested in diving into the world of embedded systems and IoT devices, MicroPython is an excellent starting point that allows you to leverage your existing Python knowledge.

---
