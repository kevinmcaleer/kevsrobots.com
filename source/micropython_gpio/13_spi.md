---
layout: lesson
title: Introduction to SPI
type: page
description: Learn about SPI communication protocol and how to use it with the Raspberry Pi Pico and MicroPython.
---

## Introduction

Welcome to Lesson 13 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, you will learn about the SPI (Serial Peripheral Interface) communication protocol and how to use it with the Raspberry Pi Pico and MicroPython. SPI is a common communication protocol used to connect microcontrollers to other devices such as sensors, displays, and EEPROMs.

## What is SPI?

SPI is a synchronous communication protocol used to transmit and receive data between two devices. It uses four wires for communication: MOSI (Master Output, Slave Input), MISO (Master Input, Slave Output), SCLK (Serial Clock), and SS (Slave Select). The SS line is used to select the slave device with which the master device wants to communicate.

## Setting up SPI on Raspberry Pi Pico

To use SPI on your Raspberry Pi Pico board, you will need to enable the SPI interface and configure the SPI pins. Follow these steps to set up SPI:

1. Enable SPI on your Raspberry Pi Pico board using the `machine.SPI` module.
2. Configure the SPI pins using the `init` method.
3. Use the `write_readinto` method to write data to and read data from the connected device.

Here's an example of setting up SPI on the Raspberry Pi Pico:

```python
from machine import Pin, SPI

# Initialize SPI
spi = SPI(0, baudrate=100000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3), miso=Pin(4))

# Write data to the connected device
spi.write(b'\x01\x02\x03')

# Read data from the connected device
data = bytearray(3)
spi.write_readinto(b'\x00\x00\x00', data)
print(data)
```

---

## Using SPI in MicroPython

Once you have set up SPI on your Raspberry Pi Pico board, you can use it to communicate with other devices. Here's an example of writing data to and reading data from an SPI device:

```python
# Write data to the connected device
spi.write(b'\x01\x02\x03')

# Read data from the connected device
data = bytearray(3)
spi.readinto(data)
print(data)
```

---

## Conclusion

In this lesson, you learned about the SPI communication protocol and how to use it with the Raspberry Pi Pico and MicroPython. You also learned how to set up SPI on your board and how to use it to communicate with other devices. You can use this knowledge to create a variety of projects that involve communicating with other devices using SPI.

---
