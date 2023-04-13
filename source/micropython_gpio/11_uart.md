---
layout: lesson
title: Introduction to UART
type: page
description: Learn about UART communication protocol and how to use it with the Raspberry Pi Pico and MicroPython.
---

Welcome to Lesson 10 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, you will learn how to use UART (Universal Asynchronous Receiver/Transmitter) to communicate between your Raspberry Pi Pico board and other devices. UART is a common serial communication protocol used to connect microcontrollers to other devices such as sensors, displays, and GPS modules.

## What is UART?

UART is a communication protocol used to transmit and receive data between two devices. It is an asynchronous protocol, which means that the transmitter and receiver are not synchronized by a clock signal. Instead, they rely on a predetermined baud rate to synchronize the transmission and reception of data.

## Configuring UART on Raspberry Pi Pico

To use UART on your Raspberry Pi Pico board, you will need to configure the UART pins and baud rate. Follow these steps to configure UART:

1. Identify the UART pins on your Raspberry Pi Pico board.
2. Configure the UART pins using the `machine.UART` module.
3. Set the desired baud rate using the `init` method.

Here's an example of configuring UART with a baud rate of 9600:

```python
from machine import UART

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
```

---

## Using UART in MicroPython

Once you have configured UART on your Raspberry Pi Pico board, you can use it to send and receive data to other devices. Here's an example of sending a message over UART:

```python
uart.write("Hello, world!")
```

And here's an example of receiving data over UART:

```python
data = uart.read(10)
```

---

## Conclusion

In this lesson, you learned how to use UART to communicate between your Raspberry Pi Pico board and other devices. You also learned how to configure UART on your board and how to use it in MicroPython. You can use this knowledge to create a variety of projects that involve communicating with other devices using UART.

---
