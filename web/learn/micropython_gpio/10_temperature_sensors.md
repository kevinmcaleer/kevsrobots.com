---
layout: lesson
title: Using Temperature Sensors
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 09_potentiometers.html
next: 11_uart.html
description: Learn how to use temperature sensors with the Raspberry Pi Pico and MicroPython.
percent: 55
duration: 4
navigation:
- name: Raspberry Pi Pico with MicroPython - GPIO Mastery
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Introduction to GPIO Pins
    content:
    - name: GPIO Pin Types
      link: 01_gpio_pin_types.html
    - name: Pin Numbering
      link: 02_pin_numbering.html
    - name: Voltage Levels
      link: 03_voltage_levels.html
  - section: Digital I/O and PWM
    content:
    - name: Digital I/O
      link: 04_digital_io.html
    - name: Pulse Width Modulation (PWM)
      link: 05_pwm.html
    - name: Controlling LEDs with PWM
      link: 06_leds.html
    - name: Controlling Motors with PWM
      link: 07_motors.html
  - section: Analog I/O (ADC)
    content:
    - name: Analog Input (ADC)
      link: 08_adc.html
    - name: Using Potentiometers
      link: 09_potentiometers.html
    - name: Using Temperature Sensors
      link: 10_temperature_sensors.html
  - section: UART, I2C, and SPI
    content:
    - name: Introduction to UART
      link: 11_uart.html
    - name: Introduction to I2C
      link: 12_i2c.html
    - name: Introduction to SPI
      link: 13_spi.html
  - section: Interfacing with Servos and Ultrasonic Range Finders
    content:
    - name: Introduction to Servos
      link: 14_servos.html
    - name: Ultrasonic Range Finders
      link: 15_ultrasonic_range_finders.html
  - section: Ground Pins
    content:
    - name: Ground Pins
      link: 16_ground_pins.html
  - section: Summary and Review
    content:
    - name: Recap and Review
      link: 17_summary.html
---


<!-- ![Cover photo of a temperature sensor connected to a Raspberry Pi Pico board](assets/raspberry_pi_pico_temperature_sensor.jpg){:class="cover"} -->

## Overview

Welcome to Lesson 10 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will learn how to use temperature sensors with the Raspberry Pi Pico and MicroPython.

## Course Content

In this lesson, you will learn:

* What temperature sensors are and how they work
* How to connect a temperature sensor to the Raspberry Pi Pico
* How to write MicroPython code to read temperature sensor values

---

## Key Results

After you have completed this lesson, you will know how to use temperature sensors with the Raspberry Pi Pico and MicroPython. You will be able to connect a temperature sensor to the Pico, write MicroPython code to read temperature sensor values, and understand the basics of temperature sensor operation.

---

## What you'll need

To follow this lesson, you will need:

* A computer, tablet, or phone to read the lesson material from
* A Raspberry Pi Pico board
* A temperature sensor (such as a DS18B20 or more commonly [DHT11](/resources/glossary#dht11)), jumper wires, and a breadboard for hands-on practice

---

## What are Temperature Sensors?

Temperature sensors are electronic devices that measure the temperature of the surrounding environment. There are many different types of temperature sensors, including thermistors, thermocouples, and digital temperature sensors.

---

## Connecting a Temperature Sensor to Raspberry Pi Pico

To connect a temperature sensor to your Raspberry Pi Pico board, follow these steps:

1. Identify the type of temperature sensor you are using and its pinout.
2. Connect the temperature sensor's pins to the appropriate GPIO pins on the Raspberry Pi Pico board.
3. If necessary, connect a pull-up or pull-down resistor to the temperature sensor's output pin.

---

## Reading Temperature Sensor Values with MicroPython

To read temperature sensor values using MicroPython, you will need to use a library specific to your temperature sensor. Here is an example of reading temperature values from a DS18B20 digital temperature sensor using the `onewire` and `ds18x20` modules:

```python
from machine import Pin
import onewire, ds18x20

# Set up the 1-wire bus
one_wire_bus = Pin(4)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(one_wire_bus))

# Scan for temperature sensors on the bus
ds_sensor.scan()

# Read the temperature value
ds_sensor.convert_temp()
temperature = ds_sensor.read_temp(roms[0])

# Print the temperature value
print("Temperature: {:.2f} degrees C".format(temperature))
```

---

## DHT11 example

```python
import machine
import dht

# Initialize the DHT11 sensor on pin 15
d = dht.DHT11(machine.Pin(15))

# Read the temperature and humidity values from the sensor
d.measure()

# Print the temperature and humidity values
print("Temperature: {} C".format(d.temperature()))
print("Humidity: {} %".format(d.humidity()))
```

In this example code, we first import the `machine` module and the `dht` module, which provides support for the DHT11 sensor.

We then initialize the DHT11 sensor by creating an instance of the `dht.DHT11` class with the GPIO pin number of the data pin connected to the sensor. In this case, we have connected the data pin of the sensor to GPIO pin 15.

Next, we use the `measure()` method of the `DHT11` class to read the temperature and humidity values from the sensor. This method updates the internal state of the `DHT11` instance with the latest sensor data.

Finally, we print the temperature and humidity values using the `temperature()` and `humidity()` methods of the `DHT11` instance, respectively.

Note that the DHT11 sensor requires a pull-up resistor on the data pin, so it is recommended to use a breadboard or other prototyping board to connect the sensor to the Raspberry Pi Pico. Also, the DHT11 sensor is known to be relatively slow to respond to commands, so it may take several seconds to read the temperature and humidity values after calling the `measure()` method.

---

## Conclusion

In this lesson, you learned how to use temperature sensors to measure the temperature in your environment, and how to read temperature sensor values using MicroPython. You can use this knowledge to create a variety of projects with your Raspberry Pi Pico board.

---
