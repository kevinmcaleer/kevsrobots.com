---
layout: how_it_works
title: Microcontrollers
short_title: How it works - Microcontrollers
short_description: Learn about Microcontrollers
date: 2023-10-29
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/microcontrollers.png
tags:
 - microcontrollers
 - arduino
 - pico
 - raspberrypi
 - How it works
 - stm32
 - microbit
---

A `microcontroller` (**MCU**) is like the brain in many of our electronic devices. While a standard computer can handle multiple tasks simultaneously, a microcontroller is optimized for specific tasks, making it efficient and responsive.

For instance, while a computer could run a word processor, a web browser, and a video game at once, a microcontroller might be dedicated to monitoring the temperature inside a refrigerator or controlling the movements of a robot.

---

## How Microcontrollers Operate

At their core, microcontrollers comprise a processor, memory, I/O ports, and various peripherals. Think of a microcontroller as the conductor of an orchestra, directing various instruments (components) to play in harmony to produce a specific output.

[![Microcontroller architecture](/assets/img/how_it_works/microcontroller_architecture.png){:class="img-fluid w-50"}](/assets/img/how_it_works/microcontroller_architecture.png)

## Microcontroller Inner Workings - Conceptual Layout

`1. Central Processing Unit (CPU)`

- The "brain" of the microcontroller.
- Executes the instructions from the program.
- Interacts with all other components of the MCU.
  
`2. Memory`

- **ROM/Flash Memory**:
  - [ROM](/resources/glossary#rom) - Stores the program code.
- **RAM**:
  - [RAM](/resources/glossary#ram) - Temporary storage while the program is running.
  - Volatile memory (loses its data when power is off).
- **EEPROM** (in some MCUs):
  - [EEPROM](/resources/glossary#eeprom) - Can store data persistently, even when power is turned off.

`3. Input/Output Ports`

- Interfaces the MCU to the external world.
- Can be digital or analog.
- Many MCUs have pins that can be configured either as input or output.

`4. Clock/Timer`

- Provides a timing signal to the MCU.
- Necessary for synchronous operations and to maintain accurate timing.

`5. Communication Interfaces`

- **UART**: [UART](/resources/glossary#uart) - Universal Asynchronous Receiver-Transmitter. Used for serial communication.
- **SPI**: [SPI](/resources/glossary#spi) - Serial Peripheral Interface. A synchronous serial data protocol.
- **I2C**: [I2C](/resources/glossary#i2c) - Inter-Integrated Circuit. A bus for connecting multiple devices.
  
`6. ADC (Analog to Digital Converter) and DAC (Digital to Analog Converter)`

- [**ADC**](/resources/glossary#adc) converts analog signals (like from sensors) to digital data.
- [**DAC**](/resources/glossary#dac) converts digital data to analog signals (like to drive speakers).

`7. Interrupt System`

- Allows the MCU to pause its main task and respond to urgent events.
  
`8. Other Peripherals (vary with the MCU model)`

- **PWM Modules**: [PWM](/resources/glossary#pwm) For generating Pulse Width Modulation signals, often used in motor control.
- **Comparators**: For comparing voltage levels.

---

## Where You Encounter Microcontrollers Daily

Everyday objects powered by microcontrollers include:

- **Home gadgets**: Thermostats, refrigerators, and smart speakers.
- **Toys**: Electronic dolls, drones.
- **Healthcare**: Glucometers, digital thermometers.
- **Transport**: Modern car's braking systems, traffic lights.

---

## Popular Microcontrollers to Know

Microcontroller                                      | Description | Manufacturer | Website
-----------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------
[`Arduino`](/resources/boards/arduino) (based on AVR)                               | A favorite among hobbyists, it's an excellent platform for beginners due to its vast community and plethora of projects | Arduino | <https://www.arduino.cc>
[`Raspberry Pi Pico`](/resources/boards/pico) and [`STM32`](/resources/boards/stm32) (both ARM Cortex series) | While Raspberry Pi Pico is becoming popular for hobby projects, STM32 is versatile, ranging from simple tasks to advanced applications | Raspberry Pi / STMicroelectronics | <https://www.raspberrypi.com> / <https://www.st.com>
[`ESP8266`](/resources/boards/esp8266) and [`ESP32`](/resources/boards/esp32)                                    | Ideal for connected projects with Wi-Fi and Bluetooth capabilities |Espressif| <https://www.espressif.com>
[`BBC micro:bit`](/resources/boards/microbit)                                        | Designed for educational purposes, it's user-friendly with integrated features, making it great for young learners | Micro:bit Educational Foundation | <https://www.microbit.org>
{:class="table table-bordered table-striped"}

---

## Starting with Microcontrollers

Embarking on your microcontroller journey is exciting. Begin with:

1. Acquiring a starter kit like [Arduino](https://www.arduino.cc) or [micro:bit](https://www.microbit.org).
2. Understanding basic electronics principles - [start with a course](/learn/micropython_gpio/).
3. Setting up your workspace with [necessary tools](https://www.youtube.com/watch?v=o8Cvr9nuccs).
4. Engaging with [online communities](/discord) for support and project ideas.

Remember, every expert was once a beginner. Take one step at a time.

---

## In Conclusion

The world of microcontrollers is vast and ever-evolving. As foundational elements in numerous devices, understanding them can open doors to innovative creations and solutions. Embrace the learning curve, and let your creativity soar!

---

## Resources for Exploration

- [Microchip's official website](https://www.microchip.com/)
- [STMicroelectronics](https://www.st.com/)
- [BBC micro:bit Tutorials for Beginners](https://microbit.org/)
- [Advanced Microcontroller Projects](https://www.avrfreaks.net/)
- [Coding: The 21st Century's most valuable skill](https://monkmakes.com/coding_book.html) - For those looking for in-depth knowledge.

---

## Suggested Courses

<div class="row row-cols-1 row-cols-md-2 row-cols-lg-4 g-3">
{% include course.html count=1 course="Raspberry Pi Pico with MicroPython - GPIO Mastery" %}
{% include course.html count=2 course="Learn how to program SMARS with Arduino"%}
</div>
---
