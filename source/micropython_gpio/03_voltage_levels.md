---
layout: lesson
title: Voltage Levels
type: page
description: Learn about the voltage levels of Raspberry Pi Pico pins and how to work with them safely.
---

## Overview

Welcome to Lesson 3 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will discuss the voltage levels of Raspberry Pi Pico pins and how to work with them safely.

---

## Lesson Content

In this lesson, you will learn:

* The voltage levels of Raspberry Pi Pico pins
* How to work with these voltage levels safely
* Converting voltages when necessary

---

## Voltage Levels

The Raspberry Pi Pico operates at 3.3V. All of its GPIO pins can be used as digital inputs or outputs with a voltage range of 0 to 3.3V. However, it is important to note that some sensors and devices may require different voltage levels.

---

## Working with Voltage Levels Safely

It is important to be aware of the voltage levels of the components you are working with and to ensure that you are using them safely. Here are some tips for working with voltage levels safely:

* Always check the voltage requirements of your components before connecting them to the Raspberry Pi Pico.
* Use voltage level shifters or level converters when necessary to ensure that components are operating within their safe voltage range.
* Avoid connecting components that require different voltage levels to the same GPIO pin. If you must do this, use a [level shifter](/resources/glossary#level-shifter) or converter to ensure that both components are operating safely.
* Use a multimeter to measure voltage levels before connecting components to ensure that they are operating within their safe range.

---

## Converting Voltages

If you need to interface with components that require a different voltage level than the Raspberry Pi Pico, you will need to use a voltage level shifter or converter. These devices can be used to convert the voltage level of a signal from one level to another.

For example, if you are interfacing with a device that operates at 5V, you can use a voltage level shifter to convert the signal to 3.3V so that it can be safely connected to the Raspberry Pi Pico.

---

## Key Results

After you have completed this lesson, you will understand the voltage levels of Raspberry Pi Pico pins and how to work with them safely. This knowledge will help you avoid potential damage to your board or components when connecting them in your projects.

---
