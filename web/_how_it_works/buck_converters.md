---
layout: how_it_works
title: Buck Converters
short_title: How it works - Buck Converters
short_description: Learn about how Buck Converters work
date: 2023-03-15
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/buckconverter.png
tags:
 - Robot
 - Tips
 - Buck Converters
 - Electronics
 - Components
 - Step-down Converter
 - How it works
---

A `buck converter`, also known as a `step-down converter`, is a type of DC-DC converter that takes an input voltage and efficiently converts it to a lower output voltage while maintaining a consistent output current.

This is accomplished through a combination of electronic components, including switches, inductors, [capacitors](/resources/glossary#capacitor), and [diodes](/resources/glossary#diode).

---

## Example

Here is a simplified explanation of how a buck converter works:

`Input voltage`: The buck converter takes a higher input voltage (Vin) and converts it to a lower output voltage (Vout).

`Switching`: The converter uses a switch (typically a transistor, such as a MOSFET) that rapidly turns on and off. When the switch is closed (on), current flows through the inductor and energy is stored in its magnetic field, while the output voltage starts to rise.

`Inductor and diode`: When the switch is open (off), the inductor's magnetic field starts to collapse, and the energy stored in the magnetic field is released as current flowing through the load. In this state, the diode becomes forward-biased, providing a path for the current to continue flowing to the load and maintaining the output voltage.

`Output capacitor`: The output capacitor plays a critical role in smoothing the voltage waveform, which is inherently pulsed due to the switching action of the converter. The capacitor stores and releases energy to provide a more stable output voltage.

`Feedback and control`: A feedback mechanism, usually involving a voltage divider and a control IC, is used to monitor the output voltage and adjust the switching duty cycle (the proportion of time the switch is on) to maintain a consistent output voltage, regardless of input voltage fluctuations or changes in load current.

---

The buck converter is highly efficient, with efficiencies of 90% or more being common in well-designed circuits. This efficiency makes them popular in a wide range of applications, such as power supplies for electronic devices, battery chargers, and voltage regulators in automotive and renewable energy systems.

---

## Where to buy buck converters

Popular buck converters are:

Manufacturer | Chip      | Spec                    | Price
-------------|-----------|-------------------------|-----:
Adafruit     | MPM3610   | 21V in 5V out at 1.2A   | £6.00
Adafruit     | LM3671    | 3.3V out at 600mA       | £4.80
Adafruit     | TLV62569  | 3.3V out at 1.2A        | £3.90
Adafruit     | MPM3610   | 21V in 3.3V out at 1.2A | £5.70
Adafruit     | MRPS62827 | 5.5V in 3.3V out at 2A  | £6.90
{:class="table table-striped"}

---
