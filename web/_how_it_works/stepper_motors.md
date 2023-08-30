---
layout: how_it_works
title: Stepper Motors
short_title: How it works - Stepper Motors
short_description: Learn about Stepper Motors
date: 2023-03-08
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/stepper_motor01.png
tags:
 - Robot
 - NEMA 17
 - Stepper Motors
 - Motors 
 - How it works
---

`Stepper Motors` are electric motors that convert electrical pulses into precise mechanical movements.

They are typically used when precise position and speed control is required, such as in robotics, 3D printing, and CNC machines.

Stepper motors work by energizing individual electromagnets in a specific sequence, causing the rotor (the moving part of the motor) to move a tiny amount with each pulse. By controlling the sequence, speed, and direction of the pulses, the motor can move the rotor in precise steps, allowing for accurate position control.

A common stepper motor, found on many 3d printers is the NEMA 17. NEMA stands for `National Electrical Manufacturers Association` (NEMA) and is a US group. The '17' in NEMA 17 is teh size of the faceplate size in Inches - `1.7"` by `1.7"`.

---

## Animation

Internally the stepper motor will have a number of electro magenets and an gear that will be attracted to and held in place by which ever magnet is energised. The magnets can be powered in sequence to turn the gear and will turn one step per magnet activated. In the example below there are 4 magnets (A - D), and they turn the gear round with each activation.

![Stepper Motor Animation](/assets/img/how_it_works/stepper_motor.gif){:class="img-fluid w-100"}

---

![Stepper Motor](/assets/img/how_it_works/stepper_motor01.png){:class="img-fluid w-100"}

---
