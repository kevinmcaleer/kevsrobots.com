---
layout: blog
title: Servos
short_title: How it works - Servos
short_description: Learn about how Servos work
date: 2023-02-17
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/servo.png
tags:
 - Servos
 - PWM
---

`Servos` are special types of motors that are used to precisely control various movements.

---

{:toc}
* toc

---

## How do Servos work

Servos are typically found in robotics, radio control cars, and other automated systems.

Servos work by taking a signal from a controller, which is usually a pulse-width modulated ([`PWM`](/resources/glossary#pwm)) signal. This signal tells the servo how much to rotate in a given direction and by how much. The servo then takes this information, and using an internal motor and gearing system, moves to the desired position. This makes them great for precise, repeatable movements and automated systems.

---

## Types of servo

There are a variety of different types of servo motors available, including [standard servos](#standard-servos), [continuous rotation servos](#continuous-rotation-servos), [lightweight servos](#lightweight-servos), [waterproof servos](#waterproof-servos), and [brushless servos](#brushless-servos).

---

### Standard servos

Standard servos are the most commonly used type, and are used for applications such as controlling robotic arms, model aircraft, and RC cars.

Standard servos typically have just 3 wires:

![Servo wiring](/assets/img/how_it_works/servo01.jpg){:class="img-fluid w-50"}

Notice how the 5v line is the middle wire - this is a thoughtful design that means you cna't accidentally short out the servo if you insert the cable incorrectly into the servo driver.

---

## Anatamoy of a Servo

![Servo diagram](/assets/img/how_it_works/servo03.jpg){:class="img-fluid w-50"}

A servo is made up of a couple of parts:

* **Servo horn** - this is a separate piece and transfers the rotation to a small arm or `horn`. This is attached to the part you want to control
* **DC Motor** - powers the servo
* **Gearbox** - Gears provide extra torque from the DC motor
* **Potentiometer** - The potentiometer tells the Control Circuit when it reaches a specific value
* **Control Circuit** - Control Circuit received the PWM signal and turns the motor
* **Information panel** - this shows the maximum weight that the servo can lift at 1 cm from the center point (eg a 20Kg servo can lift up to 20Kg, 1cm from the center of the shaft)

![Servo inner workings](/assets/img/how_it_works/servo05.jpg){:class="img-fluid w-50"}

---

## Pulse Width Modulation

![Servo inner workings](/assets/img/how_it_works/servo06.jpg){:class="img-fluid w-50"}

`Pulse Width Modulation` (PWM) is a technique used to control the amount of power delivered to an electronic device by switching it on and off at a certain frequency.

This technique is used to control the output of a circuit using varying pulse widths, or the amount of time the circuit is on or off.

PWM is also used to control the speed and direction of motors. This is achieved by varying the duty cycle of the PWM signal, which is the ratio of the “on” time to the total period of the wave. 

A higher duty cycle will result in more power to the motor, while a lower duty cycle will result in less power.

PWM is an effective and efficient way to control the output of circuits, as it allows for precise control of the output power and helps to conserve energy.

> ![Servo inner workings](/assets/img/how_it_works/servo07.png){:class="img-fluid w-50"}
>
> Typically a Pulse of:
>
> * `500ms` turns the motor to the `-90` position
> * `1500ms` is the `0` degree or mid-point.
> * `2500ms` turns the motor to `+90`
>
> Though this does depend on the server itself.

---

### Continuous rotation servos

Continuous rotation servos are used for applications such as robotics, and allow for continuous rotation in either direction.

---

### Lightweight servos

Lightweight servos are designed for lightweight applications, such as small robots and model aircraft.

---

### Waterproof servos

Waterproof servos are designed for use in wet environments, and are great for underwater robots.

---

### Brushless servos

Lastly, brushless servos are the most powerful type of servo and are used for high-torque applications such as robotic arms and  heavy-duty RC vehicles.

---

## Powering Servos

Servos typically require more power (current) than a Microcontroller can provide; this means we need to provide additional power supply, rather than trying to power a servo using a pinout from a Microcontroller, which will likely burn out the Microcontroller.

---

## Common problems with Servos

Servo motors are reliable and useful, but can occasionally experience problems due to power supply issues, mechanical damage, incorrect programming, and more.

Common problems that may be encountered include: stalling or jittering, incorrect positioning, poor responsiveness, and overheating.

To help avoid these issues, it is important to use the correct power supply, ensure proper mechanical installation, and program the servo according to the manufacturer's instructions. Additionally, regular maintenance and inspection should be performed to help identify any problems and keep the servo running smoothly.

---

### Overheating and Burnout

If a servo feels how to the touch its likely under too much load and will eventually burnout. Too much load may becaused by too much weight or power needed to maintain the current angle.

---

### Stalling and Jittering

Servos will jitter (with an accompanying whining sound) for a couple of reasons.

* Jittering can be caused by overheating, or simply that the servo is stuggling to achieve the angle specified, which may be because
  * its a cheap servo
  * or there is too much load
* Stalling may be because the servo has overheated and no longer works.

---

### Incorrect Positioning

Servos need to be calibrated to work most effectively. This is because servos are controlled by PWM and that translates into an angle as measured by the resistance of a potentiometer. There are therfore a number of factors that need to be accounted for:

* the exact pulse width required to achieve each angle (typically between 400 and 2500)

---
