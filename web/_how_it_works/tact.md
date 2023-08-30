---
layout: how_it_works
title: Tact Switches
short_title: How do Tact Switches work?
short_description: Learn about Tact Switches
date: 2023-06-06
author: Kevin McAleer
excerpt:
cover: /assets/img/how_it_works/tact.png
tags:
 - Tact Switch
 - Switch
 - Momentary Switch
 - Push Button
 - Button
 - How it works
---

`Tact` (`Tactile`) buttons, also known as momentary switches are a type of electrical switch that is only on when it is being pressed; they return to their default off position when the pressure is released.

Momentary switches are ubiquitous and are found in various applications, ranging from home electronics to industrial machinery. Buttons on your television remote, keys on your keyboard, and push buttons on a game controller are all examples of momentary switches.

---

## How do Momentary Switches Work?

When the switch is pressed, it creates a path for current to flow. When the switch is released, this path is broken, and the current flow stops. Essentially, the switch has two states:

- **Normally open (NO)**: The switch is open (`off`) when not pressed, meaning no current flows through.
- **Normally closed (NC)**: The switch is closed (`on`) when not pressed, meaning current flows through.

## Anatomy of a button

[![Tact Switch exploded view](/assets/img/how_it_works/tact04.jpg){:class="img-fluid w-100"}](/assets/img/how_it_works/tact04.jpg)

1. **The top cover** is a protective layer for the internal mechanism of the switch. It can be made of metal or other materials depending on its intended function. Some covers also include a ground terminal to safeguard against static discharge.

1. Situated beneath the cover and on top of the contact dome, **the plunger** is the part that users press to flex the dome and activate the switch. Plungers can be either flat or raised.

1. **The contact dome**, which has an arched shape, fits into the base of the switch. When the plunger makes contact with it, the dome flexes or changes shape, creating an audible click and tactile feedback. This action connects two fixed contacts in the base, completing the circuit. When the force is released, the contact dome returns to its original shape, breaking the circuit. The choice of material (such as metal or rubber) for both the contact dome and plunger affects the tactile feel and sound of the switch.

1. The molded resin **base** serves as the housing for the switch and contains the terminals and contacts that connect it to the PCB (printed circuit board).

---

## Wiring Momentary / Tack Switches

Here's a simple way to wire a momentary switch:

1. Connect one terminal of the switch to the positive terminal (`Vcc`) of the power supply.
2. Connect the other terminal to the input pin of your microcontroller, let's say a Raspberry Pi.
3. Connect a resistor (typically `10k Ohm`) from the input pin to ground (`GND`). This resistor is called a 'pull-down resistor.'

When the switch is not pressed, the input pin is connected to GND through the resistor, so the microcontroller reads a LOW signal. When the switch is pressed, the input pin is directly connected to Vcc, so the microcontroller reads a HIGH signal.

---

## Debouncing a Momentary / Tack Switch

Switch `debouncing` is a technique used to avoid the "bouncing" effect when a switch changes state. When a switch is toggled, it doesn't immediately change state. Instead, it quickly oscillates between states before settling down, known as "bouncing."

This bouncing can create multiple transitions for a single button press, which could cause issues in many applications, such as registering multiple clicks instead of one.

To overcome this, a process called 'debouncing' is employed, either via hardware (using components like resistors, capacitors, or Schmitt triggers) or software (using programming).

## Debouncing Example with Python

Let's consider a simple example of debouncing using Python on a Raspberry Pi.

First, we need to set up the GPIO pin and initialize the button:

```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
button_pin = 18
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
```

Now, let's implement a debounce function:

```python
def debounce(pin):
    button_state = GPIO.input(pin)
    previous_state = button_state

    while True:
        button_state = GPIO.input(pin)
        if button_state != previous_state:
            previous_state = button_state
            time.sleep(0.05)  # Debounce for 50 milliseconds
            if button_state != GPIO.input(pin):
                continue
        yield button_state
```

Finally, use the debounce generator in your main loop:

```python
for button_state in debounce(button_pin):
    if button_state:
        print("Button pressed!")
    else:
        print("Button released!")
```

In the code above, the function `debounce` generates the button states. If the button state changes, it waits 50 milliseconds before checking the state again. If the state is still the same after the delay, the new state is considered valid and is yielded to.

---
