---
title: Reed switches and magnets
description: How a reed switch works, how to wire it to a Pico with the internal pull-up, and your first magnet-detecting program
layout: lesson
type: page
cover: /learn/micropython_projects/assets/banners/03_reed_switches_and_magnets.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Ever wondered how a laptop knows its lid is closed, or how an alarm knows a door has opened? Very often the answer is a **reed switch**: a tiny glass tube that closes when a magnet comes near. This is the first stage of the buddy, the **Sense** stage.

Kev made a whole video on these little switches. Watch it, then come back for the wiring.

{% include youtubeplayer.html id="6SpmwUdcS_g" %}

---

## What is a reed switch?

A reed switch is two thin metal strips (the "reeds") sealed inside a glass tube. With no magnet nearby there is a tiny gap between them, so no electricity flows. Bring a magnet close and the reeds become magnetised, attract each other and touch. The circuit is closed.

Why makers love them:

- **Sealed in glass**, so dust and moisture can't get at the contacts
- **No power needed** by the switch itself
- **Long life**, because there is nothing to press or wear out
- **Cheap**, often under a pound each

## Normally open or normally closed?

| Type | Magnet away | Magnet near | Good for |
|---|---|---|---|
| Normally open (NO) | Circuit open | Circuit closed | Doors, rotation counting, proximity |
| Normally closed (NC) | Circuit closed | Circuit open | Tamper alarms, "has the magnet fallen off?" |
{:class="table table-single"}

We use a **normally open** switch. It is the most common type, and it is what the rest of this course assumes.

## Wiring it to the Pico

A reed switch has two legs and no polarity, so either way round works. Connect:

```
Reed switch, one leg    ->  GP15 (physical pin 20)
Reed switch, other leg  ->  GND  (physical pin 23)
```

There is no resistor to add. The Pico has built-in pull-up resistors that you turn on in code with `Pin.PULL_UP`. If pull-ups are new to you, [Digital I/O](/learn/micropython_gpio/) in the GPIO Mastery course explains them.

Here is what the pull-up does:

- Switch **open** (no magnet): the pull-up holds GP15 at 3.3V, so the pin reads **1**
- Switch **closed** (magnet near): GP15 is connected straight to GND, so the pin reads **0**

So the logic looks backwards. **0 means the magnet is there.** This is called *active low*, and it is the normal way to read a switch.

## Your first program

Save this on the Pico and run it. Move the magnet towards the switch and away again.

```python
from machine import Pin
import time

# GP15 with the internal pull-up switched on
reed = Pin(15, Pin.IN, Pin.PULL_UP)

print("Reed switch test")
print("Bring a magnet close to the switch...")

while True:
    if reed.value() == 0:
        print("Magnet detected")
    else:
        print("No magnet")
    time.sleep(0.2)   # look five times a second
```

**What you should see:** "No magnet" scrolling past, then "Magnet detected" while the magnet is close, then back again.

**Why this matters:** you have just proved the whole Sense stage works. Everything else in the course reacts to this one value.

## Try it Yourself

1. Change the `0.2` to `1`. What happens if you wave the magnet quickly past the switch?
2. Find how far away the magnet can be before it triggers. Is it the same from every side?
3. Challenge: change the program to print only when the state *changes*, not on every look.

## Common Issues

**Problem**: It never says "Magnet detected".

**Solution**: Check the magnet is neodymium (shiny silver, not black ceramic) and try it within 5mm of the glass. Check the wiring goes to GP15 and GND.

**Why**: Weak magnets, or a loose jumper wire, are by far the most common cause.

**Problem**: It reads the opposite way round.

**Solution**: You may have a normally closed switch. Either swap it for a normally open one, or flip the logic in your code.

**Why**: A normally closed switch is closed with no magnet, so it reads 0 when the magnet is away.

## Check your understanding

{% quiz %}
? With a normally open reed switch and `Pin.PULL_UP`, what does `reed.value()` return when the magnet is near?
- [ ] 1
- [x] 0
- [ ] True
> The magnet closes the switch, which connects the pin to GND, and the pin reads 0.

? What does `Pin.PULL_UP` do for the reed switch?
- [ ] It makes the switch more sensitive to magnets
- [x] It holds the pin at 3.3V when the switch is open, so it never floats
- [ ] It powers the switch
> Without a pull-up the pin would drift between 0 and 1 when the switch is open and give random readings.
{% endquiz %}
