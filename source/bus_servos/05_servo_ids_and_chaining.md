---
title: Servo IDs and chaining
description: Give each servo a unique ID one at a time, prove the new ID stuck, scan the bus and daisy chain servos on one cable
layout: lesson
type: page
cover: /learn/bus_servos/assets/banners/05_servo_ids_and_chaining.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Every STS3215 ships as **ID 1**. Plug in two straight from the box and you have two servos answering to the same name, so they'll both obey every command for ID 1 and both reply at once. This lesson fixes that, and it's the one step that makes chaining possible.

---

## The rule: one servo at a time

Changing an ID is a command addressed to a servo's *current* ID. If two servos share that ID, both change. So:

1. Connect **one** servo to the bus. Unplug the others.
2. Change its ID.
3. Prove the new ID works.
4. Unplug it, plug in the next one, and repeat.

Label each servo with its new number using a bit of tape. You'll thank yourself when you build the arm.

## Setting an ID

The library has `set_id(current_id, new_id)`. This program is Kev's `setup_id.py`, tidied so it reports the right servo. His original pings ID 1 in its first line even when `current_id` is something else, which can mislead you. This version pings the ID you actually say it is on.

```python
import time
from sts3215 import STS3215   # sts3215.py from the repo

CURRENT_ID = 1     # the ID the servo has now (new servos are 1)
NEW_ID = 2         # the ID you want it to have

servo = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)

found = servo.ping(CURRENT_ID)
print("servo on", CURRENT_ID, ":", found)

if not found:
    print("Nothing answers on ID", CURRENT_ID, "- check the wiring and the ID")
elif CURRENT_ID == NEW_ID:
    print("The servo already has that ID")
else:
    servo.set_id(current_id=CURRENT_ID, new_id=NEW_ID)
    time.sleep_ms(100)
    print("servo on", NEW_ID, ":", servo.ping(NEW_ID))
    print("servo on", CURRENT_ID, ":", servo.ping(CURRENT_ID))
    print("Now POWER-CYCLE the servo and run check_ids.py to confirm it stuck")
```

You want to see `True` for the new ID and `False` for the old one.

## Prove it stuck

Getting a `True` straight after the change tells you the servo answered once. It doesn't prove the new ID was *saved*. So unplug the servo's power, plug it back in, and ask again. This program scans the bus for every servo that answers, so you can run it again whenever you add a servo.

```python
from sts3215 import STS3215   # sts3215.py from the repo

FIRST_ID = 1
LAST_ID = 20        # raise this if you use higher IDs

servo = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)

print("Scanning IDs", FIRST_ID, "to", LAST_ID, "...")
found = []
for servo_id in range(FIRST_ID, LAST_ID + 1):
    if servo.ping(servo_id):
        found.append(servo_id)
        print("  found servo", servo_id)

print("Servos on the bus:", found)
if len(found) == 0:
    print("Nothing answered. Check power, wiring and jumpers")
```

After a power cycle, the new ID should still show up. If the servo has gone back to ID 1, the change didn't stick, so set it again.

## Chaining

With unique IDs, chaining is just plugging in:

```mermaid
flowchart LR
    BD["Driver board"] --> S1["Servo 1"]
    S1 -->|"second port"| S2["Servo 2"]
    S2 -->|"second port"| S3["Servo 3"]
```

Plug a cable from the **second port** of the first servo into the next servo, and so on. Power and data both run along that one chain, so there's no wire stretching across a moving arm.

Add servos one at a time and run `check_ids.py` after each. If the list shows what you expect, carry on. If a servo is missing, you know exactly which cable to look at.

**Why this matters:** the Buddy arm uses IDs 1 to 6 in a fixed order, and it relies on every servo being found at its own number.

**Common mistakes:**

- Chaining two servos that are both still on ID 1. Neither works reliably until one is renumbered
- Skipping the power cycle, then finding out an hour later that an ID reverted

## Try it Yourself

1. Give three servos the IDs 1, 2 and 3, one at a time, and run the scan after each.
2. Chain all three and scan again. Which IDs appear?
3. Challenge: write a loop that moves each found servo to 2048 in turn, so you can see which physical servo has which ID.

## Common Issues

**Problem**: Nothing moves when I chain two servos.
**Solution**: Both are probably still on ID 1. Connect them one at a time and renumber one.
**Why**: Two servos on one ID both answer, and their replies collide.

**Problem**: The scan finds the servo on the wrong ID.
**Solution**: Check `CURRENT_ID` and `NEW_ID` before running the program, and rerun the scan to see where it ended up.
**Why**: `set_id` trusts what you tell it. It doesn't look first.

---

## Check your understanding

{% quiz %}
? Why must you change IDs with only one servo connected?
- [ ] The servo can only be powered one at a time
- [x] A command to the current ID changes every servo that has that ID
- [ ] The driver board only has one port
> New servos all start on ID 1, so two connected servos would both change.

? How do you confirm a new ID really stuck?
- [ ] Ping it straight after changing it
- [x] Power-cycle the servo and ping the new ID again
- [ ] Check the label on the servo
> A quick reply proves little. Power-cycling shows the new ID survived.

? What does daisy chaining servos need?
- [ ] A separate GPIO pin for each
- [x] A unique ID on each servo and a cable from one servo's second port to the next
- [ ] A separate power supply for each
> The chain carries power and data on one cable, and the IDs decide who answers.
{% endquiz %}
