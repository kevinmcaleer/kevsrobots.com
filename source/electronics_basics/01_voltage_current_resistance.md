---
title: "Voltage, Current and Resistance"
description: "The three ideas behind every circuit, explained with a water pipe and a tiny LED example."
layout: lesson
type: page
cover: /learn/electronics_basics/assets/banners/01_voltage_current_resistance.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain voltage, current and resistance in your own words, and say which one a given measurement is.

## Overview

Three words come up in every electronics conversation: voltage, current and resistance. They sound abstract, but each one answers a simple question about your robot. In this lesson we meet all three, and by the end you will know what each one feels like on a real circuit, starting with the first step of our night light: a battery lighting an LED.

## The water pipe picture

Imagine a tank of water on a hill, connected to a pipe that runs downhill to a water wheel.

- **Voltage** is the height of the hill. It is the push that wants to move things along. Measured in **volts (V)**.
- **Current** is how much water flows through the pipe each second. Measured in **amps (A)**. Robots mostly use small currents, so you will see **milliamps (mA)**, where 1000 mA is 1 A.
- **Resistance** is how narrow the pipe is. A narrow pipe holds the flow back. Measured in **ohms**, written with the Greek letter omega, but we will just write "ohm", "k" for thousands and "M" for millions. So 4k7 or 4.7k means 4700 ohms.

A bigger hill (more voltage) pushes more water. A narrower pipe (more resistance) lets less through. That single relationship is Ohm's law, and we meet it properly in the next lesson.

The three ideas meet in one loop. Follow it from the battery round to the battery:

```mermaid
flowchart LR
    B["Battery positive<br>voltage pushes"] --> R["Resistor<br>resistance holds back"]
    R --> L["LED<br>lights up"]
    L --> N["Battery negative<br>0 V, ground"]
    N -.->|"current flows round the loop"| B
```

## Voltage

Voltage is always measured **between two points**. Saying "this wire is 3.3 volts" really means "this wire is 3.3 volts above ground". Ground (GND, 0 V) is the shared reference point that everything else is measured against.

Voltages you will meet in robotics:

- 1.5 V: one AA battery
- 3.3 V: the Pico's logic level and its 3V3 pin
- 5 V: USB power, and the Pico's VBUS pin
- 4.5 V: three AA cells in a pack, our night light's first power source

## Current

Current is the flow of electric charge around a loop. It leaves the positive terminal of a battery, travels through the parts, and returns to the negative terminal. If the loop is broken anywhere, say a wire comes out, the current stops everywhere. That is why we call it a **circuit**: a circle.

A typical indicator LED uses about 10 mA. A small hobby motor might draw 200 mA running and over 1 A when stalled. That difference is a big deal later in the course.

## Resistance

Every component resists current a little. A **resistor** is a part made to resist on purpose, so you can control how much current flows. Our night light will use resistors to protect the LED, to build a light sensor, and to switch a transistor.

## Seeing it in numbers

Here is a tiny script that lists the current our first circuit would draw. We have not met the formula yet, so just run it and read the output. It will make sense in the next lesson.

```python
# Run on your computer or the Pico
# Current through a resistor when a battery pack is connected across it
battery_volts = 4.5       # three AA cells

for resistance in (100, 330, 1000, 10000):    # ohms
    amps = battery_volts / resistance
    print(f"{resistance:>6} ohm -> {amps * 1000:.1f} mA")
```

Output:

```text
   100 ohm -> 45.0 mA
   330 ohm -> 13.6 mA
  1000 ohm -> 4.5 mA
 10000 ohm -> 0.5 mA
```

Same voltage, but more resistance means less current. That is the pipe getting narrower.

**Why this matters:** an LED lets almost any current through once it is on. Without a resistor, it would take as much as the battery could give, and burn out in a flash. Resistance is how we keep parts alive.

## Try it Yourself

1. Change `battery_volts` to 3.3 (the Pico's 3V3 pin) and rerun. How do the currents change?
2. Add 220 to the list of resistances. Which one lands closest to 20 mA?
3. Challenge: work out which resistor from the list would let 1 mA flow from 4.5 V, without running the script.

## Common Issues

- **Problem**: "Voltage flows through the wire"
- **Solution**: Say "current flows, voltage is across".
- **Why**: Voltage is the push between two points. Current is the thing that moves. Mixing them up makes later lessons confusing, especially when measuring with a multimeter.

- **Problem**: "My circuit has no current, but the battery is fine"
- **Solution**: Look for a break in the loop: a loose wire, a part not pushed in, or a gap in the breadboard.
- **Why**: Current needs a complete loop from the positive side back to the negative side.

## Check your understanding

{% quiz %}
? Which statement is correct?
- [x] Current flows through a wire, voltage is across two points
- [ ] Voltage flows through a wire, current is across two points
- [ ] Voltage and current mean the same thing
> Voltage is the push between two points. Current is the flow that moves round the loop.

? You raise the resistance in a circuit and keep the same battery. What happens to the current?
- [ ] It goes up
- [x] It goes down
- [ ] It stays the same
> More resistance is a narrower pipe, so less current flows. The script showed 45 mA at 100 ohm and 0.5 mA at 10k.

? What does 4k7 mean?
- [x] 4700 ohm
- [ ] 47 ohm
- [ ] 4.7 ohm
> The k stands for thousands, so 4k7 and 4.7k both mean 4700 ohm.
{% endquiz %}
