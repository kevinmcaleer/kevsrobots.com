---
title: Summary and What's Next
description: Review Snakie's instruments, the helpers that feed them and when to use each one, then choose where to go next
layout: lesson
type: page
cover: /learn/snakie_instruments/assets/banners/10_summary.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You can now see inside a running program. That changes how you build things: fewer guesses, more measurements.

---

## What you learned

| Instrument | Best for | Feed it with |
|---|---|---|
| **Plotter** | Values that change over time, side by side | `inst.plot(name=value)` |
| **Multimeter** | One voltage, with min, max and average | `inst.read_adc(adc)` or `inst.meter(value)` |
| **Oscilloscope** | PWM frequency and duty, signal shape | `inst.read_pwm(pwm)` or `inst.scope(value)` |
| **Data Logger** | Keeping a record of a whole run | Nothing extra. Press REC |
{:class="table table-single"}

And the ideas underneath:

- **Telemetry** is ordinary printed text starting with `SNK`. Your program keeps running while it reports
- **LIVE** lets Snakie poll the board with no code, but interrupts a running program
- **watch()** and **update()** report several objects with one line in the loop
- Every helper has a **block** in the Instruments drawer
- A **file on the board** logs data when no computer is listening, and Snakie opens CSV files as tables

## Choosing an instrument

Ask what kind of question you have.

| Your question | Reach for |
|---|---|
| "What is it right now?" | Multimeter |
| "How is it changing?" | Plotter |
| "Is the signal the right shape?" | Oscilloscope |
| "What happened during that run?" | Data Logger or a CSV file |
| "Did it ever dip?" | Multimeter MIN, or the Plotter |
{:class="table table-single"}

## A debugging habit

Next time something misbehaves, try this before changing any code:

1. **Say what you expect.** "The servo should get a 50 Hz pulse."
2. **Measure it.** Open the instrument that answers that question.
3. **Compare.** If the measurement matches, the problem is somewhere else. If not, you've found it.

It feels slower than guessing. It is nearly always faster.

## The rest of the dock

We covered four instruments. The dock has many more, and they all work the same way: open it, click **?**, copy the snippet. **I²C detect** scans for connected chips, **Range** draws a distance sensor's readings as a radar, **IMU** shows a tilt sensor in 3D, and the output panels let you drive a servo or motor by hand.

## What's next

- [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/): plan and check the circuits you're measuring, and build an obstacle-avoiding robot to try your instruments on
- [Snakie Blocks - From Blocks to Python](/learn/snakie_blocks/): if the Instruments drawer was your first look at Blocks
- [Learn MicroPython - The basics](/learn/micropython/): more Python for the programs behind the readings
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/): PWM, ADC and the rest of the Pico's pins in depth

## Final check

{% quiz %}
? Your robot's battery might be sagging when the motors start. Which reading tells you?
- [ ] The Oscilloscope's FREQ
- [x] The Multimeter's MIN
- [ ] The Plotter's legend
> MIN remembers the lowest reading, however briefly it happened.

? Which two helpers return a value your program can use?
- [ ] inst.plot() and inst.scope()
- [ ] inst.watch() and inst.update()
- [x] inst.read_adc() and inst.read_pwm()
> read_adc() returns volts and read_pwm() returns the duty, so measuring costs no extra lines.

? What's the first step of the debugging habit?
- [x] Say what you expect to see
- [ ] Change the code
- [ ] Swap the board
> A measurement only means something when you have an expectation to compare it with.
{% endquiz %}
