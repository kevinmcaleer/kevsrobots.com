---
title: Summary and Next Steps
description: Review what you learned about planning, wiring and checking circuits in Snakie's Electronics workspace, and choose what to learn next
layout: lesson
type: page
cover: /learn/snakie_electronics/assets/banners/11_summary.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to recall the order of work for any circuit project and choose where to go next.

You started with a single board on a blue canvas. You're finishing with a wired, checked, documented and programmed robot. Nice work.

---

## What you learned

| Lesson | The big idea |
|---|---|
| The workspace | A canvas for parts, a Browser for the project and a Library to draw from |
| The Parts Library | Every part knows its own pins. Read the pinout before you wire |
| Placing parts | Arrange first. Short wires that don't cross are easy to follow |
| Wiring | Power, ground and signal. And never forget the shared ground |
| The Schematic | The same circuit, drawn to show what connects to what |
| The rules check | Make your mistakes on screen, where they cost nothing |
| Drivers and help | Pin numbers in code come from the wiring diagram |
| Connections and measurements | The table is the truth. Check your build against it |
| Exporting | A build guide with a tick-box per wire beats a photo |
| The robot | Plan, wire, check, document, code, build |
{:class="table table-single"}

## The habit to keep

If you take one thing from this course, make it the order of work:

1. **Wire it on screen**
2. **Check it** with the ERC and the Connections table
3. **Write the code** with pin numbers taken from the diagram
4. **Then** pick up the real parts

It feels slower. It's much faster, because you only build the circuit once.

## What's next

**Carry on with Snakie**

- [Snakie Instruments](/learn/snakie_instruments/) is next in the Start with Snakie pathway. Watch your robot's sensor readings on a live plotter, multimeter and oscilloscope
- [Snakie Blocks](/learn/snakie_blocks/), if you'd like to program the robot by snapping blocks together

**Go deeper into robots**

- [MicroPython Robotics Projects](/learn/micropython_robotics/) takes the Pico further into robot building
- [Robotics 101](/learn/robotics_101/) covers the ideas behind every robot
- [BurgerBot](/learn/burgerbot/) is a complete two-wheeled robot you can 3D print

**Learn the language properly**

- [Learn MicroPython - The basics](/learn/micropython/)

---

## Try it Yourself

One last challenge. Design a circuit of your own from scratch: pick a sensor you haven't used from the Library, read its help page, wire it to a Pico, check it, and export the build guide. You now know every step.

## Check your understanding

{% quiz %}
? What's the right order of work?
- [ ] Build, then wire on screen, then check
- [x] Wire on screen, check, code, then build
- [ ] Code first and hope
> Planning and checking on screen means you build the real circuit once.

? Which wire do beginners most often forget?
- [ ] The Trigger wire
- [ ] The USB cable
- [x] The shared ground
> Every part has to agree on where ground is, or signals mean nothing.

? Which view is best for understanding what connects to what?
- [x] The Schematic
- [ ] The Parts Catalog
- [ ] The console
> A schematic drops the real shapes and keeps only the connections.
{% endquiz %}
