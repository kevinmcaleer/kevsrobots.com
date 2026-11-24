---
layout: lesson
title: Summary and Next Steps
author: Kevin McAleer
type: page
cover: /learn/snakie_part_editor/assets/banners/13_summary.jpg
date: 2026-11-21
previous: 12_sharing_your_parts.html
description: Review everything you built in the Snakie Part Editor course and take
  on a challenge part of your own
percent: 100
duration: 4
quiz_total: 41
date_updated: 2026-11-24
navigation:
- name: Make Your Own Parts and Boards in Snakie
- content:
  - section: Overview
    content:
    - name: Make Your Own Parts and Boards in Snakie
      link: 00_intro.html
    - name: What a Part Is
      link: 01_what_a_part_is.html
  - section: Drawing the Part
    content:
    - name: A Tour of the Part Editor
      link: 02_tour_of_the_part_editor.html
    - name: Board Size and Details
      link: 03_board_size_and_details.html
    - name: Pins, Types and Capabilities
      link: 04_pins_types_and_capabilities.html
    - name: Holes, Components and Labels
      link: 05_holes_components_and_labels.html
    - name: Tracing a Board Photo
      link: 06_tracing_a_board_photo.html
  - section: Making It Smart
    content:
    - name: Schematic Symbol and Electrical Details
      link: 07_schematic_symbol_and_electrical_details.html
    - name: Drivers and Built-In Help
      link: 08_drivers_and_built_in_help.html
    - name: Using Your Part in a Project
      link: 09_using_your_part_in_a_project.html
    - name: Giving Your Part Its Own Blocks
      link: 10_giving_your_part_its_own_blocks.html
  - section: Boards and Sharing
    content:
    - name: Making a Microcontroller Board
      link: 11_making_a_microcontroller_board.html
    - name: Sharing Your Parts
      link: 12_sharing_your_parts.html
    - name: Summary and Next Steps
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to review everything a finished part contains and pick a part of your own to make next.

You started with a board Snakie had never heard of. You finish with a part that draws itself, wires itself, checks its own voltage, installs its own driver and brings its own blocks. Nice work.

---

## What you built

| Lesson | What you added |
|---|---|
| Board size and details | A 25 by 13 mm board with a name, family and tags |
| Pins | Six pins with types, capabilities and I2C signals |
| Holes, components and labels | The physical look of the board |
| Tracing a photo | A real picture, cleaned up and scaled |
| Schematic and electrical | A symbol, an I2C address and supply limits |
| Drivers and help | One-click driver install and offline help |
| Using your part | A wired project and derived blocks |
| Own blocks | A `blocks.yml` with blocks that read like a sentence |
| Microcontroller board | A custom board in the board selector |
| Sharing | A library, versions and the registry |
{:class="table table-single"}

---

## The finished part

Here's everything in the folder, and which lesson made it:

```
vl53l0x-distance-sensor/
  parts.yml      lessons 3 to 8
  image.png      lesson 6
  help.md        lesson 8
  blocks.yml     lesson 10
```

And here's the program your blocks write, with a loop added so the robot keeps looking. As before, a stand-in sensor lets it run on this page. Press **Run**.

{% capture snakie_code %}
import time

try:
    from vl53l0x import VL53L0X
    from machine import I2C, Pin
    tof = VL53L0X(I2C(0, sda=Pin(4), scl=Pin(5)))
except ImportError:
    class FakeSensor:
        readings = [620, 480, 350, 240, 150, 90, 180, 400]

        def __init__(self):
            self.index = 0

        def ping(self):
            mm = self.readings[self.index % len(self.readings)]
            self.index = self.index + 1
            return mm

    tof = FakeSensor()

SAFE_DISTANCE = 200

for step in range(8):
    mm = tof.ping()
    if mm < SAFE_DISTANCE:
        print(mm, "mm  Obstacle! Turning")
    else:
        print(mm, "mm  Clear, driving on")
    time.sleep(0.2)
{% endcapture %}
{% include snakie.html code=snakie_code name="avoid.py" view="code" %}

---

## Your challenge

Pick a board from your parts drawer that Snakie doesn't have, and make it. Good first choices:

- An **OLED display** breakout. Four pins, and the SSD1306 driver is in the modules catalog.
- A **line sensor** module. Three pins and no driver, so it's quick.
- A **motor driver** with screw terminals. A chance to use terminal blocks and rails.
- Your favourite **microcontroller board** that isn't in the selector yet.

Work through the checklist from the last lesson. When it's good, share it.

---

## What's next

- [Coding Faster in Snakie](/learn/snakie_code_tools/): the editor tools that go with the parts you make
- [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/): build guides and circuit checks for your projects
- [Snakie Instruments](/learn/snakie_instruments/): plot the readings from your new sensor
- [Learn BetterCad](/learn/bettercad/): design a 3-D model for your part
- [Kev's YouTube channel](https://www.youtube.com/c/kevinmcaleer28): new robots and Snakie features every week

---

## Check your understanding

{% quiz %}
? Which single file is the part itself?
- [x] parts.yml
- [ ] help.md
- [ ] library.yml
> Everything else in the folder is an optional extra beside it.

? What should you set first when starting a new part?
- [ ] The blocks
- [ ] The driver
- [x] The board's real size
> Everything is positioned as a fraction of the board, so the size comes first.

? Which of these makes a part easier for other people to use?
- [x] Bundled help with wiring and a short working example
- [ ] Leaving the supply limits empty
- [ ] Renaming the pins in every version
> Help, limits and stable pin names are what make a part a pleasure to pick up.
{% endquiz %}
