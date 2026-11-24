---
layout: lesson
title: Testing the IMU, Motors and Controller
author: Kevin McAleer
type: page
cover: /learn/open_duck_mini/assets/banners/09_testing_imu_motors_and_controller.jpg
date: 2026-11-24
previous: 08_setting_up_the_pi.html
next: 10_calibration_and_offsets.html
description: Check each part of the Open Duck Mini on its own before you ask it to
  walk, starting with the IMU, then the servos, then the Xbox controller
percent: 60
duration: 4
quiz_total: 12
date_updated: 2026-11-24
navigation:
- name: Build an Open Duck Mini Walking Robot
- content:
  - section: Meet the duck
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is the Open Duck Mini?
      link: 01_what_is_open_duck_mini.html
  - section: Parts and printing
    content:
    - name: The Parts List
      link: 02_parts_list.html
    - name: Printing the Parts
      link: 03_printing_the_parts.html
    - name: Serial Bus Servos and IDs
      link: 04_bus_servos_and_ids.html
  - section: Assembly
    content:
    - name: Assembling the Legs
      link: 05_assembling_the_legs.html
    - name: Head, Body and Battery
      link: 06_head_body_and_battery.html
    - name: Wiring the Electronics
      link: 07_wiring_the_electronics.html
  - section: Bring it to life
    content:
    - name: Setting up the Pi
      link: 08_setting_up_the_pi.html
    - name: Testing the IMU, Motors and Controller
      link: 09_testing_imu_motors_and_controller.html
    - name: Calibration and Offsets
      link: 10_calibration_and_offsets.html
    - name: First Steps
      link: 11_first_steps.html
  - section: The walking brain
    content:
    - name: How the Duck Learned to Walk
      link: 12_how_the_duck_learned_to_walk.html
    - name: Training Your Own Policy
      link: 13_training_your_own_policy.html
    - name: Troubleshooting
      link: 14_troubleshooting.html
    - name: Summary
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

It's tempting to jump straight to walking. Don't! A walking robot is many parts working at once, and when it fails it's hard to tell which part let it down. So let's test each one on its own first. When all three tests pass, you'll know the hardware is good.

Log in to the Pi, go to the runtime folder and switch on the virtual environment before each session:

```bash
workon open-duck-mini-runtime
cd Open_Duck_Mini_Runtime
```

---

## Test 1: is the IMU on the bus?

First, check the Pi can see the BNO055 on I2C:

```bash
i2cdetect -y 1
```

This prints a grid of I2C addresses. You're looking for the BNO055, which usually shows up at **0x28** (that's the common default address for this sensor). If the grid is empty, go back to the wiring table in [lesson 7](/learn/open_duck_mini/07_wiring_the_electronics.html).

---

## Test 2: raw IMU readings

Now read the IMU directly:

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/raw_imu.py
```

Tilt the duck gently and watch the numbers change. If they move when you move the robot, the IMU is alive.

---

## Test 3: IMU orientation, live

Numbers are hard to picture, so the runtime can show the IMU's orientation on your computer. This needs two terminals.

On the robot:

```bash
python3 scripts/imu_server.py
```

On your computer (with the runtime cloned there too):

```bash
python3 scripts/imu_client.py --ip <robot ip>
```

Tilt the duck forward, back and to each side. The display on your computer should follow. While you're here, make a note of how your IMU is mounted, flipped or not. You'll set `imu_upside_down` to match in the next lesson.

---

## Test 4: the motors

Next, check every servo answers on the bus:

```bash
python3 scripts/check_motors.py
```

Hold the duck safely while this runs, with its legs free to move. If a servo doesn't respond, note its ID. The ID tells you exactly which joint to look at (see [lesson 4](/learn/open_duck_mini/04_bus_servos_and_ids.html)). Because the servos are daisy-chained, if several in a row are missing, look for a loose cable just before the first missing one.

---

## Test 5: the Xbox controller

The controller connects over Bluetooth. Put it into pairing mode, then on the Pi start the Bluetooth tool:

```bash
bluetoothctl
```

Inside `bluetoothctl`, scan for the controller and note its MAC address (it looks like `AA:BB:CC:DD:EE:FF`):

```bash
scan on
```

When it appears, pair, trust and connect, using its MAC address:

```bash
pair <MAC>
trust <MAC>
connect <MAC>
```

`trust` means it will reconnect by itself next time. Type `exit` to leave `bluetoothctl`, then test the controller:

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/xbox_controller.py
```

Press buttons and move the sticks. You should see the values change.

---

## Your test checklist

| Test | Passes when |
|---|---|
| `i2cdetect -y 1` | The BNO055 shows up, usually at 0x28 |
| `raw_imu.py` | The numbers change as you tilt the duck |
| `imu_server.py` and `imu_client.py` | The display follows the duck's tilt |
| `check_motors.py` | All 14 servos answer |
| `xbox_controller.py` | Buttons and sticks show up |
{:class="table table-single"}

---

## Try it Yourself

1. Unplug one servo's daisy-chain cable and run `check_motors.py` again. Which IDs go missing? Plug it back in.
2. With `imu_client.py` running, rotate the duck slowly through a full turn and watch the display.
3. Turn the controller off and on again. Does it reconnect without pairing?

## Common Issues

**Problem**: `i2cdetect` shows nothing.
**Solution**: Check I2C is on in `raspi-config`, your user is in the `i2c` group, and the IMU wiring matches lesson 7.
**Why**: The IMU is only visible once I2C is enabled and wired to pins 3 and 5.

**Problem**: `check_motors.py` can't open the serial port.
**Solution**: Check the servo board's USB cable and that the battery is powering the servos.
**Why**: The Pi needs the board to reach the servos, and the servos need power to answer.

**Problem**: The controller pairs but won't connect next time.
**Solution**: Make sure you ran `trust <MAC>` in `bluetoothctl`.
**Why**: Only trusted devices reconnect by themselves.
