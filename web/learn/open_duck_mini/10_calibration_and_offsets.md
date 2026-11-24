---
layout: lesson
title: Calibration and Offsets
author: Kevin McAleer
type: page
cover: /learn/open_duck_mini/assets/banners/10_calibration_and_offsets.jpg
date: 2026-11-24
previous: 09_testing_imu_motors_and_controller.html
next: 11_first_steps.html
description: Calibrate the BNO055 IMU, create duck_config.json, set imu_upside_down,
  find the joint offsets with find_soft_offsets.py and stand the duck up with turn_on.py
percent: 66
duration: 6
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

Your duck's parts all work. Now we make them agree with each other. The policy was trained on a perfect simulated duck, with a level IMU and every joint exactly at zero. Your real duck is a little different, and this lesson measures those differences so the software can correct them.

---

## Step 1: calibrate the IMU

The BNO055 calibrates itself as it moves. We help it along by moving the robot through lots of orientations, then save the result so it doesn't have to start from scratch every time.

```bash
cd scripts/
python calibrate_imu.py
```

Slowly rotate the duck in every direction: tilt it forward and back, roll it side to side, turn it around and hold it still in a few positions. The script reports four calibration levels. Keep going until it shows **[3,3,3,3]** and **Calibrated = True**.

The script saves a file called `imu_calib_data.pkl`. Copy it into the runtime package folder, `mini_bdx_runtime/mini_bdx_runtime/`. From the `scripts` folder:

```bash
cp imu_calib_data.pkl ../mini_bdx_runtime/mini_bdx_runtime/
```

---

## Step 2: create your config file

Your duck's personal settings live in a JSON file in your home folder. Start from the example in the runtime repo. Go back up to the `Open_Duck_Mini_Runtime` folder (`cd ..` from `scripts`), then:

```bash
cp example_config.json ~/duck_config.json
```

Open it with `nano ~/duck_config.json`. It holds three kinds of setting:

| Setting | What it does |
|---|---|
| Expression feature flags | Turn the eye LEDs, projector, speaker, antennas and ears on or off. Switch off anything you didn't fit |
| `imu_upside_down` | Tells the runtime how your IMU is mounted |
| Joint offsets | Small corrections for each joint, which you'll find in step 3 |
{:class="table table-single"}

### imu_upside_down

The assembly guide prefers the IMU **flipped along its X axis**. If you mounted it that way, set `imu_upside_down` to match. If you mounted it the other way up, set it the other way. Check `example_config.json` and the runtime README for exactly how the setting is written.

This one setting matters a lot. **With the wrong `imu_upside_down` setting, the gait oscillates badly.** The policy thinks the body is tilting the wrong way, corrects for it, and makes things worse. If your duck ever wobbles wildly, check this first.

---

## Step 3: find the soft offsets

When you fitted the servo horns in lesson 4, they landed close to zero, but not exactly. The offsets fix that in software.

First, **hang the robot so both feet are off the ground**. A loop of string around the body, or a stand, both work. Then, from the `Open_Duck_Mini_Runtime` folder:

```bash
cd scripts/
python find_soft_offsets.py
```

The script unlocks one motor at a time. You move that joint by hand into the right position, and it locks it again and records the offset. Work through every joint.

### What a good pose looks like

| Check | Good |
|---|---|
| Chassis | Level, or the nose slightly up |
| Legs and feet, seen from the side | Symmetric, left matching right |
| Foot switches | On a flat table, both click at the **same time** |
| Head | Level, or slightly up |
{:class="table table-single"}

The foot switch check is a great test. Lower the duck onto a flat table. If one foot clicks before the other, one leg is longer than the other, and you need to adjust its offsets.

Put the offsets the script finds into the joint offsets in `~/duck_config.json`, if the script doesn't write them for you. Check the runtime README for how your version handles it.

---

## Step 4: stand up

Now for a proud moment. From the runtime repo, run:

```bash
python turn_on.py
```

This puts the duck into its **standing pose with torque on**. All 14 servos hold their positions. Look at it from the front and the side. Does it stand straight? Does it look like the good pose in the table above? If not, go back to step 3 and adjust.

If `turn_on.py` isn't where you are, check the runtime README for its location on the `v2` branch.

---

## Try it Yourself

1. Lower the duck onto a flat table and listen for the foot switches. Do both click together?
2. Switch off every expression feature you didn't fit in `~/duck_config.json`.
3. Challenge: put a small spirit level, or a level app on your phone, on the duck's back while it stands. How level is it?

## Common Issues

**Problem**: The IMU calibration never reaches [3,3,3,3].
**Solution**: Move more slowly, and include some still holds in different positions.
**Why**: Each part of the sensor calibrates from a different kind of movement.

**Problem**: The duck leans to one side when standing.
**Solution**: Run `find_soft_offsets.py` again and check the legs are symmetric from the side.
**Why**: A small offset error in a hip or ankle shows up as a lean.

**Problem**: One foot switch clicks long before the other.
**Solution**: Adjust the offsets of that leg until both click together.
**Why**: The policy uses the foot switches to know when each foot is down, so they need to agree.

## Check your understanding

{% quiz %}
? What does the IMU calibration script need to show before you save the data?
- [ ] [0,0,0,0] and Calibrated = False
- [x] [3,3,3,3] and Calibrated = True
- [ ] A reading of 0x28
> 3 is the top calibration level for each part of the sensor. 0x28 is the I2C address, not a calibration level.

? Your duck's gait oscillates badly. What should you check first?
- [x] The imu_upside_down setting in duck_config.json
- [ ] The Wi-Fi password
- [ ] The speaker volume
> If the runtime thinks the IMU is the other way up, the policy corrects tilts the wrong way and the gait oscillates.

? How should the robot be set up while you run find_soft_offsets.py?
- [ ] Standing on the floor
- [x] Hanging so both feet are off the ground
- [ ] Lying on its back
> With the feet off the ground, each joint can move freely while you set its position.
{% endquiz %}
