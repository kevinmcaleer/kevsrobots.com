---
layout: lesson
title: First Steps
author: Kevin McAleer
type: page
cover: /learn/open_duck_mini/assets/banners/11_first_steps.jpg
date: 2026-11-24
previous: 10_calibration_and_offsets.html
next: 12_how_the_duck_learned_to_walk.html
description: Run the pretrained walking policy on your Open Duck Mini, drive it with
  the Xbox controller, and keep it and you safe while it learns to share your floor
percent: 72
duration: 5
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

This is the moment the whole course has been building to. Your duck is built, tested and calibrated. Let's make it walk.

---

## Safety first

A walking robot can fall, and a falling robot can break. Before every walking session:

- **Clear the floor.** Give the duck a few metres of open, flat space, away from stairs, table edges, cables and pets.
- **Be ready to catch it.** Stay close, with your hands near it, especially the first few times. A fall can snap a printed part or strain a servo.
- **Keep the controller in your hand.** You'll want to pause it quickly.
- **Leave Y (head control) off.** More on that below.

---

## Run the checklist

The runtime repo has a `checklist.md`. Open it and work through it before your first walk. It's there to catch the things that are easy to forget, and it's kept up to date with the code.

---

## Start the walking policy

The runtime repo comes with a pretrained policy, `BEST_WALK_ONNX_2.onnx`. This is the walking brain, already trained in simulation. The walk command below expects it in your home folder, so find it in the repo and copy it there:

```bash
find ~/Open_Duck_Mini_Runtime -name BEST_WALK_ONNX_2.onnx
cp <path printed above> ~/
```

Then, from the `Open_Duck_Mini_Runtime` folder:

```bash
cd scripts/
python v2_rl_walk_mujoco.py --duck_config_path ~/duck_config.json --onnx_model_path ~/BEST_WALK_ONNX_2.onnx
```

There are two important parts to that command:

| Option | What it points to |
|---|---|
| `--duck_config_path` | Your config from lesson 10, with your offsets and `imu_upside_down` setting |
| `--onnx_model_path` | The policy file the duck will run |
{:class="table table-single"}

So the same script will run any policy you give it. In lesson 13 you'll point it at one you trained yourself.

Once it starts, the runtime loops many times a second: read the servos, IMU and foot switches, ask the policy what to do, and send new positions to the servos.

---

## The controller map

| Control | What it does |
|---|---|
| Left stick | Walk |
| A | Pause or unpause |
| Hold LB | Walk faster |
| X | Toggle the projector |
| B | Play a random sound |
| LT and RT | Move the antennas |
| Y | Head control. Very experimental. Leave it off |
{:class="table table-single"}

### About the Y button

Y turns on head control. It's **very experimental**, and it can break the head. Leave it off, at least until your duck is walking well and you're happy to take the risk.

---

## Your first walk

1. Stand the duck on the cleared floor, with your hand close.
2. Start the script and wait for it to settle.
3. Push the left stick forward **gently**. The duck should start stepping.
4. Let go of the stick and watch what it does.
5. Try the other directions on the left stick, gently at first.
6. Only once it's steady, try holding LB for faster walking.

If it looks wrong, press **A** to pause it, and catch it if you need to.

Congratulations! You've built a robot that walks using a neural network trained in a simulator. That's the same idea used in some of the most advanced walking robots in the world, and it's walking on your floor.

---

## Watching it walk

Now that it walks, watch it closely. These are clues to what to fix:

| What you see | What it usually means |
|---|---|
| Wobbling side to side that gets worse | Check `imu_upside_down` |
| Drifting or leaning one way | Check your offsets |
| One foot stamping harder than the other | The foot switches may not be in sync |
| Jerky, laggy steps | Check the FTDI latency rule from lesson 8 |
{:class="table table-single"}

Lesson 14 goes through each of these in detail.

---

## Try it Yourself

1. Walk the duck to a spot across the room and back.
2. Try it on a different floor, such as carpet versus a hard floor. Does it walk differently?
3. Turn on one expression feature in your config, such as the eye LEDs, and try X and B.

## Common Issues

**Problem**: The script stops with a file not found error.
**Solution**: Check both paths in the command. `~/duck_config.json` and `~/BEST_WALK_ONNX_2.onnx` must exist in your home folder.
**Why**: The script needs both your config and a policy file to run.

**Problem**: The duck falls straight over.
**Solution**: Pause it, then check your offsets and `imu_upside_down` in lesson 10.
**Why**: The policy expects a duck that matches the simulated one. Big differences make it fall.

**Problem**: The controller doesn't do anything.
**Solution**: Run `xbox_controller.py` again, as in lesson 9, to check it's connected.
**Why**: The walk script can only follow a controller the Pi can see.
