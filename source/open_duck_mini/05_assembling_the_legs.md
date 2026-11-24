---
title: Assembling the Legs
description: Build the trunk, feet, shins, thighs and hips of the Open Duck Mini in the order the assembly guide uses, and learn when to use Loctite and when not to
layout: lesson
type: page
cover: /learn/open_duck_mini/assets/banners/05_assembling_the_legs.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Your parts are printed and your servos all have IDs. Now the duck starts to look like a duck. In this lesson we'll build the trunk and both legs, working from the ground up.

---

## Before you start

Open [`docs/assembly_guide.md`](https://github.com/apirrone/Open_Duck_Mini) in the Open_Duck_Mini repo alongside this lesson. It has the pictures and the exact parts for each step. It's marked as incomplete, so where it's thin, this lesson fills in the reasoning, and the Open Duck community can help with the rest.

Get your labelled servos out and keep them in their piles. Every servo in a leg must be the one with the right ID for that joint (see the table in [lesson 4](/learn/open_duck_mini/04_bus_servos_and_ids.html)).

---

## The Loctite rule

You'll be using Loctite 243 thread locker, and there's one rule to remember:

- **Use Loctite only on metal-to-metal screws**, the screws that go into the motors.
- **Never use it on screws going into plastic.**

Why? A motor screw going into metal can shake loose as the duck walks, and Loctite stops that. But thread locker can attack and crack some plastics, and screws into plastic or heat-set inserts don't need it anyway.

---

## Step 1: the trunk

The trunk is the duck's hips and backbone. Everything else hangs off it.

1. Press the bearings into `trunk_bottom`.
2. Fit the M3 heat-set inserts.
3. Join `trunk_bottom` and `trunk_top` with two M3x10 screws.
4. Mount the middle motor.
5. Insert `roll_motor_bottom`.

---

## Step 2: the feet

1. Join `foot_bottom_tpu` and `foot_bottom_pla` with two M3x6 screws. This gives you the soft sole on a rigid base.
2. Fit the heat-set inserts into `foot_top`.
3. Mount the foot motor so its **driver side faces `foot_top`**, as shown in the guide.
4. Optional, but recommended: fit the foot switches. They press-fit, so they trigger when the foot touches the ground.

The foot switches tell the policy when each foot is down. We'll check they click together on a flat table in lesson 10.

---

## Step 3: the shins

1. Fit four heat-set inserts into each `leg_spacer`.
2. **Plug the foot motor's cable in first**, then route it up the shin.

That second point is easy to miss. Once the shin is closed up, getting to that connector is painful, so plug it in while you can still see it.

---

## Step 4: the thighs

Take care with the orientation of the **hip_pitch** motor here. Its orientation decides where the joint's zero position ends up. Mount it the wrong way round and the leg's zero pose will be wrong, which no offset can fix. Check the picture in the assembly guide before you tighten anything.

---

## Step 5: the hips

1. Pick the right `roll_to_pitch` part. There's a `left_roll_to_pitch` and a `right_roll_to_pitch`. They're mirror images, so they look almost the same. Use the one for the leg you're building.
2. Fit `roll_motor_top` onto the hip_yaw motor from below.
3. Then fit the hip_roll motor.
4. Mount the finished leg onto the trunk.

Repeat for the other leg, then stand back and admire a pair of duck legs.

---

## Checking your work

Before you move on:

- Each motor sits in the joint that matches its label: right leg in the 10s, left leg in the 20s
- Every cable is plugged in and routed so it won't get pinched when the joint moves
- Every joint moves freely when the servos are unpowered
- Loctite is only on motor screws

---

## Try it Yourself

1. Before you close each leg, gently move each joint through its range by hand and watch the cables.
2. Hold the two finished legs side by side. They should be mirror images. If one looks different, check you used the right `roll_to_pitch` part.
3. Take a photo of each leg before you move on. It helps a lot if you need to troubleshoot later.

## Common Issues

**Problem**: A leg's motor won't respond later.
**Solution**: Check its cable is plugged in at both ends, especially the foot motor in the shin.
**Why**: The servos are daisy-chained, and a loose cable can cut off every servo after it.

**Problem**: One leg points the wrong way at its zero pose.
**Solution**: Check the hip_pitch motor orientation and the `roll_to_pitch` part.
**Why**: Those two steps decide where the leg's zero is, and mirror parts are easy to swap.

**Problem**: A plastic part cracked near a screw.
**Solution**: Reprint it, and never use Loctite on screws into plastic.
**Why**: Thread locker and overtightening both stress plastic.
