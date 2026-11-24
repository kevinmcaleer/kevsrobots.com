---
title: Head, Body and Battery
description: Build the neck and head mechanism, fit the battery pack and Pi, and close up the body of the Open Duck Mini in the right order
layout: lesson
type: page
cover: /learn/open_duck_mini/assets/banners/06_head_body_and_battery.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Legs done. Now we give the duck a neck, a head that can look around, and a body to hold it all together. This lesson also has an important bit of ordering: you'll wire the electronics part-way through, so read it all the way through before you start.

---

## The order matters

The assembly guide builds the top half in this order:

| Guide step | What you do | Where it's covered |
|---|---|---|
| 6 | Neck | This lesson |
| 7 | Head mechanism | This lesson |
| 8 | Servo driver board | Lesson 7 |
| 9 | IMU | Lesson 7 |
| 10 | Electronics wiring | Lesson 7 |
| 11 | Battery pack | This lesson |
| 12 | Head: inserts, bearing, ear motors and the Pi Zero 2W | This lesson |
| 13 | Body | This lesson |
| 14 | Software | Lessons 8 to 11 |
{:class="table table-single"}

So the plan is: build the neck and head mechanism, then do the wiring in lesson 7, then come back here for the battery, head and body. Once the body is closed, getting to the wiring is harder, so don't rush to close it up.

---

## Step 6: the neck

The neck uses the **neck_pitch** servo, ID 30. Mount it as the assembly guide shows, and route its cable so it can reach the rest of the bus without being pulled tight when the neck moves.

---

## Step 7: the head mechanism

On top of the neck, the head has three more joints: **head_pitch** (ID 31), **head_yaw** (ID 32) and **head_roll** (ID 33). Together they let it nod, turn and tilt. The guide builds this mechanism from three printed parts, in this order:

1. `head_pitch_to_yaw`
2. `head_yaw_to_roll`
3. `head_roll_mount`

Follow the guide's pictures for which servo goes where, and check each joint moves freely by hand before you add the next part.

Now **pause here and go to lesson 7** to fit the servo board and IMU and do the wiring. Come back when that's done.

---

## Step 11: the battery pack

Before the battery goes in, **charge all of its cells to the same voltage**. The assembly guide asks for this, so the pack starts out balanced. Check the BOM for the exact battery pack, and follow the charging instructions that come with it.

Fit the pack as the assembly guide shows, and keep its wires clear of any joint that moves.

---

## Step 12: the head

1. Fit the heat-set inserts in the head.
2. Fit the bearing.
3. Mount the ear motors, if you're adding them.
4. Mount the Raspberry Pi Zero 2W.

The Pi lives in the head. If you haven't flashed its SD card yet, it's easier to do it now, before it's tucked away. Lesson 8 covers that.

---

## Step 13: the body

Close up the body with `body_middle_bottom`, `body_middle_top` and `body_front`.

Before you fit the last panel, do a final check:

- Every servo cable is plugged in, and none is pinched
- The IMU is mounted firmly
- The battery is secure and its wires are clear of moving parts
- The USB cable from the servo board reaches the Pi

---

## Try it Yourself

1. With the power off, move the head through pitch, yaw and roll by hand. Listen for anything rubbing.
2. Move the neck forward and back and watch the cables. They need slack at the full range.
3. Decide where you'll leave a cable or panel easy to reach, so you can get at the Pi's SD card later.

## Common Issues

**Problem**: The head won't turn smoothly.
**Solution**: Check the bearing is seated and that no screw is overtightened.
**Why**: A tight screw or a crooked bearing adds friction the servo has to fight.

**Problem**: You're not sure which battery or charger to use.
**Solution**: Use the pack listed on the BOM and the charging instructions from its supplier.
**Why**: This course deliberately doesn't give battery details, because the BOM is kept up to date and the right pack matters for safety.

**Problem**: You closed the body and a servo is missing from the bus.
**Solution**: Open it up and check the daisy-chain cables.
**Why**: That's why we leave the body until last.
