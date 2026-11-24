---
layout: lesson
title: Exporting a Motion to MicroPython
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/12_exporting_a_motion_to_micropython.jpg
date: 2026-11-21
previous: 11_driving_the_arm_from_code.html
next: 13_urdf_in_and_out.html
description: Turn the motion you animated in Snakie into a ready-to-run MicroPython
  program and understand the code it writes
percent: 78
duration: 5
quiz_total: 46
date_updated: 2026-11-24
navigation:
- name: Build and Animate a Robot in Snakie
- content:
  - section: Overview
    content:
    - name: Build and Animate a Robot in Snakie
      link: 00_intro.html
    - name: The Build Workspace
      link: 01_the_build_workspace.html
  - section: Building the Arm
    content:
    - name: Your First Block
      link: 02_your_first_block.html
    - name: Joining Blocks
      link: 03_joining_blocks.html
    - name: Hinges, Wheels and Sliders
      link: 04_hinges_wheels_and_sliders.html
    - name: Mass, Balance and Measuring
      link: 05_mass_balance_and_measuring.html
  - section: Making It Move
    content:
    - name: Posing the Arm
      link: 06_posing_the_arm.html
    - name: Saving Poses
      link: 07_saving_poses.html
    - name: Building a Motion
      link: 08_building_a_motion.html
    - name: Joint Tracks and Keyframes
      link: 09_joint_tracks_and_keyframes.html
  - section: Servos and Code
    content:
    - name: Binding Servos
      link: 10_binding_servos.html
    - name: Driving the Arm from Code
      link: 11_driving_the_arm_from_code.html
    - name: Exporting a Motion to MicroPython
      link: 12_exporting_a_motion_to_micropython.html
  - section: Sharing
    content:
    - name: URDF In and Out
      link: 13_urdf_in_and_out.html
    - name: Render a Movie
      link: 14_render_a_movie.html
    - name: Summary and Next Steps
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to turn an animated motion into a ready-to-run MicroPython program and explain the code it writes.

## Let Snakie write the loop

You've animated a motion and bound the servos. Snakie now has everything it needs to write the program for you.

1. In **Build**, switch to **Animate** and open the **Joint tracks** tab
2. Check your keyframes from lesson 9 are still there
3. Click **Export .py**

A new file called `motion.py` opens in the editor.

![The exported motion.py in the Code workspace](/learn/snakie_build_urdf/assets/export_code.jpg){:class="img-fluid w-100"}

Notice the board picture on the right. It has labelled GP0, GP1 and GP2 with `waist_servo`, `shoulder_servo` and `elbow_servo`, read straight from the code.

---

## What's in the file

It has four parts. Here they are, shortened.

**1. Your project data.** Three blocks marked "managed by Snakie" hold your poses, your Timeline sequence and your servo bindings.

<!-- code-check: skip -->
```python
# --- snakie:poses v1 --- managed by Snakie Motion Studio
SNAKIE_POSES = { "rest": { "waist": 0, "shoulder": 0, "elbow": 0 }, ... }
# --- snakie:poses:end ---
```
{: .no-snakie }

Leave those alone. Snakie rewrites them each time you export. Everything outside the markers is yours and is kept.

**2. The servos.** One per binding, named after its joint.

```python
waist_servo = PWM(Pin(0))
waist = Servo(waist_servo, pin=0)
shoulder_servo = PWM(Pin(1))
shoulder = Servo(shoulder_servo, pin=1)
elbow_servo = PWM(Pin(2))
elbow = Servo(elbow_servo, pin=2)
SERVOS = (waist, shoulder, elbow)
```
{: .no-snakie }

This is why good joint names mattered back in lesson 2.

**3. The frames.** One row for every step of the motion, with a servo angle for each servo.

```python
FPS = 20
DT = 1 / FPS

FRAMES = (
    (90, 90, 90),
    (90, 90, 91),
    (91, 91, 92),
    (92, 93, 94),
    ...
)
```

Two seconds at 20 frames a second is 40 rows. The easing is already worked into the numbers: look how the first few rows barely change, then the steps get bigger.

**4. The player.**

```python
def play():
    for frame in FRAMES:
        for servo, angle in zip(SERVOS, frame):
            servo.angle(angle)
        time.sleep(DT)


try:
    while True:
        play()
except KeyboardInterrupt:
    pass
```

For each row, set every servo, wait one frame, and move on. That's the "lots of small steps" loop from the end of the last lesson.

---

## Run it

Press **Run** with the simulated device connected and switch to **Build**. The arm now moves smoothly through rest, reach and tuck, over and over, exactly as it did when you pressed play on the joint tracks.

![The 3-D arm playing the exported motion](/learn/snakie_build_urdf/assets/export_build.jpg){:class="img-fluid w-100"}

Press **Stop** when you've seen enough.

To put it on a real robot, save `motion.py`, connect your Pico, upload the file and run it. The servo angles are already converted using your bindings, including any you inverted.

---

## Make it yours

The generated file is a starting point. Some ideas:

- Call `play()` once when a button is pressed, in place of the endless loop
- Change `DT` to play the same motion faster or slower
- Play `reversed(FRAMES)` to run the motion backwards
- Export a second motion to another file and import both

When you change the animation, export again. Your own code outside the marked blocks survives.

---

## Try it Yourself

1. Export the motion and run it on the simulated device
2. Count the rows in `FRAMES`. Does it match duration times fps?
3. Change the fps box on the joint tracks to `10`, export again and compare
4. Edit the loop at the bottom so the motion plays three times and then stops

---

## Common Issues

**Problem**: The file says `0 servo(s)` at the top and has no frames.
**Solution**: Put keyframes on the **Joint tracks** and use **Export .py** there.
**Why**: The export is built from the joint tracks. A motion that only exists as blocks on the Timeline tab has no keyframes to bake.

**Problem**: **Export .py** is greyed out.
**Solution**: Bind at least one servo to a joint.
**Why**: With no bindings there are no servos to write code for.

**Problem**: One joint isn't in the exported code.
**Solution**: Bind a servo to it and export again.
**Why**: Joints with no servo are previewed on screen but left out of the code. Snakie adds a comment at the top of the file to tell you.

---

## Check your understanding

{% quiz %}
? Where does the exported motion come from?
- [ ] The Poses shelf
- [x] The keyframes on the Joint tracks
- [ ] The Electronics workspace
> Export bakes the joint tracks into frames.

? A 3 second motion at 20 fps has how many frames?
- [ ] 20
- [ ] 23
- [x] 60
> Duration times frames per second: 3 x 20 = 60.

? Which part of motion.py should you leave alone?
- [x] The blocks marked as managed by Snakie
- [ ] The play function
- [ ] The loop at the bottom
> Snakie rewrites the managed blocks on every export. The rest is yours.
{% endquiz %}
