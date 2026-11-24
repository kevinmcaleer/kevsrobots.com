---
title: Summary and Next Steps
description: A recap of building, animating and coding a robot arm in Snakie, with the finished project files and where to go next
layout: lesson
type: page
cover: /learn/snakie_build_urdf/assets/banners/15_summary.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to recap what you built and choose where to take your robot next.

## You built a robot

You started with an empty 3-D view. You now have a three-joint desk arm that you designed, weighed, posed, animated and drove from MicroPython, all without a single part on your desk.

That's the big idea of the Build workspace: **work it out on screen first**. Mistakes cost nothing, and by the time you print and wire the real thing you already know it moves the way you want.

---

## What you learned

| Lesson | Skill |
|---|---|
| 1 to 3 | Finding your way around Build, adding and sizing blocks, and joining them face to face |
| 4 | Fixed, Hinge, Wheel and Slider joints, axes and limits |
| 5 | Mass estimates, balance, ground contacts, Bone Mode and measuring |
| 6 to 7 | Posing with sliders and by dragging, and saving poses to the shelf |
| 8 to 9 | Building a motion on the Timeline and fine-tuning it with joint tracks and keyframes |
| 10 to 11 | Binding servos to joints and driving the model from your own code |
| 12 | Exporting a motion as a MicroPython program |
| 13 to 14 | URDF export and import, BetterCad robots, and rendering a movie |
{:class="table table-single"}

---

## The finished project

If you'd like to compare your arm with ours, or start from a known-good copy, here are the files.

- [robot.urdf](/learn/snakie_build_urdf/assets/robot.urdf), the arm itself
- [robot.yml](/learn/snakie_build_urdf/assets/robot.yml), with the poses, the motion and the servo bindings
- [arm.py](/learn/snakie_build_urdf/assets/arm.py), the program from lesson 11
- [desk-arm-urdf.zip](/learn/snakie_build_urdf/assets/desk-arm-urdf.zip), the BetterCad desk arm from lesson 13

Put `robot.urdf` and `robot.yml` in a new folder, open it in Snakie and switch to **Build**.

---

## From screen to desk

When you're ready to build an arm for real:

1. **Design the parts.** Blocks show you the sizes and the pivots. Draw the real parts in BetterCad around your servos
2. **Bring them back.** Export the URDF from BetterCad and open it in Snakie, as you did in lesson 13
3. **Reuse your motion.** Bind the servos to the same pins and export the code again
4. **Wire it and run it.** Three servos want their own 5 V supply, with the grounds joined to the Pico

---

## Challenges

1. **Add a gripper.** Two small blocks on Slider joints at the end of the forearm. Set one to copy the other so they close together
2. **A pick and place.** Save poses for above, down, lifted and across, and chain them on the Timeline
3. **A different robot.** Build a two-wheeled rover from a body block and two Wheel joints, tag its contacts and check it balances
4. **Button control.** Change the exported program so the motion plays once each time a button is pressed

---

## What's Next

- [Make It Move with BetterCad](/learn/bettercad_joints_urdf/) covers joints, poses and URDF export on the CAD side
- [Design Robot Parts with BetterCad](/learn/bettercad_robot_parts/) shows you how to design servo mounts and gears for a real arm
- [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/) covers wiring and power in the Electronics workspace
- [Snakie Instruments](/learn/snakie_instruments/) helps you watch what your servos and sensors are doing
- [Make Your Own Parts and Boards in Snakie](/learn/snakie_part_editor/) is for adding your own servo driver or sensor
- The [Snakie Power User](/learn/learning_pathways/snakie-power-user.html) pathway collects the intermediate Snakie courses

Build something, make it move, and share the movie. We'd love to see it.

---

## Check your understanding

{% quiz %}
? What is the main benefit of building a robot in Snakie before printing it?
- [ ] The printed parts come out stronger
- [x] You can test the design and the motion before anything is made
- [ ] The servos use less power
> Changing a block is free. Changing a printed part costs time and plastic.

? Which file holds your poses, motion and servo bindings?
- [ ] robot.urdf
- [x] robot.yml
- [ ] arm.py
> The URDF describes the robot. robot.yml holds what you did with it in Snakie.

? You want two gripper fingers to move together from one servo. Which joint setting helps?
- [ ] A Wheel joint
- [ ] Ground contacts
- [x] Copies, so one joint follows the other
> Copies makes a joint follow another one.
{% endquiz %}
