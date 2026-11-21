---
layout: lesson
title: URDF In and Out
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/13_urdf_in_and_out.jpg
date: 2026-11-21
previous: 12_exporting_a_motion_to_micropython.html
next: 14_render_a_movie.html
description: Export your Snakie robot as a URDF file, read what is inside it, and
  open a robot designed in BetterCad
percent: 84
duration: 6
quiz_total: 46
date_updated: 2026-11-21
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

## What a URDF is

Everything you've built in this course lives in one small text file. It's called a **URDF**, short for Unified Robot Description Format, and it's the standard way to describe a robot: its parts (links), how they connect (joints), and how each joint can move.

Snakie isn't the only program that reads it. ROS, simulators such as Gazebo and PyBullet, and [BetterCad](https://bettercad.kevsrobots.com) all speak URDF. That makes it the bridge between tools.

---

## Export your arm

1. In **Build**, look at the bottom of the panel on the left, under **Total mass**, and click **Export URDF**
2. Look in the file list. There's a new folder called `urdf` with `my_robot.urdf` inside

![The exported URDF in the project](/learn/snakie_build_urdf/assets/export_urdf.jpg){:class="img-fluid w-100"}

Snakie keeps its own working copy of the robot in the project too. The exported file is a tidy copy for sharing, with the masses from lesson 5 written in.

---

## Read it

Open `my_robot.urdf`. It's long, but it only uses two ideas over and over. Here's one link and one joint.

```xml
<link name="tower">
  <inertial>
    <mass value="0.006"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.02 0.02 0.06"/>
    </geometry>
  </visual>
</link>

<joint name="waist" type="revolute">
  <parent link="base_link"/>
  <child link="tower"/>
  <origin xyz="0 0 0.04" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

You can read your own work in it:

| In the file | What you did in Snakie |
|---|---|
| `box size="0.02 0.02 0.06"` | Made the tower 20 x 20 x 60 mm |
| `mass value="0.006"` | The 6 g estimate from the Mass section |
| `type="revolute"` | Chose **Hinge** |
| `origin xyz="0 0 0.04"` | Set Offset Z to 40 |
| `axis xyz="0 0 1"` | Chose Axis Z |
{:class="table table-single"}

Two things to notice. URDF works in **metres**, so 40 mm is written `0.04`. And it has its own words for joints:

| Snakie | URDF |
|---|---|
| Fixed | `fixed` |
| Hinge | `revolute` |
| Wheel | `continuous` |
| Slider | `prismatic` |
{:class="table table-single"}

---

## Open a robot from BetterCad

Blocks are great for working out a mechanism. For parts you'll print, you'll want real shapes, and that's what BetterCad is for. A robot designed there exports as a URDF with a `meshes` folder of STL files, one per part.

Here's one to try. It's the two-joint desk arm from the [Make It Move with BetterCad](/learn/bettercad_joints_urdf/) course.

[Download desk-arm-urdf.zip](/learn/snakie_build_urdf/assets/desk-arm-urdf.zip)

1. Unzip it. You get a folder called `Desk_arm` holding `Desk_arm.urdf`, a `meshes` folder and a tiny file called `robot.yml`
2. In Snakie, open that folder as your project
3. Switch to **Build**

![The BetterCad desk arm open in Snakie](/learn/snakie_build_urdf/assets/bettercad_arm.jpg){:class="img-fluid w-100"}

The real shapes appear, with the joints `Shoulder` and `Elbow` listed and the mass already filled in from BetterCad. Switch to **Animate** and drag a slider.

![Posing the BetterCad desk arm](/learn/snakie_build_urdf/assets/bettercad_arm_posed.jpg){:class="img-fluid w-100"}

Everything from the last eight lessons now works on this arm: poses, the Timeline, joint tracks, servo bindings and code.

### How Snakie knows which URDF to use

That `robot.yml` file is the link. It has one job here, which is to name the URDF.

```yaml
robot:
  urdf: Desk_arm.urdf
  version: 1
parts: []
connections: []
```

When you export your own design from BetterCad, put a `robot.yml` like this next to the URDF, change the file name on the second line, and open the folder.

In the Snakie desktop app there's also an **Open URDF…** button on the Build start card that does the linking for you.

---

## Add a single mesh

You don't need a whole URDF to use a printed part. Next to Export URDF, **+ STL / DAE** brings in one mesh file as a new, loose link.

![An imported STL and its properties](/learn/snakie_build_urdf/assets/stl_import.jpg){:class="img-fluid w-100"}

It arrives without a joint, so connect it with **Add a joint** as you did in lesson 3. Two differences from a block:

- Snakie can't estimate the weight of an imported mesh, so type a **Measured** weight
- The colour box recolours the whole mesh

---

## Try it Yourself

1. Export your arm and find the `shoulder` joint in the file. What is its axis?
2. Change the waist to a **Wheel**, export again, and find what changed in the file. Change it back afterwards
3. Open the BetterCad desk arm and save two poses for it
4. Bind servos to `Shoulder` and `Elbow` and run a short program that moves both

---

## Common Issues

**Problem**: The Build workspace is empty after opening the BetterCad folder.
**Solution**: Check `robot.yml` is in the same folder as the URDF and the name on the `urdf:` line matches the file exactly, capitals included.
**Why**: Snakie loads the URDF that `robot.yml` names. If the name doesn't match, there's nothing to show.

**Problem**: The joints are listed but the parts are invisible.
**Solution**: Keep the `meshes` folder next to the URDF, with the STL files inside.
**Why**: The URDF only stores the path to each shape. Move the file without its meshes and the shapes can't be found.

**Problem**: The numbers in the URDF look far too small.
**Solution**: Multiply by 1000.
**Why**: URDF uses metres and kilograms. Snakie shows millimetres and grams.

---

## Check your understanding

{% quiz %}
? What does a URDF file describe?
- [ ] The MicroPython code for a robot
- [x] A robot's links, joints and how the joints move
- [ ] The wiring between a Pico and its servos
> A URDF is a description of the robot's structure. Code and wiring live elsewhere.

? A joint origin of 0.04 in a URDF is how far in Snakie's units?
- [ ] 4 mm
- [x] 40 mm
- [ ] 400 mm
> URDF uses metres. 0.04 m is 40 mm.

? What is a Snakie Hinge called in URDF?
- [ ] prismatic
- [ ] continuous
- [x] revolute
> A hinge with limits is a revolute joint. A Wheel is continuous and a Slider is prismatic.

? What does robot.yml do for an imported robot?
- [x] It tells Snakie which URDF file to load
- [ ] It stores the STL shapes
- [ ] It runs the servos
> The urdf line in robot.yml names the file the Build workspace opens.
{% endquiz %}
