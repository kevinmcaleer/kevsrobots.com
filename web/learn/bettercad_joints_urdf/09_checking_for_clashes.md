---
layout: lesson
title: Checking for Clashes
author: Kevin McAleer
type: page
cover: /learn/bettercad_joints_urdf/assets/banners/09_checking_for_clashes.jpg
date: 2026-11-21
previous: 08_mass_and_balance.html
next: 10_sweeps_and_joint_limits.html
description: Use the BetterCad clash check to find parts that overlap or touch in
  any pose, and read the results
percent: 70
duration: 5
quiz_total: 39
date_updated: 2026-11-21
navigation:
- name: Make It Move with BetterCad
- content:
  - section: Overview
    content:
    - name: Make It Move with BetterCad
      link: 00_intro.html
  - section: Joints
    content:
    - name: Components and the Assemble Tab
      link: 01_components_and_the_assemble_tab.html
    - name: Your First Joint
      link: 02_your_first_joint.html
    - name: Posing a Joint
      link: 03_posing_a_joint.html
    - name: A Second Joint
      link: 04_a_second_joint.html
    - name: Joint Kinds and Line-up
      link: 05_joint_kinds_and_line_up.html
  - section: Poses
    content:
    - name: Named Poses
      link: 06_named_poses.html
    - name: Drag the Arm by Its Tip
      link: 07_drag_the_arm_by_its_tip.html
  - section: Checks
    content:
    - name: Mass and Balance
      link: 08_mass_and_balance.html
    - name: Checking for Clashes
      link: 09_checking_for_clashes.html
    - name: Sweeps and Joint Limits
      link: 10_sweeps_and_joint_limits.html
  - section: Drives
    content:
    - name: Drive Links
      link: 11_drive_links.html
  - section: Export
    content:
    - name: Exporting URDF
      link: 12_exporting_urdf.html
    - name: Summary and Next Steps
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

On screen, two parts can pass straight through each other and nothing complains. On the bench, something cracks. The **clash check** finds every place where two parts are trying to be in the same space.

---

## Check a pose

1. Click **Home**.
2. Click **Clashes** in the Check group.

The **Clashes** section in Properties opens and lists what it found. At home, the arm is fine:

- Under **Touching**: "Base and Upper arm touch" and "Upper arm and Forearm touch"

Touching is expected. The parts meet face to face at their joints, and that's what a joint is.

---

## Make it clash

The shoulder's limits are -90 and 90, and inside that range the arm never hits anything. Let's give it more room and see what goes wrong.

3. Right-click **Shoulder** in the Browser and choose **Edit joint**.
4. Set **Lower** to `-150` and **Upper** to `150`.

![Editing the shoulder's limits](/learn/bettercad_joints_urdf/assets/edit_joint.jpg){:class="img-fluid w-100"}

5. Click **OK**.
6. In the Pose section, set **Shoulder** to `140`.
7. If the results have gone, click **Check clashes** in Properties to switch the check back on.

![The upper arm clashing with the base plate, shown in red](/learn/bettercad_joints_urdf/assets/clash.jpg){:class="img-fluid w-100"}

The view turns red where it matters. The upper arm has swung right over and is buried in the base plate, and Properties says "Base and Upper arm overlap by 0.3 cm³".

---

## Reading the results

| Result | Meaning |
|---|---|
| **overlap by** a volume | The two parts share space. That can't be built |
| **Touching** | The parts meet but don't overlap |
| Nothing listed | The pose is clear |
{:class="table table-single"}

Each result has a **Show** button that frames the clash in the view. On a busy robot that saves a lot of hunting.

The size of the overlap is a clue. A tiny one is often a corner that needs a chamfer. A big one means the design needs a rethink.

---

## What gets checked

Above the results are three settings:

| Setting | What it does |
|---|---|
| **At** | Which pose to check: The pose now, Home, or any named pose |
| **Bodies in the same link** | Also checks bodies inside one moving part against each other |
| **Reference bodies** | Includes reference bodies, such as a servo from the catalogue |
{:class="table table-single"}

**At** is the one to remember. Pick a named pose from the list and BetterCad checks it without you going to it. Save your awkward poses once and you can test them after every design change.

**Reference bodies** is worth ticking when your model has bought parts in it. A printed arm that swings through a servo is every bit as broken as one that swings through another print.

**Bodies in the same link** is off to start with because bodies in one moving part usually overlap on purpose: a bracket sitting in its slot, a nut in its trap.

---

## Fixing a clash

There are three honest fixes:

- **Change a part.** Shorten it, notch it, round the corner that hits
- **Move a joint.** A taller upright would let this arm swing further
- **Limit the joint.** If the arm never needs to go there, stop it going there

The third is often the right one, and the next lesson has BetterCad work out the limit for you.

For now:

8. Press **Esc** to switch the check off, then click **Home**.

---

## Try it Yourself

1. Set the shoulder to 120 and check. Then 130, then 135. Roughly where does the overlap start?
2. Set **At** to your Folded pose and check it without going there.
3. Add a Library box sitting half inside the base plate, and check with **Bodies in the same link** off and then on. Delete the box afterwards.

---

## Common Issues

- **Problem**: The check lists parts as touching and you expected nothing
- **Solution**: Nothing to do, or add a small Gap to the joint
- **Why**: Parts joined face to face do touch. Touching isn't a clash

- **Problem**: The parts stay red and the list won't go away
- **Solution**: Press Esc, or click **Check clashes** again
- **Why**: Check clashes is a switch. It stays on until you turn it off

- **Problem**: A servo passes through a printed part and nothing is reported
- **Solution**: Tick **Reference bodies** and check again
- **Why**: Reference bodies are left out unless you ask

---

## Check your understanding

{% quiz %}
? The check reports "Upper arm and Forearm touch". Is that a problem?
- [ ] Yes, the joint is broken
- [x] No, parts joined face to face are meant to touch
- [ ] Only if the arm is at home
> An overlap is a clash. Touching is two parts meeting.

? How do you check a saved pose without moving the arm?
- [ ] You can't
- [ ] Right-click the pose and choose Delete
- [x] Choose it in the At list and click Check clashes
> At lists the pose now, Home, and every named pose.

? When should you tick Reference bodies?
- [x] When the model has bought parts, such as servos, that moving parts could hit
- [ ] Never
- [ ] Only before exporting an STL
> Reference bodies aren't printed, but they still take up space on the real robot.
{% endquiz %}
