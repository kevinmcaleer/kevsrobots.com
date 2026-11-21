---
title: Drive Links
description: Link two BetterCad joints through gears, a belt, a rack or a typed ratio so that turning one turns the other
layout: lesson
type: page
cover: /learn/bettercad_joints_urdf/assets/banners/11_drive_links.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Every joint so far has had its own motor, in spirit at least. Plenty of mechanisms don't work that way. One servo turns a gear, the gear turns another, and that one moves the part you care about. Two joints, one motor.

A **drive link** tells BetterCad that one joint drives another, and by how much.

---

## The servo deck

This lesson uses the geared servo deck from [Design Robot Parts with BetterCad](/learn/bettercad_robot_parts/). It has a 24-tooth gear on an SG90 and a 48-tooth gear on a post beside it. They mesh, but they don't turn.

1. Save your arm, then open your servo deck. If you don't have one, download [servo-deck.bcad](/learn/bettercad_robot_parts/assets/servo-deck.bcad) and open that.
2. Click the **Assemble** tab.

---

## Move the gears aside

Each gear needs a wheel joint. The snap you want on the deck is the top of the servo shaft, or the top of the post, and at the moment the gear is sitting on it. So lift the gears out of the way first. The joints will put them back.

3. Click **Gear 24T** and set **Position Y** to `60` in Properties.
4. Do the same for **Gear 48T**.

![The two gears moved off to the side of the deck](/learn/bettercad_joints_urdf/assets/gears_aside.jpg){:class="img-fluid w-100"}

---

## Joint the small gear

5. Click **Joint**.
6. Click the hole in the middle of **Gear 24T**.

Check the **Snap** under Moves reads `-30, 60, 23`. This gear has a servo horn recess with several snap points close together, so if you got a different one, press **Esc** once and click again nearer the centre.

7. Click the top of the servo's shaft.

The Hangs from snap should read `-30, 0, 18`.

8. Change **Kind** to **Wheel (continuous)**, so the gear can turn all the way round.

![The small gear joined to the servo shaft as a wheel](/learn/bettercad_joints_urdf/assets/g24_made.jpg){:class="img-fluid w-100"}

9. Click **OK**, then rename the joint `Servo`.

The gear lands on the shaft upside down. Parts join face to face, and you clicked its top face. A spur gear works the same either way up. If yours has a hub that must face up, turn the view to look from underneath and click the gear's bottom face.

---

## Joint the big gear

10. Click **Joint**, then click the hole in the middle of **Gear 48T**. The snap should read `6, 60, 23`.
11. Click the top of the post, near its hole. The snap should read `6, 0, 18`.
12. Change **Kind** to **Wheel (continuous)** and click **OK**.
13. Rename the joint `Output`.

![Both gears back in place, each on its own wheel joint](/learn/bettercad_joints_urdf/assets/gears_back.jpg){:class="img-fluid w-100"}

Try the two Pose sliders. Each gear spins by itself, straight through the other's teeth. Click **Home**.

---

## Add the drive

14. Click **Drive link** in the Motion group.

![The Drive link box with Servo driving Output through gears](/learn/bettercad_joints_urdf/assets/drive_dialog.jpg){:class="img-fluid w-100"}

15. Set **Driver** to **Servo** and **Driven** to **Output**.
16. Leave **Through** on **Gears**.

At the bottom, BetterCad has already done the sum: "-0.5×, the other way, from the teeth". It counted 24 teeth driving 48, so the output turns half as far, and meshed gears turn in opposite directions.

17. Click **Add drive**.

The bar says "Added Drive 1. Move either joint and the other follows."

---

## Watch it turn

In the Pose section, **Output** now says "Driven by Servo", and **Servo** has a **Play** button.

18. Type `90` in the Servo's Angle box.

![The servo gear at 90 degrees and the output gear at minus 45](/learn/bettercad_joints_urdf/assets/drive_top.jpg){:class="img-fluid w-100"}

Output reads -45. Half as far, the other way.

19. Click **Play**.

![Playing the drive, with both gears turning](/learn/bettercad_joints_urdf/assets/drive_play.jpg){:class="img-fluid w-100"}

Both gears turn together, teeth in step. The bar says "Playing Servo. Esc stops." Press **Esc** or click **Stop** when you've seen enough.

---

## Four kinds of drive

| Through | What it models | How the ratio is found |
|---|---|---|
| **Gears** | Meshed gears. The driven one turns the other way | From the tooth counts |
| **Belt** | Pulleys on a belt. The driven one turns the same way | From the tooth counts |
| **Rack** | A gear on a rack. Each turn slides the rack one pitch circle round | From the gear |
| **Custom** | Anything else | You type it |
{:class="table table-single"}

If the two parts aren't gears BetterCad made, a **Ratio** box appears and a note says "The links don't both have the gears for this, so the ratio you type is used." That covers linkages, bought gearboxes and anything else where you know the ratio.

**Turn the other way** reverses the direction if BetterCad's guess doesn't match your mechanism.

---

## Joints that move together

There's a second kind of link for a simpler job: joints that should all do the same thing. The four wheels of a rover. Two gripper fingers. A pair of servos lifting one arm.

Open the small arrow under the Motion group and choose **Link joints**. Pick a **Lead joint**, tick the joints that move with it, and tick **Other way** for any that should turn against it. From then on, moving the lead moves them all.

---

## Why this matters

**Poses are honest.** With the drive in place you can't accidentally pose the gears in a way the real mechanism couldn't reach.

**Sweeps follow the drive.** A sweep of the Servo joint takes the Output with it, so it tests what the real machine does.

**The export knows.** The drive is written into the URDF file, so a simulator turns the output gear when it turns the servo.

---

## Try it Yourself

1. Type 180 for the Servo. Where does the Output end up? Does that match the 2 : 1 ratio?
2. Add the drive link the other way round in a fresh copy: Output driving Servo. What ratio does BetterCad report?
3. Go back to your desk arm and use **Link joints** to make the Elbow follow the Shoulder. Drag the shoulder and watch the arm curl. Undo it afterwards.

---

## Common Issues

- **Problem**: A Ratio box appears though both parts are gears
- **Solution**: Check Driver and Driven are the two gears' joints, and that the gears were made with the Gear tool
- **Why**: BetterCad reads the tooth counts from gear steps. An imported gear is only a shape to it

- **Problem**: The gears turn but the teeth pass through each other
- **Solution**: Edit one gear's joint and change **Angle** under Line-up by a few degrees until the teeth sit in the gaps at home
- **Why**: The drive keeps the gears in step from wherever they start. It doesn't line the teeth up for you

- **Problem**: The gear joined on at an odd place
- **Solution**: Edit the joint and check the two Snap readings against the numbers above
- **Why**: A gear face has many snap points, and one tooth over is a different axle

---

## Check your understanding

{% quiz %}
? A 24-tooth gear drives a 48-tooth gear. The driver turns 90 degrees. What does the driven gear do?
- [ ] Turns 180 degrees the same way
- [ ] Turns 90 degrees the other way
- [x] Turns 45 degrees the other way
> Twice the teeth means half the turn, and meshed gears go in opposite directions.

? When does the Drive link box ask you to type a ratio?
- [ ] Always
- [x] When the two parts don't both have gears it can count teeth on
- [ ] Only for belts
> Custom always uses a typed ratio. The other kinds read it from the parts when they can.

? Which tool makes a rover's four wheel joints turn together?
- [ ] Rigid group
- [ ] Save pose
- [x] Link joints
> Pick a lead joint and tick the ones that move with it.
{% endquiz %}
