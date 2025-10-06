---
title: Creating Gears in Fusion 360
description: >-
    Learn how to create perfect gears and rack-and-pinion systems in Fusion 360 using the SpurGear add-in and a bit of clever math.
excerpt: >-
    Ever wanted to convert rotational energy into linear motion for your robot projects? Learn how to create gears and rack-and-pinion systems in Fusion 360 with this step-by-step guide.
layout: showcase
date: 2025-10-06
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/fusion360-gears/cover.jpg
hero: /assets/img/blog/fusion360-gears/hero.jpg
mode: light
tags:
 - Fusion 360
 - CAD
 - 3D Printing
 - Gears
groups:
 - 3dprinting
 - cad
 - design
---

## Converting Rotational Motion to Linear Motion

Ahoy there makers! In this guide, I want to show you how to create a rack and pinion system using Fusion 360. This is absolutely ideal if you want to convert rotational energy into linear motion - perfect for robot arms, linear actuators, or any project where you need precise movement control.

---

## Getting Started with the SpurGear Add-in

First things first, let's fire up Fusion 360 and get our bearings. The secret weapon we'll be using is the SpurGear add-in, which is a Python program that makes gear creation much easier than doing it manually.

### Finding the Add-in

1. Go to **Utilities** in the toolbar
2. Click on **Add-ins and Scripts**
3. Look for **SpurGear** (there's a Python version and a C++ one - I'm using the Python version)
4. Click **Run**

You'll see a dialogue box pop up with loads of options. Don't worry, we'll walk through exactly what you need!

---

## The Math Behind Perfect Gears

Here's where things get interesting. The SpurGear dialogue wants you to specify a **module**, but what if you know the outer diameter you want instead? Let's work backwards!

### Key Formulas

I've created a simple spreadsheet to help with these calculations:

- **Pitch Diameter** = Module × Number of Teeth
- **Module** = Outer Diameter ÷ (Number of Teeth + 2)
- **Rotation Angle** = 360° ÷ Number of Teeth ÷ 2

For example, if I want:
- Outer diameter: 18.5mm
- Number of teeth: 10

Then my module is: 18.5 ÷ (10 + 2) = 1.54

### Setting Up Your Gear

Let's create the gear step by step:

1. Create a new sketch on the top plane
2. Press **C** for circle and create two circles: 18mm and 12mm diameter
3. Make them construction lines
4. Run the SpurGear add-in with these settings:
   - Module: **1.54**
   - Number of teeth: **10**
   - Center hole: **3mm**
   - Gear thickness: **3mm**
   - Root fillet: **0.9mm** (needs to be under 0.9 to work with this module)

The pitch diameter should show as 15.4mm - that's the middle circle where the teeth mesh.

---

## Creating the Perfect Tooth Profile

Now we need to orient that tooth correctly for copying. Here's the trick:

1. Open the **Bodies** section and select your spur gear body
2. Press **M** to move it
3. Select **Rotate** and choose the center axis
4. Rotate by **18°** (remember that formula? 360 ÷ 10 ÷ 2 = 18°)

This gives us two teeth perfectly aligned at the top and bottom.

---

## Making the Rack

Now for the clever bit - creating the rack that our gear will mesh with!

### Copy and Position

1. Press **M** again to move/copy
2. Select **Translate** and move in the Y-axis by **-15.4mm** (the pitch diameter)
3. Make sure **Create Copy** is checked
4. Click OK

You'll see the teeth overlapping - that's exactly what we want!

### Project the Tooth Profile

1. Hide the original gear body
2. Create a new sketch on the top surface
3. Press **P** to project
4. Carefully select and project all the lines that make up one tooth profile
5. Draw a line across the bottom to close the profile
6. Press **E** to extrude the tooth out by **3mm**

### Calculate the Rack Length

For the full rack:

1. Go back into the sketch
2. Create a rectangle: **3mm high × 190mm long** (or whatever length you need)
3. Extrude this base by **3mm** as well

---

## Pattern Those Teeth!

Now we need to replicate that single tooth along the entire rack.

### Tooth Spacing Calculation

The spacing between teeth is critical:

**Spacing** = (Pitch Diameter × π) ÷ Number of Teeth

For our example: (15.4 × π) ÷ 10 = **4.84mm**

### Creating the Pattern

1. Go to **Create → Pattern → Rectangular Pattern**
2. Select the tooth extrusion as the feature
3. Set the direction along the rack
4. Set spacing to **4.84mm**
5. For a 190mm rack, you'll need about **39 teeth**

### Final Assembly

1. Select all the bodies
2. Use **Combine** with the **Join** operation
3. You now have a single, complete rack!

---

## Testing the Fit

Let's verify everything meshes properly:

1. Show the original gear body again
2. Press **M** to rotate it back by **-18°** to undo our earlier rotation
3. Zoom in and check the mesh

The teeth should fit together perfectly with no gaps or interference. If they do, you're ready to 3D print!

---

## Why This Method Works

The beauty of this approach is that we're working from the **outer diameter** - the actual physical size we need - rather than getting confused by pitch diameters and modules. The math is straightforward, and once you've got the formulas in a spreadsheet, you can quickly design gears of any size.

The involute curve of the tooth profile (that lovely curve you see) is what makes gears mesh smoothly without binding. The SpurGear add-in handles all that complex geometry for us.

---

## Tips for 3D Printing Gears

- Add a bit of tolerance (0.1-0.2mm) if your printer isn't super precise
- Print gears flat on the bed for best accuracy
- Consider the layer orientation for strength
- Test with a small sample before printing the full rack

---

## What's Next?

Now you've got the skills to create custom gears and racks for all sorts of projects! Try experimenting with different tooth counts, sizes, and ratios. You could even create complete gearbox assemblies.

Hope this helps you with your next robot build! Let me know what you create with these techniques.

Bye for now, and happy making!

---
