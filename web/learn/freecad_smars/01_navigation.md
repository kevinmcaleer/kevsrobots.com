---
layout: lesson
title: Navigation in FreeCAD
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2025-12-12
previous: 00_intro.html
next: 02_creating_the_base.html
description: Master the 3D workspace - the foundation skill for efficient CAD work
percent: 18
duration: 5
navigation:
- name: Building SMARS with FreeCAD
- content:
  - section: Getting Started
    content:
    - name: Introduction to Building SMARS with FreeCAD
      link: 00_intro.html
    - name: Navigation in FreeCAD
      link: 01_navigation.html
  - section: Designing the SMARS Base
    content:
    - name: Creating the Base
      link: 02_creating_the_base.html
    - name: Shell and Fillet the Base
      link: 03_shell_and_fillet.html
    - name: Side Holes
      link: 04_sides_and_holes.html
    - name: Front and Rear Profiles
      link: 05_front_rear_profiles.html
    - name: Arduino Slots
      link: 06_arduino_slots.html
    - name: Wheel Stubs
      link: 07_wheel_stubs.html
    - name: Motor Holder
      link: 08_motor_holder.html
  - section: Exporting for 3D Printing
    content:
    - name: Save as STL
      link: 09_save_as_stl.html
  - section: Summary
    content:
    - name: Summary and Next Steps
      link: 10_summary.html
---


## Why Navigation Matters

Before we design a single feature, you need to **see your work from every angle**. Professional CAD users can orbit, pan, and zoom without thinking - it becomes muscle memory. This lesson builds that foundation.

Think of it like driving: you don't think "press pedal, turn wheel" - you just *go*. That's what navigation should feel like in FreeCAD.

---

## Mouse Controls - Your Primary Tools

### Rotate (Orbit)

**How**: Click and hold the **middle mouse button** (scroll wheel) and move the mouse.

**Why it matters**: You'll rotate your view hundreds of times per project. When designing the SMARS base, you'll need to see inside corners, check fillet edges, and verify hole placements - all requiring constant rotation.

**Pro tip**: Small movements give fine control. If your view spins wildly, you're moving too fast!

### Pan (Slide)

**How**: Hold **Shift** + middle mouse button, then move the mouse.

**Why it matters**: Panning keeps your zoom level while sliding the view. Use it when you've zoomed in on a detail but need to see a nearby feature.

### Zoom

**How**: Scroll the mouse wheel - up zooms in, down zooms out.

**Why it matters**: Detail work (like positioning the 4.5mm motor holes) needs close zooms. Overview checks need wide zooms. You'll zoom constantly.

**Pro tip**: Zoom toward your mouse cursor position - FreeCAD zooms to where you're pointing!

---

## The Navigation Cube

Look in the **top right corner** of the 3D view. That little cube is incredibly useful:

- **Click a face** → Snap to that view (Front, Back, Top, Bottom, Left, Right)
- **Click an edge** → View from that edge
- **Click a corner** → Isometric view from that corner

**When to use it**: When you need a precise orthographic view - like when sketching on the XY plane or checking alignment.

![Navigation Cube](assets/nav02.png){:class="img-fluid w-100"}

---

## Keyboard Shortcuts - Speed Up Your Workflow

Memorize these - they'll save you hundreds of mouse movements:

| Shortcut | View | When You'll Use It |
|----------|------|-------------------|
| `0` | Isometric | Default working view - see the 3D shape |
| `1` | Front | Sketching front/rear profiles |
| `2` | Back | Checking rear features |
| `3` | Top | Overhead view for layout |
| `4` | Bottom | Rarely used, but helpful for base features |
| `5` | Left | Sketching side holes and wheel stubs |
| `6` | Right | Checking symmetric features |

### Other Essential Shortcuts

| Shortcut | Action | When You'll Use It |
|----------|--------|-------------------|
| `Ctrl + 0` | Fit all | Lost? This recenters everything |
| `F` | Focus selected | Zoom to the object you've selected |
| `Shift + F` | Focus all | Show entire scene |
| `+` / `-` | Zoom in/out | Fine zoom control |
| `Arrow keys` | Pan | Precise panning in small steps |

---

## Try It Yourself

Before moving to the next lesson, practice these exercises:

1. **The Orbit Challenge**: Rotate your view in a complete circle around the origin
2. **The Number Keys**: Press 1 through 6 and 0 - get familiar with what each view looks like
3. **Zoom + Pan Combo**: Zoom in close, then pan to each corner of your workspace
4. **Navigation Cube**: Click every face, edge, and corner of the cube

Spend 5 minutes on this. It pays off enormously later!

---

## Common Issues

### "My view spins out of control!"
**Problem**: Moving the mouse while holding middle button causes wild spinning.
**Solution**: Move slowly and deliberately. Your cursor position affects the rotation center.

### "I can't find my model anymore!"
**Problem**: After zooming or panning, the model disappeared.
**Solution**: Press `Ctrl + 0` to fit all objects in view, or press `0` for isometric view.

### "Middle mouse button doesn't work!"
**Problem**: Some laptops don't have a proper middle click.
**Solution**: Go to Edit → Preferences → Display → Navigation. Choose a navigation style that works with your hardware (like "Touchpad").

### "The view is upside down!"
**Problem**: Model appears flipped after rotating.
**Solution**: Press `0` to reset to isometric view, or click a face on the navigation cube.

---

## What You Learned

In this lesson, you mastered:

- **Mouse navigation** - Rotate, pan, and zoom
- **Navigation cube** - Quick access to standard views
- **Keyboard shortcuts** - Faster workflow

These controls become second nature with practice. By the end of this course, you won't think about navigation - you'll just *do* it.

---

## Next Up

In the next lesson, we'll create the base of our SMARS robot - a 70mm × 58mm × 32mm solid block that becomes the foundation for everything else. Time to start designing!

---
