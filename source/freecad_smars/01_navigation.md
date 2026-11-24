---
title: Navigation in FreeCAD
description: Master the 3D workspace - the foundation skill for efficient CAD work
layout: lesson
cover: /learn/freecad_smars/assets/banners/01_navigation.jpg
type: page
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to orbit, pan and zoom around a model in FreeCAD and jump to any standard view using the navigation cube and number keys.

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
{:class="table table-single"}

### Other Essential Shortcuts

| Shortcut | Action | When You'll Use It |
|----------|--------|-------------------|
| `Ctrl + 0` | Fit all | Lost? This recenters everything |
| `F` | Focus selected | Zoom to the object you've selected |
| `Shift + F` | Focus all | Show entire scene |
| `+` / `-` | Zoom in/out | Fine zoom control |
| `Arrow keys` | Pan | Precise panning in small steps |
{:class="table table-single"}

---

> **Teachers**: FreeCAD must be installed on each machine before the lesson, so ask your IT team early, as school computers often block installs. Navigation relies on a middle mouse button. If your classroom has laptops with touchpads only, have learners pick a touchpad navigation style in the preferences (see Common Issues), or supply spare USB mice. Allow about 20 minutes.

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
- **Problem**: Moving the mouse while holding middle button causes wild spinning.
- **Solution**: Move slowly and deliberately. Your cursor position affects the rotation center.
- **Why**: The rotation centre follows your cursor, so fast or off-centre movement swings the view.

### "I can't find my model anymore!"
- **Problem**: After zooming or panning, the model disappeared.
- **Solution**: Press `Ctrl + 0` to fit all objects in view, or press `0` for isometric view.
- **Why**: Zooming and panning can move the model out of the visible area, and Fit all recentres the camera.

### "Middle mouse button doesn't work!"
- **Problem**: Some laptops don't have a proper middle click.
- **Solution**: Go to Edit → Preferences → Display → Navigation. Choose a navigation style that works with your hardware (like "Touchpad").
- **Why**: The default navigation style expects a middle button, and the touchpad style replaces it.

### "The view is upside down!"
- **Problem**: Model appears flipped after rotating.
- **Solution**: Press `0` to reset to isometric view, or click a face on the navigation cube.
- **Why**: Rotating can leave the camera at an odd angle, and the standard views reset it.

---

## Check your understanding

{% quiz %}
? Which mouse action orbits (rotates) the view in FreeCAD?
- [ ] Scroll the mouse wheel
- [x] Hold the middle mouse button and move the mouse
- [ ] Hold Shift and the left mouse button
> Orbit is middle button plus movement. The scroll wheel zooms, and Shift plus the middle button pans.

? You have lost your model after zooming and panning. Which shortcut brings everything back into view?
- [ ] `1`
- [x] `Ctrl + 0`
- [ ] `Shift + F`
> `Ctrl + 0` fits all objects in view. Pressing `0` also returns you to the isometric view.

? Which number key shows the Top view?
- [ ] `1`
- [ ] `5`
- [x] `3`
> 1 is Front, 2 Back, 3 Top, 4 Bottom, 5 Left, 6 Right and 0 Isometric.
{% endquiz %}

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
