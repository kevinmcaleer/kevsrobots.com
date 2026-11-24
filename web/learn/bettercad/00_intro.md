---
layout: lesson
title: Learn BetterCad
author: Kevin McAleer
type: page
cover: /learn/bettercad/assets/banners/00_intro.jpg
date: 2026-11-20
next: 01_look_around_and_sketch.html
description: Design your own 3D printable parts in BetterCad, the free CAD tool that
  runs in your browser
percent: 7
duration: 6
quiz_total: 39
date_updated: 2026-11-24
navigation:
- name: Learn BetterCad
- content:
  - section: Overview
    content:
    - name: Learn BetterCad
      link: 00_intro.html
  - section: Your First Part
    content:
    - name: Look Around and Sketch
      link: 01_look_around_and_sketch.html
    - name: Extrude and Fillet
      link: 02_extrude_and_fillet.html
    - name: Export Your First Print
      link: 03_export_your_first_print.html
  - section: Sketching Properly
    content:
    - name: Sketching With Constraints
      link: 04_sketching_with_constraints.html
    - name: Dimensions
      link: 05_dimensions.html
  - section: Building With Shapes
    content:
    - name: Push, Pull and Shapes
      link: 06_push_pull_and_shapes.html
    - name: Combining Parts and Holes
      link: 07_combining_parts_and_holes.html
    - name: Fillets, Chamfers and Shells
      link: 08_fillets_chamfers_and_shells.html
  - section: Models That Change
    content:
    - name: The Timeline and Parameters
      link: 09_the_timeline_and_parameters.html
    - name: Saving, Sharing and Printing
      link: 10_saving_sharing_and_printing.html
  - section: Project
    content:
    - name: Project - A Phone Stand
      link: 11_project_phone_stand.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Overview

[BetterCad](https://bettercad.kevsrobots.com) is a free CAD tool for 3D printing that runs in your browser. There's nothing to install and you don't need an account. It borrows the best of two worlds: precise, constrained sketches like Fusion, and friendly push, pull and combine like Tinkercad.

In this course you'll go from an empty screen to a phone stand you designed yourself, ready to print. Along the way you'll make a filleted block, a plate with a screw hole placed to the millimetre, and a small electronics enclosure with screw posts.

![The bracket sample open in BetterCad](/learn/bettercad/assets/bracket.jpg){:class="img-fluid w-100"}

## Course Content

- Looking around, sketching a rectangle and extruding it into a solid
- Rounding edges and exporting a file your slicer can print
- Constraints and dimensions, and what "fully constrained" means
- Shapes from the Library, push/pull, move and turn
- Screw holes that fit, and cutting one shape out of another
- Fillets, chamfers and hollowing a part with Shell
- The timeline and parameters, so a model can change size without being redrawn
- Saving, sharing and the built-in print checks
- Project: a phone stand that prints with no supports

## Key Results

After this course you will be able to:

- Design a printable part from a sketch or from ready-made shapes
- Add holes that a real screw slides through on a real printer
- Change a design by editing a number, not by starting again
- Check a model for overhangs and thin walls before you print it
- Export STL, 3MF and STEP files

## What you'll need

- A computer with a recent browser. Chrome and Edge save straight back to your files. Safari and Firefox work too
- A mouse with a scroll wheel makes life easier, but a trackpad is fine
- A 3D printer and slicer if you want to print what you make. You can do the whole course without one

BetterCad works offline once it has loaded, and everything you make is saved in your browser as you go.

## A tour of the screen

Open [BetterCad](https://bettercad.kevsrobots.com). The first time, you'll see a start card and the first step of a short tour.

![BetterCad the first time you open it](/learn/bettercad/assets/first_run.jpg){:class="img-fluid w-100"}

The 3D view fills the whole window. Everything else floats over it:

| Where | What it is |
|---|---|
| Top | The **toolbar**. Tabs for Model, Assemble and Inspect, with the tools grouped underneath |
| Top right | Undo and redo, **Find a command**, **Save**, **Help** and your account |
| Left | The **Browser**: every body and sketch in your model |
| Right | **Properties**: details of whatever you have selected |
| Bottom | The **timeline**: every step you've taken, in order |
| Top right of the view | The **view cube**. Click a face to look from that side |
{:class="table table-single"}

Click **Bracket** on the start card to open a sample. Then click the bracket itself. Its size, volume and estimated print weight appear in Properties.

![The bracket selected, with its properties on the right](/learn/bettercad/assets/bracket_selected.jpg){:class="img-fluid w-100"}

Two things worth knowing from day one:

- **Every tool has a key.** Hover a tool and its tooltip shows it, like "Sketch (K)"
- **Esc always backs out one step.** If you're ever lost, press it until you're back at Select

Can't find a tool? Press <kbd>/</kbd> or <kbd>Ctrl</kbd>+<kbd>K</kbd> and type what you want.

![Find a command](/learn/bettercad/assets/command_palette.jpg){:class="img-fluid w-100"}

## How the course works

Each lesson is five to ten minutes and builds one thing. Steps you should do are numbered. Keys to press look like <kbd>K</kbd>. Names of buttons and panels are in **bold**.

BetterCad's own first-run tour covers the same ground as lessons 1 to 3, in the same words, so you can follow either. To see it again, open **Help** and choose **Take the tour**.

Every lesson ends with an exercise, some common problems and a short quiz.

> **BetterCad is in alpha.** It's being worked on all the time, so a button may have moved since these screenshots were taken. Save your work to a file now and then.

## Prerequisites

None. If you've used Tinkercad or Fusion, some of this will feel familiar, and the course points out where BetterCad does things differently. If you've never used CAD, you're in the right place.

> **Teachers**: this course suits complete beginners to CAD, with no prerequisites. It is 12 lessons after this one, each written as five to ten minutes of work, so plan several short sessions and allow longer for a class new to 3D views. Each learner needs only a computer with a recent browser: BetterCad runs in the browser, needs no install and no account, and works offline once it has loaded. A mouse with a scroll wheel helps. A 3D printer is optional, because the whole course can be done on screen. BetterCad is in alpha, so open the lesson yourself before class in case a button has moved. On shared computers, read the notes in lessons 3 and 10 about where files end up.

## Try it Yourself

1. Open the **Bracket** sample and click the bracket. How much does BetterCad think it will weigh?
2. Click the faces of the view cube to look from the top, the front and the right.
3. Press <kbd>/</kbd> and type `top`. Press <kbd>Enter</kbd>.

## Check your understanding

{% quiz %}
? Where does BetterCad run?
- [ ] In an app you install
- [x] In your browser, with no account needed
- [ ] Only on a tablet
> It runs in the browser, works offline once loaded, and never needs an account.

? What does the timeline at the bottom show?
- [ ] How long you have been working
- [x] Every step that built the model, in order
- [ ] Your print queue
> Each modelling action becomes a step on the timeline, and you can go back and change any of them.

? You're lost in a tool. Which key backs out one step at a time?
- [x] Esc
- [ ] Delete
- [ ] F1
> Esc always backs out one layer.
{% endquiz %}
