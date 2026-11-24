---
title: Saving, Sharing and Printing
description: Save BetterCad models to a file or your account, share a link, and use the Print panel to check a part before it reaches the printer
layout: lesson
type: page
cover: /learn/bettercad/assets/banners/10_saving_sharing_and_printing.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to save a model to a file, explain what an account adds, and run the overhang, thin wall and orientation checks before you print.

You can design a part now. This lesson is about looking after it: where your work is kept, how to hand it to someone else, and how to catch print problems while they're still free to fix.

---

## Where your work lives

BetterCad autosaves. A few seconds after you stop making changes, the model is written to your browser's storage, and a small **Autosaved** note appears. Close the tab by accident and the next visit offers to recover it.

That's a safety net, not a filing system. Browser storage belongs to one browser on one computer, and clearing your browsing data clears it. For anything you care about, save it properly.

## Save to a file

Press <kbd>Ctrl</kbd>+<kbd>S</kbd>, or click **Save**. Your model is saved as a `.bcad` file.

| Browser | What Save does |
|---|---|
| Chrome, Edge | Asks where to put the file the first time, then writes straight back to it on every save |
| Safari, Firefox | Downloads a fresh `.bcad` each time |
{:class="table table-single"}

A `.bcad` file holds the recipe: your timeline, sketches and parameters. It's small, and it opens with the whole history intact. An STL is the finished shape only. Keep the `.bcad`. You can always make another STL from it, but never the other way round.

The arrow beside Save opens the rest of the file menu.

![The Save menu](/learn/bettercad/assets/save_menu.jpg){:class="img-fluid w-100"}

| Item | What it does |
|---|---|
| Share… | Makes a link to a model in your account |
| Versions… | Earlier saves of a model in your account |
| Export… | STL, 3MF, STEP and more, as in lesson 3 |
| Build steps (.zip)… | A picture of every timeline step with a written guide |
| Send to kevsrobots… | Sends those build steps to one of your kevsrobots projects |
| Save as…, Open…, New model | The usual file commands |
| Import STL, STEP or 3MF… | Brings someone else's part into your model |
{:class="table table-single"}

**Build steps** deserves a second look. It turns your timeline into illustrated instructions, which is most of a build guide written for you.

## Save to your account

BetterCad never needs an account. But if you sign in with your kevsrobots.com account, three things are added:

- **My models**: your designs on any computer, not just this one
- **Versions**: every save is kept, so you can look at last Tuesday's attempt and restore it
- **Share links**: a link anyone can open to spin the model round, measure it, download it or open their own copy

A shared link opens read-only and needs no account to view. It's a good way to show a design in a Discord chat or ask "will this print?"

## Get ready to print

Open the account menu at the top right and choose **Print settings**. The Print panel opens where the Browser was. Open the **Bracket** sample to follow along.

![The Print panel](/learn/bettercad/assets/print_panel.jpg){:class="img-fluid w-100"}

Start at the top. Pick your **Printer** from the list, or choose Custom and type your bed size. BetterCad draws the build volume around the model and tells you whether it fits: **The model fits a Prusa Core One.** **Centre on bed** moves the model to the middle with its lowest point on the plate.

### Overhangs

Under **Checks**, click **Overhangs**.

![The overhang check on the bracket](/learn/bettercad/assets/overhangs.jpg){:class="img-fluid w-100"}

Anything that would print in mid-air is tinted and listed. The bracket reports **Needs support: 1 area, 13 mm²**. That's the top of the hole in the upright: a small roof with nothing under it. An area that size will bridge without trouble, and now you know it's there before the printer finds it for you.

The limit is 45° by default, which suits most printers.

### Thin walls

Click **Thin walls**.

![The thin wall check](/learn/bettercad/assets/thin_walls.jpg){:class="img-fluid w-100"}

Walls thinner than about two nozzle widths (0.8 mm with a standard nozzle) may not print at all. The bracket passes: **No walls thinner than 0.8 mm.**

### Orientation

How a part lies on the bed changes its strength, its finish and whether it needs support. Under **Orientation**, BetterCad offers the model as it lies and two alternatives. Click **Suggestion 2**.

![The bracket turned to a suggested orientation](/learn/bettercad/assets/orientation.jpg){:class="img-fluid w-100"}

BetterCad reports **Turned the model and laid it on the middle of the bed**. The suggestions weigh how much touches the bed, how much overhangs and how tall the part stands. Turning the model is a step on the timeline, so <kbd>Ctrl</kbd>+<kbd>Z</kbd> puts it back.

### The rest of the panel

Further down are the **Hole offset** and **Fits** settings that tune holes and push-fits to your own printer, and **Material** and **Infill**, which feed the weight estimate in Properties. The follow-on course, *Print It Right*, covers those properly.

## A pre-flight checklist

Before any print:

1. Does it fit the bed?
2. Is the biggest flat face down?
3. Overhang check: anything large that needs support, and can a chamfer or a turn remove it?
4. Thin wall check: anything under 0.8 mm?
5. Export a 3MF, and keep the `.bcad`.

---

> **Teachers**: autosave lives in one browser on one computer, and clearing browsing data clears it, so on shared or wiped school machines it is not a safe place for work. Have learners press Ctrl+S at the end of every session and save the `.bcad` file to a USB stick, network drive or cloud folder. An account is optional. Signing in with a kevsrobots.com account adds My models, Versions and share links, but nothing in the course needs one, so skip it if your school restricts accounts. A shared link opens read-only for anyone who has it, so agree rules before learners post links anywhere public.

## Try it Yourself

1. Run both checks on your enclosure from lessons 6 to 9. Does the cable hole show up as an overhang?
2. Open the **Phone stand** sample and run the overhang check. What does BetterCad say?
3. Use **Build steps (.zip)…** on your enclosure and open the guide it writes.

## Common Issues

- **Problem**: I closed the tab without saving.
- **Solution**: Open BetterCad again in the same browser and accept the offer to recover the model.
- **Why**: Autosave keeps your recent models in the browser's storage.

- **Problem**: Share is greyed out, or asks me to sign in.
- **Solution**: Sign in and save the model to your account first.
- **Why**: A share link points at a model stored on the server. A file on your computer can't be linked to.

- **Problem**: The overhang check flags a flat span between two walls.
- **Solution**: If it's short, ignore it. Printers can bridge a gap of a few centimetres.
- **Why**: The check doesn't recognise bridges yet, so it flags every downward face.

## Check your understanding

{% quiz %}
? Why keep the .bcad file as well as the STL?
- [ ] The slicer needs both
- [x] The .bcad holds the steps, so the model can still be edited
- [ ] STL files expire
> An STL is only the final shape. The .bcad is the recipe that rebuilds and changes it.

? What does the overhang check highlight?
- [ ] Walls that are too thin
- [ ] Parts that are too big for the bed
- [x] Downward faces that would print with nothing underneath
> Faces steeper than the limit, 45° by default, need support or a redesign.

? Do you need an account to design and export a part in BetterCad?
- [ ] Yes, always
- [x] No, an account only adds cloud saving, versions and share links
- [ ] Only for STEP files
> The editor works signed out and offline. Signing in is optional.
{% endquiz %}
