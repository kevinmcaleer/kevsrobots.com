---
title: Getting started with projects
description: A step-by-step guide to writing your first project on kevsrobots.com
excerpt: Add your parts, find them in the catalog, draw the wiring and write the build steps, one step at a time.
layout: content
permalink: /projects/getting-started/
# Help page: no "support my work" box or likes.
thanks: false
hide_likes_and_comments: true
---

{% include nav_projects.html %}

<link rel="stylesheet" href="/assets/css/project-getting-started.css?v={{ site.time | date: '%s' }}">

<div class="kr-guide" markdown="1">

# {{page.title}}

This guide walks you through your first project, from an empty page to a write-up other makers can follow. You don't need to know Markdown to start: most things have a button, and the editor shows the result as you type.

**On this page**

1. [Start a project](#start-a-project)
2. [Add your parts: the Bill of Materials](#add-your-parts-the-bill-of-materials)
3. [Find parts in the catalog](#find-parts-in-the-catalog)
4. [Add a part that isn't in the catalog](#add-a-part-that-isnt-in-the-catalog)
5. [Draw the wiring diagram](#draw-the-wiring-diagram)
6. [Write the assembly instructions](#write-the-assembly-instructions)
7. [Tables](#tables)
8. [Callouts](#callouts)
9. [Publish it](#publish-it)

---

## Start a project

Log in, then click **Create New** in the Projects bar. You get an empty project in the editor.

1. Type a **title** at the top. A clear name ("Line Follower Bot", not "My robot") helps people find it.
2. Add a **short description**: one line that says what it does.
3. Set the **difficulty**, the **build time** in minutes and a few **tags**, such as `pico` or `3d-printing`.
4. Click into the page under the toolbar and start writing.

![The project editor, with the title, details and the start of the write-up](/assets/img/projects/getting-started/editor-overview.webp)

Your work saves as you go. The **On this page** list on the left fills in from your headings, and the **Project completeness** box in the corner lists what's left to add.

> **The `/` menu is the quickest way in.** Type `/` at the start of an empty line to add a section (Bill of Materials, Assembly instructions, Wiring diagram, Gallery, Videos), a part, a file or a diagram. Keep typing to narrow the list, then press Enter.

The **?** button at the end of the toolbar opens the Markdown guide: a short reference for everything the editor can do.

---

## Add your parts: the Bill of Materials

The Bill of Materials is the list of parts someone needs to buy or print to build your project. It's a table under a heading called **Bill of Materials**.

1. On an empty line, type `/` and pick **Bill of Materials**.
2. The editor adds the heading and a table with Part, Qty, Price, Total and Notes columns, with the first part name ready to type over.
3. Type the part's name, press Tab to move to **Qty**, and Tab again for **Price**.
4. Click the **+** under the table to add another row.

![A Bill of Materials with prices, totals and a Total row](/assets/img/projects/getting-started/bom-table.webp)

A few things are done for you:

- **Totals.** With Qty and Price columns, each row's Total and the Total row at the bottom are worked out as you type.
- **Prices.** Write prices with a dot (`4.99`). Pick a currency from the Price column's menu (the small `…` above it) and every price is shown in it, so `4` becomes `£4.00`.
- **Quantity.** Click a Qty cell and use its up and down arrows, or Alt+Up and Alt+Down.

### Link each part to the catalog

A plain part name is fine, but a part **linked to the catalog** shows its picture, its details and where to buy it. Readers can hover over it to see a card, and the wiring diagram can draw it with its real pins.

1. Click in the part's cell and type `/` then **Part**.
2. Start typing the part's name. Matching parts from the catalog appear with their pictures.
3. Pick one. The cell becomes a link to that part, and if the catalog has a price for it, the editor offers to fill in the Price.

If some parts aren't linked yet, a yellow bar above the editor says how many. Click **Review** to go through them.

---

## Find parts in the catalog

The [Parts Catalog](/parts/) is a shared list of the parts makers use in their projects, with photos, pinouts, specifications and shop links.

- **Search** by name, part number, maker or tag in the box at the top.
- **Browse** by category: each one is a shelf. Click **Show all** to see the whole category.
- **Filter** on the left by manufacturer, by status (Verified, Draft) or to show only parts with a price.

Click a part to open its page. It has an **Overview** with specifications and pinout, the **Symbol** used in diagrams, **Photos**, the edit history under **Versions**, and **Talk** for questions about the part.

### Pick several parts at once

From the editor, type `/` then **Part** on an empty line outside the table and choose **Browse the catalog…** at the bottom of the list. Tick the parts you need and click **Add to Bill of Materials**: each one gets its own row, already linked.

---

## Add a part that isn't in the catalog

If your part isn't there yet, you can add it. It helps the next maker too.

When a row in your Bill of Materials isn't linked, hover over its name and click **New part**.

![The New part button on a Bill of Materials row](/assets/img/projects/getting-started/bom-new-part-button.webp)

The **Add a part to the catalog** box opens with the name from your table filled in.

![The Add a part to the catalog box](/assets/img/projects/getting-started/bom-new-part-dialog.webp)

1. Check the **name**. If a similar part is already in the catalog, it's listed under the name: click **Use this** to link to it instead.
2. Add **pictures**: drop, paste or click to choose. Say who took them, so the right licence goes with them.
3. Fill in **Where to buy it** and the **Price** if you know them.
4. Open **More details** to add a description, category, datasheet, manufacturer and part number. Only the name is needed.
5. Click **Add part**. Your table row is linked to the new part.

You can also type `/` then **Part** in a cell, type the name, and choose **Create "…"** at the bottom of the list. New parts can be used in your project straight away, and join the catalog for everyone once they've been checked.

---

## Draw the wiring diagram

The wiring workspace draws your parts with their real pins, and you wire them up by dragging between pins. It uses the parts in your Bill of Materials, so add those first.

1. Give your project a title (the wiring is saved with the project).
2. On an empty line, type `/` and pick **Wiring diagram**. The editor adds a **Wiring** heading and opens the workspace in the page.

![The wiring workspace with the Bill of Materials panel on the right](/assets/img/projects/getting-started/wiring-workspace.webp)

The **Bill of Materials** panel on the right lists every row of your table:

- **Add** puts that part on the canvas. **Add all** puts them all on.
- A part linked to the catalog is drawn as its photo with its pins. Your microcontroller (a Pico, say) becomes the board.
- A part that isn't linked yet shows as a plain card. Click **Link…** to find it in the catalog, or **+** to add it as a new part.
- A catalog part with no pins yet gets an **Edit part** button, where you can add its pins.

![Parts on the canvas, ready to wire](/assets/img/projects/getting-started/wiring-parts.webp)

To wire it up:

1. **Drag from one pin to another** to connect them. The connections are listed under the canvas.
2. Switch between **Life-like** and **Schematic** at the top to change the drawing style.
3. Use **Add parts** to bring in a part from the catalog. It's added to your Bill of Materials too.
4. Warnings, such as a pin that isn't wired, appear across the top. They're only a reminder: you can save anyway.
5. Click **Save**. The drawing is placed in your page under the Wiring heading.

Click **Full screen** for more room (Esc brings you back), and **Pinout** (**Insert pinout table** in full screen) to add a table of every connection under the diagram. **Export** saves the drawing as an SVG or PNG. To change the wiring later, hover over the diagram in the editor and click **Edit wiring**.

---

## Write the assembly instructions

This is the part readers come for: how to build it, one step at a time.

1. On an empty line, type `/` and pick **Assembly instructions**. The editor adds the heading and starts a numbered list.
2. Write one action per step, and press Enter for the next number.
3. For a longer build, break it into stages with smaller headings (`## Fit the motors`, `## Wire the sensor`). They show in the **On this page** list, so readers can jump to a stage.

Some tips that make instructions easier to follow:

- **Add a picture to each step.** Drag a photo onto the page, or paste it, and it's uploaded where you drop it. Click a picture to give it a title (shown as a caption), change its size (Full, Half, Cover or Small) or swap it for another.
- **Label your photos.** Click a picture and choose **Draw on it** to add arrows, numbers and labels over it.
- **Link the parts you mention.** Type `/` then **Part** in the middle of a sentence to link to a part from your Bill of Materials.
- **Point out the tricky bits** with a callout (see [Callouts](#callouts) below), such as a warning before you solder or a tip that saves time.
- **Split a very long build into pages.** Type `/` then **Page break** between two sections. Readers get Previous and Next buttons, and can still choose to read it as one page.

A short example of how a step looks in Markdown:

```markdown
# Assembly instructions

## Fit the motors

1. Push the two N20 motors into the chassis, wires facing in.
2. Screw the motor brackets down with four M2 screws.

![The motors fitted in the chassis](motors-fitted.jpg){half}

> [!warning] Mind the polarity
> Check both motors turn the same way before you solder the wires.
```

---

## Tables

Tables are handy for anything with rows and columns: pin connections, settings, print settings or a parts list.

To add one, click the **table** button on the toolbar, or type `/` then **Table**. You can also type it yourself:

```markdown
| Pin  | Connects to | Notes        |
| ---- | ----------- | ------------ |
| GP0  | Motor A in1 | PWM          |
| GP1  | Motor A in2 |              |
```

Once it's in the page, click any cell to edit the table directly:

![The table column menu with insert, delete and alignment options](/assets/img/projects/getting-started/table-menu.webp)

- **Tab** moves to the next cell. **+** to the right adds a column, **+** underneath adds a row.
- The `…` above a column opens its menu: insert a column left or right, delete it, or align it left, centre or right.
- The handle at the start of a row opens the row menu: insert or delete rows, or **Split table here**.
- Drag a row or column by its handle to move it.
- Select text in a cell to get a small formatting bar for bold, italics and links. **Shift+Enter** starts a new line in the same cell.

---

## Callouts

A callout is a coloured box that makes a note stand out from the text around it: a warning, a tip, or something to check before carrying on.

On an empty line, type `/` then the kind you want, such as **Tip callout** or **Warning callout**. Or type it yourself:

```markdown
> [!tip] Print the chassis first
> It takes about four hours, so start it before you do the wiring.
```

The first line sets the type and the title. Every line under it starts with `>`.

![Picking a callout type from the list](/assets/img/projects/getting-started/callout-types.webp)

To change the type, click the callout's icon, or put the cursor on its type, and pick another from the list. The types you'll use most in a project:

| Type | Use it for |
| ---- | ---------- |
| `[!note]` | Something worth knowing |
| `[!tip]` | A way to save time or get a better result |
| `[!info]` | Background or extra detail |
| `[!warning]` | Something that could go wrong: check this first |
| `[!danger]` | Something that could hurt you or break a part, such as mains power or a LiPo battery |
| `[!success]` | How to tell a step worked |
| `[!question]` | A common question and its answer |

Add `-` after the type (`> [!info]- Full parts spec`) to make a callout that starts folded, so long detail doesn't get in the way.

---

## Publish it

Projects start as a **Draft**, which only you can see. When you're ready:

1. Check the **Project completeness** box in the corner. It lists anything missing, such as a cover picture or a difficulty level. Click an item to go to it.
2. Click **View** at the top to see the page as readers will.
3. Turn on the **Published** switch. Your project now appears in the Projects Hub and on your profile.

You can keep editing after publishing. Changes save as you go and show on the project page straight away.

**Ready to start?** [Create a new project](/projects/editor.html) or browse the [Projects Hub](/projects/hub) to see how other makers write theirs.

</div>
