---
layout: lesson
title: Sharing Your Parts
author: Kevin McAleer
type: page
cover: /learn/snakie_part_editor/assets/banners/12_sharing_your_parts.jpg
date: 2026-11-21
previous: 11_making_a_microcontroller_board.html
next: 13_summary.html
description: Package your Snakie parts as a library with versions, add a 3D model
  and learn how the community registry shares parts with other makers
percent: 91
duration: 6
quiz_total: 41
date_updated: 2026-11-24
navigation:
- name: Make Your Own Parts and Boards in Snakie
- content:
  - section: Overview
    content:
    - name: Make Your Own Parts and Boards in Snakie
      link: 00_intro.html
    - name: What a Part Is
      link: 01_what_a_part_is.html
  - section: Drawing the Part
    content:
    - name: A Tour of the Part Editor
      link: 02_tour_of_the_part_editor.html
    - name: Board Size and Details
      link: 03_board_size_and_details.html
    - name: Pins, Types and Capabilities
      link: 04_pins_types_and_capabilities.html
    - name: Holes, Components and Labels
      link: 05_holes_components_and_labels.html
    - name: Tracing a Board Photo
      link: 06_tracing_a_board_photo.html
  - section: Making It Smart
    content:
    - name: Schematic Symbol and Electrical Details
      link: 07_schematic_symbol_and_electrical_details.html
    - name: Drivers and Built-In Help
      link: 08_drivers_and_built_in_help.html
    - name: Using Your Part in a Project
      link: 09_using_your_part_in_a_project.html
    - name: Giving Your Part Its Own Blocks
      link: 10_giving_your_part_its_own_blocks.html
  - section: Boards and Sharing
    content:
    - name: Making a Microcontroller Board
      link: 11_making_a_microcontroller_board.html
    - name: Sharing Your Parts
      link: 12_sharing_your_parts.html
    - name: Summary and Next Steps
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to share a part as a folder or a library, choose the right version number, and explain how the community registry works.

A part you've made is useful to you. A part you've **shared** is useful to everyone with the same board. Because parts are just folders of text files, sharing them is refreshingly simple.

---

## Share one part

The quickest way to share is to send the folder.

1. Click **Reveal the parts folder** in the Library toolbar.
2. Open `my-parts`.
3. Zip the `vl53l0x-distance-sensor` folder and send it.

Your friend unzips it into their own `my-parts` folder and clicks **Refresh library**. Everything travels together: the pins, the photo, the help and the blocks.

---

## Make a library

For more than a couple of parts, make a library of your own. It's a folder with a `library.yml` and one folder per part:

```
robot-club-parts/
  library.yml
  vl53l0x-distance-sensor/
    parts.yml
    image.png
    help.md
    blocks.yml
  tiny-robot-brain/
    parts.yml
```

The `library.yml` describes the collection:

```yaml
id: robot-club-parts
name: Robot Club Parts
description: Sensors and boards we use at robot club.
author: Robot Club
homepage: https://github.com/your-name/robot-club-parts
version: 1.0.0
```

Put that folder inside Snakie's parts folder and it shows up in the Library as its own section. In the Part Editor, **Saved to** in the top bar becomes a list, so you can choose which library a part is saved into.

Put the folder on GitHub and you get history, backups and an easy way for other people to suggest fixes. The Git lesson in [Coding Faster in Snakie](/learn/snakie_code_tools/) covers the basics.

---

## Versions

Parts and libraries both carry a version number in three parts, such as `1.2.0`.

| Change | Example | Bump |
|---|---|---|
| A fix that changes nothing for users | Nudged a label | 1.2.0 to 1.2.1 |
| Something new that doesn't break projects | Added blocks or a photo | 1.2.0 to 1.3.0 |
| A change that could break projects | Renamed or removed a pin | 1.2.0 to 2.0.0 |
{:class="table table-single"}

Renaming a pin is the one to be careful with. Projects remember their wires by pin name, so a rename can leave wires with nowhere to go.

---

## The community registry

Snakie has a **community registry**: a list of approved part libraries kept in a GitHub repository. Click **Add library** in the Library toolbar and Snakie fetches the list and shows the libraries you don't have yet. Installing one downloads it into your parts folder.

Snakie also compares each installed library's version with the registry. When there's a newer one, you're offered the update. The **Refresh library** button checks straight away.

To get your own library listed, you open a pull request against the registry, adding an entry like this:

```json
{
  "id": "robot-club-parts",
  "name": "Robot Club Parts",
  "description": "Sensors and boards we use at robot club.",
  "author": "Robot Club",
  "repo": "https://github.com/your-name/robot-club-parts",
  "version": "1.0.0",
  "tags": ["sensor", "robot"]
}
```

The registry lives at [github.com/kevinmcaleer/snakie-parts](https://github.com/kevinmcaleer/snakie-parts), along with the Standard Parts. It's also a great place to read well-made parts.

---

## Add a 3-D model

One more thing makes a part complete. Open it in the Part Editor and click **3-D**.

![The 3-D view of the Part Editor](/learn/snakie_part_editor/assets/model_tab.jpg){:class="img-fluid w-100"}

Click **Link a model…** and choose an **STL** or **DAE** file. Snakie copies it into the part's folder, so it travels with the part. Then:

- **Size** sets the units, in case your model was saved in metres.
- **Orientation** turns it until it lies flat, with Z pointing up.
- **Position** slides it until it sits on the part's footprint.

Snakie uses the model wherever the part is shown in 3-D, such as the **Build** workspace. You can design the model yourself. The [Learn BetterCad](/learn/bettercad/) course shows you how, and exports STL files that work here.

---

## A checklist before you share

- The size is measured, not guessed
- Every pin has the right name, type and capabilities
- I2C pins have their signal set, and the address is recorded
- Supply limits are filled in
- The driver installs, and the help has a working example
- You have the right to share the photo
- The version number has gone up

> **Teachers**: For a class, zipping the part folder is the simplest way to share, and keeps work inside the group. The community registry is public: adding a library means a pull request on GitHub, which needs a GitHub account and puts the learner's name and work on a public page, so decide with your school's rules in mind before anyone does it. Learners should only share photos and files they have the right to share.

---

## Try it Yourself

1. Zip your part's folder and look inside. Which files are there?
2. Copy it to a new folder with its own `library.yml`.
3. Click **Refresh library** and find your new library in the list.
4. Click **Add library** and see what the registry offers.
5. Bump your part's version to `0.2.0` and save.

---

## Common Issues

**Problem**: **Add library** says no libraries were found.
**Solution**: Check your internet connection and click **Retry**.
**Why**: The registry is fetched from GitHub, so it needs a connection. You'll see the same message when you already have every listed library.

**Problem**: My friend's copy of the part has no picture.
**Solution**: Send the whole folder, not just `parts.yml`.
**Why**: The image, help and blocks are separate files beside it.

**Problem**: My new library doesn't appear.
**Solution**: Check it has a `library.yml` with an `id`, then click **Refresh library**.
**Why**: The manifest is what makes a folder a library.

---

## Check your understanding

{% quiz %}
? What do you send to share one part completely?
- [ ] Just parts.yml
- [x] The whole part folder
- [ ] A screenshot
> The photo, help and blocks are separate files in the same folder.

? You rename a pin from SDA to DATA. What kind of version bump is that?
- [ ] A patch, 1.2.0 to 1.2.1
- [ ] A minor bump, 1.2.0 to 1.3.0
- [x] A major bump, 1.2.0 to 2.0.0
> Existing projects wire to the pin by name, so this can break them.

? How does a library get into the community registry?
- [ ] It's added automatically when you save
- [x] Through a pull request to the registry on GitHub
- [ ] By emailing the folder to a friend
> The registry is a list in a GitHub repository, and changes to it are reviewed.
{% endquiz %}
