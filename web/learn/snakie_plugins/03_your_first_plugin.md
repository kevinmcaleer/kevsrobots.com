---
layout: lesson
title: Your First Plugin
author: Kevin McAleer
type: page
cover: /learn/snakie_plugins/assets/banners/03_your_first_plugin.jpg
date: 2026-11-21
previous: 02_the_plugin_rack.html
next: 04_when_plugins_break.html
description: Create the ClubKit plugin folder, write a one-command Snakie plugin in
  Python and load it into the Plugin Rack
percent: 28
duration: 5
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Extend Snakie with Python Plugins
- content:
  - section: Overview
    content:
    - name: Extend Snakie with Python Plugins
      link: 00_intro.html
    - name: How Plugins Work
      link: 01_how_plugins_work.html
    - name: The Plugin Rack
      link: 02_the_plugin_rack.html
  - section: Commands
    content:
    - name: Your First Plugin
      link: 03_your_first_plugin.html
    - name: When Plugins Break
      link: 04_when_plugins_break.html
    - name: Commands That Read Code
      link: 05_commands_that_read_code.html
    - name: Commands That Change Code
      link: 06_commands_that_change_code.html
  - section: Linters
    content:
    - name: Your First Linter
      link: 07_your_first_linter.html
    - name: Quick Fixes
      link: 08_quick_fixes.html
    - name: Rules for Your Hardware
      link: 09_rules_for_your_hardware.html
  - section: Blocks and Sharing
    content:
    - name: Blocks from a Plugin
      link: 10_blocks_from_a_plugin.html
    - name: Sharing Your Plugin
      link: 11_sharing_your_plugin.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to create the ClubKit plugin folder, write a one-command plugin and load it into the Plugin Rack.

## Meet ClubKit

Imagine you help run a robot club. Everyone writes MicroPython for the same little robot, and everyone makes the same handful of mistakes. Over this course you'll build **ClubKit**, a plugin that helps.

It starts very small: one button that says hi.

---

## Make the folder

1. Go to the `.snakie/plugins` folder from lesson 1
2. Inside it, make a folder called `clubkit`
3. Inside that, make an empty file called `__init__.py`

You should end up with this:

```
.snakie/
  plugins/
    clubkit/
      __init__.py
```

The folder name becomes the plugin's name in the rack. Use lower case and no spaces, because it has to be a valid Python name.

You can write the file in any editor, including Snakie itself. In Snakie, use **Open Folder** and choose the `clubkit` folder.

---

## Write the plugin

Put this in `__init__.py`:

```python
from snakie import Context, message, plugin


@plugin.command("clubkit.hi", "Say hi")
def say_hi(ctx: Context):
    return message("info", f"Hi from ClubKit! You're editing {ctx.file.name}.")
```

That's a complete plugin. Let's read it.

**Line 1** imports three things from `snakie`, the toolkit Snakie gives every plugin:

- `plugin` is how you register things
- `message` builds a message to send back
- `Context` is the type of `ctx`. It's only there so your editor can autocomplete `ctx.file`

**`@plugin.command("clubkit.hi", "Say hi")`** is a **decorator**. A decorator is a line starting with `@` that sits on top of a function and does something with it. This one tells Snakie "here's a command". It takes two pieces of text:

| Part | Purpose |
|---|---|
| `"clubkit.hi"` | The command's **id**. It must be different from every other command, so start it with your plugin's name |
| `"Say hi"` | The **title**. This is the label next to the RUN button |
{:class="table table-single"}

**The function** takes `ctx` and returns a message. `message` needs a level and some text. The level is `"info"`, `"warning"` or `"error"`.

---

## Load it

1. Save the file
2. In Snakie, open the **Plugins** panel
3. Click the **Reload** arrow

![ClubKit mounted in the Plugin Rack](/learn/snakie_plugins/assets/first_plugin.jpg){:class="img-fluid w-100"}

`clubkit` appears at the top of the rack, with **Say hi** listed under its name. The MOUNTED count has gone up by one.

4. Open a Python file to act on. Any will do
5. Click the `clubkit` module to open it, then click **RUN**

![The Say hi message](/learn/snakie_plugins/assets/say_hi.jpg){:class="img-fluid w-100"}

Your message appears at the bottom of the panel. You've extended your editor.

---

## The edit and reload loop

Plugins are loaded once, when the host starts. Snakie doesn't watch your file for changes. So the rhythm of plugin writing is:

1. Change the code
2. Save
3. Click **Reload**
4. Try it

Forgetting step 3 is the most common reason a change "doesn't work".

---

## A sample program to work on

From here on, ClubKit needs a robot program to chew on. Make a new folder somewhere for the club's code, and save this in it as `patrol.py`.

```python
from machine import Pin
import time
import clubbot

led = Pin(25, Pin.OUT)
buzzer = Pin(15, Pin.OUT)
sense = Pin(29, Pin.IN)

for lap in range(4):
    clubbot.forward(30)
    clubbot.turn(90)
    led.toggle()
    time.sleep(500)

clubbot.stop()
```

It drives the robot round a square. `clubbot` is the club's own helper module, which we're pretending lives on the robot. You don't need it, because we never run this program. ClubKit only reads it.

There are two mistakes hiding in there. Can you spot them? ClubKit will find both by lesson 9.

---

## Try it Yourself

1. Change the message text, save, and run the command **without** reloading. What do you see?
2. Reload and run it again
3. Change the level from `"info"` to `"warning"`, then `"error"`. How does the message change?
4. Add a second command called `clubkit.bye` with the title "Say bye". Copy the three lines and change them

---

## Common Issues

**Problem**: `clubkit` doesn't appear after Reload.
**Solution**: Check the path is exactly `.snakie/plugins/clubkit/__init__.py`, with two underscores each side of `init`.
**Why**: The host only loads `.py` files directly in `plugins`, and folders that contain `__init__.py`.

**Problem**: The message still shows the old text.
**Solution**: Save the file and click Reload.
**Why**: The host keeps the version it loaded until it's restarted.

**Problem**: Two of my commands do the same thing.
**Solution**: Give each one its own id.
**Why**: The id is how Snakie tells commands apart. Two commands with one id clash.

---

## Check your understanding

{% quiz %}
? What does the line starting with @plugin.command do?
- [ ] Runs the function straight away
- [x] Registers the function below it as a command
- [ ] Imports the plugin toolkit
> The decorator tells Snakie that the function underneath is a command, with an id and a title.

? You've edited your plugin and saved it. What else must you do?
- [ ] Restart your computer
- [ ] Reconnect the board
- [x] Click Reload in the Plugins panel
> The host loads plugins when it starts. Reload restarts it.

? Which of these is a good command id for ClubKit?
- [ ] hi
- [x] clubkit.hi
- [ ] Say hi
> Starting the id with the plugin's name keeps it from clashing with other plugins.
{% endquiz %}
