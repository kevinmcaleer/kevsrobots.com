---
layout: lesson
title: Commands That Change Code
author: Kevin McAleer
type: page
cover: /learn/snakie_plugins/assets/banners/06_commands_that_change_code.jpg
date: 2026-11-21
previous: 05_commands_that_read_code.html
next: 07_your_first_linter.html
description: Use the edit action in a Snakie plugin to rewrite the open file, and
  learn the rules for commands that change code safely
percent: 49
duration: 5
quiz_total: 36
date_updated: 2026-11-21
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

## From reading to writing

So far ClubKit only looks. Now it gets to change things.

The club has a rule: every program starts with a header comment saying which club it belongs to and what the file is called. Nobody remembers to add it. One button will.

---

## The edit action

A command changes the file by returning an **edit**:

```python
edit(new_content)
```

`new_content` is the **whole file** as you want it to be. Snakie swaps the text in the editor for your new text. There's no "insert at line 3" for commands. You take `ctx.file.content`, build the new version, and hand all of it back.

Two things to know:

- The edit changes the editor, not the file on disk. The tab is marked as unsaved, and it's up to the user to save
- Because it's a normal change to the editor, the user can look it over before saving

---

## The header command

Add `edit` to the import line and a constant under `PIN_USE`:

```python
from snakie import Context, edit, message, plugin, status

HEADER = "# Robot Club\n"
```

Then add the command. Put it above `list_pins`:

```python
@plugin.command("clubkit.header", "Add club header")
def add_header(ctx: Context):
    """Put the club's header comment at the top of the file."""
    if ctx.file.content.startswith(HEADER):
        return message("info", "This file already has the club header.")
    return [
        edit(HEADER + f"# File: {ctx.file.name}\n\n" + ctx.file.content),
        message("info", f"Added the club header to {ctx.file.name}."),
    ]
```

Save, Reload, open `patrol.py` and run **Add club header**.

![The header added to patrol.py](/learn/snakie_plugins/assets/header.jpg){:class="img-fluid w-100"}

Three lines have appeared at the top of the file, the tab shows it has unsaved changes, and the panel has two new messages. One is yours. The other, "Applied edit to patrol.py", is Snakie confirming it made the change.

---

## Run it twice

Click **RUN** again. This time you get "This file already has the club header", and the file isn't touched.

That check is the most important line in the function. Without it, every click would add another header. A command that's safe to run again and again is called **idempotent**, and it's what you want from any command that edits.

Ask of every editing command: *what happens if someone clicks it twice?*

---

## Rules for commands that edit

Changing someone's code is a serious thing to do. Good plugins follow a few rules.

**Change as little as you can.** Build the new text from the old text. Don't reformat lines you weren't asked to touch.

**Say what you did.** Return a message with the edit, so the user isn't left wondering what changed.

**Do nothing when there's nothing to do.** Return a message, not an edit that puts back the same text. An edit marks the file as changed even when it isn't.

**Keep the line endings.** Add `\n` after anything you insert, and keep the file's final newline.

---

## Try it Yourself

Here's a function to practise on. It's the heart of the header command with Snakie taken away, so you can run it right here. Change the code so it also adds a line saying `# Checked by ClubKit`.

{% capture snakie_code %}HEADER = "# Robot Club\n"

def add_header(name, content):
    if content.startswith(HEADER):
        return content
    return HEADER + "# File: " + name + "\n\n" + content

program = "import clubbot\nclubbot.forward(30)\n"

once = add_header("patrol.py", program)
twice = add_header("patrol.py", once)

print(once)
print("Same after a second run?", once == twice)
{% endcapture %}
{% include snakie.html code=snakie_code name="header.py" view="code" %}

Then, in your plugin:

1. Run **Add club header** on a new, empty file
2. Write `clubkit.tidy`, titled "Trim line ends", that removes spaces from the end of every line. `line.rstrip()` trims one line, and `"\n".join(lines)` puts them back together. Remember the final newline
3. Make `clubkit.tidy` report how many lines it changed, and return only a message when that number is zero

---

## Common Issues

**Problem**: The header is added again every time.
**Solution**: Check whether it's already there before you edit.
**Why**: The command gets the current text each time it runs, including the header it added last time.

**Problem**: The whole file went onto one line, or lost its last line.
**Solution**: Join lines with `"\n"` and add a final `"\n"`.
**Why**: `splitlines()` removes the line endings, so you have to put them back.

**Problem**: The file on disk hasn't changed.
**Solution**: Save the tab.
**Why**: An edit changes the editor. Saving is left to the user.

---

## Check your understanding

{% quiz %}
? What do you pass to edit()?
- [ ] Only the lines you want to add
- [x] The complete new text of the file
- [ ] A line number and some text
> A command's edit replaces everything in the editor, so you build and return the whole file.

? What does idempotent mean for a command?
- [ ] It runs very quickly
- [ ] It can't be undone
- [x] Running it twice gives the same result as running it once
> A safe editing command checks whether its work is already done.

? After a command returns an edit, is the file saved?
- [ ] Yes, straight away
- [x] No, the tab is marked unsaved and the user decides
- [ ] Only if a board is connected
> The edit changes the editor's text. The user saves it when they're happy.
{% endquiz %}
