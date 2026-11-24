---
layout: lesson
title: When Plugins Break
author: Kevin McAleer
type: page
cover: /learn/snakie_plugins/assets/banners/04_when_plugins_break.jpg
date: 2026-11-21
previous: 03_your_first_plugin.html
next: 05_commands_that_read_code.html
description: Learn what Snakie shows when a plugin has a syntax error or crashes,
  and how to find and fix the bug quickly
percent: 35
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

## Break it on purpose

Every plugin author breaks their plugin sooner or later. It's much less alarming when you've seen it before, so let's do it now, on purpose, twice.

There are two kinds of failure, and Snakie shows them in different places.

| Kind | When it happens | Where you see it |
|---|---|---|
| The plugin won't **load** | When you click Reload | In the rack |
| A command **crashes** | When you click RUN | In the messages |
{:class="table table-single"}

---

## Failure one: it won't load

Open `__init__.py` and delete the colon from the end of the `def` line:

<!-- code-check: skip -->
```python
@plugin.command("clubkit.hi", "Say hi")
def say_hi(ctx: Context)
    return message("info", f"Hi from ClubKit! You're editing {ctx.file.name}.")
```

Save and click **Reload**.

![A plugin that failed to load](/learn/snakie_plugins/assets/broken.jpg){:class="img-fluid w-100"}

`clubkit` has dropped out of MOUNTED into a separate group at the bottom, and where its commands used to be it now says:

```
Failed to load — SyntaxError: expected ':' (__init__.py, line 5)
```

That one line tells you what's wrong (`expected ':'`), which file, and which line. If the text is cut short, hover over the module to read all of it, or drag the edge of the panel to make it wider.

Notice what **didn't** happen. Snakie didn't crash. The other five plugins loaded as usual. One broken plugin can't take the rest down.

Put the colon back, save and Reload. `clubkit` returns to MOUNTED.

---

## Failure two: it crashes when run

Some bugs only show up when the code runs. Add a line that can't work:

```python
@plugin.command("clubkit.hi", "Say hi")
def say_hi(ctx: Context):
    laps = 10 / 0
    return message("info", f"Hi from ClubKit! You're editing {ctx.file.name}.")
```

Save and Reload. This time the plugin loads happily, because the file is valid Python. Now click **RUN**.

![A command that crashed](/learn/snakie_plugins/assets/crash.jpg){:class="img-fluid w-100"}

The message area says:

```
Say hi failed: ZeroDivisionError: division by zero
```

You get the command's title, the kind of error and its message. Again, nothing else is harmed. Delete the bad line, save and Reload.

---

## Finding a bug

You don't get a line number for a crash, so here are three ways to track one down.

**1. Return what you know.** A message is the plugin author's `print`. Send back the value you're unsure about:

```python
return message("info", f"content has {len(ctx.file.content)} characters")
```

**2. Test the idea outside Snakie.** Most plugin code is ordinary string handling. Try the tricky line in a Python prompt first, where you get a full traceback.

**3. Catch the error yourself.** For a command that might fail on odd files, say what went wrong in your own words:

```python
@plugin.command("clubkit.hi", "Say hi")
def say_hi(ctx: Context):
    if not ctx.file.name:
        return message("warning", "Open a file first.")
    return message("info", f"Hi from ClubKit! You're editing {ctx.file.name}.")
```

A friendly warning beats a `TypeError` every time, especially if other people will use your plugin.

> `print()` inside a plugin doesn't appear in Snakie. The host keeps printed text away from its conversation with the editor, so use a message when you want to see a value.

---

## Three habits that save time

- **Reload after every small change.** If it breaks, you know which change did it
- **Read the rack first.** If your plugin isn't under MOUNTED, nothing in it will work until the load error is fixed
- **Keep commands quick.** While your function runs, the host can't answer anything else

---

## Try it Yourself

1. Misspell `message` as `mesage` in the `import` line, Reload, and read the error
2. Misspell it only inside the function this time. Does the plugin load? What happens on RUN?
3. Fix both and confirm **Say hi** works again

---

## Common Issues

**Problem**: The error text is cut off with "…".
**Solution**: Hover over the module, or widen the panel by dragging its right edge.
**Why**: The rack only has room for one line.

**Problem**: I fixed the bug but the plugin still says Failed to load.
**Solution**: Save the file, then click Reload.
**Why**: The rack shows the result of the last load.

**Problem**: My `print()` output is nowhere to be seen.
**Solution**: Return a `message` in its place.
**Why**: Printed text is kept out of the channel the host uses to talk to Snakie.

---

## Check your understanding

{% quiz %}
? Your plugin has a syntax error. When do you find out?
- [x] When you click Reload
- [ ] When you click RUN
- [ ] When you connect a board
> A syntax error stops the file loading at all, so the rack reports it on Reload.

? One plugin fails to load. What happens to the others?
- [ ] They all fail too
- [x] They load as normal
- [ ] Snakie closes
> Each plugin is loaded separately, and a failure is reported for that plugin only.

? What is the plugin author's substitute for print()?
- [ ] An edit
- [x] A message
- [ ] A block
> Return a message with the value you want to inspect.
{% endquiz %}
