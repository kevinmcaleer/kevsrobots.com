---
title: Sharing Your Plugin
description: Package a Snakie plugin for a club or class, write install instructions, and learn how to decide whether a plugin is safe to trust
layout: lesson
type: page
cover: /learn/snakie_plugins/assets/banners/11_sharing_your_plugin.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to package a plugin for a club or class, write install instructions and decide whether a plugin is safe to trust.

## One plugin, many computers

ClubKit is only useful to the club if the club has it. A Snakie plugin is a folder of Python, so sharing it is refreshingly low-tech.

---

## Get it ready

Before you hand it over, give the folder a tidy.

**1. Add a README.** A short text file answers the questions everyone asks.

```
clubkit/
  __init__.py
  README.md
```

```markdown
# ClubKit

A Snakie plugin for our robot club.

- Add club header: stamps the club header on the open file
- List the pins: shows every GPIO pin the program uses
- Warns about sleep() calls that look like milliseconds
- Flags pins that aren't on the Pico's header
- Adds a ClubKit drawer of blocks for the club robot

Needs the Snakie desktop app and Python 3.

Install: copy this folder into .snakie/plugins in your home folder,
then click Reload in the Plugins panel.
```

**2. Keep the docstring.** The first line of `__init__.py`, in triple quotes, tells the next person what they're looking at.

**3. Delete `__pycache__`.** Python creates it when the plugin runs. Nobody else needs it.

**4. Stick to the standard library.** ClubKit only imports `re` and `snakie`, so it works on any computer with Python. If your plugin needs an extra package, every user has to install that too, into the same Python that Snakie uses.

---

## Three ways to share

| Way | Good for | How |
|---|---|---|
| A zip file | A club night, a memory stick | Zip the `clubkit` folder. Others unzip it into `.snakie/plugins` |
| A Git repository | A class or team that will get updates | Clone it into `.snakie/plugins`. `git pull` fetches new versions |
| A Python package | Publishing more widely | The Snakie docs describe declaring a `snakie.plugins` entry point so an installed package is found |
{:class="table table-single"}

For a club, the first two cover almost everything. With Git you also get history, so when a new rule annoys everyone you can see exactly what changed. [Coding Faster in Snakie](/learn/snakie_code_tools/) covers Snakie's Source Control panel.

Whichever way you pick, the last step is the same for everyone: open the Plugins panel and click **Reload**.

---

## Updating

When you improve ClubKit:

1. Tell people what changed, in a line or two
2. Have them replace the folder, or `git pull`
3. Reload

Be careful with changes that affect people's existing work. Renaming a block's `id` is the big one: a block program saved with the old id won't find it any more. Add new blocks freely, and leave old ids alone.

---

## Trust

This is the serious part of the course. At the top of the Plugin Rack, Snakie says:

> Modules run Python code on your machine. Only mount modules you trust.

It means it. A plugin runs as **you**, with no sandbox. It can read your files, delete them, and use the network, the same as any program you run. Snakie can't check what a plugin does. Only a person reading it can.

So before installing someone else's plugin, ask:

- **Who wrote it?** Someone you know, or a stranger?
- **Can I read it?** A plugin is plain Python. ClubKit is about a hundred lines, which takes five minutes
- **Does it do more than it says?** A linter has no reason to import `socket`, `subprocess` or `os`, or to open files outside the project
- **Where did this copy come from?** Get it from the author, not a forwarded copy of a copy

And when you're the author:

- Keep it small and readable, so people *can* check it
- Don't reach for the network or the file system unless the plugin truly needs to, and say so in the README when it does
- Never run a command or change a file the user didn't ask you to

Blocks are the exception to the worry. As you saw in the last lesson, a block can only produce text. The Python in the same plugin file is still Python, though, so the plugin as a whole needs the same care.

> **Teachers**: Plugins run with no sandbox, as the person using them. Set a simple class rule: learners only install plugins they or their teacher wrote or trust, and read the code first using the four trust questions in this lesson. A zip file or a Git repository both work for handing a plugin out to a class.

---

## Try it Yourself

1. Write a README for your version of ClubKit, including the rules you added in the Try it Yourself tasks
2. Zip the folder, move the original out of `.snakie/plugins`, and install from the zip as a club member would
3. Open one of the plugins that ships with Snakie (`hello` is the shortest) and read it with the four trust questions in mind

---

## Common Issues

**Problem**: It works for me but not for a friend.
**Solution**: Check they have Python 3 installed, they're using the desktop app, and the folder is `.snakie/plugins/clubkit` with `__init__.py` directly inside.
**Why**: Unzipping often makes a folder inside a folder, such as `clubkit/clubkit/__init__.py`, and the host only looks one level down.

**Problem**: A friend's plugin says `ModuleNotFoundError`.
**Solution**: Install the missing package for the Python that Snakie uses, or ask the author to remove the dependency.
**Why**: Plugins run in your own Python and can only import what's installed there.

**Problem**: Old block programs broke after an update.
**Solution**: Put the old block `id` back.
**Why**: Saved programs refer to blocks by id.

---

## Check your understanding

{% quiz %}
? What is the simplest way to give ClubKit to another club member?
- [x] Send them the clubkit folder to put in .snakie/plugins
- [ ] Flash it to their Pico
- [ ] Email them a screenshot
> A plugin is a folder of Python. Copy it into place and click Reload.

? What can a Snakie plugin's Python code do on your computer?
- [ ] Only read the open file
- [ ] Only change the editor
- [x] Anything you could do yourself
> Plugins run as you with no sandbox, which is why you should only install ones you trust.

? Which import would make you look twice at a plugin that claims to be a linter?
- [ ] re
- [x] subprocess
- [ ] snakie
> A linter reads text. It has no need to run other programs.
{% endquiz %}
