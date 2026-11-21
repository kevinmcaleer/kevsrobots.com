---
title: The Plugin Rack
description: Tour the Snakie Plugins panel, see the plugins that ship with the app and run your first plugin command
layout: lesson
type: page
cover: /learn/snakie_plugins/assets/banners/02_the_plugin_rack.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Open the rack

Before writing a plugin, let's use one. Open the Snakie desktop app, open any Python file, and click the **Plugins** icon in the activity bar on the left. It's the jigsaw piece.

![The Plugin Rack with the bundled plugins](/learn/snakie_plugins/assets/rack.jpg){:class="img-fluid w-100"}

Snakie calls this the **Plugin Rack**, and it looks like a rack of studio equipment. Each plugin is a module bolted into the rack.

| Part | What it does |
|---|---|
| The round arrow at the top | **Reload**. Restarts the plugin host and loads every plugin again |
| The **PWR** light | Green when the host is running |
| The search box | Filters the list by name |
| The note under it | A reminder that plugins run Python code on your machine |
| **MOUNTED** | The plugins that loaded |
{:class="table table-single"}

---

## The plugins that come with Snakie

A fresh install has five. They're examples, and one of them does real work.

| Plugin | What it shows |
|---|---|
| `hello` | Three commands: a message, an edit and a status bar link |
| `lint_demo` | A tiny linter that flags spaces at the end of a line and `# TODO` comments |
| `python_linter` | The real Python linter behind the squiggles and the Problems panel |
| `refactor_demo` | A classroom "house style" example |
| `blocks_demo` | Four blocks for an imaginary club robot |
{:class="table table-single"}

Each module shows its name, the commands it offers, and where it was loaded from. A plugin that says "No commands registered" isn't broken. It does its work some other way, such as linting.

---

## Run a command

1. Click the `hello` module. It opens to show its commands
2. Click **RUN** next to **Hello**

![Running the Hello command](/learn/snakie_plugins/assets/hello.jpg){:class="img-fluid w-100"}

A message appears at the bottom of the panel, naming your file and how long it is. That text was put together by a Python function and sent back to Snakie.

3. Click **RUN** next to **Show status link**

![A plugin message in the status bar](/learn/snakie_plugins/assets/status.jpg){:class="img-fluid w-100"}

Now look at the status bar along the bottom of the window. The plugin has put a line count there, and it's a link.

There's a third command, **Uppercase file**. It replaces everything in the editor with CAPITALS. Try it on a file you don't mind changing, then press **Ctrl + Z** (**Cmd + Z** on a Mac) or close the file without saving.

---

## Commands work on the active file

Every command acts on the file in the editor tab you're looking at. Switch tabs and run **Hello** again, and the message names the other file.

With no file open, a command still runs, but it has nothing to read.

---

## Read the example

The `hello` plugin is about thirty lines. Here's the part behind the button you pressed.

```python
from snakie import Context, edit, message, plugin, status


@plugin.command("hello", "Hello")
def hello(ctx: Context):
    """Greet the user with the active file's name and size."""
    name = ctx.file.name or "(no file)"
    size = len(ctx.file.content)
    return message("info", f"Hello from Snakie! Editing {name} ({size} chars).")
```

You can already read most of it. A function receives `ctx`, looks at the file name and content, and returns a message. The `@plugin.command` line is what turns the function into a button. You'll write one of your own in the next lesson.

---

## Try it Yourself

1. Run **Hello** on two different files and compare the messages
2. Type `lint` in the search box. Which modules are left?
3. Add a space to the end of a line in your file. `lint_demo` underlines it. Hover over the squiggle to read why
4. Click the Reload arrow and watch the PWR light

---

## Common Issues

**Problem**: The panel says Python wasn't found and shows `pip install snakie`.
**Solution**: Install Python 3, restart Snakie and open the panel again. You don't need to install anything with pip for this course.
**Why**: The host needs Python to run. Snakie brings its own copy of the plugin toolkit, so Python itself is the only thing missing.

**Problem**: There's no Plugins icon.
**Solution**: Use the desktop app.
**Why**: The browser version can't start a Python program on your computer, so it hides the panel.

**Problem**: RUN does nothing visible.
**Solution**: Open a file first and look at the bottom of the Plugins panel for the message.
**Why**: Messages collect at the bottom of the panel, newest first, and it's easy to miss them.

---

## Check your understanding

{% quiz %}
? What does the Reload arrow do?
- [ ] Reloads the file from disk
- [x] Restarts the plugin host and loads every plugin again
- [ ] Reconnects to the board
> Reload restarts the host, which is how Snakie picks up new or edited plugins.

? A module says "No commands registered". What does that mean?
- [ ] It failed to load
- [x] It has no buttons, but may still do other work such as linting
- [ ] It needs a board connected
> Linters and block providers have no commands to run by hand.

? Which file does a command act on?
- [x] The one in the active editor tab
- [ ] Every open file
- [ ] The file called main.py
> Commands receive the active file as their context.
{% endquiz %}
