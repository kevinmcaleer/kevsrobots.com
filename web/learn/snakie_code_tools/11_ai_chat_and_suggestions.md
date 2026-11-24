---
layout: lesson
title: AI Chat and Suggestions
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/11_ai_chat_and_suggestions.jpg
date: 2026-11-21
previous: 10_version_control_with_git.html
next: 12_menus_and_shortcuts.html
description: Set up Snakie's AI chat pane and optional ghost-text suggestions with
  your own API key, and learn how to check what an AI writes
percent: 72
duration: 6
quiz_total: 42
date_updated: 2026-11-24
navigation:
- name: Coding Faster in Snakie
- content:
  - section: Overview
    content:
    - name: Coding Faster in Snakie
      link: 00_intro.html
  - section: Writing
    content:
    - name: Smart Autocomplete
      link: 01_smart_autocomplete.html
    - name: Hover Help and Definitions
      link: 02_hover_help_and_definitions.html
    - name: Find and Replace
      link: 03_find_and_replace.html
  - section: Checking
    content:
    - name: Linting and the Problems Panel
      link: 04_linting_and_problems.html
    - name: MicroPython Refactors
      link: 05_micropython_refactors.html
    - name: YAML and JSON Validation
      link: 06_yaml_and_json.html
  - section: Libraries
    content:
    - name: Installing Packages
      link: 07_installing_packages.html
    - name: The Modules Shelf
      link: 08_the_modules_shelf.html
    - name: Compiling to mpy
      link: 09_compiling_to_mpy.html
  - section: Working Like a Pro
    content:
    - name: Version Control with Git
      link: 10_version_control_with_git.html
    - name: AI Chat and Suggestions
      link: 11_ai_chat_and_suggestions.html
    - name: Menus and Keyboard Shortcuts
      link: 12_menus_and_shortcuts.html
  - section: Project
    content:
    - name: Project - A Tidy Sensor Driver
      link: 13_project_sensor_driver.html
    - name: Summary and What's Next
      link: 14_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to set up the AI chat pane and ask it good questions, and to check any code it gives you.

> **Desktop app**: the chat pane is in the Snakie desktop app. It's optional, and it needs your own API key from an AI provider, which is usually a paid service.

Snakie can put an AI assistant beside your code. Used well, it's like a patient colleague who has read every datasheet. Used carelessly, it's a fast way to get code you don't understand. This lesson covers both the setup and the habits.

---

## Opening the pane

Click **Chat** at the top right of the editor. A panel opens on the right.

![The chat pane beside the editor](/learn/snakie_code_tools/assets/chat_pane.jpg){:class="img-fluid w-100"}

Until it's set up, it says *No API key* and points you to Settings.

## Setting it up

Open **Settings** (the cog at the bottom of the activity bar), then the **Chat** tab.

![The Chat tab in Settings](/learn/snakie_code_tools/assets/settings_chat.jpg){:class="img-fluid w-100"}

1. Pick a **provider**. The choices are Anthropic Claude, OpenAI, Google Gemini, Grok (xAI) and GitHub Copilot, which is marked experimental and signs in differently
2. Use **Get a key** to create an API key on the provider's website
3. Paste it in and click **Save key**

The key is stored on your computer and only used to talk to the provider you chose.

> **Teachers and parents**: an API key is tied to a payment account, and anything typed in the chat is sent to that provider. Decide who holds the key before switching this on for a class.

## Asking good questions

At the bottom of the pane are the controls that matter:

| Control | What it does |
|---|---|
| **Include active file** | Sends the file you have open along with your question |
| **Attach console (since last Run)** | Sends the console output from your last run |
| **Provider**, **Model**, **Effort** | Which AI answers, and how hard it thinks |
| **Send** | Sends the message |
| **Clear** | Starts a fresh conversation |
{:class="table table-single"}

Those two tick boxes are what make this better than a chat window in a browser. With the file included, "find the bug in this" works. With the console attached, "why did this crash?" comes with the actual traceback. The console also has a **Send to chat** button that does it in one click.

Questions that work well:

- *Explain what line 14 does, as if I'm new to PWM.*
- *Why does this traceback happen?* (with the console attached)
- *Write a function that turns a 0 to 100 speed into a duty value. MicroPython on a Pico.*
- *What would break if I changed this sleep to 1 ms?*

Always say **MicroPython** and name your board. An AI's default is desktop Python, and it will happily use modules your Pico doesn't have.

## Ghost text

The same providers can suggest code as you type. In **Settings**, **Chat**, find **Autocomplete**. It's **Off** by default, for a reason the setting states honestly: *it spends tokens on every typing pause*.

Switch it on and a faint grey suggestion appears ahead of your cursor when you stop typing. <kbd>Tab</kbd> accepts it. Keep typing to ignore it.

This is separate from the autocomplete in lesson 1. That one is built in, free, and only offers names that really exist. Ghost text is a guess about what you might write next.

## Check its work

An AI can be confidently wrong. The good news is that you've spent this course learning the tools that catch it:

| Check | Tool |
|---|---|
| Does that method exist? | Hover, and autocomplete. If `pwm.` doesn't list it, be suspicious |
| Is that module on my board? | The Modules tab |
| Any undefined names or unused imports? | The Problems panel |
| Is it written well for a microcontroller? | Refactoring hints |
| What exactly did it change? | The diff in Source Control |
| Does it work? | Run it, on the simulator first |
{:class="table table-single"}

Commit before you paste in anything large. Then the diff shows you precisely what arrived, and **Discard changes** takes it away again.

And the oldest rule: don't run code you can't explain. Ask the chat to explain it. That's what it's best at.

---

## Try it Yourself

1. With the active file included, ask the chat to describe what your program does. Is it right?
2. Run a program with a deliberate error, attach the console, and ask why it failed.
3. Ask for a function, paste it in, and put it through the six checks above. Did it pass all of them?

## Common Issues

- **Problem**: There's no Chat button.
- **Solution**: Use the Snakie desktop app.
- **Why**: The chat pane isn't part of the browser version.

- **Problem**: The pane says *No API key*.
- **Solution**: Add a key in **Settings**, **Chat**, for the provider selected at the bottom of the pane.
- **Why**: Each provider needs its own key, and the pane uses whichever provider is selected.

- **Problem**: The code it wrote uses a module my board doesn't have.
- **Solution**: Tell it your board and that you're using MicroPython, and check the Modules tab.
- **Why**: Without that, an AI assumes desktop Python.

## Check your understanding

{% quiz %}
? What does "Include active file" do?
- [ ] Uploads the file to the board
- [x] Sends the open file with your question so the AI can see your code
- [ ] Saves the file
> It's what makes "find the bug in this" possible.

? Why is ghost-text autocomplete off by default?
- [x] It uses your provider's tokens every time you pause typing
- [ ] It's broken
- [ ] It only works on a Mac
> The setting says so itself. It's a cost you opt in to.

? An AI suggests led.set_brightness(50). What's the quickest check?
- [ ] Trust it
- [ ] Reinstall Snakie
- [x] Type led. and see whether autocomplete lists that method
> Built-in autocomplete only offers names that really exist.
{% endquiz %}
