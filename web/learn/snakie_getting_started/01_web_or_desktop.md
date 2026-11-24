---
layout: lesson
title: Web App or Desktop App
author: Kevin McAleer
type: page
cover: /learn/snakie_getting_started/assets/banners/01_web_or_desktop.jpg
date: 2026-11-20
previous: 00_intro.html
next: 02_tour_of_the_workspaces.html
description: Choose between Snakie in the browser and the installed desktop app, and
  install it on Windows, macOS, Linux or Raspberry Pi
percent: 16
duration: 4
quiz_total: 23
date_updated: 2026-11-24
navigation:
- name: Getting Started with Snakie
- content:
  - section: Overview
    content:
    - name: Getting Started with Snakie
      link: 00_intro.html
    - name: Web App or Desktop App
      link: 01_web_or_desktop.html
    - name: A Tour of the Workspaces
      link: 02_tour_of_the_workspaces.html
  - section: Your First Programs
    content:
    - name: Your First Program on the Simulator
      link: 03_first_program_on_the_simulator.html
    - name: The REPL and the Console
      link: 04_the_repl_and_console.html
    - name: Seeing Your Pins
      link: 05_seeing_your_pins.html
  - section: Real Hardware
    content:
    - name: Finding Your Board
      link: 06_finding_your_board.html
    - name: Flashing MicroPython
      link: 07_flashing_micropython.html
    - name: Connect, Run and Stop
      link: 08_connect_run_stop.html
    - name: Files on Your Computer and on the Board
      link: 09_files_local_and_device.html
  - section: Project
    content:
    - name: Project - Traffic Light
      link: 10_project_traffic_light.html
    - name: Summary
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to choose between the Snakie web app and the desktop app, and open or install the one that suits you.

Snakie comes in two forms that look and work the same: a web app that runs in your browser, and a desktop app you install. Let's pick the right one for you.

---

## The web app

Go to [app.snakie.org](https://app.snakie.org/). That's it. There is nothing to install, and it works on Windows, macOS, Linux and Chromebooks.

Snakie opens already connected to the **Simulated device (offline)**, a pretend board that lives inside the app, so you can run MicroPython straight away.

To use a real board from the web app you need **Chrome or Edge**. Those browsers support Web Serial, which is how a web page talks to a USB device. Firefox and Safari can still run everything on the simulator.

## The desktop app

The desktop app is the same Snakie, installed on your computer. It works in any setup, updates itself, and has a full set of application menus (File, Device, Tools and View).

Download the installer for your computer from the [Snakie releases page](https://github.com/kevinmcaleer/Snakie/releases/latest):

| Your computer | File to download |
|---|---|
| Windows (x64) | `Snakie.Setup.<version>.exe` |
| macOS, Apple Silicon | `Snakie-<version>-arm64.dmg` |
| macOS, Intel | `Snakie-<version>.dmg` |
| Linux (x64) | `Snakie-<version>.AppImage` or `snakie_<version>_amd64.deb` |
| Raspberry Pi 4 or 5, 64-bit Pi OS | `Snakie-<version>-arm64.AppImage` |
{:class="table table-single"}

The macOS builds are signed and notarized, so they open like any other app. The Windows and Linux builds are not signed yet, so Windows may ask you to confirm before it runs the installer.

### Raspberry Pi

On a Raspberry Pi the download is an AppImage: a single file you make executable and run.

```bash
chmod +x ~/Downloads/Snakie-*-arm64.AppImage
~/Downloads/Snakie-*-arm64.AppImage
```

An AppImage doesn't add itself to the menu. To put Snakie under **Programming** with its icon, run the helper script once:

```bash
curl -fsSL https://raw.githubusercontent.com/kevinmcaleer/Snakie/master/scripts/install-linux-menu.sh -o install-linux-menu.sh
bash install-linux-menu.sh ~/Downloads/Snakie-<version>-arm64.AppImage
```

## Which should you choose?

| | Web app | Desktop app |
|---|---|---|
| Install needed | No | Yes |
| Simulator | Yes | Yes |
| Real boards | Chrome or Edge only | Yes |
| Works offline | After the first visit | Yes |
| Application menus and shortcuts sheet | No | Yes |
| Updates | Automatic on reload | One click from the status bar |
{:class="table table-single"}

If you're just starting, use the web app. Everything in this course works there. Move to the desktop app when you want Snakie on a machine that isn't always online, or you prefer menus and keyboard shortcuts.

## Keeping Snakie up to date

The version number sits at the bottom right of the window, in the status bar. In the desktop app it turns into an **Update** button when a new release is out. In the web app Snakie suggests a reload, and it keeps your open tabs when you do.

---

> **Teachers**: The web app needs no install, so it suits locked-down school machines and Chromebooks, and the simulator works in any browser. Real boards need Chrome or Edge. The desktop installer for Windows and Linux is not signed yet, so Windows may ask for confirmation, and a school may need its IT team to approve the install.

## Try it Yourself

1. Open [app.snakie.org](https://app.snakie.org/) in your browser.
2. Find the version number in the status bar at the bottom right.
3. Look at the bottom left of the status bar. What does it say you are connected to?

## Common Issues

- **Problem**: The web app can't see my board.
- **Solution**: Use Chrome or Edge, and check your USB cable carries data.
- **Why**: Only those browsers support Web Serial, and many cheap cables only carry power.

- **Problem**: On Linux, double-clicking the AppImage does nothing.
- **Solution**: Make it executable with `chmod +x`, as shown above.
- **Why**: Downloaded files are not allowed to run until you say so.

## Check your understanding

{% quiz %}
? Which browsers can connect the Snakie web app to a real USB board?
- [ ] Any browser
- [x] Chrome or Edge
- [ ] Only Safari
> Chrome and Edge support Web Serial, which a web page needs to talk to a USB board.

? What does Snakie connect to when you first open it?
- [ ] Nothing, you must plug in a board first
- [x] The Simulated device (offline)
- [ ] The last board you used
> Snakie connects its built-in simulated board so you can run Python straight away.
{% endquiz %}
