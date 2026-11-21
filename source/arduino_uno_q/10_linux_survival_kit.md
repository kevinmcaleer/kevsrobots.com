---
title: Linux survival kit
description: Get a terminal on your Uno Q, learn the everyday Linux commands, and know when to use sudo, apt, the App CLI and a backup
layout: lesson
type: page
cover: /learn/arduino_uno_q/assets/banners/10_linux_survival_kit.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Your Uno Q has a real Linux computer on it, and App Lab hides most of it from you. That is lovely until the day something needs fixing, and then you want a terminal. This lesson is your small survival kit: how to get in, the dozen commands you will actually use, and the habits that keep the board healthy.

Kev made a video of his own five favourite tips for managing the board. Watch it first, then come back and follow along here.

{% include youtubeplayer.html id="e5mNvGAk_Cg" %}

The video goes with Kev's blog post [Uno Q tips](/blog/uno-q-tips.html), which goes deeper on a few of these.

---

## Two ways in

A terminal on the board is a text window where you type commands and the board's Linux runs them. There are two ways to get one.

| | ADB over USB | SSH over the network |
| --- | --- | --- |
| What you need | USB-C cable and the ADB tool on your computer | Both machines on the same network |
| Works before first setup | Yes | No, it is enabled after App Lab's first-time setup |
| Works with no network | Yes | No |
| Password | `arduino` if the board is not set up yet | The board password you chose in [lesson 2](/learn/arduino_uno_q/02_first_boot_app_lab_and_wifi.html) |
{:class="table table-single"}

### ADB over USB

ADB (Android Debug Bridge) is a tool you install on your computer. Pick the line for your system and type it **on your computer**.

```bash
brew install android-platform-tools
```

That one is for macOS. On Windows use `winget install Google.PlatformTools`. On Debian or Ubuntu use `sudo apt-get install android-sdk-platform-tools`.

Check it installed:

```bash
adb version
```

Now plug the board in with USB-C and ask ADB to list devices. The board can take up to a minute to appear.

```bash
adb devices
```

When it shows up, open the board's terminal. Type `exit` to leave again.

```bash
adb shell
```

### SSH over the network

Once App Lab has finished first-time setup, you can SSH in from **your computer**. Type `yes` the first time it asks, then the board password from lesson 2.

```bash
ssh arduino@<boardname>.local
```

Swap `<boardname>` for the name you gave your board. If `.local` names do not resolve on your network, use the board's IP address instead. You find it by running `hostname -I` on the board, for example over ADB.

The user is called `arduino`, its home folder is `/home/arduino`, and your Apps live in `~/ArduinoApps/`.

---

## Everyday commands

Everything from here is typed **on the board** (in an ADB shell or SSH session) unless it says otherwise.

| Command | What it does |
| --- | --- |
| `ls` | List the files here |
| `cd folder` | Move into a folder |
| `cat file` | Print a file on screen |
| `nano file` | Edit a file. Ctrl+X, then Y, then Enter to save and leave |
| `df -h` | How much disk space is left |
| `du -sh *` | The size of each item in this folder |
| `free -h` | How much memory is free |
| `top` | Live list of running processes. Press `q` to quit |
| `hostname -I` | The board's IP address |
{:class="table table-single"}

Try them in order: `cd ~/ArduinoApps`, then `ls`. There are your Apps, one folder each.

Keep an eye on `df -h`. The board's storage is eMMC, 16 GB or 32 GB depending on the model, and you cannot swap it like an SD card. Space is finite, so it pays to look now and then, and `du -sh *` tells you which folder is the hungry one.

---

## sudo, and when not to use it

`sudo` means "do this as the administrator". Use it for system-level jobs only: installing software with `apt`, editing files in `/etc/`, restarting services.

Do not use it for your own files. A file made with `sudo` belongs to root, so you then need `sudo` to edit it again, and that is how you meet "Permission denied" later.

Not sure which is which? Have a go.

{% capture items %}
Install tmux with apt | right | Installing software changes the system, so apt needs sudo.
Edit main.py inside your App folder | left | It is your own file in your own home folder.
Restart the arduino-router service | right | Services belong to the system.
List your Apps with arduino-app-cli app list | left | Listing Apps changes nothing and needs no special rights.
Edit a file in /etc | right | System settings live in /etc and are protected.
Check free disk space with df -h | left | Looking is always allowed.
Copy your Apps folder to a backup | left | They are your files. Using sudo here would make root own the copies.
Halt the board before unplugging it | right | Stopping the whole system is a system-level job.
{% endcapture %}
{% include sorter.html title="sudo or not?" left="No sudo" right="sudo" items=items %}

Notice the pattern: looking and your own files need no sudo. Changing the system does.

---

## Installing software with apt

`apt` is Debian's package manager. These are the commands to know.

```bash
sudo apt update
sudo apt upgrade
sudo apt install tmux
apt search <keyword>
sudo apt remove <name>
```

`update` refreshes the list of what is available, `upgrade` installs updates, `install` adds a program, `search` finds one by keyword, and `remove` takes one away. `tmux` is a safe one to practise on.

### The container limit

Here is the catch that surprises people. An App runs inside a container without root. That means an App cannot use programs you `apt install` on the board, and it cannot run `apt` itself.

So where do an App's Python packages go? In `python/requirements.txt`, one package per line. App Lab installs them the next time you press **Run**, and the board needs internet for that. Use `apt` for tools you use at the terminal, and `requirements.txt` for what your App imports.

---

## The Arduino App CLI

The board comes with a command-line tool, `arduino-app-cli`, that does what App Lab's buttons do. It is handy over SSH when no computer is running App Lab. Start by listing your Apps.

```bash
arduino-app-cli app list
```

Every App has an ID. Your own Apps start with `user:` and the built-in examples start with `examples:`, for example `examples:blink`. Find your Q Board App in the list and copy its ID exactly. In the commands below, `<app ID>` stands for what you copied.

```bash
arduino-app-cli app start <app ID>
arduino-app-cli app stop <app ID>
arduino-app-cli app restart <app ID>
arduino-app-cli app logs <app ID> --follow
arduino-app-cli monitor
```

`logs ... --follow` shows the Python output live, like App Lab's **Python** tab. `monitor` shows the sketch's serial output, like the **Serial Monitor** tab.

And if the Bridge acts up, the router check from [lesson 9](/learn/arduino_uno_q/09_when_the_bridge_goes_wrong.html) is also typed here:

```bash
systemctl status arduino-router
```

---

## Back up your Apps

Your Q Board is a folder of files, so backing it up means copying the folder. Type this **on your computer**, not on the board.

```bash
scp -rp arduino@<boardname>.local:/home/arduino/ArduinoApps ./uno-q-apps-backup
```

That copies every App into a folder called `uno-q-apps-backup`. To put one back, again from your computer:

```bash
scp -rp "my-folder" arduino@<boardname>.local:/home/arduino/ArduinoApps/
```

Kev's [tips post](/blog/uno-q-tips.html) also covers Samba file sharing, rsync, tar archives and git. It mentions a full disk image with `dd` too, but that advice is written for SD-card boards. The Uno Q's storage is soldered-on eMMC, so back up the Apps folder instead.

---

## Shutting down

Pulling the plug on a running computer is rude to its files. Shut Linux down first.

`sudo shutdown now` and `sudo poweroff` both shut Linux down cleanly, but the board restarts shortly afterwards. To stop it for storage or travel, use this instead, then unplug:

```bash
sudo halt
```

After `halt` the board stays powered but Linux has stopped, so it is safe to pull the cable.

---

## Try it Yourself

1. **Look around.** Get a terminal by whichever route suits you. Run `cd ~/ArduinoApps`, then `ls`, then `du -sh *`. Which App is the biggest?
2. **Find your App's ID.** Run `arduino-app-cli app list` and copy the ID of your Q Board. Use it with `app logs <app ID> --follow` while the App runs, and watch your Python output appear.
3. **Challenge: take a backup.** From your computer, run the `scp` backup command, then open the backup folder and find your `main.py` and `sketch.ino` inside.

---

## Common Issues

**Problem**: `adb devices` shows nothing.
**Solution**: Wait up to a minute, and check you are using a USB-C data cable.
**Why**: The board takes a while to appear, and a charge-only cable carries power but no data.

**Problem**: `ssh` cannot find `<boardname>.local`.
**Solution**: Use the IP address instead. Run `hostname -I` on the board to read it, then `ssh arduino@` followed by that address.
**Why**: Some networks do not resolve `.local` names.

**Problem**: "Permission denied" when editing a file.
**Solution**: The file was probably created with `sudo`, so root owns it. Create your own files without `sudo`.
**Why**: A file made with `sudo` belongs to root, and you then need `sudo` to change it.

**Problem**: My App cannot find a program I installed with `apt`.
**Solution**: Put Python packages in `python/requirements.txt` and press **Run** again.
**Why**: Apps run in a container without root, so they cannot use what you `apt install` on the board.

---

## Check your understanding

{% quiz %}
? You run `sudo nano main.py` in your App folder and save it. What problem are you setting up?
- [ ] The file becomes read-only for everyone
- [x] The file now belongs to root, so you will need sudo to edit it again
- [ ] App Lab deletes the file on the next Run
> A file made with sudo belongs to root. Use sudo for system-level jobs, not for your own files.

? Your App imports a Python package. Where do you list it so App Lab installs it?
- [ ] Run `sudo apt install` on the board
- [ ] In `app.yaml`, edited by hand
- [x] In `python/requirements.txt`, one package per line
> Apps run in a container without root, so they cannot use programs installed with apt. App Lab installs requirements.txt packages on the next Run.

? You want to unplug the board and put it away. What should you do first?
- [ ] Run `sudo poweroff`, which stops it for good
- [x] Run `sudo halt`, then unplug it
- [ ] Nothing, Linux handles a cut in power safely
> `shutdown now` and `poweroff` shut Linux down but the board restarts soon after. `halt` leaves Linux stopped so you can unplug.
{% endquiz %}
