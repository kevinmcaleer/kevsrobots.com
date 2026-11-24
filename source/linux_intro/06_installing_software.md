---
title: Installing Software
description: Use the apt package manager to refresh the package list, search for tools and install them on Raspberry Pi OS.
layout: lesson
type: page
cover: /learn/linux_intro/assets/books.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to update the package list, search for a package and install it with `apt`, and say where the software came from.

## Software from a trusted shelf

On a phone you get apps from an app store. On Raspberry Pi OS the equivalent is **apt**, the Advanced Package Tool, which Debian-based systems (including Ubuntu) share. You ask for a tool by name, `apt` fetches it from a **repository** (a trusted server of software), checks it is genuine, and installs it along with anything it depends on.

Installing from apt is much safer than downloading random programs from the internet, because packages are signed and kept up to date together. It is also how Docker's own installer works behind the scenes.

```mermaid
flowchart LR
    A["sudo apt update: refresh the list"] --> B["apt search / apt show: find the package"]
    B --> C["sudo apt install: download and install"]
    C --> D["dependencies installed automatically"]
```

## Step 1: refresh the list

Your Pi keeps a local list of what is available, and it goes stale. Always refresh it before installing:

```bash
$ sudo apt update
Hit:1 http://deb.debian.org/debian trixie InRelease
Hit:2 http://deb.debian.org/debian trixie-updates InRelease
Hit:3 http://archive.raspberrypi.com/debian trixie InRelease
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
12 packages can be upgraded. Run 'apt list --upgradable' to see them.
```

The names and numbers on your Pi will differ (older releases say `bookworm`, not `trixie`). The important line is the last one: `update` only refreshes the list. It does not install anything yet.

**Repositories**: the lines starting `Hit` or `Get` are the servers your Pi asks. Raspberry Pi OS uses Debian's servers plus Raspberry Pi's own for firmware and Pi-specific software.

## Step 2: find a package

```bash
$ apt search htop
Sorting... Done
Full Text Search... Done
htop/stable 3.x arm64
  interactive processes viewer

$ apt show htop
Package: htop
Description: interactive processes viewer
 Htop is an ncursed-based process viewer similar to top, but it
 allows one to scroll the list vertically and horizontally ...
```

`apt search` looks at names and descriptions, so it may list a lot. `apt show` gives the details for one package. Version numbers will vary.

## Step 3: install it

We will install two tools we need in later lessons: `tree` (draws folders) and `htop` (a friendly process viewer).

```bash
$ sudo apt install tree htop
Reading package lists... Done
Building dependency tree... Done
The following NEW packages will be installed:
  htop tree
0 upgraded, 2 newly installed, 0 to remove and 12 not upgraded.
After this operation, 600 kB of additional disk space will be used.
Do you want to continue? [Y/n] y
```

`apt` always tells you what it plans to do and waits. Read the list, then press `Y` and `Enter`. Try them:

```bash
$ tree ~/homelab
/home/pi/homelab
├── backups
│   └── notes-2026-11-24.txt
├── notes.txt
├── projects
│   └── hello-docker
└── scripts
    ├── hello.sh
    ├── oops.txt
    └── secrets.env

4 directories, 5 files
```

If you used `htop` earlier, the Pi may already have it, and `apt` will say `htop is already the newest version`. That is fine.

## Where did it go?

```bash
$ which tree
/usr/bin/tree
```

`which` shows where a program lives. Software installed by `apt` goes in system folders such as `/usr/bin`, so it works for every user, and that is why `apt` needs `sudo`.

## Installing safely

- **Update first**, so you get the current version.
- **Read the plan** before pressing `Y`. If it wants to remove many packages you recognise, stop.
- **Avoid `-y` while learning**. It skips the question. It is useful in scripts, but it means you have not read what is about to happen.
- **Be careful with random instructions**. A line such as `curl ... | sudo bash` downloads a script and runs it as root without you reading it. Docker's own docs offer a convenience script like that. We will look at it properly in the Docker course.

> **Teachers**: `apt` needs internet access to Debian and Raspberry Pi servers. School filters, proxies and slow shared Wi-Fi can make `apt update` fail or take many minutes when a whole class does it at once. Try it first, and stagger the class or pre-install `tree` and `htop` on the image. Each install uses a little storage, so check the SD cards have room (we measure that in the monitoring lesson).

Pi progress: you can install software safely from the official repositories. Next, keeping it up to date and removing it again.

## Try it Yourself

1. Run `sudo apt update`, then `apt list --upgradable` to see which packages have newer versions.
2. Search for `cowsay` with `apt search cowsay`. Read the description, and check the details with `apt show cowsay`.
3. Use `tree -L 1 ~` to show only one level of your home folder.
4. Challenge: install `cowsay` (it is harmless and fun), run `cowsay "Pi is ready"`, and find out where it lives with `which cowsay`.

## Common Issues

- **Problem**: `E: Unable to locate package something`.
- **Solution**: Run `sudo apt update`, then check the spelling with `apt search something`.
- **Why**: Your local package list is out of date, or the package name is different from what you typed.

- **Problem**: `E: Could not get lock /var/lib/dpkg/lock-frontend`.
- **Solution**: Wait a minute. Another update (often the desktop's automatic one) is running. Do not delete the lock file.
- **Why**: Only one program may change installed software at a time, so apt locks the database.

- **Problem**: `Temporary failure resolving 'deb.debian.org'` or `Could not resolve host`.
- **Solution**: Check your network with `ping -c 3 8.8.8.8`, then fix Wi-Fi or Ethernet. See the networking lesson.
- **Why**: Without a network connection, your Pi cannot reach the repository.

## Check your understanding

{% quiz %}
? What does `sudo apt update` do?
- [ ] Installs every available update
- [x] Refreshes the list of available packages
- [ ] Removes old packages
> `update` only refreshes the list. `upgrade` installs the newer versions.

? Why does `apt install` need `sudo`?
- [ ] It is slow
- [x] It changes system folders that normal users cannot write to
- [ ] To hide your password
> Software is installed into system folders such as `/usr/bin`, which only root may change.

? What is a repository?
- [ ] A kind of folder on your desktop
- [x] A server that stores software packages
- [ ] A way to delete packages
> Repositories are the trusted servers `apt` downloads packages from.

? Which tool shows you where an installed program lives?
- [x] which
- [ ] whoami
- [ ] pwd
> `which tree` prints `/usr/bin/tree`.
{% endquiz %}
