---
layout: lesson
title: Installing Git
author: Kevin McAleer
type: page
cover: /learn/git/assets/cover.jpg
date: 2026-11-04
previous: 01_what-is-git.html
next: 03_your-first-repository.html
description: Install Git on Windows, macOS, Linux or Raspberry Pi, and set your name,
  email and default branch
percent: 21
duration: 5
quiz_total: 35
navigation:
- name: Git for Beginners
- content:
  - section: Overview
    content:
    - name: Course Overview
      link: 00_intro.html
  - section: Getting Started
    content:
    - name: What is Git?
      link: 01_what-is-git.html
    - name: Installing Git
      link: 02_installing-git.html
    - name: Your First Repository
      link: 03_your-first-repository.html
  - section: Saving Your Work
    content:
    - name: Staging and Committing
      link: 04_staging-and-committing.html
    - name: Viewing History
      link: 05_viewing-history.html
    - name: Undoing Changes
      link: 06_undoing-changes.html
    - name: Ignoring Files
      link: 07_ignoring-files.html
  - section: Branches
    content:
    - name: Branches
      link: 08_branches.html
    - name: Merging and Conflicts
      link: 09_merging-and-conflicts.html
  - section: Working with GitHub
    content:
    - name: GitHub and Remotes
      link: 10_github-and-remotes.html
    - name: Pulling and Collaborating
      link: 11_pulling-and-collaborating.html
  - section: Putting It All Together
    content:
    - name: 'Project: Version Control a Robot'
      link: 12_project-version-control-a-robot.html
    - name: Cheat Sheet and Next Steps
      link: 13_cheat-sheet-and-next-steps.html
---


## Is Git already installed?

Git may already be on your computer, especially on a Mac, Linux or a Raspberry Pi. Open a terminal and type:

```bash
git --version
```

If you see something like this, you're ready to skip ahead to [Telling Git who you are](#telling-git-who-you-are):

```text
git version 2.47.1
```

Any version from 2.28 onwards is fine for this course. If you get `command not found`, install it using the steps for your system below.

> ## Where's the terminal?
>
> * **Windows**: after installing Git, open **Git Bash** from the Start menu. You can also use PowerShell or the terminal in VS Code.
> * **macOS**: press `Cmd + Space`, type **Terminal** and press Enter.
> * **Raspberry Pi and Linux**: open **Terminal** from the menu, or press `Ctrl + Alt + T`.

---

## Installing on Windows

1. Download **Git for Windows** from [git-scm.com](https://git-scm.com/downloads/win).
2. Run the installer. The default options are fine for almost everyone. When it asks for a default editor, pick one you know (such as **Visual Studio Code** or **Notepad**) instead of the default, Vim.
3. When it's finished, open **Git Bash** from the Start menu and run `git --version`.

Git for Windows also installs **Git Credential Manager**, which will handle signing in to GitHub for you later.

You can also install it from PowerShell with `winget install --id Git.Git -e`.

---

## Installing on macOS

The simplest way is to type `git --version` in Terminal. If Git isn't installed, macOS offers to install the **Command Line Developer Tools**, which include Git. Click **Install** and wait a few minutes.

If you use [Homebrew](https://brew.sh), you can get a newer version with:

```bash
brew install git
```

---

## Installing on Raspberry Pi OS, Ubuntu and Debian

Git is usually installed on Raspberry Pi OS already. If not:

```bash
sudo apt update
sudo apt install git
```

On Fedora use `sudo dnf install git`.

---

## Telling Git who you are

Every commit records the name and email of the person who made it, so Git needs to know yours. Run these two commands, using your own details:

```bash
git config --global user.name "Ada Lovelace"
git config --global user.email "ada@example.com"
```

`--global` means "for every repository on this computer", so you only have to do this once per machine. If you plan to use GitHub, use the same email address as your GitHub account so your commits are linked to your profile.

> ## Keep your email private
>
> Commits on GitHub are public, and so is the email in them. GitHub gives every account a private `noreply` address you can use instead. Find it under **Settings > Emails** on GitHub.

---

## Two more useful settings

**Call the first branch `main`.** Newer versions of Git and GitHub both use `main` as the name of the first branch. Older versions of Git used `master`. Set it once so everything matches:

```bash
git config --global init.defaultBranch main
```

**Pick an editor you know.** Some Git commands open a text editor for you to write a message. If you don't set one, you might land in Vim, which is famously hard to quit (the answer is to type `:q!` and press Enter). To use VS Code instead:

```bash
git config --global core.editor "code --wait"
```

Or for the simple terminal editor nano, which is great on a Raspberry Pi:

```bash
git config --global core.editor "nano"
```

---

## Checking your settings

To see everything you've set:

```bash
git config --global --list
```

```text
user.name=Ada Lovelace
user.email=ada@example.com
init.defaultbranch=main
core.editor=nano
```

---

## Try it yourself

1. Install Git (if needed) and check the version.
2. Set your name, email, default branch and editor.
3. Run `git config --global --list` and check it's all correct.
4. Run `git help commit`. Git has a full manual for every command, and you can open it any time with `git help <command>`. Press `q` to quit the manual.

---

## Common issues

- **Problem**: `git: command not found` (or `'git' is not recognized` on Windows)
- **Solution**: Install Git, then close and reopen the terminal.
- **Why**: Your terminal only looks for programs in certain folders, and only checks for new ones when it starts.

- **Problem**: Commits show the wrong name or email
- **Solution**: Run the two `git config --global user...` commands again with the right details.
- **Why**: Git stamps each new commit with whatever is in your settings at the time. Changing them won't alter old commits, only new ones.

- **Problem**: A strange editor opened and you can't get out
- **Solution**: Press `Esc`, type `:q!` and press Enter. Then set `core.editor` as shown above.
- **Why**: That's Vim, Git's default editor on many systems.

---

## Check your understanding

{% quiz %}
? What does `--global` mean in `git config --global user.name "Ada"`?
- [ ] The name is shared with everyone on GitHub
- [x] The setting applies to every repository on this computer
- [ ] The setting only applies to the current folder
> Global settings are stored in your home folder and apply to all your repositories. You only need to set them once per computer.

? Why set `init.defaultBranch` to `main`?
- [ ] Git won't work without it
- [x] So new repositories start on a branch called `main`, matching GitHub
- [ ] It makes Git run faster
> Older versions of Git call the first branch `master`. Setting it to `main` avoids mismatches when you push to GitHub.
{% endquiz %}

---
