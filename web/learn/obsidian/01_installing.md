---
layout: lesson
title: Installing Obsidian
author: Kevin McAleer
type: page
cover: /learn/obsidian/assets/cover.jpg
date: 2025-08-24
previous: 00_intro.html
next: 02_first_vault.html
description: Learn how to install Obsidian on your computer.
percent: 14
duration: 1
navigation:
- name: Obsidian
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Getting Started with Obsidian
    content:
    - name: Installing Obsidian
      link: 01_installing.html
    - name: Creating Your First Vault
      link: 02_first_vault.html
  - section: Taking Notes and Organization
    content:
    - name: Creating Notes
      link: 03_creating_notes.html
    - name: Tagging Notes
      link: 04_tagging_notes.html
    - name: Linking Notes
      link: 05_linking_notes.html
  - section: Advanced Features
    content:
    - name: Canvas in Obsidian
      link: 06_canvas.html
    - name: Tasks in Obsidian
      link: 07_tasks.html
    - name: Dataview and Bases
      link: 08_dataview_and_bases.html
    - name: Properties in Obsidian
      link: 09_properties.html
    - name: Graph View in Obsidian
      link: 12_graph_view.html
  - section: Markdown in Obsidian
    content:
    - name: Markdown in Obsidian
      link: 10_markdown_in_obsidian.html
    - name: 'Learn More: Markdown Course'
      link: 11_markdown_course_link.html
---


# Installing Obsidian

Obsidian is available for Windows, macOS, and Linux. Visit the official website <https://obsidian.md/> and download the installer for your operating system. Follow the installation instructions to get started.

Once installed, launch Obsidian and you'll be greeted with the option to create your first vault.

---

## Installing Obsidian on a Raspberry Pi

To install Obsidian on Raspberry Pi there are two main routes:

- [flatpak](#installing-via-flatpak)
- [pi-apps](#installing-via-pi-apps)

---

### Installing via flatpak

1. Install flatpak if you haven't already:

```sh
sudo apt install flatpak
```

2. Add the Flathub repository:

```sh
flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo
```

3. Install Obsidian:

```sh
flatpak install flathub md.obsidian.Obsidian
```

---

### Installing via Pi-Apps

1. Open the [Pi-apps](/blogs/install_pi_apps) on your Raspberry Pi.
2. Search for "Obsidian" in the store.
3. Click "Install" and follow the prompts.

---

Once installed, launch Obsidian and you'll be greeted with the option to create your first vault.

---
