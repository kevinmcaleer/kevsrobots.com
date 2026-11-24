---
title: Installing FreeCAD
description: Step-by-step guide to installing FreeCAD on various operating systems, ensuring you are ready to start designing.
layout: lesson
type: page
cover: assets/1.png
date_updated: 2026-11-24
---

![Installing FreeCAD]({{ page.cover }}){:class="cover"}

By the end of this lesson you will be able to install FreeCAD on your computer and open it successfully.

> **Teachers**: Installing software on school machines usually needs an administrator. Ask your IT team well before the lesson, and check which FreeCAD version they will install, because buttons differ between versions. If installs are blocked, ask whether a portable (no-install) download is allowed, or run the lesson as a demo on one teacher machine. Also agree where learners save files, because school computers often wipe local folders at log-out.

## Getting Ready to Install FreeCAD

Before diving into the installation process, ensure your system meets the necessary requirements. Here’s a quick checklist:

- Operating System version (Windows, macOS, Linux)
- Sufficient disk space for installation - [check FreeCAD's official documentation](https://wiki.freecad.org/Download) for specific requirements.
- Graphics requirements for 3D modeling applications - FreeCAD is not very demanding, but a dedicated GPU can improve performance.

---

## Detailed Installation Steps

### Windows

1. **Download the Installer:**
   - Visit the [FreeCAD download page](https://www.freecad.org/downloads.php) and choose the installer for Windows.
2. **Run the Installer:**
   - Open the downloaded file and follow the on-screen instructions. Choose whether to install optional components.
3. **Complete Setup:**
   - Launch FreeCAD from the Start menu to check that it opens.

---

### macOS

1. **Download the .dmg File:**
   - Navigate to the same download page and select the macOS version.
2. **Install the Application:**
   - Drag FreeCAD into your Applications folder and then open it to ensure it runs correctly. macOS may ask you to confirm that you want to open an app downloaded from the internet.

---

### Linux & Raspberry Pi OS

1. **Using Terminal:**
   - For Debian-based distributions such as Ubuntu and Raspberry Pi OS, open your terminal and type: `sudo apt install freecad`
   - The version in the package list can be older than the one on the website, so lessons may look slightly different.
2. **Verify Installation:**
   - Launch FreeCAD from your applications menu to check if it was installed successfully.

---

## Post-Installation Steps

- **Update FreeCAD:** Check the download page for a newer version and install it if you want the latest features.
- **Configure Preferences:** Open the preferences from the menus (the menu name varies by operating system) and look at the unit settings. Millimetres suit robot parts.

---

## Troubleshooting Installation Issues

- **Permission Errors:** Make sure you have administrative access.
- **Failed Installations:** Check the error message and search the [FreeCAD forums](https://forum.freecad.org/) for it.

---

## Try it Yourself

1. Install FreeCAD, open it, and find the version number in the Help menu (look for an About FreeCAD entry).
2. Open any example or recent file offered when FreeCAD starts, and spin the model around with the mouse.
3. Challenge: change the units preference to millimetres, then close and reopen FreeCAD to prove the setting stuck.

---

## Common Issues

- **Problem**: The installer will not run on a school or work computer.
- **Solution**: Ask the administrator to install it, or ask about a portable version.
- **Why**: Ordinary accounts are usually not allowed to install programs.

- **Problem**: `sudo apt install freecad` says the package cannot be found.
- **Solution**: Run `sudo apt update` first, then try again.
- **Why**: The package list on your machine may be out of date.

- **Problem**: FreeCAD is slow or the 3D view is jerky.
- **Solution**: Update your graphics drivers and close other heavy programs.
- **Why**: The 3D view needs graphics support from the computer.

---

## Check your understanding

{% quiz %}
? Which command installs FreeCAD on Ubuntu or Raspberry Pi OS?
- [ ] `install freecad.exe`
- [x] `sudo apt install freecad`
- [ ] `drag freecad`
> The lesson uses apt on Debian-based Linux.

? Why might the Linux version look different from the lessons?
- [x] The version in the package list can be older than the website version
- [ ] Linux has no 3D view
- [ ] apt installs a different program
> Package versions often lag behind the latest download.

? What do you do on macOS after downloading the .dmg?
- [ ] Run a terminal command
- [x] Drag FreeCAD into your Applications folder
- [ ] Nothing, it runs from the download
> The macOS steps are to drag the app into Applications and open it.
{% endquiz %}

---

## Additional Resources

- [FreeCAD Download page](https://www.freecad.org/downloads.php)
- [FreeCAD Community Support Forums](https://forum.freecad.org/)
