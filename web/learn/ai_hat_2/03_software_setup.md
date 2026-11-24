---
layout: lesson
title: Software setup
author: Kevin McAleer
type: page
cover: /learn/ai_hat_2/assets/banners/03_software_setup.jpg
date: 2026-11-24
previous: 02_fitting_the_hat.html
next: 04_object_detection_with_rpicam_apps.html
description: Update Raspberry Pi OS, install the Hailo packages for the AI HAT+ 2
  and check the HAT is detected
percent: 28
duration: 2
quiz_total: 6
date_updated: 2026-11-24
navigation:
- name: Raspberry Pi AI HAT+ 2 - Vision and Generative AI
- content:
  - section: Meet the HAT
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is the AI HAT+ 2?
      link: 01_what_is_the_ai_hat_2.html
    - name: Fitting the HAT
      link: 02_fitting_the_hat.html
    - name: Software setup
      link: 03_software_setup.html
  - section: Eyes
    content:
    - name: Object detection with rpicam-apps
      link: 04_object_detection_with_rpicam_apps.html
    - name: Pose and segmentation
      link: 05_pose_and_segmentation.html
    - name: Reading detections from Python
      link: 06_reading_detections_in_python.html
    - name: Detections to robot behaviour
      link: 07_detections_to_robot_behaviour.html
  - section: A brain
    content:
    - name: A language model on the HAT
      link: 08_a_language_model_on_the_hat.html
    - name: A chat page in your browser
      link: 09_a_chat_page_in_your_browser.html
    - name: Asking the robot what it sees
      link: 10_asking_the_robot_what_it_sees.html
  - section: Choosing and fixing
    content:
    - name: Performance and choosing hardware
      link: 11_performance_and_choosing_hardware.html
    - name: Troubleshooting
      link: 12_troubleshooting.html
    - name: Summary
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

With the hardware fitted, let's install the software. These commands come straight from the Raspberry Pi AI software documentation at [raspberrypi.com/documentation/computers/ai.html](https://raspberrypi.com/documentation/computers/ai.html). The docs target a Raspberry Pi 5 running 64-bit Raspberry Pi OS (Trixie).

## PCIe speed

Older kits needed you to switch the PCIe port to Gen 3.0 by hand. The docs say the AI HAT+ and AI HAT+ 2 apply this automatically, so there is nothing to do.

## Update the Pi

Update the OS and the firmware, then reboot:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo rpi-eeprom-update -a
sudo reboot
```

## Install the Hailo packages

The AI HAT+ 2 uses a different package from the AI HAT+. This is the one for the AI HAT+ 2:

```bash
sudo apt install dkms
sudo apt install hailo-h10-all
```

**Important:** the docs warn that `hailo-all` (for the AI Kit and AI HAT+) and `hailo-h10-all` cannot co-exist. If you tried an older guide and installed `hailo-all`, you will need to sort that out before carrying on.

Reboot, then verify.

## Check it works

```bash
hailortcli fw-control identify
dmesg | grep -i hailo
```

The first command asks the HAT to identify itself. The second searches the kernel log for Hailo messages. The docs note that the AI HAT+ and AI HAT+ 2 may show `<N/A>` for Serial Number, Part Number and Product Name. That is expected, so don't panic.

## Check the camera

```bash
sudo apt update && sudo apt install rpicam-apps
rpicam-hello
```

If the camera works, `rpicam-hello` shows a preview for five seconds.

## Try it Yourself

1. Run `hailortcli fw-control identify` and read what it prints.
2. Run `dmesg | grep -i hailo` and find the line that mentions the device.
3. Challenge: write the three commands you ran, in order, in a notes file so you can set up your next Pi in minutes.

## Common Issues

**Problem**: `hailortcli` reports no device.
**Solution**: Power off, reseat the PCIe ribbon cable at both ends, and check you installed `hailo-h10-all`, not `hailo-all`.
**Why**: The HAT is a PCIe device, so it needs both a good connection and the right driver package.

**Problem**: The serial number shows `<N/A>`.
**Solution**: Nothing, carry on.
**Why**: The docs say this is expected for the AI HAT+ and AI HAT+ 2.
