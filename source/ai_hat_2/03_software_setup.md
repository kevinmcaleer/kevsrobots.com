---
title: Software setup
description: Update Raspberry Pi OS, install the Hailo packages for the AI HAT+ 2 and check the HAT is detected
layout: lesson
type: page
cover: /learn/ai_hat_2/assets/banners/03_software_setup.jpg
date_updated: 2026-11-24
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
