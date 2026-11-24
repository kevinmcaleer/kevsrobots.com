---
title: Install by drag and drop
description: Download a UF2 file from micropython.org and drag it onto your Pico in BOOTSEL mode, with no extra software needed.
layout: lesson
type: page
cover: /learn/how_to_install_micropython/assets/banners/04_drag_and_drop.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to install MicroPython on a Pico by copying a UF2 file onto it, without any special software.

Thonny's installer is lovely, but it hides what is really happening. The truth is much simpler: the Pico pretends to be a USB memory stick, and installing firmware is just copying one file onto it. Once you know that, you can install MicroPython from any computer, even one without Thonny.

## The steps

1. Open [micropython.org/download](https://micropython.org/download/) and choose your board page. Last lesson's table shows which one.
2. Download the `.uf2` file for the latest release. Remember where your browser saves it, often your Downloads folder.
3. Unplug the Pico from the USB cable.
4. Hold down the **BOOTSEL** button, plug the cable in, then let go of the button.
5. The Pico appears as a USB drive: `RPI-RP2` on a Pico or Pico W, `RP2350` on a Pico 2 or Pico 2 W. Open it and you will see a couple of small files.
6. Drag the `.uf2` file onto that drive.
7. Wait a moment. The Pico copies the file, restarts by itself, and the drive vanishes. That is how you know it worked. Do not unplug it before then.

```mermaid
flowchart LR
    A["UF2 file in<br>Downloads"] -->|drag and drop| B["Pico drive<br>RPI-RP2 or RP2350"]
    B --> C["Pico copies the<br>firmware and restarts"]
    C --> D["Drive disappears,<br>MicroPython runs"]
```

## A handy shortcut

If MicroPython is already on your Pico, you do not need to reach for the button. Type this at the REPL and the board reboots into bootloader mode, ready for a new UF2 file:

```python
import machine
machine.bootloader()
```

This is documented on every board's download page at micropython.org.

## Why the file must match the board

The UF2 file contains instructions for one particular chip and board. The Pico's bootloader is built into the chip and will copy whatever you give it, so it is up to you to pick the right one. If you pick the wrong file, the board may not start properly. It will not damage it. Put the Pico into BOOTSEL mode again and copy the correct file.

> **Teachers**: Some school computers block writing to USB drives, and some Chromebooks cannot do this at all. Test one machine before the lesson. If drag and drop is blocked, learners can use the Thonny method on an unrestricted computer, or you can pre-install MicroPython on a batch of Picos.

## Try it Yourself

1. Install MicroPython by drag and drop on your Pico. Time yourself.
2. Open the `RPI-RP2` or `RP2350` drive before copying and read the two small files you find. What do you think they are for? (Hint: one is an HTML shortcut.)
3. Challenge: use `machine.bootloader()` from the REPL to reboot into BOOTSEL mode, without touching the button. Then reinstall the firmware.

## Common Issues

- **Problem**: The drive appears but my computer will not let me copy the file.
- **Solution**: Check the file really ends in `.uf2` and was not saved as `.uf2.txt` or opened as a text file by the browser. Re-download if you are not sure.
- **Why**: The Pico only accepts a genuine UF2 file.

- **Problem**: The drive never appears.
- **Solution**: Try another USB cable or port, and make sure you are holding BOOTSEL as you plug the cable in, not after.
- **Why**: The Pico only looks at the button as it powers up.

- **Problem**: I copied the file but the drive is still there.
- **Solution**: Wait a few seconds. If it stays, the file may be for the wrong board or may not have finished downloading. Download it again.
- **Why**: The Pico restarts only after it has accepted a complete, valid firmware file.

## Check your understanding

{% quiz %}
? What does the Pico look like to your computer in BOOTSEL mode?
- [ ] A serial port
- [x] A USB drive
- [ ] A keyboard
> That is why you can install firmware just by copying a file.

? What happens once the UF2 file has been copied?
- [ ] You must press RESET
- [x] The Pico restarts by itself and the drive disappears
- [ ] Thonny opens
> A disappearing drive means the install worked.

? Which command reboots a Pico that already runs MicroPython into bootloader mode?
- [x] `machine.bootloader()`
- [ ] `machine.reset()`
- [ ] `os.format()`
> It saves you reaching for the BOOTSEL button.
{% endquiz %}
