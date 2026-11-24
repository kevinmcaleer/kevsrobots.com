---
title: Setting Up the Toolchain
description: Install the Raspberry Pi Pico C/C++ SDK and build the official blink example into a .uf2 file for Pico, Pico W or Pico 2.
layout: lesson
type: page
cover: assets/6.jpg
date_updated: 2026-11-24
---

![Cover]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to install the Pico SDK, build the official `blink` example, and copy the result onto your board.

To turn C into something the Pico can run, you need a **toolchain**: a cross-compiler that makes code for the Pico's ARM chip, the CMake build tool that drives it, and the **Pico SDK** that holds all the Pico-specific code. This is the longest setup in the course, but you only do it once.

```mermaid
flowchart LR
    A["blink.c<br/>+ CMakeLists.txt"] --> B["CMake<br/>plans the build"]
    B --> C["arm-none-eabi-gcc<br/>compiles for the Pico"]
    C --> D["blink.uf2"]
    D --> E["Copied to the Pico<br/>over USB"]
```

---

## Which boards does this cover?

Everything from here on applies to the **Pico** and **Pico W** (both RP2040), and to the **Pico 2** (RP2350). Where they differ we say so. In short:

- You tell CMake which board you have with `-DPICO_BOARD=...`. The default is the original Pico. Use `pico_w` for a Pico W and `pico2` for a Pico 2.
- In BOOTSEL mode the board shows up as a drive called `RPI-RP2` (Pico, Pico W) or `RP2350` (Pico 2).

## Choose a route

**Route A: the official VS Code extension.** Raspberry Pi provides a Raspberry Pi Pico extension for Visual Studio Code that installs the tools for you and works on Windows, macOS and Linux. If you are on Windows or macOS, or you like an editor with buttons, start with Raspberry Pi's [Getting Started guide](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html), follow its VS Code section, then carry on with lesson 7 (the code and CMake ideas are the same). The documentation notes the extension is not supported on 32-bit Raspberry Pi OS or Raspberry Pi OS Lite.

**Route B: the command line, which we use here.** It shows you exactly what is going on, and it is how the SDK's own README describes setup. These steps are for Linux (including Raspberry Pi OS and Ubuntu under WSL). On Raspberry Pi OS Lite or the 32-bit version, where the extension is not supported, Raspberry Pi's documentation points to a `pico_setup.sh` script that sets up the toolchain; see the page linked above.

---

## Step 1: Install the tools (Linux)

These are the packages from the SDK's README:

```bash
sudo apt update
sudo apt install cmake python3 build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

## Step 2: Get the SDK and the examples

```bash
mkdir -p ~/pico
cd ~/pico
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
cd ..
git clone https://github.com/raspberrypi/pico-examples.git
```

The `git submodule update --init` step downloads extra libraries the SDK depends on, including TinyUSB, which we need for printing over USB in lesson 7.

## Step 3: Tell CMake where the SDK is

```bash
echo 'export PICO_SDK_PATH=$HOME/pico/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

The SDK's build scripts look for the environment variable `PICO_SDK_PATH`. (If you use zsh, put it in `~/.zshrc` instead. You can also pass `-DPICO_SDK_PATH=...` to `cmake` each time.)

## Step 4: Build the blink example

```bash
cd ~/pico/pico-examples
mkdir build
cd build
cmake ..
make blink
```

For a Pico W use `cmake -DPICO_BOARD=pico_w ..`, and for a Pico 2 use `cmake -DPICO_BOARD=pico2 ..`. Each example gets its own folder inside `build`, so your finished file is `build/blink/blink.uf2`.

If you switch boards later, make a fresh build folder. CMake remembers its settings in the folder.

## Step 5: Copy it to the Pico

1. Hold down the **BOOTSEL** button on the Pico.
2. While holding it, plug the USB cable into your computer, then let go.
3. A drive appears, named `RPI-RP2` (Pico, Pico W) or `RP2350` (Pico 2).
4. Drag `blink.uf2` onto that drive.

The board reboots by itself and the onboard LED starts blinking. On a Pico W the example also works when you built with `-DPICO_BOARD=pico_w`, because the Pico W's LED hangs off the wireless chip and the example code handles that.

> **Teachers**: installing the toolchain usually needs administrator rights, which school computers often block. Test the setup on one machine first, or use a shared build computer and let learners copy the `.uf2` over. The SDK and its submodules are a large download, so fetch it once and copy the folder around on a USB stick or shared drive instead of everyone downloading together. Allow around 90 minutes for this lesson. Check your school's rules on USB mass-storage devices before the first BOOTSEL drive appears.

---

## Try it Yourself

1. Open `~/pico/pico-examples/blink/blink.c` and change the delay (`LED_DELAY_MS`). Rebuild with `make blink` and copy the new file across.
2. Build another example, such as `hello_world`, and look in `build` for its folder.
3. Challenge: find where in `blink/CMakeLists.txt` the `.uf2` file is requested (look for `pico_add_extra_outputs`).

## Common Issues

- **Problem**: CMake says `SDK location was not specified. Please set PICO_SDK_PATH`.
- **Solution**: Open a new terminal (or run `source ~/.bashrc`) and check `echo $PICO_SDK_PATH` prints your SDK folder.
- **Why**: The variable is only read when a terminal starts.

- **Problem**: CMake says the directory does not appear to contain the Raspberry Pi Pico SDK.
- **Solution**: Check the path has no typo and that the clone finished.
- **Why**: The import script looks for `pico_sdk_init.cmake` inside that folder.

- **Problem**: No `RPI-RP2` drive appears.
- **Solution**: Unplug, hold BOOTSEL, then plug in. Try a different USB cable.
- **Why**: Many cables carry power only, with no data wires. BOOTSEL must be held at power-up.

- **Problem**: The build worked but the board does nothing.
- **Solution**: Check you built for the right board with `-DPICO_BOARD`.
- **Why**: Pico, Pico W and Pico 2 use different chips or LED wiring.

## Check your understanding

{% quiz %}
? Which CMake option tells the SDK you have a Pico 2?
- [ ] `-DPICO_SDK_PATH=pico2`
- [x] `-DPICO_BOARD=pico2`
- [ ] `-DUSB=on`
> `PICO_BOARD` selects the board. The default is the original Pico.

? Where does `PICO_SDK_PATH` need to point?
- [ ] At your project folder
- [x] At the folder you cloned `pico-sdk` into
- [ ] At the Pico's USB drive
> The build scripts import the SDK from that path.

? How do you get the Pico to appear as a USB drive?
- [x] Hold BOOTSEL while plugging it in
- [ ] Press the reset pin twice
- [ ] Type a command in the terminal
> Holding BOOTSEL at power-up starts the board's built-in drive.
{% endquiz %}
