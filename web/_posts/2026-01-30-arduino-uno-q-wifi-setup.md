---
title: "How to Set Up WiFi on the Arduino Uno Q"
description: >-
  Configure your Arduino Uno Q's WiFi using App Lab or the command line with ADB and nmcli. Two methods, one guide.
excerpt: >-
  Get your Arduino Uno Q online using App Lab or the command line. This guide covers both methods for configuring WiFi over USB-C.
layout: showcase
mode: light
date: 2026-01-30
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/arduino_uno_q_wifi/cover.jpg
hero: /assets/img/blog/arduino_uno_q_wifi/hero.png
tags:
  - arduino
  - wifi
  - iot
  - app lab
  - cli
  - adb
groups:
  - arduino
  - electronics
---

Ahoy there makers! If you've just picked up the shiny new **Arduino Uno Q** and you're wondering how to get it connected to your WiFi network, you're in the right place. The Uno Q is a fantastic hybrid board that combines a full Linux single-board computer with a real-time microcontroller, and getting it online is one of the first things you'll want to do.

This guide covers **two methods** for setting up WiFi:

1. **Using App Lab** --- the graphical IDE from Arduino (great for beginners)
2. **Using the command line** --- via ADB and `nmcli` (for when you want to skip App Lab entirely or need to fix a misconfigured network)

Let's walk through both.

---

## What is the Arduino Uno Q?

The Arduino Uno Q isn't your typical Uno. Under the hood it packs a **Qualcomm Dragonwing QRB2210 processor** running Debian Linux alongside an **STM32U585 microcontroller** running sketches under Zephyr OS. For wireless connectivity, it has an on-board **WCBN3536A module** providing dual-band WiFi 5 (2.4GHz and 5GHz) and Bluetooth 5.1, both with built-in antennas.

The key tool for programming and configuring this board is **Arduino App Lab** --- a desktop IDE that supports Python, JavaScript, and C++ development, complete with modular "Bricks" (prebuilt libraries) for tasks like computer vision, audio, and web hosting.

---

## What You'll Need

Before you start, make sure you have:

- An **Arduino Uno Q** board
- A **USB-C data cable** (not a charging-only cable --- this is a common gotcha)
- A computer running **Windows 10+**, **macOS 11+**, or **Ubuntu 22.04+** (all 64-bit)
- Your **WiFi network name (SSID) and password**

For the **App Lab method**, you'll also need:
- **Arduino App Lab** installed on your computer ([download here](https://www.arduino.cc/en/software))

For the **CLI method**, you'll also need:
- **ADB (Android Debug Bridge)** installed on your computer ([download here](https://developer.android.com/tools/releases/platform-tools#downloads))

> **Heads up:** Use a USB-C or USB 3.0 port on your computer for the best results. The Arduino Uno Q is not compatible with Apple USB-C hubs and may not be detected through some third-party USB hubs either.

---

## Method 1: Using App Lab

This is the easiest approach and the one Arduino recommends for first-time setup. App Lab handles WiFi configuration through a guided wizard.

### Step 1: Download and Install Arduino App Lab

Head over to the [Arduino Software page](https://www.arduino.cc/en/software) and download **Arduino App Lab** for your operating system. Install it following the standard process for your platform.

App Lab is a separate application from the classic Arduino IDE --- it's purpose-built for the Uno Q and its hybrid architecture.

---

### Step 2: Connect the Uno Q via USB-C

Plug your USB-C data cable into the Uno Q and connect the other end directly to your computer. Avoid using USB hubs for this initial setup.

Once connected, the board will begin to boot. You'll see the **LED matrix display a swirling Arduino logo animation** during startup. Wait about 30 seconds until the animation changes to a **heartbeat pattern** --- that means the board has finished booting and is ready to go.

---

### Step 3: Launch App Lab and Detect the Board

Open Arduino App Lab. It will automatically scan for any Arduino Uno Q boards connected via USB or available on your local WiFi network.

After a moment, your board should appear in the device list. Select the **USB** option to connect to it over the cable.

If the board doesn't appear:

- Double-check that your USB cable supports data transfer (not just charging)
- Try a different USB port on your computer
- Make sure the board has fully booted (heartbeat LED animation)

---

### Step 4: Set a Password and Name Your Board

On first connection, App Lab will walk you through initial setup:

1. **Set a board password** --- choose something memorable. You'll need this to log into the board in the future.
2. **Confirm the password** and log in.
3. **Name your board** --- give it a unique name to identify it on your network, especially useful if you have multiple boards (great for classroom setups). You can type in a custom name or pick one of the randomly generated suggestions.

---

### Step 5: Connect to Your WiFi Network

This is the key step. App Lab will now present you with a list of available WiFi networks that the Uno Q can see.

1. **Select your WiFi network** from the list
2. **Enter your WiFi password**
3. Click **Confirm** to connect

The Uno Q supports dual-band WiFi 5, so you'll see both 2.4GHz and 5GHz networks if your router broadcasts them. Pick whichever you prefer --- 5GHz offers faster speeds but shorter range, while 2.4GHz has better range but lower throughput.

Once connected, the board will remember these credentials and automatically reconnect to the same network on future boots.

---

### Step 6: Install Updates

After connecting to WiFi, the Uno Q will check for firmware and App Lab updates. If updates are available, you'll be prompted to install them.

Let the updates complete --- this may involve a restart. After updating, close App Lab and relaunch it from your desktop. Your board should reconnect automatically.

---

### Step 7: Switch to Wireless Development

Here's the nice part. Now that WiFi is configured, you no longer need the USB cable for day-to-day development.

As long as your computer and the Uno Q are on the **same WiFi network**, App Lab will detect the board wirelessly. It will appear in the device list with a **"Network" tag** instead of "USB".

Simply:

1. Disconnect the USB cable
2. Power the Uno Q using a **5V/3A USB-C power supply**
3. Open App Lab --- your board will appear over the network
4. Select it and start coding

You can even connect to **multiple Uno Q boards** simultaneously over the network.

---

## Method 2: Using the Command Line (ADB + nmcli)

If you don't want to use App Lab --- or if you've misconfigured your WiFi and App Lab is stuck trying to check for updates --- you can configure WiFi entirely from the command line using **ADB (Android Debug Bridge)** and **nmcli**.

This is also the approach to use if you prefer working with tools like VS Code, want to script your setup, or are setting up multiple boards in a headless environment.

### Step 1: Install ADB

ADB is part of the Android SDK Platform Tools. You don't need the full Android SDK --- just the platform tools package.

Download it for your operating system from [developer.android.com](https://developer.android.com/tools/releases/platform-tools#downloads).

- **macOS:** Extract the zip and add the `platform-tools` folder to your PATH, or run the commands using the full path
- **Linux:** Extract and add to PATH, or install via your package manager (e.g. `sudo apt install adb` on Ubuntu)
- **Windows:** Extract the zip and open PowerShell in that folder

### Step 2: Connect the Uno Q and Verify

Plug your Uno Q into your computer with a USB-C data cable and wait for it to boot (about 30 seconds --- look for the heartbeat LED animation).

Then verify ADB can see the board:

```bash
adb devices
```

You should see your device listed. If it shows as "unauthorized", check the board's screen for a confirmation prompt.

### Step 3: Open a Shell on the Board

Drop into a shell session on the Uno Q:

```bash
adb shell
```

You're now running commands directly on the board's Debian Linux system. The default user is `arduino` and the default password is `arduino`.

### Step 4: Scan for WiFi Networks

List all available WiFi networks to find your SSID:

```bash
nmcli device wifi list
```

This will display a table of nearby networks including their SSID, signal strength, and security type.

### Step 5: Connect to Your WiFi Network

Connect to your network by replacing `YOUR_SSID` and `YOUR_PASSWORD` with your actual credentials:

```bash
sudo nmcli dev wifi connect "YOUR_SSID" password "YOUR_PASSWORD"
```

If your network name or password contains special characters, make sure to wrap them in quotes as shown.

For a **hidden network**, add the `hidden yes` flag:

```bash
sudo nmcli dev wifi connect "YOUR_SSID" password "YOUR_PASSWORD" hidden yes
```

You should see a confirmation message like `Device 'wlan0' successfully activated with...`

### Step 6: Verify the Connection

Check that you're connected and find your IP address:

```bash
nmcli
```

Look for the `wlan0` section --- your IP address will be listed under `inet4`, typically something like `192.168.x.x`.

You can also use:

```bash
hostname -I
```

This prints just the IP address, which is handy for scripting.

### Step 7: Enable SSH for Remote Access

Once WiFi is working, you can set up SSH so you don't need the USB cable or ADB at all:

```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start sshd
```

Now you can connect from your computer over the network:

```bash
ssh arduino@192.168.x.x
```

Replace `192.168.x.x` with the IP address you found in the previous step. The default password is `arduino` --- you should change this with `passwd` once you're logged in.

> **Tip:** If your router supports it, you can assign a static IP to your Uno Q using its MAC address (visible in the `nmcli` output under `wlan0`). This means the IP won't change between reboots.

### Step 8: Change WiFi Networks Later

Already connected but need to switch networks? You can do this at any time over SSH or ADB:

```bash
# Disconnect from current network
nmcli device disconnect wlan0

# Connect to a different network
sudo nmcli dev wifi connect "NEW_SSID" password "NEW_PASSWORD"
```

To see all saved network connections:

```bash
nmcli connection show
```

To delete a saved connection:

```bash
nmcli connection delete "CONNECTION_NAME"
```

---

## Troubleshooting

Here are some common issues and how to fix them:

**Board not detected over USB**
- Use a data-capable USB-C cable, not a charging-only cable
- Connect directly to your computer, not through a USB hub
- Wait for the heartbeat LED animation before opening App Lab

**Board not detected over WiFi**
- You must configure WiFi over USB first --- there's no way to set it up wirelessly from scratch
- Make sure your computer and the Uno Q are on the same network
- Check that the WiFi password was entered correctly during setup

**App Lab updates failing**
- Ensure your WiFi connection is stable
- Try restarting both the Uno Q and App Lab
- If the board becomes unresponsive, reconnect over USB

**Serial Monitor not showing output**
- The Uno Q's architecture works differently from a standard Arduino. Use `Monitor.println` via the `Arduino_RouterBridge.h` library instead of the usual `Serial.println`

**ADB doesn't see the board**
- Make sure ADB is installed and on your PATH (`adb version` should print a version number)
- Check that you're using a data-capable USB-C cable
- Try `adb kill-server && adb start-server` to restart the ADB daemon

**nmcli says "No WiFi device found"**
- The WiFi hardware may not have initialised yet --- wait a few more seconds after boot and try again
- Run `nmcli device status` to check the state of all network interfaces

**App Lab stuck checking for updates**
- This happens when the board connects to a WiFi network that requires a browser-based login (captive portal). Use the ADB + nmcli method to switch to a different network:
  ```bash
  adb shell
  sudo nmcli dev wifi connect "CORRECT_SSID" password "CORRECT_PASSWORD"
  ```

---

## Try It: Your First Wireless Sketch

Now that you're connected wirelessly, let's verify everything works with a quick test. Create a new App in App Lab and add the following code to your MCU sketch:

```cpp
#include <Arduino_RouterBridge.h>

void setup() {
  Monitor.begin();
  Monitor.println("Hello from the Uno Q!");
  Monitor.println("WiFi is working - welcome to wireless development!");
}

void loop() {
  Monitor.println("Still online...");
  delay(5000);
}
```

> **Note:** On the Uno Q, use `Monitor.println` instead of the usual `Serial.println`. The `Arduino_RouterBridge.h` library handles communication between the microcontroller and the Linux side of the board.

Upload the sketch wirelessly through App Lab and check the Sketch Console for output. If you see your messages appearing, everything is set up correctly.

**Challenge:** Try modifying the sketch to print the current millis() value alongside each message, so you can see how long the board has been running.

---

## What's Next?

With your Uno Q online, you can start exploring everything the board has to offer:

- **Build web interfaces** to control hardware remotely
- **Use computer vision Bricks** for object detection projects
- **Set up Docker containers** for isolated development environments
- **Run voice recognition** and AI workloads directly on the board

The Arduino Uno Q opens up a whole new world of possibilities by combining the simplicity of Arduino with the power of a Linux SBC. Getting WiFi set up is the gateway to all of it.

Happy making!

---

## Useful Links

- [Arduino Uno Q Documentation](https://docs.arduino.cc/hardware/uno-q)
- [Uno Q User Manual](https://docs.arduino.cc/tutorials/uno-q/user-manual/)
- [Arduino App Lab Getting Started](https://docs.arduino.cc/software/app-lab/tutorials/getting-started)
- [Arduino App Lab Overview](https://docs.arduino.cc/software/app-lab/)
- [Android SDK Platform Tools (ADB)](https://developer.android.com/tools/releases/platform-tools#downloads)
- [Shawn Hymel's CLI Guide for the Uno Q](https://shawnhymel.com/3074/how-to-use-the-command-line-cli-with-the-arduino-uno-q/)
- [Edge Impulse Uno Q Setup (includes CLI WiFi steps)](https://docs.edgeimpulse.com/hardware/boards/arduino-uno-q)

---
