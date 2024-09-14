---
title: Connecting the Arduino Nano
description: Learn how to connect the Arduino Nano to your computer, install the necessary drivers, and configure it in the Arduino IDE.
layout: lesson
cover: /learn/arduino_intro/assets/nano_connection.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

The Arduino Nano is a compact and versatile microcontroller board, ideal for projects that require a small form factor but still offer significant power. It provides most of the same functionality as the larger Arduino Uno, but in a more space-efficient package. In this lesson, you'll learn how to connect the Arduino Nano to your computer, install any necessary drivers, and configure the Arduino IDE to work with it. You will also upload your first sketch to test the connection.

---

## Learning Objectives

- Understand the features and applications of the Arduino Nano.
- Learn how to connect the Arduino Nano to your computer.
- Install drivers specific to the Arduino Nano, if necessary.
- Configure the Arduino IDE to work with the Nano.
- Upload a basic sketch to verify that the Arduino Nano is working correctly.

---

### What is the Arduino Nano?

The Arduino Nano is a small, breadboard-friendly version of the Arduino family, designed to offer full microcontroller functionality in a smaller package. Key features of the Nano include:

- **Microcontroller**: ATmega328P, the same as the Uno, but newer versions might use ATmega328PB.
- **Size**: Roughly 45mm x 18mm, making it much smaller than the Arduino Uno.
- **Digital I/O Pins**: 14 digital pins, 6 of which can be used for PWM output.
- **Analog Inputs**: 8 analog input pins, more than the 6 on the Arduino Uno.
- **Power Input**: Powered via USB (mini-USB or micro-USB) or external 5V supply.
- **USB Connection**: The Nano uses a mini-USB or micro-USB connector, depending on the version.

Due to its size, the Nano is a popular choice for compact projects, wearables, and applications where space is at a premium.

---

## Connecting the Arduino Nano to Your Computer

### Step 1: Identify Your Arduino Nano Version

Before connecting your Nano to your computer, it's important to know which version of the board you have. The Nano comes in different versions:

1. **Original Arduino Nano**: Uses the mini-USB connector and typically does not require any special drivers.
2. **Clone or Newer Nano (with CH340/CH341 USB Chip)**: Some clones and newer versions use the CH340/CH341 chip for USB-to-serial communication. These boards may require additional drivers to be installed on your computer for proper functionality.

Look on the back of your board for the CH340/CH341 chip, or check your vendor's description to confirm which version you have.

---

### Step 2: Install the Necessary Drivers

If you’re using a Nano with the CH340/CH341 chip, follow these steps to install the necessary drivers:

1. **Download the Driver**: Visit the [CH340/CH341 driver page](https://www.wch.cn/downloads/CH341SER_ZIP.html) and download the appropriate driver for your operating system.
2. **Install the Driver**:
   - **For Windows**: Run the `.exe` file and follow the installation instructions.
   - **For macOS**: Unzip the file and run the installation script.
   - **For Linux**: The drivers may already be installed, but if not, you may need to manually install them.
3. **Restart Your Computer**: After installation, restart your computer to ensure the drivers are correctly loaded.

Once the driver is installed, your Arduino Nano should be ready for use.

---

### Step 3: Connect the Arduino Nano to Your Computer

1. **Use the Correct Cable**: Depending on your Nano version, connect it to your computer using either a mini-USB or micro-USB cable.
2. **Check for Power**: Once connected, the Nano should power up, and the onboard power LED (usually labeled "ON") should light up, indicating that the board is receiving power from your computer.
3. **Verify the Connection**: If the drivers are properly installed, the Arduino IDE should recognize the board once it’s connected.

---

## Configuring the Arduino IDE for the Arduino Nano

After connecting the Arduino Nano to your computer, you need to configure the Arduino IDE to work with the Nano:

### Step 1: Select the Arduino Nano Board in the IDE

1. **Open the Arduino IDE**: Launch the IDE on your computer if it’s not already open.
2. **Go to Tools > Board**: In the menu, select **Arduino Nano** from the list of available boards. This tells the IDE that you are working with a Nano and will ensure compatibility during code uploads.

---

### Step 2: Choose the Correct Processor

The Arduino Nano comes with different bootloaders depending on the version. You need to select the right processor to match your Nano version:

1. **Go to Tools > Processor**:
   - **ATmega328P**: Select this option for most modern Nano boards.
   - **ATmega328P (Old Bootloader)**: Select this if you have an older Nano, or if the standard ATmega328P option does not work when uploading sketches.

If you're not sure which bootloader your Nano uses, try uploading a sketch with both processor options to see which one works.

---

### Step 3: Select the Correct Port

1. **Go to Tools > Port**: This will list all the serial ports available on your computer. The port for the Arduino Nano will usually be labeled as `COM3`, `COM4`, etc., on Windows, or `/dev/ttyUSB0` or `/dev/tty.usbmodem` on macOS/Linux.
2. **Select the Port**: Choose the correct port corresponding to your Nano. If no port appears, ensure that the Nano is properly connected, and the correct drivers are installed.

---

## Uploading Your First Sketch to the Arduino Nano

Once everything is set up, it’s time to upload a basic sketch to the Nano to verify that it’s working correctly.

### Step 1: Open the Blink Sketch

1. **Go to File > Examples > 01.Basics > Blink**: This will open the Blink sketch, a simple program that blinks the onboard LED (connected to pin 13) at one-second intervals.

The Blink sketch code looks like this:

```cpp
void setup() {
  // Initialize pin 13 as an output.
  pinMode(13, OUTPUT);
}

void loop() {
  // Turn the LED on (HIGH is the voltage level)
  digitalWrite(13, HIGH);
  // Wait for a second
  delay(1000);
  // Turn the LED off by making the voltage LOW
  digitalWrite(13, LOW);
  // Wait for a second
  delay(1000);
}
```

### Step 2: Upload the Sketch to Your Arduino Nano

1. **Click the Upload Button**: Once you’ve selected the correct board and port, click the **Upload** button in the IDE (right arrow icon). The IDE will compile the sketch and upload it to the Nano.
2. **Wait for the Upload**: You’ll see progress messages at the bottom of the IDE window, and when the upload is complete, the message "Done uploading" will appear.

### Step 3: Verify the Sketch is Working

1. **Check the LED**: The Arduino Nano has an onboard LED connected to pin 13. After the Blink sketch is uploaded, this LED should start blinking on and off at 1-second intervals.

---

## Troubleshooting Common Issues

### No COM Port Detected
- Ensure the USB cable is securely connected to both your computer and the Arduino Nano.
- Try using a different USB cable or port.
- Make sure the CH340/CH341 drivers are installed if you are using a Nano clone.

### Sketch Not Uploading
- Double-check that you’ve selected **Arduino Nano** as the board and chosen the correct processor.
- Verify that the correct port is selected in the `Tools > Port` menu.
- Try pressing the **Reset** button on the Nano and then re-uploading the sketch.

### LED Not Blinking
- Re-upload the Blink sketch to ensure it was uploaded correctly.
- Ensure that the correct pin (pin 13) is referenced in the code.

---

## Summary

In this lesson, you learned how to connect the Arduino Nano to your computer, install the necessary drivers, and configure the Arduino IDE to work with the Nano. You also uploaded your first sketch and verified that it’s working. In the next lesson, we’ll explore the Arduino Nano ESP32, which introduces advanced features such as Wi-Fi and Bluetooth.
```

---