---
title: Connecting the Arduino Uno R3
description: Learn how to connect the Arduino Uno R3 to your computer, set it up in the Arduino IDE, and upload your first sketch.
layout: lesson
cover: /learn/arduino_intro/assets/uno_r3_connection.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

The Arduino Uno R3 is one of the most widely used and beginner-friendly microcontroller boards in the Arduino ecosystem. It features easy connectivity, a wide range of compatible shields, and a solid community for support. In this lesson, you’ll learn how to connect the Arduino Uno R3 to your computer, set it up in the Arduino IDE, and upload your first sketch to verify that everything is working properly.

---

## Learning Objectives

- Understand the features of the Arduino Uno R3 and its pin layout.
- Connect the Arduino Uno R3 to your computer.
- Set up the Arduino IDE to work with the Uno R3.
- Upload a basic sketch to the board and verify the connection.

---

### What is the Arduino Uno R3?

The Arduino Uno R3 is a microcontroller board based on the ATmega328P. It has:

- **14 digital input/output pins**: 6 of which can be used as PWM outputs.
- **6 analog inputs**: For reading analog signals, such as from sensors.
- **USB connection**: To upload code from your computer and power the board.
- **Power jack**: For powering the board externally with an adapter.
- **Reset button**: To reset the board when necessary.

The Arduino Uno R3 is commonly used for prototyping and educational purposes, offering a solid platform for learning electronics and programming.

---

## Connecting the Arduino Uno R3 to Your Computer

### Step 1: Prepare Your Arduino Uno R3

1. **USB Cable**: The Arduino Uno R3 uses a standard USB Type-B cable (the type commonly used for printers). Ensure you have a working USB cable that fits the port on the board.
2. **Check the Board**: Make sure the board is in good condition and that there are no visible damages to the pins or components.

### Step 2: Connect the Arduino Uno to Your Computer

1. **Plug the USB Cable into the Board**: Connect one end of the USB cable to the Arduino Uno and the other end to a USB port on your computer.
2. **Power LED**: Once connected, the green power LED labeled "ON" on the Arduino Uno should light up, indicating that the board is receiving power from your computer.

---

## Configuring the Arduino IDE for the Arduino Uno R3

To ensure that the Arduino IDE can communicate with your Arduino Uno R3, you need to configure the board settings and select the correct port:

### Step 1: Select the Arduino Uno Board

1. **Open the Arduino IDE**: If you haven’t already done so, launch the Arduino IDE on your computer.
2. **Go to Tools > Board**: From the menu, select **Arduino Uno**. This tells the IDE that you're working with the Uno R3.

### Step 2: Choose the Correct Port

1. **Go to Tools > Port**: The port where your Arduino is connected should appear in the list. On Windows, it may be labeled as `COM3`, `COM4`, etc., while on macOS and Linux, it will appear as something like `/dev/tty.usbmodem` or `/dev/ttyUSB`.
2. **Select the Port**: Choose the correct port that corresponds to your connected Arduino.

If no port is detected:
- Ensure the USB cable is properly connected.
- Try a different USB port or cable.
- Verify that the necessary drivers for the Arduino Uno are installed (this is generally automatic, but in rare cases, you might need to install drivers manually from the [Arduino website](https://www.arduino.cc/en/Guide/ArduinoUno#toc4)).

---

## Uploading Your First Sketch to the Arduino Uno R3

To verify that everything is working, let’s upload a simple sketch to your Arduino Uno R3. We'll use the **Blink** example sketch, which will make the onboard LED blink on and off at regular intervals.

### Step 1: Open the Blink Sketch

1. **Go to File > Examples > 01.Basics > Blink**: This will open the Blink sketch, which is a pre-loaded example in the Arduino IDE.

The code for the Blink sketch looks like this:

```cpp
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}

void loop() {
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(13, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(13, LOW);
  // wait for a second
  delay(1000);
}
```

### Step 2: Upload the Sketch to Your Arduino Uno

1. **Click the Upload Button**: In the top-left corner of the IDE, click the **Upload** button (right arrow icon). The IDE will first compile the sketch and then upload it to your Arduino Uno.
   
   - You’ll see status messages at the bottom of the IDE indicating the progress of the upload.
   - Once the upload is complete, the message "Done uploading" should appear.

### Step 3: Verify the Sketch is Working

1. **Check the LED**: The Arduino Uno has an onboard LED connected to pin 13. After uploading the Blink sketch, this LED should start blinking on and off at 1-second intervals.
   
   If the LED is blinking, congratulations! You’ve successfully uploaded your first sketch to the Arduino Uno.

---

## Troubleshooting Common Issues

### No COM Port Detected
- Ensure that the USB cable is firmly connected to both your computer and the Arduino Uno.
- Try using a different USB cable or port on your computer.
- If necessary, reinstall the Arduino IDE or drivers for the Uno.

### Sketch Not Uploading
- Double-check that you've selected **Arduino Uno** as your board and the correct port under the `Tools` menu.
- Ensure there are no issues with the USB cable or the power supply.
- Press the **Reset** button on the Uno before trying to upload the sketch again.

### LED Not Blinking
- Verify that the correct sketch was uploaded by re-uploading the Blink sketch.
- Ensure that there are no hardware issues with the Arduino Uno board.

---

## Summary

In this lesson, you learned how to connect the Arduino Uno R3 to your computer, configure the Arduino IDE to recognize the board, and upload your first sketch. The Arduino Uno is now set up and ready for use in your projects. In the next lesson, we’ll explore connecting and configuring the Arduino Nano, a smaller but equally powerful board.
```

---
