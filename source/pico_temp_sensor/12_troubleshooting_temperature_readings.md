---
title: Troubleshooting Temperature Readings
description: Learn how to diagnose and fix common issues with your Raspberry Pi Pico temperature monitoring system.
layout: lesson
type: page
---

## Introduction

Even the best-designed systems can encounter issues. This lesson will guide you through diagnosing and resolving common problems you may face with your Raspberry Pi Pico temperature monitoring system.

---

## Common Issues and Solutions

### 1. **Inaccurate Temperature Readings**

   **Symptoms**:

   - Temperature values appear much higher or lower than expected.
   - Temperature fluctuates wildly.

   **Possible Causes**:

   - The temperature sensor is measuring the Pico chip's temperature, which may be affected by heat from the board itself.
   - Incorrect conversion formula.

   **Solutions**:

   - Remember the sensor reads the chip's own temperature, which is normally a few degrees above the room.
   - Ensure the Pico is not under load (e.g., running intensive scripts) during measurements.
   - Verify the conversion formula is correct, and that it is given the **voltage**, not the raw ADC value:

     ```python
     voltage = raw_value * 3.3 / 65535
     temperature = 27 - (voltage - 0.706) / 0.001721
     ```
     {: .no-snakie }

   - Average several readings to smooth out jitter. Readings naturally jump in steps of about half a degree, because each ADC step is about 0.47°C.
   - Apply a calibration offset measured against a thermometer (see the lesson on getting more accurate readings). The formula assumes a perfect 3.3 V reference, and a 1% error there shifts the result by about 4°C.

   - Test the readings in a stable room temperature environment and compare them with a thermometer.

---

### 2. **No Readings Displayed**

   **Symptoms**:
   - Thonny console shows no output or an error message.

   **Possible Causes**:
   - The script isn’t running properly.
   - The ADC channel is wrong. The temperature sensor is ADC channel 4, not a GPIO pin.
   - Thonny is set to the wrong interpreter (for example, your computer's own Python instead of MicroPython on the Pico).

   **Solutions**:
   - Check that the ADC pin is correctly initialized:

     ```python
     sensor_adc = machine.ADC(4)
     ```
     {: .no-snakie }
   - Read the last line of any error message in the Shell. For example, `AttributeError: 'tuple' object has no attribute 'tm_hour'` means the code was written for Python on a PC: in MicroPython `time.localtime()` returns a tuple, so use `timestamp[3]` for the hour.
   - Verify the script is saved and run in Thonny.
   - Restart the Pico by unplugging and reconnecting it.

---

### 3. **Temperature Readings Do Not Change**

   **Symptoms**:
   - Temperature stays constant regardless of changes in the environment.

   **Possible Causes**:
   - ADC readings are not being updated.
   - Sensor data is not being processed correctly.

   **Solutions**:
   - Add debugging statements to ensure the ADC values are being updated:

     ```python
     print(f"Raw ADC Value: {raw_value}")
     ```
     {: .no-snakie }
   - Verify the while loop is executing:

     ```python
     import time

     count = 0
     while True:
         print(f"The loop is running {count}")
         count += 1  # Increment the count
         time.sleep(1)  # without this the loop floods the Shell
     ```

---

### 4. **Script Crashes or Freezes**

   **Symptoms**:
   - The script stops responding or Thonny freezes.

   **Possible Causes**:
   - Infinite loop or too many prints in a short time.
   - Thonny struggling to handle high-frequency updates.

   **Solutions**:
   - Add a delay to the loop:
     ```python
     time.sleep(1)
     ```
     {: .no-snakie }
   - Limit the frequency of printed messages by updating only when necessary.

---

### 5. **Voltage Conversion Issues**

   **Symptoms**:
   - Voltage values seem too high or too low.

   **Possible Causes**:
   - Incorrect formula for converting ADC to voltage.
   - Miscalibrated ADC range.

   **Solutions**:
   - Ensure the correct formula is used:
     ```python
     voltage = raw_value * 3.3 / 65535
     ```
     {: .no-snakie }
   - Test the ADC separately by connecting a known voltage between 0 V and 3.3 V to GP26 (ADC channel 0) and reading `machine.ADC(26).read_u16()`. Never connect more than 3.3 V to an ADC pin.

---

## Debugging Tips

1. **Use Debugging Prints**:
   - Print intermediate values like `raw_value`, `voltage`, and `temperature` to pinpoint where the issue lies.

2. **Simplify the Script**:
   - Strip down your script to just the ADC reading and print functions to isolate the problem.

3. **Check Hardware Connections**:
   - Ensure the Pico is properly connected to your computer.
   - Verify the Pico is running MicroPython by checking the Thonny interpreter settings.

4. **Restart the System**:
   - Unplug and reconnect the Pico to reset it.

---

## What You’ve Learned

- How to identify and fix common issues with temperature readings.
- Debugging techniques for MicroPython scripts.
- Best practices for ensuring accurate and reliable temperature monitoring.

---

## Final Thoughts

Troubleshooting is an essential part of working with microcontrollers and sensors. By systematically identifying and resolving issues, you’ll not only fix your project but also strengthen your problem-solving skills.

Congratulations on completing this lesson—and the course! Keep experimenting and building exciting projects with your Raspberry Pi Pico.

---
