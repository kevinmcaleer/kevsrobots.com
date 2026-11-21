---
title: Reading Temperature with the ADC
description: Learn how to write a Python script to read raw data from the Raspberry Pi Pico's built-in temperature sensor using its ADC.
layout: lesson
type: page
---

## Introduction

In this lesson, you’ll write your first script to interact with the Raspberry Pi Pico’s temperature sensor. You’ll read raw ADC values and display them in the Thonny console.

---

## Step 1: Understanding the Code

The Pico’s ADC provides a digital representation of the temperature sensor’s reading. In Python, you can use the `machine` module to access the ADC.

---

## Step 2: Writing the Script

1. Open Thonny.
2. Type the following code:

   ```python
   import machine
   import time

   # Initialize the ADC for the temperature sensor (channel 4)
   sensor_adc = machine.ADC(4)

   while True:
       # Read the raw ADC value
       raw_value = sensor_adc.read_u16()
       print("Raw ADC Value:", raw_value)
       time.sleep(1)
   ```

3. Save the script to your Pico as read_temp.py.

---

### Step 3: Running the Script

1. Run the script in Thonny by pressing the green Run button.
2. Observe the raw ADC values printed in the Shell, one per second.
3. The `while True:` loop runs forever. Click the red **Stop** button (or press **Ctrl+C**) to stop it.

---

### What You’ll See

You should see numbers somewhere around 13,000 to 15,000 at room temperature, for example:

```raw
Raw ADC Value: 14018
Raw ADC Value: 14034
Raw ADC Value: 14002
```

The raw ADC value changes with the temperature of the Pico’s chip. Because the sensor’s voltage *falls* as the chip warms up, the number goes **down** when the Pico gets warmer. Try holding your finger on the chip in the middle of the board and watch it drop. In the next lesson, you’ll learn to convert these values into meaningful temperature readings.

---
