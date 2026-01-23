---
title: Reed Switches - Magical Magnetic Switches
description: Learn what reed switches are, how they work, and build amazing projects like door sensors, bike speedometers, and water flow meters with MicroPython and Arduino.
excerpt: Discover the magic of magnetic sensing! Learn how to use reed switches in your maker projects with complete wiring guides, code examples, and three real-world project tutorials.
layout: showcase
date: 2025-10-29
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/reed_switches/cover.jpg
hero: /assets/img/blog/reed_switches/hero.png
mode: light
tags:
 - micropython
 - circuitpython
 - arduino
 - electronics
 - sensors
 - reed_switch
groups:
 - arduino
 - micropython
 - electronics
videos:
  - 6SpmwUdcS_g
---

Ahoy there makers! Ever wondered how your laptop knows when the lid is closed? Or how security systems detect when a door opens? The secret is a simple but clever component called a **reed switch** - a tiny glass tube that acts like magic when you bring a magnet near it!

These fascinating little sensors are incredibly useful for all sorts of maker projects. They're cheap (often under £1), require no power to operate, and can last for millions of operations. Best of all, they're super easy to use with your Raspberry Pi Pico, Arduino, or any microcontroller.

Today we're diving deep into reed switches. You'll learn what they are, how they work, and most importantly - how to use them in your own projects. By the end of this post, you'll be able to build everything from door sensors to rotation counters to water flow meters!

---

## What is a Reed Switch?

A **reed switch** is a type of electrical switch that's operated by a magnetic field. It consists of two thin, flexible metal strips (called "reeds") sealed inside a glass tube filled with inert gas.

Here's what makes them special:

- **Hermetically sealed**: The reeds are sealed in glass, protecting them from dust, moisture, and corrosion
- **Magnetically activated**: No physical contact needed - just bring a magnet close
- **Long-lasting**: No mechanical wear from pressing buttons - can last 100 million+ operations
- **Zero power consumption**: The switch itself uses no electricity (unlike Hall effect sensors)
- **Simple to use**: Just two wires - connect them like any switch

The reeds are typically made from ferromagnetic materials (often nickel-iron alloy) that respond to magnetic fields. When you bring a magnet close, the reeds become magnetized, attract each other, and make electrical contact - closing the circuit!

---

## How Reed Switches Work

The operation is beautifully simple:

1. **At rest**: The two reeds are separated by a tiny gap (usually less than 0.1mm) inside the glass tube
2. **Magnet approaches**: When a magnet comes within range (typically 10-20mm), its magnetic field passes through the glass
3. **Reeds magnetize**: The magnetic field magnetizes the reeds with opposite polarities
4. **Contact made**: The reeds attract each other and touch, closing the electrical circuit
5. **Magnet removed**: When the magnet moves away, the reeds lose their magnetism and spring back apart, breaking the circuit

Think of it like two tiny diving boards made of magnetic material. When there's no magnet, they just sit there separated. But when a magnet appears, they bend toward each other and touch!

The glass envelope serves multiple purposes:
- Protects the reeds from environmental contamination
- Contains inert gas (often nitrogen) to prevent oxidation
- Provides a stable mounting for the reeds
- Allows the magnetic field to pass through unaffected

---

## Types of Reed Switches

Reed switches come in two main varieties:

### Normally Open (NO)

This is the most common type. **The circuit is open (off) when there's no magnet**, and closes (turns on) when a magnet approaches.

**Use cases:**
- Door/window sensors (trigger when opened)
- Counting rotations (trigger once per rotation)
- Proximity detection (trigger when object approaches)
- Bike speedometers (trigger when magnet passes)

### Normally Closed (NC)

Less common, but useful for specific applications. **The circuit is closed (on) when there's no magnet**, and opens (turns off) when a magnet approaches.

**Use cases:**
- Security systems (trigger alarm if magnet removed)
- Tamper detection (break circuit if cover removed)
- Fail-safe applications (detect if magnet falls off)

Some reed switches have **three terminals** (SPDT - Single Pole Double Throw), giving you both NO and NC connections in one package. The middle terminal (common) is connected to either the NO or NC terminal depending on magnet presence.

For this tutorial, we'll focus on **Normally Open** switches as they're the most common and versatile.

---

## Bill of Materials

Here's what you'll need to follow along with the examples in this post:

Item                    | Description                   | Quantity | Approx Price | Where to Buy
------------------------|-------------------------------|----------|--------------|-------------
Raspberry Pi Pico       | Microcontroller (or Pico W)   | 1        | £5.80        | [Pimoroni](https://shop.pimoroni.com/products/raspberry-pi-pico), [The Pi Hut](https://thepihut.com/products/raspberry-pi-pico)
Reed Switch (NO)        | Normally Open magnetic sensor | 1-5      | £0.30-£2.00  | [eBay](https://www.ebay.co.uk), [Amazon](https://www.amazon.co.uk), AliExpress
Neodymium Magnet        | 10mm diameter, strong         | 1        | £0.50        | [eBay](https://www.ebay.co.uk), Hardware stores
Jumper Wires            | Female-to-female              | 3        | £0.30        | [Pimoroni](https://shop.pimoroni.com), [The Pi Hut](https://thepihut.com)
Breadboard (optional)   | For prototyping               | 1        | £2.00        | [Pimoroni](https://shop.pimoroni.com), [The Pi Hut](https://thepihut.com)
10kΩ Resistor (optional)| External pull-up if needed    | 1        | £0.05        | [Pimoroni](https://shop.pimoroni.com), [The Pi Hut](https://thepihut.com)
**Total**               |                               |          | **£9-£11**   |
{:class="table table-striped"}

**Notes:**
- Reed switches are often sold in packs of 5-10, making them very economical
- You probably already have jumper wires and a breadboard if you're into maker projects
- Strong neodymium magnets work best - the cheap ceramic magnets may not have enough field strength
- An Arduino Uno, Nano, or any other microcontroller works just as well as the Pico

---

## Wiring Guide

Reed switches are incredibly simple to wire - they're just a switch with two terminals. However, proper wiring is important for reliable readings.

### Wiring to Raspberry Pi Pico

The best approach is to use the Pico's **internal pull-up resistor**:

```
Reed Switch Terminal 1  →  GPIO 15 (physical pin 20)
Reed Switch Terminal 2  →  GND (physical pin 23)
```

**Why this works:**
- The internal pull-up resistor keeps GPIO 15 at 3.3V (HIGH) when the switch is open
- When the magnet approaches and the switch closes, it connects GPIO 15 directly to GND (LOW)
- Your code reads HIGH = no magnet, LOW = magnet present

**Do I need an external resistor?**

No! The Pico has built-in pull-up resistors that you enable in code with `Pin.PULL_UP`. However, if you want to use an external pull-up resistor (for example, to share the circuit between multiple microcontrollers or for specific voltage levels), here's the circuit:

```
3.3V  →  10kΩ Resistor  →  GPIO 15  →  Reed Switch  →  GND
                              ↓
                         (Read pin here)
```

**Physical Connection Steps:**

1. Identify your reed switch terminals (they're usually not polarized - either way works)
2. Connect one terminal to **GPIO 15** using a female-to-female jumper wire
3. Connect the other terminal to **GND**
4. That's it! Now let's write the code.

### Wiring to Arduino

The wiring is identical to the Pico:

```
Reed Switch Terminal 1  →  Digital Pin 2
Reed Switch Terminal 2  →  GND
```

Arduino boards also have internal pull-up resistors enabled with `INPUT_PULLUP` mode.

**Arduino Wiring with External Pull-up** (if preferred):

```
5V  →  10kΩ Resistor  →  Digital Pin 2  →  Reed Switch  →  GND
```

---

## Code Examples

Now for the fun part - let's make these switches actually do something!

### MicroPython - Basic Reading

This simple example reads the reed switch state and prints when a magnet is detected:

```python
from machine import Pin
import time

# Setup reed switch on GPIO 15 with internal pull-up
reed_switch = Pin(15, Pin.IN, Pin.PULL_UP)

print("Reed Switch Test")
print("Bring a magnet close to the switch...")

while True:
    # Read the pin - LOW (0) means magnet present
    if reed_switch.value() == 0:
        print("🧲 Magnet detected!")
    else:
        print("No magnet")

    time.sleep(0.2)  # Check 5 times per second
```

**How it works:**
- `Pin.PULL_UP` enables the internal pull-up resistor
- `reed_switch.value()` returns `0` (LOW) when magnet is present, `1` (HIGH) when absent
- We check every 0.2 seconds (5 times per second)

**Why active LOW?** When the switch closes, it connects the GPIO pin directly to ground (0V), which reads as LOW. This is standard practice for switches with pull-up resistors.

---

### MicroPython - Debouncing for Accurate Counting

If you're counting events (like door openings or wheel rotations), you need **debouncing** to prevent multiple triggers from a single event:

```python
from machine import Pin
import time

# Setup reed switch
reed_switch = Pin(15, Pin.IN, Pin.PULL_UP)

# Debouncing variables
last_trigger_time = 0
DEBOUNCE_MS = 50  # Ignore triggers within 50ms of previous
count = 0

print("Reed Switch Counter with Debouncing")
print("Pass the magnet by the switch to count events...")

while True:
    current_time = time.ticks_ms()

    # Check if switch is closed (magnet present)
    if reed_switch.value() == 0:
        # Calculate time since last trigger
        time_diff = time.ticks_diff(current_time, last_trigger_time)

        # Only count if enough time has passed (debouncing)
        if time_diff > DEBOUNCE_MS:
            count += 1
            print(f"Trigger #{count}")
            last_trigger_time = current_time

            # Wait for switch to open before looking for next trigger
            while reed_switch.value() == 0:
                time.sleep(0.01)

    time.sleep(0.01)  # Check frequently (100 times per second)
```

**Why debouncing matters:**

Reed switches can "bounce" - the contacts may make and break connection several times in a few milliseconds when opening or closing. This mechanical bounce would cause your code to count one event as multiple events!

Our debouncing solution:
1. Records the time of each trigger
2. Ignores any new triggers within 50ms of the previous one
3. Waits for the switch to open again before looking for the next trigger

This gives you accurate, reliable counting for rotation sensors, door counters, etc.

---

### MicroPython - Interrupt-Based Detection

For better responsiveness and lower power consumption, use **interrupts** to detect switch changes:

```python
from machine import Pin
import time

# Counter for interrupt events
trigger_count = 0
last_trigger_time = 0
DEBOUNCE_MS = 50

def reed_callback(pin):
    """Called when reed switch changes state"""
    global trigger_count, last_trigger_time

    current_time = time.ticks_ms()
    time_diff = time.ticks_diff(current_time, last_trigger_time)

    # Debouncing - only count if enough time has passed
    if time_diff > DEBOUNCE_MS:
        # Check if switch is actually closed (falling edge)
        if pin.value() == 0:
            trigger_count += 1
            print(f"🧲 Interrupt! Count: {trigger_count}")
            last_trigger_time = current_time

# Setup reed switch with interrupt on falling edge (HIGH → LOW)
reed_switch = Pin(15, Pin.IN, Pin.PULL_UP)
reed_switch.irq(trigger=Pin.IRQ_FALLING, handler=reed_callback)

print("Interrupt-based Reed Switch")
print("Pass magnet to trigger interrupt...")
print("Main program can do other things now!")

# Main loop is free to do other tasks
while True:
    # Your main program can do other work here
    # The reed switch will trigger interrupts automatically
    print(f"Current count: {trigger_count}")
    time.sleep(2)  # Update display every 2 seconds
```

**Advantages of interrupt-based detection:**
- **More responsive**: Triggers immediately when magnet approaches
- **Lower power**: CPU can sleep between events (important for battery projects)
- **Non-blocking**: Main loop can do other tasks while monitoring the switch

**How it works:**
- `Pin.IRQ_FALLING` triggers the interrupt when the pin goes from HIGH to LOW (magnet approaches)
- The `reed_callback()` function runs automatically when triggered
- We still use debouncing inside the callback to prevent false counts

---

### Arduino - Basic Reading

Here's the Arduino equivalent for reading a reed switch:

```cpp
const int REED_PIN = 2;  // Reed switch connected to digital pin 2

void setup() {
  Serial.begin(9600);

  // Enable internal pull-up resistor
  pinMode(REED_PIN, INPUT_PULLUP);

  Serial.println("Reed Switch Test");
  Serial.println("Bring a magnet close...");
}

void loop() {
  // Read pin - LOW means magnet present
  int state = digitalRead(REED_PIN);

  if (state == LOW) {
    Serial.println("🧲 Magnet detected!");
  } else {
    Serial.println("No magnet");
  }

  delay(200);  // Check 5 times per second
}
```

---

### Arduino - Debounced Counting

```cpp
const int REED_PIN = 2;
const int DEBOUNCE_MS = 50;

unsigned long lastTriggerTime = 0;
int count = 0;

void setup() {
  Serial.begin(9600);
  pinMode(REED_PIN, INPUT_PULLUP);

  Serial.println("Reed Switch Counter");
  Serial.println("Pass magnet to count...");
}

void loop() {
  unsigned long currentTime = millis();
  int state = digitalRead(REED_PIN);

  // Check if switch is closed (magnet present)
  if (state == LOW) {
    // Calculate time since last trigger
    unsigned long timeDiff = currentTime - lastTriggerTime;

    // Only count if enough time has passed (debouncing)
    if (timeDiff > DEBOUNCE_MS) {
      count++;
      Serial.print("Trigger #");
      Serial.println(count);
      lastTriggerTime = currentTime;

      // Wait for switch to open before next count
      while (digitalRead(REED_PIN) == LOW) {
        delay(10);
      }
    }
  }

  delay(10);  // Check frequently
}
```

---

## Project Examples

Now let's build three real-world projects using reed switches!

### Project 1: Door Sensor

Build a simple door/window sensor that detects when a door opens - perfect for home automation or security systems.

**How it works:**
- Mount the reed switch on the door frame
- Attach the magnet to the door edge
- When the door is closed, the magnet is close to the switch (switch closed)
- When the door opens, the magnet moves away (switch opens)
- Your microcontroller detects the change and can trigger an alarm, send a notification, or log the event

**Complete Code:**

```python
from machine import Pin
import time

# Setup
reed_switch = Pin(15, Pin.IN, Pin.PULL_UP)
led = Pin(25, Pin.OUT)  # Built-in LED on Pico

door_state = "unknown"
print("Door Sensor Active")

while True:
    current_state = reed_switch.value()

    if current_state == 0:  # Magnet present = door closed
        if door_state != "closed":
            print("🚪 Door CLOSED")
            led.off()
            door_state = "closed"
    else:  # Magnet absent = door open
        if door_state != "open":
            print("🚨 Door OPENED!")
            led.on()  # Turn on LED as visual indicator
            door_state = "open"
            # Here you could: send notification, trigger alarm, log to file, etc.

    time.sleep(0.1)
```

**Mounting Tips:**
- Use hot glue or double-sided tape to mount the switch on the frame
- Position the magnet on the door so it's within 10-20mm when closed
- Test the alignment before permanent mounting
- Consider weatherproofing for outdoor installations

**Enhancements:**
- Add WiFi notification using Pico W (send HTTP request when door opens)
- Log all door events to a file with timestamps
- Add multiple door sensors (one per door/window)
- Create a web dashboard showing status of all doors

---

### Project 2: Bike Speedometer

Measure your bike's speed by detecting wheel rotations!

**How it works:**
- Attach a small magnet to one of the wheel spokes
- Mount the reed switch on the bike frame near the wheel
- Each time the wheel rotates, the magnet passes the switch (one trigger = one rotation)
- Calculate speed from the rotation frequency and wheel circumference

**Complete Code:**

```python
from machine import Pin
import time

# Configuration
reed_switch = Pin(15, Pin.IN, Pin.PULL_UP)
WHEEL_CIRCUMFERENCE = 2.1  # meters (700c road bike wheel ≈ 2.1m)
DEBOUNCE_MS = 20

# Variables
last_trigger_time = 0
last_speed_calc_time = 0
rotation_count = 0

print("Bike Speedometer")
print(f"Wheel circumference: {WHEEL_CIRCUMFERENCE}m")

while True:
    current_time = time.ticks_ms()

    # Detect magnet pass (wheel rotation)
    if reed_switch.value() == 0:
        time_diff = time.ticks_diff(current_time, last_trigger_time)

        if time_diff > DEBOUNCE_MS:
            # Calculate time for one rotation
            rotation_time_ms = time_diff

            # Calculate speed (m/s then convert to km/h)
            if rotation_time_ms > 0:
                speed_ms = WHEEL_CIRCUMFERENCE / (rotation_time_ms / 1000)
                speed_kmh = speed_ms * 3.6

                print(f"Speed: {speed_kmh:.1f} km/h ({speed_ms:.1f} m/s)")

            rotation_count += 1
            last_trigger_time = current_time

            # Wait for magnet to pass
            while reed_switch.value() == 0:
                time.sleep(0.01)

    # Calculate distance every 5 seconds
    if time.ticks_diff(current_time, last_speed_calc_time) > 5000:
        distance_m = rotation_count * WHEEL_CIRCUMFERENCE
        distance_km = distance_m / 1000
        print(f"Total distance: {distance_km:.2f} km ({rotation_count} rotations)")
        last_speed_calc_time = current_time

    time.sleep(0.01)
```

**Installation Tips:**
- Use a strong neodymium magnet (cheap ones may not trigger consistently)
- Position the switch 5-10mm from the spoke path
- Secure all wiring so it doesn't get caught in the wheel
- Power the Pico from a USB power bank
- Mount the Pico on the handlebars for easy viewing

**Wheel Circumference Reference:**
- 700c road bike: ~2.1m
- 26" mountain bike: ~2.07m
- 29" mountain bike: ~2.29m
- Measure yours: (wheel diameter in meters) × π

**Enhancements:**
- Add OLED display to show speed in real-time
- Log rides to SD card (distance, time, average speed)
- Add cadence sensor (second reed switch on crank)
- Calculate calories burned based on speed and time

---

### Project 3: Water Flow Meter

Measure water flow without breaking into the plumbing!

**How it works:**
- Inside a water pipe, install a small turbine/propeller with a magnet attached
- Mount the reed switch on the outside of the pipe (magnetic field passes through plastic/PVC)
- As water flows, the turbine spins
- Each rotation triggers the reed switch
- Count triggers over time to calculate flow rate

**Complete Code:**

```python
from machine import Pin
import time

# Configuration
reed_switch = Pin(15, Pin.IN, Pin.PULL_UP)
PULSES_PER_LITER = 450  # Typical for common flow sensors (YF-S201)
DEBOUNCE_MS = 10

# Variables
pulse_count = 0
last_trigger_time = 0
last_display_time = 0
total_liters = 0

print("Water Flow Meter")
print("Start water flow to begin measuring...")

while True:
    current_time = time.ticks_ms()

    # Detect pulse
    if reed_switch.value() == 0:
        time_diff = time.ticks_diff(current_time, last_trigger_time)

        if time_diff > DEBOUNCE_MS:
            pulse_count += 1
            last_trigger_time = current_time

            # Wait for switch to open
            while reed_switch.value() == 0:
                time.sleep(0.001)

    # Calculate and display flow every second
    if time.ticks_diff(current_time, last_display_time) > 1000:
        # Calculate flow rate (liters per minute)
        flow_rate_lpm = (pulse_count / PULSES_PER_LITER) * 60

        # Calculate total volume
        liters_this_second = pulse_count / PULSES_PER_LITER
        total_liters += liters_this_second

        if flow_rate_lpm > 0:
            print(f"Flow: {flow_rate_lpm:.2f} L/min | Total: {total_liters:.2f} L")

        # Reset for next second
        pulse_count = 0
        last_display_time = current_time

    time.sleep(0.001)  # Check very frequently for accurate measurement
```

**Hardware Setup:**

For a DIY flow meter, you have two options:

1. **Buy a pre-made flow sensor** (£3-£5 on eBay/AliExpress):
   - Common models: YF-S201, YF-S402, YF-B1
   - These have the turbine and reed switch built in
   - Just connect the 3 wires: VCC, GND, Signal

2. **Build your own** (more advanced):
   - Install a small propeller/turbine inside a T-junction pipe
   - Attach a tiny magnet to one propeller blade
   - Mount reed switch on outside of pipe aligned with magnet path
   - Seal everything to prevent leaks

**Applications:**
- Monitor shower water usage (conservation)
- Detect leaks (unexpected flow when nothing is on)
- Irrigation system monitoring
- Aquarium auto-top-up systems
- Measure beverage dispensing (homebrew kegs!)

**Calibration:**

The `PULSES_PER_LITER` value depends on your sensor:
- YF-S201: ~450 pulses per liter
- YF-S402: ~5880 pulses per liter (high precision)
- DIY setup: Fill a measuring jug with exactly 1 liter, count pulses, adjust value

**Enhancements:**
- Add automatic shutoff valve (solenoid) to stop flow after X liters
- Send water usage data to home automation system
- Create daily/weekly usage reports
- Alert on abnormal flow (leak detection)
- Display on OLED screen or web dashboard

---

## Troubleshooting

Here are solutions to common issues you might encounter:

### Problem: Erratic readings or multiple triggers per event

**Symptom**: Getting 5-10 trigger events for a single magnet pass, or readings that jump randomly

**Causes**:
- **Switch contact bounce**: The mechanical contacts vibrate when closing, causing rapid on/off transitions
- **Electrical noise**: Nearby motors or power supplies inducing voltage spikes
- **No debouncing**: Code counting every state change without filtering

**Solutions**:
1. **Add software debouncing** (shown in code examples above):
```python
DEBOUNCE_MS = 50
if time.ticks_diff(current_time, last_trigger_time) > DEBOUNCE_MS:
    # Valid trigger
```

2. **Add hardware debouncing** - 0.1µF capacitor across the switch terminals

3. **Increase debounce time**: If 50ms isn't enough, try 100ms or more

**Why it works**: Debouncing ignores rapid changes within a short time window, filtering out mechanical bounce and electrical noise.

---

### Problem: Reed switch doesn't respond to magnet

**Symptom**: No trigger events no matter how close you bring the magnet

**Checklist**:
1. **Is the magnet strong enough?**
   - Test: Neodymium magnets work best (silver/metallic looking)
   - Cheap ceramic magnets (black) may be too weak
   - Try bringing the magnet within 5mm - if still no response, magnet is too weak

2. **Is the wiring correct?**
   - Verify GPIO pin number matches your code
   - Check connections are secure (not loose)
   - Confirm GND is actually connected to ground

3. **Is the pull-up enabled?**
   - MicroPython: Check you have `Pin.PULL_UP` in code
   - Arduino: Check you have `INPUT_PULLUP` mode
   - Without pull-up, readings will be random/unreliable

4. **Is the switch broken?**
   - Test with multimeter: Measure resistance between terminals
   - Should be infinite (open circuit) without magnet
   - Should be ~0Ω (closed circuit) with magnet close
   - If always open or always closed, switch is faulty

5. **Is the switch oriented correctly?**
   - Most reed switches work from any direction
   - But some are directional - try rotating the magnet/switch

**Quick test**: Remove the switch from your circuit and test with a multimeter + magnet separately to verify the switch itself works.

---

### Problem: Switch works but reading is backwards (HIGH when should be LOW)

**Symptom**: Code reads HIGH when magnet is present, LOW when absent

**Cause**: You're using a Normally Closed (NC) switch instead of Normally Open (NO), or the logic is inverted

**Solutions**:
1. **Simplest**: Just flip your logic in code:
```python
if reed_switch.value() == 1:  # Change 0 to 1
    print("Magnet detected!")
```

2. **Verify switch type**: Check if your switch is NO or NC (should be marked on packaging)

3. **Use correct switch**: Replace with NO switch if you meant to use NO

---

### Problem: Switch works but only at very close range (< 5mm)

**Symptom**: Need to get magnet extremely close for it to trigger

**Causes**:
- Weak magnet (most common)
- Low-sensitivity reed switch
- Thick barrier between magnet and switch

**Solutions**:
1. **Upgrade to neodymium magnet**: 10mm diameter disc magnets work great (£0.50 on eBay)
2. **Use multiple magnets**: Stack 2-3 magnets together for stronger field
3. **Choose high-sensitivity switch**: Some reed switches are more sensitive than others (check datasheet)
4. **Reduce barrier thickness**: If mounting through a surface, make it as thin as possible

**Testing magnets**: A good neodymium magnet should trigger a typical reed switch from 10-20mm away. If yours only works at 2-3mm, the magnet is too weak.

---

### Problem: Switch triggers randomly without magnet nearby

**Symptom**: Getting false triggers with no magnet present

**Causes**:
- **No pull-up resistor**: Pin is floating, picking up electrical noise
- **Nearby electromagnetic interference**: Motors, relays, power supplies
- **Broken switch**: Internal short circuit

**Solutions**:
1. **Verify pull-up is enabled**:
```python
reed_switch = Pin(15, Pin.IN, Pin.PULL_UP)  # Make sure PULL_UP is there!
```

2. **Add external pull-up**: If internal isn't strong enough, add 10kΩ resistor between GPIO and 3.3V

3. **Move away from interference**: Keep reed switch wires away from motors and power cables

4. **Test switch**: Use multimeter to verify switch is normally open without magnet

---

### Problem: Switch stops working after a while

**Symptom**: Works for a few minutes/hours then stops responding

**Causes**:
- **Overheating** (if switching high current)
- **Magnetic saturation** (from too-strong permanent magnet field)
- **Loose connection**

**Solutions**:
1. **Check current rating**: Reed switches typically handle 0.5-1A max. If you're switching higher loads, use a relay or transistor
2. **Remove permanent magnets**: Don't store the switch near strong magnets when not in use
3. **Verify connections**: Check for loose wires, cold solder joints
4. **Let it cool**: If switch is hot, reduce current or add heat sinking

---

### Problem: Reading is noisy or unstable

**Symptom**: Value flickers between HIGH and LOW rapidly

**Causes**:
- Long wires picking up electrical noise
- Missing or insufficient pull-up resistor
- Loose connections

**Solutions**:
1. **Shorten wires**: Keep reed switch wires as short as practical
2. **Use shielded cable**: For long runs (>1m), use shielded cable with shield connected to GND
3. **Add capacitor**: 0.1µF ceramic capacitor across switch terminals filters noise
4. **Increase pull-up strength**: Try 4.7kΩ instead of 10kΩ external resistor
5. **Check connections**: Ensure all solder joints are solid, no loose wires

---

## Try It Yourself

Ready to experiment? Here are some challenge projects to expand your skills:

### Beginner Challenges

1. **LED Door Indicator**: Build a circuit where an LED turns on when a door is closed (magnet present) and off when open.

2. **Event Counter**: Create a program that counts how many times a reed switch is triggered and displays the count. Reset the count with a button press.

3. **Two-State Detector**: Use two reed switches and magnets to detect three states: both magnets present, one present, or none present.

### Intermediate Challenges

4. **RPM Meter**: Attach a magnet to a motor shaft or fan blade. Use the reed switch to measure and display RPM (rotations per minute).

5. **Window Status Monitor**: Create a multi-window security system with multiple reed switches. Display which windows are open or closed on an OLED display.

6. **Pump Controller**: Build an automatic pump that turns on when water level drops (float with magnet) and off when level rises.

### Advanced Challenges

7. **Bike Computer**: Build a complete bike computer with speed, distance, and trip time. Add an OLED display and buttons for mode selection.

8. **Home Automation Integration**: Connect your reed switch door sensor to a WiFi-enabled Pico W and send MQTT messages to Home Assistant or similar home automation platform.

9. **Anemometer**: Build a wind speed sensor by creating a spinning cup anemometer with a magnet on the shaft. Calculate wind speed from rotation frequency.

10. **Multi-Point Security System**: Create a comprehensive security system monitoring 5+ doors/windows with individual status LEDs, alarm, and logging to SD card.

---

## Taking It Further

Congratulations! You now understand how reed switches work and how to use them in practical projects. Here are some ideas for expanding your knowledge:

### Advanced Techniques

- **Latching reed switches**: Special reed switches that stay closed after magnet is removed (until reset)
- **Mercury switches**: Similar concept but use liquid mercury (though becoming rare due to environmental concerns)
- **Hall effect sensors**: Alternative to reed switches using solid-state electronics (no moving parts)
- **Multiple switches for direction detection**: Use two switches spaced apart to detect direction of motion

### Related Projects

- **Rotary encoders**: Use two reed switches 90° apart to detect rotation direction
- **Position sensing**: Multiple reed switches in a line to detect position along a track
- **Vehicle detection**: Bury reed switches in driveway to detect cars passing over
- **Safe lock**: Create a magnetic combination lock using multiple reed switches

### Learn More

Want to dive deeper into sensors and GPIO control? Check out these related courses on KevsRobots.com:

- **[MicroPython GPIO](/learn/micropython_gpio/)** - Master GPIO control with comprehensive examples
- **[MicroPython - The Basics](/learn/micropython/)** - Learn MicroPython from scratch
- **[Getting Started with Raspberry Pi Pico](/learn/pico/)** - Complete Pico introduction

### Shopping for Reed Switches

When buying reed switches, consider:

- **Sensitivity**: Distance at which it triggers (measured in Ampere-Turns or AT)
- **Current rating**: Maximum current the switch can handle (0.5-1A typical)
- **Voltage rating**: Maximum voltage (100-200V typical for small switches)
- **Size**: Smaller switches are less sensitive but more compact
- **With or without leads**: Some have pre-attached wires, others are bare

**Recommended sources**:
- **AliExpress**: Bulk packs (100 switches for £2-3) - but slow shipping
- **eBay**: UK sellers with faster delivery
- **Amazon**: Convenient but slightly more expensive
- **SparkFun/Adafruit**: High quality, good documentation, but premium pricing

---

## Conclusion

Reed switches are magical little devices that open up a world of sensing possibilities! They're:

- **Simple to use**: Just two wires and basic code
- **Incredibly reliable**: No wear parts, can last decades
- **Dirt cheap**: Often under £1 each
- **Versatile**: Door sensors, speedometers, flow meters, and more
- **Low power**: Draw zero current themselves

Whether you're building home automation, robot sensors, or fun projects like bike computers, reed switches are an essential tool in your maker toolkit.

Now it's your turn - grab some reed switches, a magnet, and your microcontroller, and start building! Share your projects in the comments below - I'd love to see what you create!

**Happy making!** 🧲

---

## Additional Resources

- **Video Tutorial**: Check out the accompanying video at the top of this post for visual demonstrations
- **Code Repository**: Download all example code from [GitHub](https://github.com/kevinmcaleer/reed_switch_examples)
- **Community**: Join discussions on [KevsRobots Discord](https://discord.gg/kevsrobots)
- **Questions?**: Leave a comment below and I'll help you troubleshoot!

---
