---
title: Blinking an LED
description: Build your first complete hardware project by making an LED blink using the Raspberry Pi Pico and C.
layout: lesson
type: page
cover: assets/9.jpg
date_updated: 2025-06-15
---

![Cover]({{page.cover}}){:class="cover"}

---

It's time to bring everything together!  
In this lesson, you’ll build your **first real hardware project**: a blinking LED using the Raspberry Pi Pico and C.

This is the “Hello World” of embedded programming — and it's just the beginning.

---

## What You’ll Build

You’ll connect an LED to a GPIO pin and write a C program that blinks it on and off every second.

---

## What You’ll Need

- Raspberry Pi Pico (or Pico W)
- Breadboard (optional)
- 1x LED
- 1x 330Ω resistor
- Jumper wires

### Wiring the Circuit

1. **GPIO 15** → long leg of LED (anode)
2. **Short leg (cathode)** → 330Ω resistor → GND

```text

\[Pico GPIO 15] ---\[LED]---\[330Ω]---\[GND]

```

---

## How It Works

Your program will:

- Set GPIO 15 as an **output**
- Turn the pin **HIGH** (3.3V) to light the LED
- Turn the pin **LOW** (0V) to turn it off
- Repeat the cycle with a 1 second delay

---

## Full Code Example

### `blink.c`

```c
#include "pico/stdlib.h"

int main() {
    const uint LED_PIN = 15;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(LED_PIN, 1);  // Turn LED on
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);  // Turn LED off
        sleep_ms(1000);
    }
}
```

---

## Build and Upload

1. Create a `CMakeLists.txt` similar to earlier lessons.
2. Compile your project:

    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```

3. Hold **BOOTSEL**, plug in your Pico
4. Copy the `blink.uf2` file to the **RPI-RP2** drive

The LED should start blinking!

---

## Try This

- Change the blink timing (try 250ms)
- Blink multiple LEDs on different pins
- Use a `for` loop to blink 10 times, then stop

---

## Summary

You’ve now:

- Built a simple electronic circuit
- Controlled it using C
- Completed your first embedded systems project!

---

Next up: [Course Summary and Next Steps](10_summary)

---
