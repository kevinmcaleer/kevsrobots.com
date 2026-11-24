---
title: Blinking an LED
description: Build your first complete hardware project, an external LED blinking on GP15, bringing together loops, functions, GPIO and USB output.
layout: lesson
type: page
cover: assets/9.jpg
date_updated: 2026-11-24
---

![Cover]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to wire an LED and resistor to a Pico, then build and flash a C program that blinks it in a pattern.

It's time to bring everything together. This project uses a loop (lesson 4), a function (lesson 5), a GPIO pin (lesson 8) and USB printing (lesson 7). When the LED blinks, you'll have written a program that makes something happen in the real world.

---

## What you'll build

An LED on GP15 that blinks three quick flashes, pauses, and repeats, while the Pico prints which burst it is on.

## What you'll need

- A Raspberry Pi Pico, Pico W or Pico 2
- A breadboard
- One LED (any colour)
- One 330 ohm resistor
- Two jumper wires
- A USB data cable

## Wiring the circuit

Always unplug the Pico while you wire.

| Pico pin | Goes to |
|----------|---------|
| GP15 (physical pin 20) | Long leg of the LED (the anode) |
| LED short leg (the cathode) | One end of the 330 ohm resistor |
| Other end of the resistor | Any GND pin on the Pico, such as physical pin 18 |
{:class="table table-single"}

```mermaid
flowchart LR
    A["GP15<br/>pin 20"] --> B["LED<br/>long leg in"]
    B --> C["LED<br/>short leg out"]
    C --> D["330 ohm<br/>resistor"]
    D --> E["GND<br/>pin 18"]
```

A handy way to remember the LED: long leg is live. Current flows in at the long leg and out of the short one. The resistor limits the current. Without it the LED can draw far too much and burn out, or put strain on the Pico's pin. With a 330 ohm resistor and a typical red LED, the current works out at only a few milliamps, which is safe. (The resistor can sit on either side of the LED.)

You can't take a photo of this from the text, so check your wiring against a Pico pinout diagram from Raspberry Pi's documentation: find GP15 and a GND pin and trace the loop.

> **Teachers**: LEDs and a resistor at 3.3 V are low risk, but resistors are tiny and easy to lose, and an LED fitted without a resistor is the usual way to break one. Hand out kits that count the parts out and back in. Learners often share one USB cable and port, so plan for who plugs in when. This lesson needs a computer with the toolchain from lesson 6. If you have time, let pairs swap roles: one wires, one types, then swap.

---

## The code

Create a new project folder, `~/pico/blink-led`, and copy `pico_sdk_import.cmake` into it as in lesson 7. Then create `blink.c`:

```c
#include <stdio.h>
#include "pico/stdlib.h"

const uint LED_PIN = 15;

void flash(int times, int on_ms, int off_ms) {
    for (int i = 0; i < times; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(on_ms);
        gpio_put(LED_PIN, 0);
        sleep_ms(off_ms);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    int burst = 0;
    while (true) {
        burst++;
        printf("Burst number %d\n", burst);
        flash(3, 150, 150);
        sleep_ms(1000);
    }
}
```

And the matching `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(blink_led C CXX ASM)

pico_sdk_init()

add_executable(blink blink.c)

target_link_libraries(blink pico_stdlib)

pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 0)

pico_add_extra_outputs(blink)
```

## How it works

- `flash(times, on_ms, off_ms)` is our own function. It uses a `for` loop to switch the pin on, wait, switch it off, wait, `times` times. Giving the pattern a name means `main` reads like a sentence.
- `LED_PIN` is declared once at the top, outside every function, so both `flash` and `main` can see it.
- The setup (`gpio_init`, `gpio_set_dir`) runs once. The `while (true)` loop then repeats the pattern forever.
- `printf` tells your computer each time a burst starts. The program works the same with no terminal open.

## Build and upload

```bash
mkdir build
cd build
cmake ..
make
```

Add `-DPICO_BOARD=pico_w` or `-DPICO_BOARD=pico2` to the `cmake` command if you have one of those boards. Then hold BOOTSEL, plug in the Pico, and copy `blink.uf2` from the `build` folder onto the `RPI-RP2` (or `RP2350`) drive. The LED should flash three times, pause, and repeat. Open a serial terminal as in lesson 7 to see the burst numbers.

**No circuit yet?** On a plain Pico or Pico 2 you can test the pattern on the onboard LED by changing `LED_PIN` to `25`. On a Pico W that will not work, because its LED is on the wireless chip, so wire the external LED.

---

## Try it Yourself

1. Change the call to `flash(5, 100, 100)` and watch the difference.
2. Make a slow heartbeat: `flash(2, 80, 120)` followed by a longer pause.
3. Add a second LED on GP14 with its own resistor, and flash both in turn.
4. Challenge: send an SOS in Morse code, three short, three long, three short, using `flash` with different timings.

## Common Issues

- **Problem**: The LED never lights.
- **Solution**: Turn the LED around. Check that GP15 is in the right hole and the resistor leads to GND.
- **Why**: An LED only passes current one way, and the circuit must be a complete loop.

- **Problem**: The LED is on all the time, dim, or does not blink.
- **Solution**: Check you copied the newest `blink.uf2` and that your delays are not tiny.
- **Why**: A blink faster than a few milliseconds looks like a steady glow.

- **Problem**: `make` fails with `Cannot find source file`.
- **Solution**: The names in `add_executable(blink blink.c)` must match the real file name exactly.
- **Why**: CMake builds only the files you list.

- **Problem**: The LED works but the terminal shows nothing.
- **Solution**: Open the serial port after the Pico has started, and see lesson 7 for the permission and device name tips.
- **Why**: USB serial depends on your computer connecting to the board.

## Check your understanding

{% quiz %}
? Why does the circuit need a resistor?
- [ ] To make the LED brighter
- [x] To limit the current through the LED
- [ ] To tell the Pico which pin to use
> The resistor limits current so the LED and the pin are not overloaded.

? Which leg of the LED goes towards the GPIO pin?
- [x] The long leg (anode)
- [ ] The short leg (cathode)
- [ ] Either, it makes no difference
> Current flows in through the long leg and out of the short one.

? In `flash(3, 150, 150)`, what do the numbers mean?
- [x] 3 flashes, each on for 150 ms and off for 150 ms
- [ ] 3 seconds, 150 pins, 150 volts
- [ ] 150 flashes, 3 ms on
> The parameters are the number of flashes and the on and off times in milliseconds.
{% endquiz %}
