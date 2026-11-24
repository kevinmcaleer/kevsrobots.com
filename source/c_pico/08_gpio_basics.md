---
title: GPIO Basics
description: Learn what GPIO pins are and how gpio_init, gpio_set_dir and gpio_put control a pin from C on the Raspberry Pi Pico.
layout: lesson
type: page
cover: assets/8.jpg
date_updated: 2026-11-24
---

![Cover]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to configure a GPIO pin as an output and switch it high and low with the Pico SDK.

So far the Pico has only talked to your computer. GPIO pins are how it talks to the world: lights, motors, buttons and sensors all connect through them. **GPIO** stands for General Purpose Input/Output, and each pin is a tiny switch your code controls.

---

## Which pins can I use?

The Pico and Pico 2 bring out **26 GPIO pins** to the edge of the board: GP0 to GP22, and GP26 to GP28. (GP23, GP24 and GP25 exist inside the chip but are used by the board itself, for example GP25 drives the onboard LED on a plain Pico.) The Pico W is similar, but its onboard LED is on the wireless chip, not on a GPIO pin you can name.

Note that "GP15" is the **GPIO number** your code uses, which is not the same as the **physical pin number** counted around the board. GP15 sits at physical pin 20. Always check a pinout diagram.

Each pin works in one of two directions:

- **Output**: your program sets the pin to high (3.3 V) or low (0 V). Use this to drive an LED.
- **Input**: your program reads whether something outside is pulling the pin high or low. Use this for buttons.

```mermaid
flowchart LR
    A["Your C code"] -- "gpio_put(15, 1)" --> B["GP15 at 3.3 V"]
    B --> C["330 ohm resistor"]
    C --> D["LED"]
    D --> E["GND at 0 V"]
```

---

## The four calls you need

```c
#include "pico/stdlib.h"

int main() {
    const uint LED_PIN = 15;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
}
```

This is a complete program. To build it, use the same `CMakeLists.txt` as lesson 7 but name the program `blink` instead of `hello`, and you can leave out the two `pico_enable_stdio` lines because we are not printing. We build and wire it in the next lesson.

What each call does:

- `gpio_init(pin)` gets the pin ready for ordinary use as a GPIO.
- `gpio_set_dir(pin, GPIO_OUT)` makes it an output. `GPIO_IN` makes it an input.
- `gpio_put(pin, value)` sets it high with `1` (or `true`) and low with `0` (or `false`).
- `sleep_ms(ms)` pauses for that many milliseconds.

`uint` is an unsigned whole number type the SDK defines for you, a good fit for pin numbers because they are never negative. `const` means the number can't be changed, so you can't accidentally move your LED to another pin halfway through.

**Why a named constant?** If you write `15` all through the code and then move the LED, you must change every one. With `LED_PIN` you change one line.

## Reading a pin

Outputs are only half the story. A taste of inputs, which we will not wire up in this course:

```c
gpio_init(14);
gpio_set_dir(14, GPIO_IN);
gpio_pull_up(14);
bool pressed = !gpio_get(14);
```

`gpio_pull_up` turns on a resistor inside the chip that holds the pin high when nothing is connected. A button wired from the pin to GND then pulls it low when pressed, so we flip the result with `!`. These four lines are a fragment, not a whole program.

---

## Try it Yourself

1. Change `500` to `100` in both `sleep_ms` calls. What does the blink look like?
2. Use `true` and `false` in place of `1` and `0`.
3. Make the LED stay on longer than it stays off, for example 900 ms on and 100 ms off.
4. Challenge: change `LED_PIN` to `16`. Which physical pin on the Pico is GP16? Use a pinout diagram.

## Common Issues

- **Problem**: The compiler says `'uint' undeclared` or `gpio_init` is unknown.
- **Solution**: Add `#include "pico/stdlib.h"` at the top, and check `pico_stdlib` is in `target_link_libraries`.
- **Why**: Those names are defined by the SDK, not by plain C.

- **Problem**: The pin never goes high.
- **Solution**: Check you called `gpio_set_dir(pin, GPIO_OUT)` and used the GP number, not the physical pin number.
- **Why**: Pins start as inputs, and GP numbers differ from the numbers printed along the board.

- **Problem**: The code works but a connected LED stays dark.
- **Solution**: Check the LED is the right way round (long leg towards the pin) and the circuit goes back to GND.
- **Why**: An LED only conducts one way, and needs a complete loop.

## Check your understanding

{% quiz %}
? What voltage does an output pin give when set high on a Pico?
- [ ] 5 V
- [x] 3.3 V
- [ ] 12 V
> Pico GPIO pins work at 3.3 V.

? Which call makes a pin an output?
- [ ] `gpio_init(pin)`
- [x] `gpio_set_dir(pin, GPIO_OUT)`
- [ ] `gpio_put(pin, 1)`
> `gpio_set_dir` chooses the direction. `gpio_init` only prepares the pin.

? GP15 is which kind of number?
- [x] The GPIO number your code uses, not the physical pin position
- [ ] The physical pin number on the board
- [ ] The memory address of the pin
> GP15 is at physical pin 20. Your code uses 15.
{% endquiz %}
