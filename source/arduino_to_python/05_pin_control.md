---
title: Pin Control - Goodbye pinMode, Hello Pin Class!
description: Learn how to control GPIO pins in MicroPython using the Pin class instead of pinMode, digitalWrite and digitalRead, including pull-ups, debouncing and interrupts
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/banners/05_pin_control.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to set up output and input pins with the `Pin` class, read a button with a pull-up, and debounce it.

## Pins become objects

Arduino has three global functions: `pinMode()`, `digitalWrite()` and `digitalRead()`. MicroPython has one class, `machine.Pin`. You create a `Pin` object once, say whether it is an input or an output, and then call methods on it.

| Arduino C++ | MicroPython |
|-------------|-------------|
| `pinMode(13, OUTPUT);` | `led = Pin(13, Pin.OUT)` |
| `pinMode(2, INPUT_PULLUP);` | `button = Pin(2, Pin.IN, Pin.PULL_UP)` |
| `digitalWrite(13, HIGH);` | `led.on()` or `led.value(1)` |
| `digitalWrite(13, LOW);` | `led.off()` or `led.value(0)` |
| `digitalRead(2)` | `button.value()` |
{:class="table table-single"}

## Pin numbers on the Pico

On a Raspberry Pi Pico family board you use the GP number printed on the pinout. `Pin(14)` is GP14, which is physical pin 19, not pin 14 on the header. The on-board LED has its own name:

```python
from machine import Pin

led = Pin("LED", Pin.OUT)   # on-board LED on a Pico W or Pico 2 W
external = Pin(15, Pin.OUT) # GP15, with an LED and resistor
```

On the original Pico (non-W) the LED is GP25, so `Pin(25, Pin.OUT)` works there, and recent firmware also accepts `"LED"`. Other boards, including the Arduino Nano ESP32, number their pins differently, so always check the board's MicroPython pinout rather than reusing Arduino numbers.

## Writing and blinking

**Arduino C++:**
```cpp
void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin
import time

led = Pin("LED", Pin.OUT)

while True:
    led.toggle()      # Arduino has no equivalent: flip the current state
    time.sleep(0.5)
```

`.toggle()` is a handy bonus. `.on()`, `.off()` and `.value(x)` do the same job as `digitalWrite`, and `.value()` with no argument reads the state back.

## Reading a button with a pull-up

A button needs a resistor so the pin is never left floating. The Pico has built-in pull-up and pull-down resistors you switch on in code, as `INPUT_PULLUP` does on an Arduino.

```mermaid
flowchart LR
    V["3.3V"] --- R["Internal pull-up"] --- P["GP14 pin"]
    P --- B["Button"] --- G["GND"]
```

With `Pin.PULL_UP` the pin reads `1` when the button is open, and `0` when it is pressed, because pressing connects the pin to GND. This is called active low. `Pin.PULL_DOWN` is the reverse, with the button connecting to 3.3 V.

**MicroPython:**
```python
from machine import Pin

led = Pin("LED", Pin.OUT)
button = Pin(14, Pin.IN, Pin.PULL_UP)   # button between GP14 and GND

while True:
    if button.value() == 0:    # pressed
        led.on()
    else:
        led.off()
```

## Debouncing

Mechanical buttons bounce: the contact chatters for a few milliseconds when pressed. The Arduino debounce example uses `millis()`. In MicroPython you use `time.ticks_ms()` and always compare with `time.ticks_diff()` (lesson 7 explains why).

**MicroPython:**
```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)
led = Pin("LED", Pin.OUT)

DEBOUNCE_MS = 50
stable_state = 1
last_reading = 1
last_change = time.ticks_ms()

while True:
    reading = button.value()
    if reading != last_reading:
        last_change = time.ticks_ms()          # the pin moved: restart the clock
        last_reading = reading
    if time.ticks_diff(time.ticks_ms(), last_change) > DEBOUNCE_MS:
        if reading != stable_state:
            stable_state = reading
            if stable_state == 0:              # a clean press
                led.toggle()
                print("Button pressed")
    time.sleep_ms(5)
```

The program only accepts a new state once it has been steady for 50 ms, so one press gives exactly one toggle.

## Many pins, one list

Instead of copying `pinMode` for each LED, build a list of `Pin` objects:

**MicroPython:**
```python
from machine import Pin
import time

leds = [Pin(n, Pin.OUT) for n in (10, 11, 12, 13)]

while True:
    for led in leds:           # running light, left to right
        led.on()
        time.sleep(0.2)
        led.off()
    for led in reversed(leds): # and back again
        led.on()
        time.sleep(0.2)
        led.off()
```

## Interrupts

Instead of checking a pin in a loop, you can ask the hardware to call a function when the pin changes, like `attachInterrupt()`:

**Arduino C++:**
```cpp
attachInterrupt(digitalPinToInterrupt(2), onPress, FALLING);
```

**MicroPython:**
```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)
led = Pin("LED", Pin.OUT)
pressed = False

def on_press(pin):
    global pressed
    pressed = True            # keep handlers tiny: just set a flag

button.irq(trigger=Pin.IRQ_FALLING, handler=on_press)

while True:
    if pressed:
        pressed = False
        led.toggle()
        print("Press handled")
    time.sleep_ms(20)
```

The handler runs at interrupt time, so keep it very short, and do the printing and delays in the main loop. The triggers are `Pin.IRQ_FALLING`, `Pin.IRQ_RISING`, or both combined with `|`.

> **Teachers**: A breadboard with one LED and one button per pair is plenty. Check LED resistors before the first power-up, since an LED connected straight to a GPIO pin without a resistor can damage the LED or the pin. If learners share boards, give each pair a labelled pinout printout so everyone uses the same GP numbers.

## Try it Yourself

1. Wire an LED and a resistor to GP15 and blink it with `.toggle()` at half-second intervals.
2. Make the LED follow the button: on while pressed, off when released.
3. Make a four-LED binary counter on GP10 to GP13: each button press adds one, and it wraps at 16. Hint: `leds[i].value((count >> i) & 1)`.
4. Challenge: add a second button that cycles through three patterns (all on, alternating, running light) each time it is pressed.

## Common Issues

- **Problem**: The LED does nothing, and there is no error.
- **Solution**: Check the GP number against the pinout, and check the LED is the right way round with a resistor.
- **Why**: `Pin(15)` is GP15, not the 15th pin on the header.

- **Problem**: The button reads `1` when pressed instead of `0`.
- **Solution**: With `Pin.PULL_UP` and the button to GND, pressed is `0`. Test with `== 0`.
- **Why**: Pressing connects the pin to ground, so the logic is inverted.

- **Problem**: One press toggles the LED several times.
- **Solution**: Use the debounce pattern above, or ignore changes for 50 ms.
- **Why**: Mechanical contacts bounce for a few milliseconds.

- **Problem**: `ValueError: invalid pin` or `OSError`.
- **Solution**: Use a GP number from the pinout, or the name `"LED"`.
- **Why**: The pin does not exist on that board.

## Check your understanding

{% quiz %}
? What does `button = Pin(14, Pin.IN, Pin.PULL_UP)` replace in Arduino?
- [ ] `digitalWrite(14, HIGH)`
- [x] `pinMode(14, INPUT_PULLUP)`
- [ ] `analogRead(14)`
> It sets the pin as an input with the internal pull-up resistor turned on.

? A button is wired between the pin and GND with `Pin.PULL_UP`. What does `button.value()` return when it is pressed?
- [ ] 1
- [x] 0
- [ ] None
> Pressing the button connects the pin to ground, so it reads low.

? Which method flips an LED between on and off?
- [ ] `led.flip()`
- [x] `led.toggle()`
- [ ] `led.swap()`
> `toggle()` is a MicroPython convenience that has no Arduino equivalent.

? What should an interrupt handler do?
- [ ] Run long delays and print lots of text
- [x] Do as little as possible, such as setting a flag
- [ ] Contain the whole program
> Interrupt handlers pause the main program, so they should be short.
{% endquiz %}
