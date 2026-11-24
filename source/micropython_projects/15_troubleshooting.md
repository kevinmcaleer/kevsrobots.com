---
title: Troubleshooting
description: A single place to look when the status buddy doesn't behave, grouped by stage: Sense, Show, Talk, Connect and porting
layout: lesson
type: page
cover: /learn/micropython_projects/assets/banners/15_troubleshooting.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Every maker's project misbehaves at some point. That's not failure, it's the job. This lesson collects the problems you are most likely to meet, grouped by stage, so you can check your symptom and fix it fast.

---

## A debugging habit that saves hours

When the whole buddy won't work, **don't debug the whole buddy**. Go back to the stage-by-stage programs and test each part alone: the polling program from lesson 3 for the switch, the "hello" program from lesson 6 for the screen, the three lines from lesson 9 for SAM. The part that fails is the part to fix.

## Sense

| Problem | Solution | Why |
|---|---|---|
| Never detects the magnet | Use a neodymium magnet within 5mm. Check GP15 and GND wiring | Weak magnets and loose jumpers are the usual causes |
| Reads backwards (1 when the magnet is near) | You probably have a normally closed switch. Swap it, or flip the logic | A normally closed switch is closed with no magnet |
| One movement counts several times | Raise `DEBOUNCE_MS` to 100 | Contacts bounce for a few milliseconds when they close |
| Random readings | Make sure `Pin.PULL_UP` is set | Without a pull-up, an open switch leaves the pin floating |
| No reaction to the door, but polling works | Check both edges are set and the handler has no brackets | `on_edge()` calls the function. `on_edge` hands it over |
{:class="table table-single"}

If one setting isn't enough for a noisy switch, Kev's reed switch post also suggests a 0.1 microfarad capacitor across the switch terminals as hardware debouncing.

## Show

| Problem | Solution | Why |
|---|---|---|
| Screen is blank | Check VCC, GND and 3.3V (not 5V). Call `oled.show()` | Drawing only changes a buffer until you call `show()` |
| I2C scan finds nothing | Check SDA and SCL aren't swapped and match your code | I2C needs both wires on the right pins |
| `ENODEV` error | Run the scan. Check the address in `config.py` | The display isn't answering at that address |
| Garbled text | Lower the I2C speed with `freq=200000`. Check for loose wires | A fast bus on poor connections corrupts data |
| `no module named 'ssd1306'` | Copy `ssd1306.py` to the Pico's root | Imports only come from the Pico's own files |
| Old drawings linger | Start each frame with `oled.fill(0)` | The buffer keeps what you drew before |
{:class="table table-single"}

## Talk

| Problem | Solution | Why |
|---|---|---|
| Silence | Check speaker is on GP0 and GND, and polarity | SAM drives a single pin |
| `ImportError` for SAM | Copy the whole `sam` folder and `sam_render.mpy` | SAM is a package and a native module |
| Garbled or distorted | Add the 1K resistor. Make sure nothing else uses GP0 | The resistor softens the PWM, and sharing the pin corrupts it |
| Works, then breaks after updating MicroPython | Get a matching `sam_render.mpy` | The native module must match the MicroPython version |
| OLED and speaker fight | Keep the OLED on GP4 and GP5, not GP0 and GP1 | GP0 belongs to SAM in this course |
{:class="table table-single"}

## Connect

| Problem | Solution | Why |
|---|---|---|
| Never connects to Wi-Fi | Use a 2.4 GHz network. Check case in `secrets.py` | The Pico W doesn't speak 5 GHz, and the password is case-sensitive |
| `no module named 'cbor2'` or similar | Run `install_requirements.py` with Wi-Fi up | The cloud client needs those MIP packages |
| `no module named 'arduino_iot_cloud'` | Copy the folder to `/lib/` on the Pico | MIP can't fetch this library |
| Dashboard stays empty | Check `DEVICE_ID` and `SECRET_KEY` | A wrong key is silently rejected. The secret is only shown once |
| Dashboard is slow | This is normal with `interval=10` | The client only sends every 10 seconds |
| `Address already in use` on a web server | Add `SO_REUSEADDR` before `bind()` | The port stays locked for about a minute after a crash |
{:class="table table-single"}

## Porting

| Problem | Solution | Why |
|---|---|---|
| Wrong pin or `ValueError` | Look up the GPIO number behind each silkscreen label | `Pin()` takes GPIO numbers, not labels like D1 |
| I2C bus won't start | Check `I2C_ID` for those pins, or try `SoftI2C` | Hardware I2C belongs to specific pins |
| No LED | Set `LED_PIN = None` | Not every board has a user LED named `"LED"` |
| Wi-Fi code fails | Use a board with Wi-Fi and set `USE_CLOUD` to `False` otherwise | RP2040 and RP2350 XIAOs have no radio |
{:class="table table-single"}

## When all else fails

1. Disconnect everything except the Pico, and run a blink program to prove the board works
2. Add one part at a time, testing after each
3. Read the exact error message. MicroPython's errors usually name the line and the problem
4. Ask in the comments under Kev's videos. Someone has probably met your problem before

## Try it Yourself

1. Deliberately break one wire (carefully, with the power off) and note what error you get. Knowing what a broken wire looks like makes later faults easier to spot.
2. Write your own "problem, solution, why" row for something that went wrong while *you* built the buddy.

## Check your understanding

{% quiz %}
? The whole buddy fails and you aren't sure why. What is the best first step?
- [ ] Rewrite `main.py` from scratch
- [x] Test each part on its own with the small programs from earlier lessons
- [ ] Replace the Pico
> Testing one part at a time shows which part is at fault, so you fix one thing instead of guessing.

? Your screen is blank but the I2C scan finds the OLED at 0x3c. What is a likely cause?
- [ ] The speaker is on the wrong pin
- [x] `oled.show()` was never called
- [ ] The reed switch is wired backwards
> The scan proves the wiring works. Drawing only changes a buffer until `show()` sends it.
{% endquiz %}
