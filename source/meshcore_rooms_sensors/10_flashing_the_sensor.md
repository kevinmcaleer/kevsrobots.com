---
title: Flashing the Sensor Node
description: Load sensor firmware, confirm the role, and let the board auto-detect what you have wired up
type: page
layout: lesson
---

Third board, third firmware variant. The process is identical; the interesting part comes at first boot, when the firmware goes looking for sensors.

---

## Flashing

1. Fit the antenna
2. Plug the board in with a USB-C **data** cable
3. Open [flasher.meshcore.io](https://flasher.meshcore.io) in Chrome or Edge
4. Select **Heltec V3**
5. Select the **Sensor** firmware variant
6. Select the **EU/UK (Narrow)** preset
7. Flash

> ## If there is no Sensor variant listed
>
> Sensor support is compiled per board, and not every board gets a prebuilt sensor image. If your board has no sensor build:
>
> - Check whether a newer firmware release has added one
> - Consider a board that does have one - the RAK WisBlock and Seeed variants are common sensor platforms
> - Or build it yourself from [the MeshCore repository](https://github.com/meshcore-dev/MeshCore), enabling the sensor flags in `platformio.ini`
>
> Lesson 13 covers the custom build route in more detail.

---

## Wire the sensor before first boot

**Do this now, before you power it up properly.** The firmware scans the I2C bus **at boot**. A sensor plugged in afterwards will not be noticed until the next reset.

The wiring is four wires, and lesson 11 walks through it properly. For now:

| Sensor pin | Heltec V3 pin |
|---|---|
| VCC / VIN | 3V3 |
| GND | GND |
| SDA | **GPIO33** |
| SCL | **GPIO34** |
{:class="table table-single"}

**Not GPIO17 and GPIO18.** Those are the OLED's bus. The sensor firmware scans a separate bus - see lesson 2.

If you would rather flash first and wire later, that is fine - just remember to `reboot` afterwards.

---

## First boot and auto-detection

Connect the console and check the basics:

```text
ver
board
get role
get radio
```

Then ask what it found:

```text
sensor list
```

**This is the moment of truth.** If your BME280 is wired correctly and supported by this build, it appears here. If the list is empty or shows only battery, the firmware did not find your sensor.

`sensor list` takes an optional starting index if you have more sensors than fit in one response:

```text
sensor list 5
```

---

## The other two sensor commands

```text
sensor get <key>
sensor set <key> <value>
```

`sensor get` reads a single value by its key, which is useful for checking one thing without pulling the whole telemetry set. `sensor set` adjusts a sensor setting - the classic example being turning a GPS module on or off at runtime.

The available keys depend entirely on what is attached, which is why `sensor list` is where you always start.

---

## Confirm the radio settings

Same as every other node:

```text
get radio
```

Expect 869.618 MHz, 62.5 kHz, SF8, CR8 - matching the rest of your mesh. If not:

```text
set radio 869.618,62.5,8,8
reboot
```

---

## Name it and place it

```text
set name Greenhouse-Sensor
set lat 53.4084
set lon -2.9916
set owner.info Kev - kevsrobots.com
```

**Name it after where it is, not what it is.** `Greenhouse` tells you something useful when the reading looks odd. `Sensor-02` does not.

---

## Transmit power on a battery node

```text
get tx
set tx 17
```

Full power makes sense for mains nodes. For a battery sensor, dropping from 22 to 17 dBm roughly halves the transmit current for about 40% of the range. If your sensor has a repeater within easy reach, that is a good trade; if it is right at the edge, keep the power up.

The valid range is 1 to 22 dBm.

---

## Send an advert

```text
advert
```

Your client will not know the sensor exists until it hears an advert. Plain `advert` floods across the mesh; `advert.zerohop` announces only to nodes in direct range.

---

## Try it Yourself

1. Run `sensor list` with nothing wired up. What does it report? (Battery and uptime should still be there.)
2. Wire the BME280, reboot, and run `sensor list` again. Compare the two.
3. Challenge: unplug the sensor while the node is running, then run `sensor list`. Does it notice? Now reboot and check again. This tells you a lot about when detection actually happens.

---

## Common Issues

**Problem**: `sensor list` shows nothing but battery.

**Solution**: Check the wiring, confirm the sensor is on the supported list, then `reboot` so the I2C scan runs again.

**Why**: Detection happens at boot. A sensor connected after startup is invisible until the next reset.

**Problem**: `sensor list` is not a recognised command.

**Solution**: You are on repeater or companion firmware, or a build without sensors compiled in.

**Why**: The sensor commands only exist when sensor support is compiled into that firmware image.

**Problem**: I wired the sensor to GPIO17 and GPIO18 and nothing was detected.

**Solution**: Move it to GPIO33 and GPIO34, then `reboot`.

**Why**: GPIO17 and GPIO18 are the OLED's bus. The sensor firmware scans a second, separate bus - so a perfectly wired sensor on the wrong bus is invisible.

**Problem**: The sensor works on USB but not on battery.

**Solution**: Check the Vext rail. On the Heltec V3, GPIO36 controls power to the external 3V3 rail.

**Why**: Some power-saving configurations switch the external rail off, which cuts power to anything you have wired to 3V3.

---
