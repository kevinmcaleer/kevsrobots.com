---
title: The Hardware
description: Boards and sensors for both builds, and which sensors MeshCore actually supports
type: page
layout: lesson
---

Good news first: you already know the board. Both builds in this course run on the same Heltec WiFi LoRa 32 V3 you used in part one. The new spending is on sensors, and it is small.

---

## The boards

**Room server:** one Heltec WiFi LoRa 32 V3, **868MHz**, around £15. It behaves much like the repeater you already built - fixed location, mains power, no phone attached.

**Sensor node:** one Heltec WiFi LoRa 32 V3, **868MHz**, around £15, plus whatever you are measuring.

The same warning applies as in part one: **check the frequency variant**. The 863-928MHz boards are the ones sold as "868MHz" or "915MHz". A 433MHz board is the wrong hardware and cannot be fixed in software.

---

## A word on sensor hardware choice

The Heltec V3 is a fine board to *learn* on, and that is why this course uses it. For a sensor node you leave outdoors for months, it is not the best choice - the ESP32-S3 is thirsty compared to the alternatives.

| Board | Why you would pick it |
|---|---|
| Heltec V3 | Cheap, you already have one, has a screen for debugging |
| Heltec T114 | nRF52840 - dramatically lower idle current, the right answer for solar |
| RAK WisBlock | Modular sensor slots, industrial enclosures, higher cost |
| Seeed Xiao variants | Tiny, low power, good for tucking into small places |
| Seeed T1000-E | A sealed card-sized tracker with sensors built in |
{:class="table table-single"}

**Build it on a V3 first.** Get it working on the desk where you can see the screen and reach the USB port. Move it to a T114 once you know what you are doing and where it is going.

---

## Which sensors does MeshCore support?

This is the part worth reading carefully, because it saves you buying the wrong thing.

MeshCore's sensor firmware **scans the I2C bus at boot and auto-detects** what is connected. There is no configuration file, no pin mapping to edit, no sensor to declare. You wire it up, you power on, it appears.

The catch is that it detects a *specific list* of parts, and support is compiled into each board's firmware build. Commonly supported I2C sensors include:

| Sensor | Measures | Notes |
|---|---|---|
| **BME280** | Temperature, humidity, pressure | The recommended all-rounder, about £5 |
| BME680 | The above plus air quality / IAQ | More expensive, needs the BSEC library |
| BMP280 / BMP085 | Temperature, pressure | No humidity |
| AHT10 / AHT20 | Temperature, humidity | Cheap and cheerful |
| SHTC3 / SHT4x | Temperature, humidity | Better accuracy than AHT |
| LPS22HB | Temperature, pressure | Common on nRF dev boards |
| **INA219 / INA226 / INA260 / INA3221** | Voltage, current, power | Battery and solar monitoring |
| MLX90614 | Non-contact infrared temperature | Point it at a thing |
| **VL53L0X** | Distance (time of flight) | Tank levels, door open or shut |
| RAK12035 | Soil moisture and temperature | For growing things |
| RAK12500 / NMEA | GPS position | Location reporting |
{:class="table table-single"}

**If a sensor is not on that list, it will not just work.** A random DHT22 or a one-wire DS18B20 is not I2C and is not auto-detected. That does not make them useless - lesson 13 covers the custom firmware route - but it does mean they are not a five minute job.

> ## Check your board's build
>
> Sensor support is compiled per board. Before you buy, open the flasher at [flasher.meshcore.io](https://flasher.meshcore.io), select your board, and look at which firmware variants are offered. If there is no sensor build listed for your hardware, you will be building firmware yourself.

---

## The shopping list

### For the room server

| Item | Qty | Approx cost |
|---|---|---|
| Heltec V3, 868MHz | 1 | £15 |
| 868MHz antenna | 1 | £5 |
| 5V USB mains charger | 1 | £5 |
| **Total** | | **~£25** |
{:class="table table-single"}

### For the environment sensor

| Item | Qty | Approx cost |
|---|---|---|
| Heltec V3, 868MHz | 1 | £15 |
| 868MHz antenna | 1 | £5 |
| BME280 breakout, I2C | 1 | £5 |
| Dupont jumper wires | 4 | £2 |
| 3.7V LiPo with SH1.25 connector | 1 | £8 |
| **Total** | | **~£35** |
{:class="table table-single"}

Add an INA219 (£4) if you want battery and solar current, or a VL53L0X (£6) for tank level.

---

## Buying BME280 breakouts - one gotcha

BME280 breakouts are sold everywhere for a couple of pounds, and a meaningful number of the cheapest ones are actually **BMP280** - the version without a humidity sensor. The silkscreen sometimes says BME280 even when the chip is a BMP280.

**How to tell:** both chips have a hole in the metal lid, so presence of a hole proves nothing. The tells are **shape and position** - a BME280 is a 2.5 x 2.5 mm square package with the hole roughly **centred**; a BMP280 is a 2.0 x 2.5 mm rectangle with the hole in a **corner**. If the listing price looks too good, you are probably buying a BMP280.

**The certain test:** wire it up and see whether humidity appears. The chip IDs differ (0x60 for BME280, 0x58 for BMP280) and the firmware reports what it finds.

**Does it matter?** If you only care about temperature and pressure, no. If you wanted humidity in the greenhouse, yes.

Also check the breakout is the **I2C** version - some boards break out SPI only, or need a solder jumper moved to select I2C.

---

## The Heltec V3 pins you need

This is the detail that trips people up, so it is worth getting right before you wire anything.

The Heltec V3 has **two separate I2C buses** in the MeshCore sensor build. The onboard OLED sits on one; your sensor goes on the other.

| Function | Pin |
|---|---|
| **Sensor bus SDA** | **GPIO33** |
| **Sensor bus SCL** | **GPIO34** |
| OLED bus SDA (not for your sensor) | GPIO17 |
| OLED bus SCL (not for your sensor) | GPIO18 |
| OLED reset | GPIO21 |
| Vext control (switches the external 3V3 rail) | GPIO36 |
| Battery voltage ADC | GPIO1 |
{:class="table table-single"}

**Why this matters:** wire your BME280 to GPIO17 and GPIO18 - the pins most tutorials mention, because they are the OLED's - and the sensor firmware will not find it. It is scanning the other bus.

> ## Confirm against your own firmware
>
> These are the sensor-bus pins in the stock Heltec V3 sensor build (`ENV_PIN_SDA` and `ENV_PIN_SCL` in the board's `platformio.ini`). Different boards use different pins, and builds do change. If `sensor list` comes up empty on correct wiring, check the variant configuration in [the MeshCore repository](https://github.com/meshcore-dev/MeshCore) for your board.

---

## Try it Yourself

1. Open the flasher, select Heltec V3, and note down every firmware variant offered. Is there a sensor build?
2. Look at your BME280 breakout under a magnifier or phone camera. Can you see the vent hole in the metal lid?
3. Challenge: price up a solar sensor node on a Heltec T114 versus a V3. How much more is the T114, and how many months of runtime does it buy you?

---

## Common Issues

**Problem**: I bought a sensor and MeshCore does not see it.

**Solution**: Confirm it is on the supported list, that it is the I2C version, and that it is wired to the right pins.

**Why**: The firmware only detects the specific parts compiled into that build. Anything else is invisible to it, no matter how well wired.

**Problem**: My BME280 reports temperature and pressure but no humidity.

**Solution**: You have a BMP280.

**Why**: The two chips share a bus address and a footprint. The BMP280 simply has no humidity element, and the firmware reports what the chip offers.

---
