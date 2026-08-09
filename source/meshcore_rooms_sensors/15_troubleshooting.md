---
title: Troubleshooting
description: A systematic checklist for room servers and sensor nodes that will not behave
type: page
layout: lesson
---

Work down this list in order. As in part one, most problems are in the first few items, and most of those are radio settings or wiring.

---

## The two minute checklist

Before anything clever:

1. **Antennas fitted?** Still the first question, still catches people
2. **Radio settings identical across all nodes?** `get radio` on each - 869.618, 62.5, 8, 8
3. **Has the node adverted?** Nothing knows it exists until it does
4. **Right firmware variant?** `get role` tells you what you actually flashed
5. **Clock set?** Especially critical on a room server

---

## Room server problems

### Nobody can find the room

1. `get radio` on the server. Compare all four numbers against a working client
2. `get role` - confirm it really is running room server firmware
3. Run `advert` on the console while someone watches their contacts list
4. Move the client and server a few metres apart - too close overloads the receiver

### People find it but cannot log in

1. `get guest.password` - is it what you told them?
2. `clock` - is the server's time right? Sync it
3. Have them wait a minute and try again - a client whose clock has jumped backwards is refused until it catches up
4. `get allow.read.only` - if on, they may be joining read-only without realising

### The room keeps emptying

1. `stats-core` - what is the uptime? If it keeps resetting, it is a power problem
2. Suspect the USB power bank first. Idle draw is often below its auto-shutdown threshold
3. Check for a loose USB connector - the classic slow-motion failure
4. Add a LiPo on the SH1.25 connector as backup

### People can read but not post

`set guest.password` and give them the password. Read-only access looks like a successful join until the first post fails.

### Posts arrive for some people and not others

1. Check the ones who miss out have actually logged in, not just come into range
2. `stats-core` on the server - is the queue backing up?
3. Consider whether those users have a working path to the server specifically, rather than just hearing a repeater

---

## Sensor node problems

### `sensor list` shows nothing but battery

In order:

1. **Check all four wires.** VCC, GND, SDA, SCL
2. **Confirm 3V3, not 5V**
3. **`reboot`.** Detection runs at boot only - this is the single most common cause
4. **Confirm the part is supported.** A DHT22 or DS18B20 will never appear
5. **Check for an address clash** if you have two sensors on the bus
6. **Confirm the firmware has sensors compiled in** - if `sensor list` is not a recognised command, it does not

### The OLED went blank when I wired the sensor

You have a short on the I2C bus. Check SDA to SCL and SDA to GND. The screen shares the bus, which makes it a free fault indicator.

### Telemetry requests time out

1. Can you message the node normally? If not, fix that first - telemetry rides on messaging
2. Is the node powered? Check battery voltage if you can reach it
3. How many hops away is it? Trace the path
4. Has it adverted recently?

### Readings look wrong

| Symptom | Likely cause | Fix |
|---|---|---|
| Temperature 1-2°C high | Self-heating from the board | Move sensor onto a short lead in free air |
| Temperature wildly high | Direct sunlight | Shade it |
| Humidity stuck at 100% | Condensation on the element | Dry it out somewhere warm |
| No humidity at all | It is a BMP280, not a BME280 | Check the lid for the vent hole |
| Pressure a few hPa off | Normal absolute accuracy | Use the trend, not the value |
| Battery voltage wrong | ADC multiplier needs calibrating | `set adc.multiplier` |
| Distance jumping around | Dirty lens or awkward target | Clean it, aim square at the surface |
{:class="table table-single"}

### The node dies overnight

1. Check voltage **under load**, not idle
2. Suspect cell capacity rather than cell voltage - an aged LiPo reads fine and collapses under current
3. Check the Vext rail if the sensor works on USB but not battery
4. Reduce `set tx` and lengthen advert intervals

---

## Diagnosing across both roles

These commands are your instrument panel:

| Command | Tells you |
|---|---|
| `ver` / `board` | Firmware and hardware - start here |
| `get role` | What this node actually is |
| `get radio` / `get freq` | Whether it can talk to anything |
| `stats-core` | Uptime, battery, queue depth |
| `stats-radio` | RSSI, SNR, noise floor |
| `stats-packets` | Whether it is hearing anything at all |
| `sensor list` | What sensors were detected at boot |
| `clock` | Time - critical for room servers |
| `get acl` | Who has what permission (serial only) |
{:class="table table-single"}

**A useful habit:** run `stats-radio` when everything is working and write down the noise floor. When something degrades in six months, you will have a baseline to compare against, and that turns guesswork into diagnosis.

---

## When to start over

Sometimes the fastest fix is a clean slate:

```text
erase
reboot
```

Then reflash and reconfigure. Ten minutes, and it eliminates every accumulated mistake at once.

**For a room server, note what you are losing:** `erase` clears configuration *and* the buffer. Tell your group first.

---

## Try it Yourself

1. Deliberately break something - unplug the sensor's SDA wire - and work down the checklist until you find it.
2. Record `stats-radio` output from every node in your mesh. That set of baselines is your future self's best diagnostic tool.
3. Challenge: write a one-page card for your own setup listing each node's name, role, radio settings, passwords and normal readings. Tape it inside the enclosure lid.

---

## Common Issues

**Problem**: Everything worked on the desk and nothing works once deployed.

**Solution**: The problem is physical - range, obstruction, antenna orientation, or power.

**Why**: Settings problems are all-or-nothing and would have failed on the desk too. Gradual, position-dependent failures are always physical.

**Problem**: I have tried everything and it still will not work.

**Solution**: `erase`, reflash, and rebuild the configuration on the desk with all nodes a few metres apart.

**Why**: Debugging two unknowns at once is much harder than starting from a known good state and adding one variable at a time.

---
