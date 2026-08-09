---
title: Radio Settings for the UK
description: The EU/UK Narrow preset, what the numbers mean, and why both nodes must match exactly
type: page
layout: lesson
---

Both boards are flashed. Now we make sure they can actually hear each other - and that they can hear everybody else in the UK too.

---

## The rule that catches everyone

**Frequency, bandwidth, spreading factor and coding rate must match on every node that needs to talk to each other.** There is no negotiation, no auto-detect, no fallback. If one number is different, the nodes are silent strangers.

That applies to your two boards, and it applies to every other MeshCore node in your area. Which is why the community agrees on presets.

---

## The UK/EU preset

At the time of writing, the UK and European MeshCore community standard is the **EU/UK (Narrow)** preset:

| Setting | Value |
|---|---|
| Frequency | **869.618 MHz** |
| Bandwidth | **62.5 kHz** |
| Spreading Factor | **8** |
| Coding Rate | **8** |
{:class="table table-single"}

Some networks run the same settings with **CR5** instead of CR8 for slightly faster transmissions - and usefully, CR5 and CR8 nodes can still hear each other, because coding rate is carried in the packet header. Frequency, bandwidth and spreading factor are the three that absolutely must match.

> ## Check before you commit
>
> Regional conventions do change - the UK moved from an older 869.525MHz / BW250 / SF11 setup to these narrow settings. Before you finalise, check the current preset list in the flasher or app, and check with your local community: [localmesh.co.uk](https://localmesh.co.uk) for Britain.

---

## Why these numbers?

**869.618MHz** sits inside the 869.4 - 869.65MHz sub-band, which is the slice that allows 500mW ERP at a 10% duty cycle in the UK. It is the most generous corner of the 868 band.

**62.5kHz bandwidth** is narrow enough to slide between the smart meters, weather stations and doorbells that clutter the band, and it gains about 6dB of sensitivity over a 250kHz channel.

**SF8** keeps airtime down. Combined with the narrow bandwidth it lands at a similar data rate to the old SF11/BW250 setup, but it is far more robust in the presence of interference and it uses a quarter of the spectrum.

**CR8** adds generous error correction, which pays off on the marginal long links that repeaters are there to serve.

---

## Setting them on the repeater

If you selected the EU/UK (Narrow) preset in the flasher, this is already done. To check, connect the console and run:

```text
get freq
get radio
```

To set them explicitly:

```text
set freq 869.618
set radio 869.618,62.5,8,8
```

The `set radio` command takes `frequency,bandwidth,spreading factor,coding rate` in one go. Then confirm:

```text
get radio
```

Reboot the board to be sure the settings have stuck:

```text
reboot
```

---

## Setting them on the client

The companion node is configured from the app rather than the console. Once you have paired the phone in lesson 9, you will find the radio settings in the app's settings screen - and there is a preset menu there too, usually behind a three-dot icon next to the frequency field. Pick **EU/UK (Narrow)** and apply.

We will walk through this in lesson 9. For now, just know that the same four numbers have to appear on both devices.

---

## Transmit power

While we are in the radio settings, check the transmit power:

```text
get tx
```

The Heltec V3 tops out at around **21** (dBm) - Heltec quote 21 &plusmn;1dBm for this board. Stock firmware usually sets this sensibly. To change it:

```text
set tx 20
```

**Should you always run maximum power?** For a fixed repeater, yes - it is well within the legal ERP limit with a normal antenna. For a battery powered node, dropping to 17dBm roughly halves the transmit current for about 40% of the range, which can be a good trade.

---

## Try it Yourself

1. Run `get radio` on your repeater and write the four numbers down. You will want them when you configure the client.
2. Deliberately set your client to a different spreading factor and try to message the repeater. Confirm that nothing at all happens - it is a useful failure to have seen once.
3. Challenge: work out the theoretical data rate for SF8/BW62.5 using an online LoRa calculator, then compare it to SF11/BW250.

---

## Common Issues

**Problem**: `set freq 869618` does not work.

**Solution**: The value is in **MHz**, so it is `869.618`, not the raw kHz figure.

**Why**: MeshCore expects megahertz with a decimal point.

**Problem**: My nodes still cannot hear each other with matching settings.

**Solution**: Reboot both, and check that both are actually running the frequency you think - `get radio` on the repeater, and read the app's settings screen on the client rather than assuming.

**Why**: Settings changes sometimes need a reboot to take effect, and the flasher preset only applies at flash time.

**Problem**: I can hear my own nodes, but not any of the local community repeaters.

**Solution**: Ask your local group which preset they are on - some regions still run legacy settings.

**Why**: Regional presets have changed over time, and older repeaters do not always get updated.

---
