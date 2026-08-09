---
title: Running a Room Server
description: Power, placement, the repeat question, and keeping a room alive long term
type: page
layout: lesson
---

A room server is infrastructure. Once your group depends on it, the interesting problems stop being about firmware and start being about electricity.

---

## The power problem

Say it once more, because it governs everything else:

> **Posts live in RAM. Lose power, lose the posts.**

A repeater that reboots comes back in seconds and nobody notices. A room server that reboots comes back empty, and the first person to log in sees an empty noticeboard with no explanation.

### Ranking the power options

**Best: mains USB with a battery backup.** A 5V mains charger for normal running, plus a LiPo on the SH1.25 connector as a fallback. The board runs from USB, charges the cell, and rides through a cut without dropping.

**Good: mains USB alone.** Simple and reliable, right up until the power goes off.

**Poor: USB power bank.** Many power banks shut down when the load drops below a threshold, and an idle room server draws very little. If you use one, get a model with a low-current or always-on mode and test it for a week before trusting it.

**Bad: battery only, no charging.** It will die at an unpredictable moment and take the room's contents with it.

### Sizing a backup cell

A Heltec V3 is not a low-power board. Heltec quote around **90mA** with the radio receiving, and **200 to 230mA** while transmitting depending on power level. A 2000mAh LiPo therefore gives you roughly **20 hours** of standalone running - enough to ride out a typical power cut, but not a long outage.

If you need days rather than hours, the answer is different hardware: a Heltec T114 on nRF52840 draws a fraction of this.

---

## Where to put it

A room server has an easier life than a repeater, because it does not need to reach the horizon - it only needs to reach the people who use it.

**Good enough:** a shelf indoors, upstairs, antenna vertical, near enough to a socket. If your group is in one village, that will very likely do.

**Better:** the same place your repeater lives. Everyone who can reach the repeater can reach the room.

**Do not:** put it in a metal cabinet, behind a fridge, or wedged next to a WiFi router. And keep the antenna vertical - horizontal costs you range for free.

**A useful trick:** if you already have a repeater at a good site, put the room server next to it on the same power supply. The two boards together cost £30, use one socket, and give you a room that reaches everyone the repeater reaches.

---

## The repeat question

A room server *can* also forward packets. The setting is:

```text
get repeat
set repeat off
```

**Room server firmware ships with forwarding off**, so a fresh room server is not repeating unless you turned it on. Check anyway - it takes a second and it is worth knowing.

**The case for leaving repeat on:** one board, one socket, two jobs. Tempting when you have one good site.

**The case against - and it is the stronger one:**

- The official guidance is to run a room server with repeat off and use a separate board for repeating, because a room server with repeat on lacks the full set of repeater and remote administration features
- Two jobs on one board means one failure takes out both
- A room server's RAM is holding your posts. Anything that increases the chance of a reset is working against you
- Repeating adds transmit load, which adds heat and current draw

**The recommendation:** if the room matters, `set repeat off` and spend another £15 on a dedicated repeater. If you are experimenting, leave it on and see how you get on.

---

## Monitoring it

Log in over the mesh from your phone occasionally and run:

```text
stats-core
stats-radio
```

**What to look for:**

**Uptime** is the important one. If uptime keeps resetting, you have a power problem - and every reset emptied the room. Track it over a week.

**Queue depth** in `stats-core` should sit near zero. A persistently non-zero queue means the node is receiving faster than it can transmit.

**Noise floor** in `stats-radio` tells you whether new interference has appeared nearby. A rising noise floor quietly eats your range.

---

## A maintenance rhythm

Room servers are low maintenance, but not zero:

**Monthly:** check uptime. Anything under a month suggests unplanned resets.

**Quarterly:** check for firmware updates, and confirm your group can still get in.

**Annually:** rotate the guest password, and re-read your own `owner.info` to check it is still accurate.

**Before any event:** reboot it deliberately at a quiet moment, so the buffer is clear and you know it is starting healthy.

---

## Telling your group the truth

The single best operational habit is setting expectations honestly. Something like:

> The room keeps roughly the last 30 posts. If you have been away a long time you will not get all of it. If the power goes off, everything in the room is lost. Anything important, say twice.

People are entirely fine with limitations they know about. What annoys them is a system they thought was reliable quietly not being.

---

## Try it Yourself

1. Run `stats-core` and note the uptime. Check again in a week - has it kept counting?
2. Deliberately pull the power for ten seconds, then log back in. Confirm the buffer is empty. Now you know exactly what a power cut looks like.
3. Challenge: set up mains power with a LiPo backup, then pull the mains and confirm the room keeps running and keeps its posts.

---

## Common Issues

**Problem**: The room empties itself every few days.

**Solution**: Check `stats-core` uptime, then suspect the power supply - especially if it is a USB power bank.

**Why**: Many power banks auto-shutdown below a current threshold, and an idle room server draws very little. The board reboots, RAM clears, posts are gone.

**Problem**: My room server is also repeating and I want to stop it.

**Solution**: `set repeat off`, then confirm with `get repeat`.

**Why**: Forwarding is off by default on room server firmware, so if it is repeating, somebody enabled it - possibly you, experimenting.

**Problem**: People near the repeater can use the room but people at the edge cannot.

**Solution**: Move the room server to the repeater's site, or add a repeater between the room and the people who cannot reach it.

**Why**: Room traffic is point to point, so a client needs a working path to the server specifically. Being able to hear a repeater is not the same as being able to reach the room through it.

---
