---
layout: lesson
title: Configuring the Room Server
author: Kevin McAleer
type: page
cover: /learn/meshcore_rooms_sensors/assets/cover.png
date: 2026-08-05
previous: 04_flashing_the_room_server.html
next: 06_joining_a_room.html
description: Name, location, admin and guest passwords, read-only access and advert
  intervals
percent: 30
duration: 6
navigation:
- name: MeshCore Room Servers and Sensors
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Beyond Repeaters and Clients
      link: 01_beyond_repeaters.html
    - name: The Hardware
      link: 02_hardware.html
  - section: The Room Server
    content:
    - name: What is a Room Server?
      link: 03_what_is_a_room_server.html
    - name: Flashing the Room Server
      link: 04_flashing_the_room_server.html
    - name: Configuring the Room Server
      link: 05_room_server_config.html
    - name: Joining a Room
      link: 06_joining_a_room.html
    - name: Permissions and Moderation
      link: 07_permissions.html
    - name: Running a Room Server
      link: 08_room_operations.html
  - section: The Sensor Node
    content:
    - name: What is a Sensor Node?
      link: 09_what_is_a_sensor_node.html
    - name: Flashing the Sensor Node
      link: 10_flashing_the_sensor.html
    - name: Building an Environment Sensor
      link: 11_environment_sensor.html
    - name: Battery and Solar Telemetry
      link: 12_battery_telemetry.html
    - name: Contact and Level Sensing
      link: 13_contact_and_level.html
    - name: Reading Telemetry
      link: 14_reading_telemetry.html
  - section: Going Further
    content:
    - name: Troubleshooting
      link: 15_troubleshooting.html
    - name: Where to Go Next
      link: 16_next_steps.html
---


The board is flashed and on the right frequency. Now we give it an identity and lock it down, because the passwords it shipped with are printed in the public documentation.

---

## Step 1 - name it

```text
set name Liverpool-Makers-Room
```

**Why this matters:** the name is what your group looks for in their contacts list. Make it obvious what the room is *for*. `Kev-Room-01` tells nobody anything; `Village-Noticeboard` tells them everything.

Names can run to roughly 24 to 32 bytes depending on the field, so keep it reasonably short - and remember it goes out over the air on every advert.

---

## Step 2 - change the admin password

**Do this before anything else touches the air.**

```text
password <your-new-admin-password>
```

The factory default is `password`. It is documented publicly, which means every MeshCore user in the world already knows it. Until you change it, anyone in radio range can log in as admin and reconfigure your node.

Write the new one down somewhere you will still have it in a year.

---

## Step 3 - set the guest password

```text
set guest.password <your-group-password>
```

The factory default is `hello`, and it is equally public.

**How to think about this password:** it is the one you actually hand out. It is what a member of your group types once to join the room. Pick something memorable and speakable - people will be reading it to each other. `hedgehog-fields` is a better group password than `Xk9$2p`.

Verify it took:

```text
get guest.password
```

---

## Step 4 - decide about read-only access

```text
set allow.read.only on
```

With this on, anyone can read the room's posts without a password. They still need the guest password to post.

**When to turn it on:** a public noticeboard - emergency coordination, event info, a village mesh where you want passers-by to be able to read.

**When to leave it off:** anything that is meant to be a private group conversation.

**A note on what "private" means here:** posts to a room are carried across the mesh as ordinary MeshCore traffic. Read-only off keeps strangers out of the room, which is real access control. Do not treat it as a guarantee that nobody anywhere could ever observe traffic. For genuinely sensitive things, direct messages between two nodes are the right tool.

---

## Step 5 - set the location

```text
set lat 53.4084
set lon -2.9916
```

Same guidance as the repeater in part one: use your own coordinates, and round to three decimal places (roughly 100 metres) if you would rather not publish your exact address.

---

## Step 6 - set the owner info

```text
set owner.info Kev - kevsrobots.com
```

**Why this matters:** if your room server ever misbehaves, floods, or needs turning off, this is how somebody finds you. It is a small courtesy that makes you a good neighbour on a shared mesh.

Pipe characters in this field convert to newlines, so you can fit a couple of lines in if you want.

---

## Step 7 - advert intervals

```text
set advert.interval 60
set flood.advert.interval 12
```

`advert.interval` is in **minutes** and accepts 60 to 240. `flood.advert.interval` is in **hours** and accepts 3 to 168.

The same etiquette applies as on a repeater: flood adverts are rebroadcast by every repeater in the region, so a room server that floods every three hours is being greedy with everyone's airtime. Twelve hours is plenty for a fixed node.

Then announce yourself:

```text
advert
```

Note that plain `advert` sends a **flood** advert - it goes across the whole mesh. For a local-only announcement to nodes in direct range, use:

```text
advert.zerohop
```

---

## Step 8 - check your work

```text
get name
get role
get radio
get guest.password
get allow.read.only
get lat
get lon
stats-core
```

Take a screenshot or copy the output somewhere. When something goes odd in three months, a record of the known-good configuration is worth a great deal.

---

## The room server settings, summarised

| Command | Sets | Notes |
|---|---|---|
| `set name <name>` | Node name | What your group looks for |
| `password <pwd>` | Admin password | Default `password` - change it |
| `set guest.password <pwd>` | Guest password | Default `hello` - change it |
| `set allow.read.only on/off` | Passwordless reading | Default off |
| `set lat` / `set lon` | Location | For the map and for routing sense |
| `set owner.info <text>` | Who to contact | Pipe becomes newline |
| `set advert.interval <mins>` | Local advert | 60 to 240 |
| `set flood.advert.interval <hrs>` | Mesh-wide advert | 3 to 168 |
| `set repeat on/off` | Also act as a repeater | Default on - see lesson 8 |
{:class="table table-single"}

---

## Try it Yourself

1. Set the guest password to something, then run `get guest.password` to confirm. Now try logging in from the app with the old one.
2. Turn `allow.read.only` on and off and note how the room appears differently to a client that has never authenticated.
3. Challenge: write out the exact joining instructions you will send your group - node name, guest password, and the three taps in the app. Test them on somebody who has not seen the room before.

---

## Common Issues

**Problem**: I changed the admin password and now I cannot get in.

**Solution**: Connect over USB serial and run `password <new-password>` again.

**Why**: The serial console is always trusted - the password only ever gates access over the air. Physical access is always the way back in.

**Problem**: `set guest.password` returns an error.

**Solution**: Check you are on room server firmware, not repeater firmware.

**Why**: A guest password only exists on roles that have guests. A repeater has no concept of one.

**Problem**: My group can read the room but cannot post.

**Solution**: They have joined as read-only guests. Give them the guest password.

**Why**: With `allow.read.only on`, joining without a password succeeds and grants read access only. It looks like a working join right up until they try to say something.

---
