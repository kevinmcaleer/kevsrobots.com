---
title: What is a Room Server?
description: How store-and-forward messaging works, and what a room server does that a channel cannot
type: page
layout: lesson
cover: /learn/meshcore_rooms_sensors/assets/banners/03_what_is_a_room_server.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain how a room server stores and forwards posts, and what its limits are.

A room server is the closest thing MeshCore has to an inbox. It is a node that keeps recent posts and hands them over to people who were not around when they were made.

---

## The core idea: store and forward

![How store and forward works](/learn/meshcore_rooms_sensors/assets/room_flow.svg){:class="img-fluid w-100"}

Everything else on a MeshCore network is **live**. A direct message goes from you to a specific node right now. A channel message floods across the mesh right now. If the recipient is not listening at that moment, the packet dissipates and that is the end of it.

A room server breaks that pattern. When someone posts to the room:

1. The post travels to the room server like any other message
2. The room server **writes it into a buffer** and keeps it
3. Anyone currently logged in and in range receives it immediately
4. Anyone out of range gets it **the next time they log in**

Step 4 is the whole point. That is the difference between a broadcast and a noticeboard.

---

## What is actually stored

The room server keeps a **fixed-size cyclic buffer of recent posts**. The official FAQ says a client logging in receives the previous 32 unseen messages, so plan around that figure. When the buffer is full, each new post overwrites the oldest one.

**Why this matters:** a room server is not an archive. It is a rolling window of recent conversation. On a quiet village mesh, a few dozen posts might be a fortnight of history. On a busy group during an event, it might be an afternoon. Nobody gets to scroll back to last month.

Alongside the buffer, the server tracks a **sync point per user**. When you log in, it works out which posts are newer than the last time you synced and sends you those - and it skips your own posts, so you do not get an echo of things you wrote yourself.

---

## The thing that will catch you out

**Posts live in RAM.** Pull the power and they are gone.

This is the single most important operational fact about running a room server, and it drives most of lesson 8. A room server on a wall socket during a power cut loses every post it was holding. A room server on a battery-backed supply does not.

It is worth being upfront with your group about this. "The room keeps the last few dozen posts, and it forgets everything if it reboots" sets expectations that a silent surprise reset never will.

---

## Room server versus channel

You could reasonably ask why not just use a channel. Both are group conversation. Here is the difference:

| | Channel | Room Server |
|---|---|---|
| Needs dedicated hardware | No | Yes - a board |
| Delivers to people out of range | No | Yes |
| Keeps history | No | Yes, recent posts |
| Access control | Anyone with the key | Password tiers, ACL |
| Airtime cost | Floods the whole mesh | Point to point to the server |
| Works with nobody else online | Yes, nobody hears it | Yes, and it is kept |
{:class="table table-single"}

**The airtime point deserves attention.** Channel messages always flood, because a channel has no single destination. A post to a room server is addressed to one specific node, so it can use a stored path and only the repeaters on that path forward it. On a busy mesh, a room is meaningfully cheaper than a channel carrying the same conversation.

---

## Room server versus direct messages

Direct messages are already point to point and already encrypted. Why not just DM people?

Because a DM goes to **one** node, and it only arrives if that node is reachable. A room is for the group, and the group is never all reachable at once. If you want six people to see something and you have no idea which of them are in range, that is exactly the job a room does.

---

## Who can do what

A room server has three tiers of access, and you will set them up in lesson 5:

| Tier | Gets in with | Can do |
|---|---|---|
| **Read-only guest** | No password, if `allow.read.only` is on | Read posts |
| **Guest** | The guest password | Read and post |
| **Admin** | The admin password | Everything, plus reconfigure the node |
{:class="table table-single"}

The official FAQ says the default admin password is **`password`** and the default room guest password is **`hello`**. Both are published, which means both are effectively public. (The CLI reference lists the guest password default as blank, so defaults can differ between builds.) Changing them is lesson 5's first job.

---

## Can it repeat as well?

On some firmware versions, yes: the official FAQ says a room server can also repeat with `set repeat on`, but that this is not recommended or encouraged, and that a repeater and a room server work best on separate devices. The room server source sets forwarding off by default, and recent builds may reject `set repeat` entirely.

If you want a room and a repeater at the same site, use two boards. It is cheaper than debugging one board doing two jobs.

We cover the trade-off properly in lesson 8.

---

> **Teachers**: A room server is a shared noticeboard, so treat it like any group chat. Decide who gets the guest password, do not post personal details such as pupils' names with locations, and remember that posts are stored in the board's memory and vanish on reboot, so the room is not a record you can rely on.

---

## Try it Yourself

1. Count how many messages your group sends in a typical week. Would a 32-post buffer hold a useful amount of history for you?
2. Think about where your room server would live. Is that socket on the same circuit as anything that trips?
3. Challenge: write the one-paragraph explanation you would send your group to explain what the room is and how to join. Being able to explain it simply is the test of whether you understand it.

---

## Common Issues

- **Problem**: I posted to the room and my friend never saw it, even after coming back into range.
- **Solution**: Check they are actually logged into the room, not just in range of it.
- **Why**: Sync happens on login. A node that is nearby but has never authenticated has no sync point, so there is nothing for the server to catch it up on.

- **Problem**: Older posts have vanished from the room.
- **Solution**: Nothing is broken - that is the cyclic buffer doing its job.
- **Why**: The server holds a fixed number of recent posts and overwrites the oldest as new ones arrive. It is a rolling window, not an archive.

---

## Check your understanding

{% quiz %}
? What happens to the posts in a room server when it loses power?
- [x] They are lost, because they live in RAM
- [ ] They are saved to your phone
- [ ] They are kept for a year
> The room server source stores posts in a memory array and only saves its access list.

? Why is a post to a room usually cheaper in airtime than a channel message?
- [x] It is addressed to one node, so it need not flood the mesh
- [ ] It is shorter
- [ ] It skips encryption
> Channel messages flood because they have no single destination.

? Which tier lets someone post to the room?
- [x] Guest password (read-write)
- [ ] No password at all
- [ ] Nothing, posting is always allowed
> The guest password grants read-write; no password grants read-only if enabled.

? Why change the default passwords straight away?
- [x] They are published in the documentation
- [ ] They stop the radio working
- [ ] They expire after a week
> Anyone who has read the FAQ knows them.

{% endquiz %}
