---
title: A Webhook for the Pico
description: Add a Webhook node so devices on your network can send data to n8n, and protect it with a key
layout: lesson
type: page
cover: /learn/n8n_homelab/assets/banners/06_a_webhook_for_the_pico.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Stage 4, part one: the butler gets ears. A **webhook** is a web address that n8n listens on. When a device sends it an HTTP request, the workflow starts. It is the easiest way to get data from a Pico W, a script or another service into n8n.

---

## Add the Webhook node

Create `Butler - sensor inbox` and add a **Webhook** node as the trigger. The settings you need:

| Setting | What to choose |
|---|---|
| HTTP Method | `POST`, because the Pico is sending data |
| Path | A name you choose, such as `pico-reading`. Left alone, n8n generates a random one |
| Authentication | Header auth (we add it below) |
| Respond | Immediately |
{:class="table table-single"}

The node shows two addresses, selected with **Test URL** and **Production URL**.

## Test URL and Production URL

| | Test URL | Production URL |
|---|---|---|
| When it works | After you click **Listen for Test Event** or **Execute workflow**. Stays listening for 120 seconds | Once the workflow is published |
| Where the data shows | In the editor, on the node | Only in the **Executions** tab |
{:class="table table-single"}

Use the Test URL while you build, then switch the Pico to the Production URL. Mixing them up is the most common webhook problem, and lesson 12 lists it.

## Try it from the Pi

Click **Listen for Test Event**, then from another terminal send a request, pasting the Test URL from the node:

```bash
curl -X POST "PASTE_THE_TEST_URL_HERE" \
  -H "Content-Type: application/json" \
  -d '{"device": "pico-lounge", "temperature_c": 21.5}'
```

The node lights up with the data. An incoming request is an item with four parts: `body`, `headers`, `params` and `query`. Our JSON arrives inside `body`, so the temperature is at `body.temperature_c`. Remember that: lesson 8 reads from there.

## Protect it with a key

A webhook is a door into your butler. Anyone who can reach the address can knock. On a home network the risk is small, but build the habit now: never expose a webhook to the internet without authentication and HTTPS.

1. In the Webhook node, set **Authentication** to **Header auth**.
2. Create a new credential. It asks for a header **name** (for example `X-Pico-Key`) and a **value**, which is your shared secret. Generate a long random one, such as `openssl rand -hex 24`.
3. Save, then repeat the curl request with `-H "X-Pico-Key: YOUR_SHARED_SECRET"`. A request without the right header is refused.

Keep the secret out of screenshots and out of Git. The Pico will need a copy, and lesson 7 shows where it goes.

## Try it Yourself

1. Send the curl request without the header. What does curl print?
2. Add a second path, `pico-test`, on a duplicate Webhook node, and compare their Production URLs.
3. Challenge: look at the Webhook node's options. Which one would limit callers to the Pico's IP address?

## Common Issues

**Problem**: The Production URL does nothing.
**Solution**: Publish the workflow first.
**Why**: Production webhooks are only registered for published workflows.

**Problem**: The Test URL worked once and now fails.
**Solution**: Click **Listen for Test Event** again.
**Why**: The test listener only stays open for 120 seconds, for one request.
