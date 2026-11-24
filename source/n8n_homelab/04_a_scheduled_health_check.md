---
title: A Scheduled Health Check
description: Use the Schedule Trigger and HTTP Request nodes to check a homelab service automatically
layout: lesson
type: page
cover: /learn/n8n_homelab/assets/banners/04_a_scheduled_health_check.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Stage 2: the butler checks on things without being asked. A health check is the classic homelab automation, because a quiet failure at 3 a.m. is worse than a noisy one.

---

## The Schedule Trigger

Build a new workflow called `Butler - health check` and start it with a **Schedule Trigger** node. You add one or more **Trigger Rules**, each with a **Trigger Interval**:

| Interval | Fields you set |
|---|---|
| Minutes | Minutes Between Triggers |
| Hours | Hours Between Triggers, Trigger at Minute |
| Days | Days Between Triggers, Trigger at Hour, Trigger at Minute |
| Custom (Cron) | Expression |
{:class="table table-single"}

(There are also Seconds, Weeks and Months.) For now choose **Minutes** and set **Minutes Between Triggers** to `5`.

**Timezone:** the node uses the workflow's timezone if one is set, otherwise the instance timezone, which is the `GENERIC_TIMEZONE` you set in lesson 2.

**Publish needed:** a schedule only runs on its own after you **Publish** the workflow. While you build, use **Execute workflow** to test.

## The HTTP Request node

Add an **HTTP Request** node after the trigger. It has the same settings you used before, plus a few that matter for health checks:

- **Method** and **URL**, such as `GET` and the address of your service
- **Options > Timeout**, in milliseconds, so a hung service doesn't hang the butler. Try `5000`
- **Options > Response > Include Response Headers and Status**, which returns the status code along with the body
- **Options > Response > Never Error**, which stops a non-2xx reply from failing the node

By default the node counts only 2xx replies as success, and treats anything else as an error. For a health check we want to **see** a 500 and act on it, so turn on both **Never Error** and **Include Response Headers and Status**.

## Pick a target

Start with n8n itself so you can test with no other setup:

```
http://localhost:5678/healthz
```

Then try a real service on your network, using the Pi's LAN address:

```
http://YOUR_PI_IP:8080/
```

**Why not localhost for other services?** From inside the n8n container, `localhost` is the container, so it can't see ports published on the Pi. Use the Pi's LAN IP (`hostname -I`), or put the services on a shared Docker network and use their container names. The [Docker networking lesson](/learn/docker/06_networking.html) explains how.

## Look at the output

Click **Execute workflow** and open the HTTP Request output. With **Include Response Headers and Status** on, the output includes the status code alongside the body. Find the status field name on your own screen, because we are about to use it in lesson 5.

## Try it Yourself

1. Add a second service URL by duplicating the HTTP Request node (copy and paste it on the canvas) and run both.
2. Set the **Timeout** to `1` and point it at a slow site. What happens?
3. Challenge: change the trigger to a cron-style rule that runs only at 07:00 on weekdays. The docs list the Custom (Cron) interval and its Expression field.

## Common Issues

**Problem**: The schedule never runs.
**Solution**: **Publish** the workflow, and check the timezone.
**Why**: Production schedules only run for a published workflow, and the default timezone is America/New_York.

**Problem**: A stopped service makes the node fail instead of returning a status.
**Solution**: Expect this. **Never Error** covers bad status codes, but a service that can't be reached at all is a failed run, and lesson 5 shows how to be told about those too.
**Why**: There is no reply to read when nothing is listening.
