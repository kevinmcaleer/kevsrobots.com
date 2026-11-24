---
layout: lesson
title: Building Your First Docker Image
author: Kevin McAleer
type: page
cover: /learn/docker/assets/banners/04_building.jpg
date: 2024-01-21
previous: 03_basics.html
next: 05_managing.html
description: Write a small Flask robot status page, package it with a Dockerfile,
  build an image and run it as a container.
percent: 45
duration: 6
quiz_total: 30
date_updated: 2026-11-24
navigation:
- name: Docker
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
    - name: Introduction to Docker
      link: 01_what-is-docker.html
    - name: Installing Docker
      link: 02_installing-docker.html
  - section: Images and Containers
    content:
    - name: Docker Basics
      link: 03_basics.html
    - name: Building Your First Docker Image
      link: 04_building.html
    - name: Managing Docker Containers
      link: 05_managing.html
    - name: Docker Networking Basics
      link: 06_networking.html
    - name: Docker Volumes and Persistent Data
      link: 07_volumes.html
  - section: Docker Compose
    content:
    - name: Creating a Docker Compose File
      link: 08_docker-compose.html
    - name: 'Project: The Robot Status Stack'
      link: 09_dockerise.html
    - name: Conclusion and Next Steps
      link: 10_conclusion.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write a Dockerfile, build an image from it, and run your own app as a container.

Time to build our project. We'll make a **robot status page**: a small Python web app that shows the robot's name, its CPU temperature and a visit counter. Everything here is typed on the Pi or Linux PC.

## Step 1: Make a project folder

```bash
mkdir ~/robot-status
cd ~/robot-status
```

## Step 2: Write the app

Create `app.py` with `nano app.py`, paste in the whole program, then save with `Ctrl+O`, `Enter`, `Ctrl+X`:

```python
import os
import platform
import socket
import time

from flask import Flask, jsonify

app = Flask(__name__)

ROBOT_NAME = os.environ.get("ROBOT_NAME", "Sparky")
REDIS_HOST = os.environ.get("REDIS_HOST")
STARTED = time.time()

# Visit counter: kept in memory by default, kept in Redis when REDIS_HOST is set
store = None
memory_count = 0
if REDIS_HOST:
    import redis
    store = redis.Redis(host=REDIS_HOST, port=6379, decode_responses=True)


def count_visit():
    global memory_count
    if store is not None:
        return int(store.incr("visits"))
    memory_count += 1
    return memory_count


def cpu_temp():
    """CPU temperature in Celsius, or None if this machine does not report it."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read().strip()) / 1000, 1)
    except (OSError, ValueError):
        return None


def status():
    return {
        "robot": ROBOT_NAME,
        "container": socket.gethostname(),
        "machine": platform.machine(),
        "uptime_seconds": int(time.time() - STARTED),
        "cpu_temp_c": cpu_temp(),
        "storage": "redis" if store is not None else "memory",
        "visits": count_visit(),
    }


@app.route("/")
def home():
    s = status()
    temp = "not available" if s["cpu_temp_c"] is None else f'{s["cpu_temp_c"]} C'
    return (
        f"<h1>{s['robot']} is online</h1>"
        f"<p>Container: {s['container']} ({s['machine']})</p>"
        f"<p>CPU temperature: {temp}</p>"
        f"<p>Visits: {s['visits']} (stored in {s['storage']})</p>"
    )


@app.route("/status")
def status_json():
    return jsonify(status())


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
```

Two details are worth a note. The app reads its settings from **environment variables** (`ROBOT_NAME`, `REDIS_HOST`), which is how we'll configure containers without changing the code. And `host="0.0.0.0"` tells Flask to listen on every network interface; inside a container, the default of `127.0.0.1` would be unreachable from outside.

For now there's no Redis, so the visit counter lives in memory. We'll upgrade that in lessons 6 and 7.

## Step 3: List the dependencies

Create `requirements.txt` containing two lines:

```text
flask
redis
```

We haven't pinned versions to keep things simple. For a project you rely on, pin them (for example `flask==3.0.3`) so a rebuild next year gives the same result.

## Step 4: Write the Dockerfile

A **Dockerfile** is the recipe for an image. Create a file called exactly `Dockerfile` (capital D, no extension):

```dockerfile
# Start from an official Python image (slim = smaller download)
FROM python:3.12-slim

# Show print output and errors straight away in docker logs
ENV PYTHONUNBUFFERED=1

# All following commands run inside /app in the image
WORKDIR /app

# Copy the dependency list first, then install
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Now copy the app itself
COPY app.py .

# Document the port the app listens on
EXPOSE 5000

# The command that runs when a container starts
CMD ["python", "app.py"]
```

Each instruction does one job:

| Instruction | What it does |
| --- | --- |
| `FROM` | Picks the base image to build on |
| `ENV` | Sets an environment variable |
| `WORKDIR` | Sets the working folder, creating it if needed |
| `COPY` | Copies files from your folder into the image |
| `RUN` | Runs a command while **building** the image |
| `EXPOSE` | Documents a port; it does not publish it |
| `CMD` | The command to run when a container **starts** |
{:class="table table-single"}

**Why copy `requirements.txt` first?** Docker builds an image in **layers** and caches each one. If only `app.py` changes, Docker reuses the slow `pip install` layer and your rebuild takes a second. Put the things that change least at the top.

## Step 5: Build and run

```bash
docker build -t robot-status .
```

`-t robot-status` names (tags) the image and the final `.` means "use this folder as the build context". The first build downloads Python and installs Flask, so it takes a minute or two on a Pi. Then run it:

```bash
docker run -d --name robot-status -p 5000:5000 robot-status
```

Open `http://localhost:5000` (or `http://<your-pi-address>:5000`). Refresh a few times and watch the visit counter climb. You can also try `curl http://localhost:5000/status` to see the JSON version. Leave the container running for the next lesson.

## Common Issues

- **Problem**: `failed to read dockerfile: open Dockerfile: no such file or directory`.
- **Solution**: Run `ls` and check you are in `~/robot-status` and the file is named exactly `Dockerfile`.
- **Why**: `docker build .` looks for a file called `Dockerfile` in the folder you give it.

- **Problem**: The container starts, but the browser says the connection was reset or refused.
- **Solution**: Check `app.run(host="0.0.0.0", port=5000)` is in `app.py`, then rebuild with `docker build -t robot-status .` and remove and re-run the container.
- **Why**: Flask listening only on `127.0.0.1` can't be reached from outside its container.

- **Problem**: You edited `app.py` but the running page hasn't changed.
- **Solution**: Rebuild the image, then `docker rm -f robot-status` and run it again.
- **Why**: A container is made from a snapshot of the image. Changing your files doesn't change images that already exist.

## Try it Yourself

1. Change the greeting in `app.py` (for example "is online and ready"), rebuild, and replace the container. Notice how fast the second build is.
2. Run the container with a different name for your robot: `docker run -d --name robot2 -p 5001:5000 -e ROBOT_NAME=Wall-E robot-status`. Open port 5001.
3. Challenge: run `docker history robot-status` and find the layer made by `pip install`. Roughly how big is it compared with the rest?

## Check your understanding

{% quiz %}
? What is the difference between `RUN` and `CMD` in a Dockerfile?
- [x] `RUN` executes while the image is built, `CMD` when a container starts
- [ ] They are identical
- [ ] `RUN` starts the container, `CMD` stops it
> `RUN pip install` happens once at build time. `CMD ["python", "app.py"]` happens each time a container starts.

? Why do we copy `requirements.txt` and install before copying `app.py`?
- [ ] Docker insists on alphabetical order
- [x] So the slow install layer is cached when only the app code changes
- [ ] Python cannot import app.py before requirements.txt
> Docker reuses unchanged layers. Putting rarely changing steps first makes rebuilds fast.

? Why does the app use `host="0.0.0.0"`?
- [ ] To make Flask faster
- [x] So it accepts connections from outside the container
- [ ] To hide it from the network
> `127.0.0.1` means only the container itself. `0.0.0.0` means every network interface.
{% endquiz %}
