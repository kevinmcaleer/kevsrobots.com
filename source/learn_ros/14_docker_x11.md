---
layout: lesson
title: Getting Rviz2 and RQT to work in docker
type: page
description: Let's learn how to redirect our X11 Display
cover: /learn/learn_ros/assets/x11cover.jpg
---

![Cover Photo]({{page.cover}}){:class="cover"}

---

To get the ROS visualisation tool `rviz2` to work in our docker container we need to redirect any graphics calls to outside our container into the hosts's X11 environment.

## What is X11

`X11` is a software package developed by the Massachusetts Institute of Technology (MIT) in the 1980s that enables graphical user interfaces (GUIs) to be created and used on Unix-based systems.
It works by using a network protocol called the X Window System Protocol to communicate between the server and the client.
The **X server** runs on the computer with the screen, keyboard and mouse, and draws everything. The **clients** are the applications, such as RViz2, which send drawing commands to the server.
Because it's a network protocol, an application running in one place (like our Docker container) can draw windows on a screen somewhere else (like the Raspberry Pi desktop).

X11 is the traditional display system on Linux, and X servers are available for macOS and Windows too. (Newer Raspberry Pi OS releases use Wayland, but still run X11 applications through XWayland.)

![X11 example desktop](assets/x11.jpg){:class="img-fluid w-100"}

---

## Redirecting the X11 Display to the host

To redirect any X11 display commands from within the docker container running ROS2, to outside the container (our Raspberry Pi 4 OS), we need to do two things:

1. Use `xhost` to enable other processes to access the X11 server (this isn't exactly secure, but for now it works)
1. `export` the `DISPLAY` environment variable - this will complete the plumbing so we can run [RViz2](/resources/glossary#rviz2).

* **xhost** - from the raspberry pi terminal type:

```bash
xhost local:root
```

* **Add display to docker-compose.yml** - Edit the docker-compose.yml file to include the lines:

```yaml
environment:
  - DISPLAY
```

*We also add `network_mode: host` so the container shares the Pi's network, which ROS 2 needs to find nodes running outside the container.*

The full docker-compose.yml file should now look like this:

```yaml
version: "3.9"
services:
  ros2:
    build: .
    network_mode: host
    volumes:
      - /home/kev/ros:/home/ros
      - /home/kev/cubie-1:/ros2
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    tty: true
    environment:
      - DISPLAY
```

---

## Bring up the new docker-container

Now that we have updated our docker-compose file we need to bring the new container online; docker-compose has a simple command for this called `up`:

* **docker compose up -d** - from the Raspberry Pi's terminal, in the folder where the updated docker-compose.yml is, type:

```bash
docker compose up -d
```

**Note** the `-d` parameter means run the container detached (in the background). If you don't include it, all the container output will display on this terminal, and the container will stop when the terminal session ends.

---
