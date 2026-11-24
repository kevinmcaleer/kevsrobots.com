---
layout: lesson
title: A chat page in your browser
author: Kevin McAleer
type: page
cover: /learn/ai_hat_2/assets/banners/09_a_chat_page_in_your_browser.jpg
date: 2026-11-24
previous: 08_a_language_model_on_the_hat.html
next: 10_asking_the_robot_what_it_sees.html
description: Add the Open WebUI chat page in Docker so you can talk to the language
  model from any browser
percent: 70
duration: 3
quiz_total: 6
date_updated: 2026-11-24
navigation:
- name: Raspberry Pi AI HAT+ 2 - Vision and Generative AI
- content:
  - section: Meet the HAT
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is the AI HAT+ 2?
      link: 01_what_is_the_ai_hat_2.html
    - name: Fitting the HAT
      link: 02_fitting_the_hat.html
    - name: Software setup
      link: 03_software_setup.html
  - section: Eyes
    content:
    - name: Object detection with rpicam-apps
      link: 04_object_detection_with_rpicam_apps.html
    - name: Pose and segmentation
      link: 05_pose_and_segmentation.html
    - name: Reading detections from Python
      link: 06_reading_detections_in_python.html
    - name: Detections to robot behaviour
      link: 07_detections_to_robot_behaviour.html
  - section: A brain
    content:
    - name: A language model on the HAT
      link: 08_a_language_model_on_the_hat.html
    - name: A chat page in your browser
      link: 09_a_chat_page_in_your_browser.html
    - name: Asking the robot what it sees
      link: 10_asking_the_robot_what_it_sees.html
  - section: Choosing and fixing
    content:
    - name: Performance and choosing hardware
      link: 11_performance_and_choosing_hardware.html
    - name: Troubleshooting
      link: 12_troubleshooting.html
    - name: Summary
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Typing `curl` commands gets old fast. Open WebUI gives the language model a friendly chat page in your browser. It is optional, but it is a nice way to try prompts before you put them in robot code.

## Why Docker?

The Raspberry Pi docs explain that Open WebUI needs Docker here, because it is incompatible with the Python 3.13 that comes with Raspberry Pi OS Trixie. So Docker runs it in a container, away from your system Python.

## Install Docker

Installing Docker means adding Docker's apt repository, and the docs list the exact commands, which are long and change over time. Please follow the **Install Docker** step on the [AI software page](https://raspberrypi.com/documentation/computers/ai.html) and, where it points to it, the Docker Engine on Debian guide. The docs say the `Suites` line in `/etc/apt/sources.list.d/docker.sources` should say `trixie`, and to finish with:

```bash
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl start docker
```

Then let your user run Docker without `sudo`:

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

Test it:

```bash
docker run hello-world
```

## Start Open WebUI

Make sure `hailo-ollama` is running first, in its own terminal. Then, from the docs:

```bash
docker pull ghcr.io/open-webui/open-webui:main
docker run -d -e OLLAMA_BASE_URL=http://127.0.0.1:8000 -v open-webui:/app/backend/data --name open-webui --network=host --restart always ghcr.io/open-webui/open-webui:main
docker logs open-webui -f
```

Starting up can take up to a minute. Then open `http://127.0.0.1:8080` in a browser on the Pi. Follow the page's sign-up prompts, then pick a model and start chatting.

**What the options do:** `OLLAMA_BASE_URL` tells Open WebUI where `hailo-ollama` is listening, port 8000. `--network=host` lets the container see that port. `--restart always` brings it back after a reboot.

## Real robot use

A chat page is great for prompt practice. Try the exact wording you plan to send from code, such as "Answer with one word: forward, stop or search". When a prompt gives you reliable answers here, copy it into your Python program.

## Try it Yourself

1. Start a chat and ask the model to introduce itself as a robot called Bolt.
2. Ask for a one-word answer and see whether it obeys.
3. Challenge: open the page from another computer on your network. Which address will you need? (Hint: the Pi's own address, not 127.0.0.1.)

## Common Issues

**Problem**: The page shows no models.
**Solution**: Check `hailo-ollama` is running and that you pulled a model.
**Why**: Open WebUI only lists what the server offers.

**Problem**: `permission denied` talking to Docker.
**Solution**: Run the `usermod` and `newgrp` commands, or sign out and back in.
**Why**: Your user has to be in the `docker` group.
