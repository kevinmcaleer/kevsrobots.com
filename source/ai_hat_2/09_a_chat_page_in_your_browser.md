---
title: A chat page in your browser
description: Add the Open WebUI chat page in Docker so you can talk to the language model from any browser
layout: lesson
type: page
cover: /learn/ai_hat_2/assets/banners/09_a_chat_page_in_your_browser.jpg
date_updated: 2026-11-24
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
