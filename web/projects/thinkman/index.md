---
title: "Thinkman"
description: "Portable, Raspberry Pi Based Offline AI Assistant"
excerpt: >-
    Learn how to build your own portable AI assistant using a Raspberry Pi, 3D printing, and Python. This project combines hardware and software to create a modern take on the classic Sony Walkman, allowing you to interact with AI models offline.
layout: showcase
date: 2025-06-06
date_updated: 2025-06-06
author: Kevin McAleer
difficulty: beginner
cover: /projects/thinkman/assets/cover.jpg
hero:  /projects/thinkman/assets/hero.png
mode: light
videos:
  - OJTa4qAjBqg
tags:
 - Raspberry Pi
 - ai
groups:
 - raspberrypi
 - ai
code:
 - https://www.github.com/kevinmcaleer/thinkman
stl:
 - name: Body
   link: /projects/thinkman/assets/thinkman_body_v1.stl
 - name: Top
   link: /projects/thinkman/assets/thinkman_top_v1.stl
 - name: Flap
   link: /projects/thinkman/assets/thinkman_flap_v1.stl
---

## Inspiration

This was an idea I had ever since running Ollama on a Raspberry Pi 5 for the first time, however it was the PiSugar  team (<https://www.pisugar.com>) who kindly provided me with a PiSugar 3 Plus battery pack at Makers Central this year that made this project possible.

The physical design is an homage to Sony Walkman; I had one of these as a kid, and I took it everywhere with me, so I wanted to create a modern version of this, but with a Raspberry Pi and AI.

---

## Hardware

{% include gallery.html images="/projects/thinkman/assets/thinkman06.jpg" titles="" descriptions="" noborder=true small_title=true cols=1 %}

This project uses the following hardware:

- **Raspberry Pi 5** - the latest Raspberry Pi, with 4Gb of RAM or higher - I used a 16Gb Pi 5 for this project.
- **PiSugar 3 Plus** - a battery pack that allows you to power the Raspberry Pi on the go.
- **3D Printed Case** - a custom case that holds the Raspberry Pi and the battery pack together, designed to look like a Sony Walkman.  
- **USB Headset** - for audio input/output, to allow you to speak to the AI assistant, and hear responses.
- **M.2 HAT+ and NVMe SSD** - for additional storage, allowing you to run larger AI models and store more data.
- **Active cooling** - a small fan to keep the Raspberry Pi cool during operation, especially when running AI models.

---

## Setup

1. On your Raspberry Pi, open up a terminal.
2. First we need to clone the **Thinkman** project.
3. Clone the thinkman project:

    `git clone https://github.com/kevinmcaleer/thinkman`

    then

    `cd thinkman`

---

## Software Architecture

{% include card.html img="/projects/thinkman/assets/thinkman07.jpg" cardtitle="Software Architecture" description="Thinkman software stack" tl="1" dl="1" small_title=true noborder=true %}

---

Thinkman is made up of a few different components:

- **ollama** - this is a local AI model server that allows you to run large language models on your Raspberry Pi.
- **vosk** - this is a speech recognition library that allows you to convert speech to text.
- **espeak-ng** - this is a text-to-speech engine that allows you to convert text to speech.
- **docker** - this is used to run the ollama server and the open webui.

## Python setup

On Raspberry Pi we need to install the following packages:

- `libportaudio2` - this is the PortAudio library, which is used for audio input and output.
- `portaudio19-dev` - this is the development files for PortAudio, which are needed to build the Python bindings.
- `espeak-ng` - this is a text-to-speech engine that will be used to speak the responses from the AI models.

We also need to download the `vosk` model.

Luckily both of these tasks are taken care of by running the `install.sh` script.

---

> ## `install.sh` 
> The `install.sh` script will install the required packages and download the Vosk model for you. It will also install `uv`, the Ultimate Virtual environment and package manager for Python, set up the Python virtual environment and install the required Python dependencies.
> Downloads and installs the vosk language model for accurate speech recognition 
> installs portaudio & espeak-ng
> installs uv - the Ultra Fast Virtualenv and Package Manager
> sets up the virtual python environment
> installs the python dependencies
> brings up the ollama docker container
> downloads 2 large language models; deepseek-r1:1.5b and tinyllama

---

Next we need to setup a virtual environment, lets install UV as its much faster than the traditional method:

`curl -LsSf https://astral.sh/uv/install.sh | sh`

Next setup the virtual environment with:

`uv venv venv`

Now lets install the python dependencies:

`uv pip install -r requirements.txt`

ollama and open webui install via docker

## Install Docker

If you've not got docker installed, follow these instructions: <https://www.kevsrobots.com/learn/docker/02_installing-docker.html>

1. Run the docker command to bring up ollama and the open webui:

    `docker compose up -d`

1. This will install docker and bring up open webui as well.

1. Finally we need to download the `deepseek-r1:1.5b` model into ollama.

    `docker exec -it ollama ollama run deepseek-r1:1.5b`

    and to install the `TinyLlama` model, use:

    `docker exec -it ollama ollama run tinyllama`

1. Type `/bye` to exit the ollama prompt

1. Now run the code with `python main.py`

---

## Assembly

The 3D printed case is designed to hold the Raspberry Pi, the PiSugar battery pack, and the M.2 HAT+ with NVMe SSD. 

The case is printed in three parts: body, top, and flap.

Assembling the parts is simple and doesn't require glue. 2x M2.5 screws are used to attach the PiSugar battery pack to the body. The flap is held in place with a friction fit.

{% include gallery.html images="/projects/thinkman/assets/thinkman01.jpg,/projects/thinkman/assets/thinkman02.jpg,/projects/thinkman/assets/thinkman03.jpg,/projects/thinkman/assets/thinkman04.jpg,/projects/thinkman/assets/thinkman05.jpg" titles="" descriptions="" noborder=true small_title=true cols=2 %}

---
