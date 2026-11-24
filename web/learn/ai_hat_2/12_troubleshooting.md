---
layout: lesson
title: Troubleshooting
author: Kevin McAleer
type: page
cover: /learn/ai_hat_2/assets/banners/12_troubleshooting.jpg
date: 2026-11-24
previous: 11_performance_and_choosing_hardware.html
next: 13_summary.html
description: A table of the common problems with the AI HAT+ 2, what to do and why
percent: 91
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

Things will go wrong. That is normal, and nearly every problem on this list is a small one. Work down from the top, because the early items cause the later ones.

| Problem | Solution | Why |
|---|---|---|
| `hailortcli fw-control identify` finds nothing | Power off and reseat the PCIe ribbon cable at both ends | The HAT talks to the Pi over PCIe, and a loose cable looks like no device |
| Software won't install, or conflicts appear | Use `hailo-h10-all`, not `hailo-all`, and don't install both | The docs say the two packages cannot co-exist |
| Serial Number shows `<N/A>` | Ignore it | The docs say this is expected on the AI HAT+ and AI HAT+ 2 |
| `rpicam-hello` shows no camera | Reseat the camera cable, and attach the camera before the AI hardware | The docs say to attach the camera first |
| The post-process JSON is not found | Install `rpicam-apps`, update the OS, and list `/usr/share/rpi-camera-assets/` | The files ship with the camera software |
| No preview over SSH | Add `-n -v 2` | There is no screen to draw on |
| `curl` says connection refused on port 8000 | Start `hailo-ollama` in another terminal | The server has to be running |
| Open WebUI shows no models | Check `hailo-ollama` is running and a model is pulled | It lists what the server offers |
| Docker says permission denied | Add yourself to the `docker` group and run `newgrp docker` | Docker needs group access |
| SD card full | Use a bigger card or remove models | Model files are large, and one review filled a 32 GB card |
| Speeds are slower than you hoped | Fit the heatsink and fan, and compare with the review in lesson 11 | A hot chip and bandwidth limits both matter |
| Old AI HAT+ code fails | Check you are using models compiled for the Hailo-10H | A commenter on the announcement said existing apps need models compiled for the H10 |
{:class="table table-single"}

## A debugging order that works

1. Is it detected? `hailortcli fw-control identify` and `dmesg | grep -i hailo`.
2. Does the camera work alone? `rpicam-hello`.
3. Does a detection demo work? The YOLOv8 command from lesson 4.
4. Does the server answer? `curl --silent http://localhost:8000/hailo/v1/list`.
5. Only then look at your own code.

```python
# The same order, as a checklist you can print.
steps = [
    "HAT detected (hailortcli fw-control identify)",
    "Camera works alone (rpicam-hello)",
    "Detection demo runs (lesson 4)",
    "hailo-ollama answers (curl the list URL)",
    "Then, and only then, debug your own code",
]
for number, step in enumerate(steps, start=1):
    print(f"{number}. {step}")
```

## Where to ask

The Hailo community forum at [community.hailo.ai](https://community.hailo.ai/) and the `hailo-apps` repository are good places for questions that aren't answered here.

## Try it Yourself

1. Run the first two checks on your Pi and note the results.
2. Break something on purpose, such as stopping `hailo-ollama`, and see which check catches it.
3. Challenge: write your own troubleshooting row for a problem you hit.
