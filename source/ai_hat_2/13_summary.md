---
title: Summary
description: What you have built, what to remember, and where to go next with the AI HAT+ 2
layout: lesson
type: page
cover: /learn/ai_hat_2/assets/banners/13_summary.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You gave a robot eyes and a brain. Let's recap what you can now do.

## What you learned

- The AI HAT+ 2 carries a Hailo-10H (40 TOPS INT4) and its own 8 GB of memory, and it is the one board in the family that runs LLMs and VLMs
- How to fit the HAT, install `hailo-h10-all`, and check it with `hailortcli`
- How to run object detection, pose estimation and segmentation with `rpicam-hello` and the Hailo post-process files
- Where to find the official Python routes to detection data, and how to start a pipeline from Python
- How to turn detections into a person-following decision in plain Python
- How to run `hailo-ollama`, talk to it from the terminal and Python, and chat in the browser with Open WebUI
- How to turn a vision-language description into a cautious decision
- How to choose between the AI Camera, the AI HAT+ and the AI HAT+ 2, and read a review's numbers sensibly

## Key points to remember

- Follow the official docs for commands, because the software moves quickly
- Keep your decision code separate from your detector, so you can swap either one
- Don't let a language model be the only thing between your robot and a wall
- Fit the heatsink and fan for heavy work

## Where to go next

- [Computer Vision on Raspberry Pi with CVZone](/learn/cvzone/), for more camera tricks
- [Building Object Detection Models with Raspberry Pi AI Camera](/learn/object_model/), to train your own detector
- [Reinforcement learning](/learn/reinforcement_learning/), to let a robot learn behaviour by trial and error
- [Learn ROS](/learn/learn_ros/), to wire your detections into a bigger robot system

Thank you for building along. Now go and give a robot some eyes, and see what it says about what it sees.
