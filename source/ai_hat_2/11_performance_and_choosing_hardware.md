---
title: Performance and choosing hardware
description: Real-world numbers from a review, and when to pick the AI Camera, the AI HAT+ or the AI HAT+ 2
layout: lesson
type: page
cover: /learn/ai_hat_2/assets/banners/11_performance_and_choosing_hardware.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You now have a robot that can see and talk. The last design question is: which board should your next robot use? Let's separate what is official from what one reviewer measured.

## Honest numbers

The official numbers are the specifications: 40 TOPS (INT4), 8 GB of on-board memory, and vision performance roughly equal to the 26 TOPS AI HAT+.

The next numbers come from one review, the CNX Software review of January 2026. They used a Raspberry Pi 5 2GB with the HAT, and compared against a CM5 devkit with 4GB running Ollama on its CPU:

| Model | CPU (tokens per second) | AI HAT+ 2 (tokens per second) |
|---|---|---|
| DeepSeek R1 1.5B | 9.04 | 6.72 |
| Qwen2 1.5B | 11.29 | 5.89 |
| Llama3.2 3B | 4.78 | 2.60 |
| Qwen2.5 instruct 1.5B | 11.73 | 6.74 |
| Qwen2.5 Coder 1.5B | 10.26 | 8.06 |
{:class="table table-single"}

Read that carefully. In this review, the HAT was **slower** than the CPU at generating text for these models. The reviewer said it was "more like an AI decelerator than an AI accelerator" for LLMs, and noted that the HAT's figures may include prompt evaluation time. He also reported Eben Upton saying that speed is limited by memory bandwidth, and that the aims are better time-to-first-token and offloading the host CPU and memory.

The same review measured power with DeepSeek R1 1.5B: 7.2 to 7.6 W for the Pi 5 2GB with the HAT, against 10.2 to 10.6 W for the CM5 using Ollama. For vision, the reviewer said performance was about the same as the first-generation AI HAT+.

**What this means for a robot:** the win is not raw token speed. The model runs on the HAT, leaving the Pi 5's CPU and memory free for your robot code, and uses less power in that test. Treat these as one reviewer's figures, and measure your own.

## Picking the board

This table combines the Raspberry Pi pages, the Raspberry Pi AI software docs and this site's [AI Camera course](/learn/object_model/):

| | AI Camera | AI HAT+ | AI HAT+ 2 |
|---|---|---|---|
| What it is | A camera with the AI chip in the camera module | A Pi 5 add-on board | A Pi 5 add-on board |
| Chip | Sony IMX500 sensor | Hailo-8L (13 TOPS) or Hailo-8 (26 TOPS) | Hailo-10H (40 TOPS INT4) |
| Own memory | Not covered here | Not covered here | 8 GB |
| Vision (detection, pose) | Yes | Yes | Yes, about like the 26 TOPS AI HAT+ |
| LLMs and VLMs | No | No, per the docs | Yes |
| Needs a Pi 5 | Works on other Pis, see the AI Camera course | Yes | Yes |
{:class="table table-single"}

**Rules of thumb:**

- **AI Camera** if you want the simplest vision and to keep the Pi's PCIe free. Train your own models in the [AI Camera course](/learn/object_model/).
- **AI HAT+** if you only need vision on a Pi 5 and want to spend less.
- **AI HAT+ 2** if your robot should also run a language model or a VLM locally, with no cloud.

The AI HAT+ 2 and an M.2 board both want the Pi 5's one PCIe port, so one of them has to go. This came from a commenter in Raspberry Pi's own announcement, so check before you plan a build around it.

On price, the announcement and the product page quoted different figures when this course was written, so check the product page for today's price.

## Try it Yourself

1. Write down your next robot's job in one sentence.
2. Use the table to choose a board, and say why.
3. Challenge: if your answer is the AI Camera, which course would you do next?

## Common Issues

**Problem**: "Faster AI" turned out to mean slower tokens.
**Solution**: Compare what you care about: CPU free, power, or speed.
**Why**: In the review the HAT freed the CPU and saved power, but did not out-run the CPU on tokens per second.
