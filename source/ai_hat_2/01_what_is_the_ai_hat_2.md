---
title: What is the AI HAT+ 2?
description: What the Hailo-10H board adds to a Raspberry Pi 5, and what it can and cannot do
layout: lesson
type: page
cover: /learn/ai_hat_2/assets/banners/01_what_is_the_ai_hat_2.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Let's start by knowing exactly what is on the board, because it explains everything we do later.

## The key facts

The Raspberry Pi documentation and product page give these specifications:

| Feature | AI HAT+ 2 |
|---|---|
| AI chip | Hailo-10H |
| Rating | 40 TOPS (INT4) |
| On-board memory | 8 GB, its own, separate from the Pi's |
| Host | Raspberry Pi 5, over its PCIe port |
| Runs | Vision models, large language models (LLMs) and vision-language models (VLMs) |
{:class="table table-single"}

The documentation says it can run LLMs and VLMs "up to ~6 billion parameters". It also says the original AI HAT+ does not support LLMs or VLMs, and that only the AI HAT+ 2 does.

**Why the 8 GB matters:** a language model has to live somewhere while it runs. On the AI HAT+ 2 it lives in the HAT's own memory, so the Pi's RAM and CPU stay free for your robot code. Raspberry Pi describes this as keeping the host Pi 5 free for other tasks.

## What TOPS means

TOPS is "trillions of operations per second". Think of it as how many small sums the chip can do each second. More is faster, but only for models the chip can run, and the number is for INT4 (very compact numbers).

```python
# A rough picture of "operations": every pixel of a camera frame
# gets multiplied by a weight, many times over, inside a model.
width, height = 640, 640
weights_per_pixel = 1000      # an invented number, to show the scale
operations = width * height * weights_per_pixel
print(f"One frame is roughly {operations:,} operations")
```

Do not worry about the exact number. The point is that a robot camera makes the chip do a lot of sums, and a dedicated chip does them far more efficiently than the Pi's CPU.

## Vision: the same as before, mostly

For vision work, the Raspberry Pi announcement says performance is roughly equal to the 26 TOPS AI HAT+. So if you already know the AI HAT+, the camera apps in this course will feel familiar. The new part is the generative AI.

## Software in the box

Raspberry Pi OS detects the accelerator automatically. The camera software, `rpicam-apps` and `Picamera2`, can offload supported tasks to it. Other software components and models are installed separately, which is what lesson 3 covers.

## What it is not

Hailo's own announcement says the HAT is "not designed to be a replacement for cloud inference or large LLMs". Expect small models that fit in 8 GB. That is plenty for a robot that needs to describe a scene or answer a simple question.

## Try it Yourself

1. Write down three things you'd like a robot to notice, such as a person, a cup or a doorway.
2. For each one, decide if it needs vision only, or a language model as well.
3. Challenge: which of your three would you want to work with no internet at all?

## Common Issues

**Problem**: "I have an AI HAT+ and want to run a language model."
**Solution**: You need the AI HAT+ 2.
**Why**: The Raspberry Pi documentation says only the AI HAT+ 2 supports LLMs.

## Check your understanding

{% quiz %}
? Which Raspberry Pi AI board can run large language models?
- [ ] The AI HAT+ with Hailo-8
- [x] The AI HAT+ 2 with Hailo-10H
- [ ] Both of them
> Raspberry Pi's documentation says only the AI HAT+ 2 supports LLMs.

? Why does the HAT's own 8 GB of memory help a robot project?
- [ ] It makes the Pi 5 boot faster
- [x] The model lives on the HAT, so the Pi's CPU and RAM stay free for your code
- [ ] It lets you remove the Pi's microSD card
> The model is held in the HAT's memory rather than in the Pi's.
{% endquiz %}
