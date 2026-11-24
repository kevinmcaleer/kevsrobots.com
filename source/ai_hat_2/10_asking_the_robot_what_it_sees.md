---
title: Asking the robot what it sees
description: Use a vision-language model on the HAT to describe a camera view, and turn the description into a robot decision
layout: lesson
type: page
cover: /learn/ai_hat_2/assets/banners/10_asking_the_robot_what_it_sees.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Object detection gives labels from a fixed list. A **vision-language model** (VLM) looks at a picture and answers in plain English, so you can ask it anything: "Is the doorway clear?" or "What is on the table?" This is the second half of eyes and a brain.

## What the sources say

- The Raspberry Pi docs say the AI HAT+ 2 supports VLMs, through Hailo's `hailo-apps` repository.
- The Raspberry Pi announcement mentions a demo where a VLM describes a camera stream.
- The CNX Software review ran Qwen2-VL-2B-Instruct through the Hailo apps VLM chat demo. It gave no speed figures, and the author reported that the camera colours looked off.
- Hailo's own announcement suggests VLMs for event triggering, logging, captioning and smart search.

The exact command to start the VLM demo is not given on the pages we follow. So, as in lesson 6, please use the `hailo-apps` repository: look in `hailo_apps/python/gen_ai_apps/` and the repository's `doc/user_guide/running_applications.md`. Run the VLM demo there and ask it what it sees.

## From a sentence to a decision

A description is just text. A robot needs a decision. Here we write plain Python that reads a description and picks a command. It uses sample descriptions so you can test the logic anywhere, and it is the same code whichever way you collect the real text.

```python
# What the VLM might say. These are invented examples.
descriptions = [
    "A clear hallway with a wooden floor and a door at the end.",
    "A person is standing in front of the camera holding a box.",
    "A chair and a cardboard box block the way ahead.",
    "A dog is lying on a rug near the sofa.",
]

BLOCKERS = ["box", "chair", "wall", "person", "dog"]


def decide(description):
    text = description.lower()
    if "clear" in text and not any(word in text for word in BLOCKERS):
        return "forward"
    found = [word for word in BLOCKERS if word in text]
    if "person" in found:
        return "stop and say hello"
    if found:
        return "slow down and go round: " + ", ".join(found)
    return "stop and ask for help"


for text in descriptions:
    print(f"{decide(text):45} <- {text}")
```

Run it and look at the output. The hallway description says "clear" and mentions no blocker word, so it goes `forward`. The person description gets a greeting, and the chair and box description makes the robot slow down and steer round.

**Why this matters:** words are slippery. "A clear hallway" and "no clear path" both contain "clear". A keyword check is a quick start, not a safety system. For anything a robot could bump into, trust a depth or distance sensor first, and use the language model to explain or choose.

**A better habit:** ask the model for a short, fixed-format answer, such as one word, and compare it against a list of allowed words. If the answer isn't on the list, do nothing.

## Try it Yourself

1. Add a fifth description of your own, and see what `decide` does with it.
2. Add `"stairs"` to `BLOCKERS`. What should the robot do about stairs?
3. Challenge: write a function that only accepts one of `forward`, `stop` or `search` and returns `stop` for anything else.

## Common Issues

**Problem**: The description is long and rambling.
**Solution**: Ask for one short sentence, or a single word, in your prompt.
**Why**: Short, fixed answers are easier for code to check.

**Problem**: The colours look wrong in the VLM demo.
**Solution**: Compare with a plain `rpicam-hello` view, and check the hailo-apps issues list.
**Why**: The CNX Software reviewer saw this too. It is a software matter, not your wiring.
