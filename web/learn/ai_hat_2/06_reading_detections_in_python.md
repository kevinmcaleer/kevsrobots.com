---
layout: lesson
title: Reading detections from Python
author: Kevin McAleer
type: page
cover: /learn/ai_hat_2/assets/banners/06_reading_detections_in_python.jpg
date: 2026-11-24
previous: 05_pose_and_segmentation.html
next: 07_detections_to_robot_behaviour.html
description: Start a Hailo camera pipeline from Python and find the official routes
  for reading detection data
percent: 49
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

So far the detections have only been drawn on a screen. A robot needs them in code. This is the lesson where we're careful, because the HAT is new and details matter.

## What the docs tell us

The Raspberry Pi documentation says `rpicam-apps` and `Picamera2` can offload supported tasks to the HAT. The Hailo-apps repository, at [github.com/hailo-ai/hailo-apps](https://github.com/hailo-ai/hailo-apps), provides Python examples. Its README lists these folders:

- `hailo_apps/python/pipeline_apps/` has detection (`hailo-detect-simple`), pose (`hailo-pose`), segmentation (`hailo-seg`) and depth (`hailo-depth`)
- `hailo_apps/python/standalone_apps/` has minimal Python examples
- `hailo_apps/python/gen_ai_apps/` has the LLM, VLM and speech apps for the Hailo-10H

For install steps, the README sends you to its installation guide (`doc/user_guide/installation.md`) and prerequisites page. For how to run each app, see `doc/user_guide/running_applications.md`. We won't copy those steps here, because they change as the project grows. Follow them for your version.

For Picamera2, the camera software page links the Picamera2 manual. Use that for the Python side of the camera.

## Starting the pipeline from Python

One thing we can do safely is have Python start the exact `rpicam-hello` command from lesson 4. This is a complete program. By default it only prints the command, so you can read it first. Add `--run` to start the camera.

```python
import subprocess
import sys

MODELS = {
    "yolov8": "/usr/share/rpi-camera-assets/hailo_yolov8_inference.json",
    "yolox": "/usr/share/rpi-camera-assets/hailo_yolox_inference.json",
    "yolov5": "/usr/share/rpi-camera-assets/hailo_yolov5_inference.json",
    "pose": "/usr/share/rpi-camera-assets/hailo_yolov8_pose.json",
}


def build_command(model, headless=True):
    """Build the rpicam-hello command from the docs for a chosen model."""
    command = ["rpicam-hello", "-t", "0"]
    if headless:
        command += ["-n", "-v", "2"]  # no viewfinder, text output only
    command += ["--post-process-file", MODELS[model]]
    return command


if __name__ == "__main__":
    command = build_command("yolov8")
    print("Command:", " ".join(command))
    if "--run" in sys.argv:
        subprocess.run(command)
    else:
        print("Add --run to start the camera")
```

Save it as `start_eyes.py` and try `python3 start_eyes.py`, then `python3 start_eyes.py --run`.

**Why this matters:** a robot program can now choose its "eyes" in one line, and switch from detection to pose without you retyping long paths.

## Where the detection data comes from

The text output you get with `-v 2` is for people to read. This course doesn't document its exact format, because the docs we follow don't specify it, so please don't write a parser on a guess. For robot code, use one of the routes above: the Picamera2 manual, or the Python apps in `hailo-apps`.

Whichever you use, think of each detection as three things: **what** it is (a label such as `person`), **how sure** the model is (a confidence), and **where** it is (a box). In lesson 7 we write the decision logic around exactly that shape of data, so it will plug into any of the routes.

## Try it Yourself

1. Run `start_eyes.py` without `--run` and read the command it prints.
2. Change `"yolov8"` to `"pose"` and print the command again.
3. Challenge: open the hailo-apps repository and find the folder for the detection example. What files does it contain?

## Common Issues

**Problem**: `FileNotFoundError: rpicam-hello`.
**Solution**: Run the program on the Pi, and install `rpicam-apps` as in lesson 3.
**Why**: The program starts a command that only exists on Raspberry Pi OS with the camera software.

**Problem**: The hailo-apps install steps differ from what you expected.
**Solution**: Follow the version in the repository's installation guide.
**Why**: It is under active development, and its guide is the source of truth.
