---
layout: lesson
title: Object detection with rpicam-apps
author: Kevin McAleer
type: page
cover: /learn/ai_hat_2/assets/banners/04_object_detection_with_rpicam_apps.jpg
date: 2026-11-24
previous: 03_software_setup.html
next: 05_pose_and_segmentation.html
description: Run YOLO object detection on the camera with rpicam-hello and a Hailo
  post-processing file
percent: 35
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

Time to give the robot eyes. The camera apps from Raspberry Pi can hand each frame to the HAT, and a JSON **post-processing file** tells them which model to use and how to draw the results.

## Your first detection

The docs give this pattern: run `rpicam-hello` with `-t 0` (run until stopped) and a post-process file.

```bash
rpicam-hello -t 0 --post-process-file /usr/share/rpi-camera-assets/hailo_yolov8_inference.json
```

You should see the camera preview with boxes drawn round the things the model recognises. Point it at a person, a cup or a chair. Press Ctrl+C in the terminal to stop.

## The models to choose from

The docs list these object detection files, all in `/usr/share/rpi-camera-assets/`:

| Model | Post-process file | Notes from the docs |
|---|---|---|
| YOLOv6 | `hailo_yolov6_inference.json` | Object detection |
| YOLOv8 | `hailo_yolov8_inference.json` | Object detection |
| YOLOX | `hailo_yolox_inference.json` | Lightweight, fast |
| YOLOv5 | `hailo_yolov5_inference.json` | People and face detection |
{:class="table table-single"}

**Real robot example:** for a robot that follows people, `hailo_yolov5_inference.json` is the one the docs point to for people and faces. For a robot that has to notice many kinds of object, start with the YOLOv8 file. If the robot's frame rate feels sluggish, try YOLOX, which the docs call lightweight and fast.

## Useful flags

The docs mention two flags worth knowing:

- `-n` turns off the viewfinder window, which is what you want on a robot with no screen
- `-v 2` shows text output only

```bash
rpicam-hello -t 0 -n -v 2 --post-process-file /usr/share/rpi-camera-assets/hailo_yolov8_inference.json
```

The docs also note that other `rpicam-apps` such as `rpicam-vid` and `rpicam-still` may need option changes.

## Why this matters

Everything in the camera pipeline, the capture, the model and the drawing, is already wired up. You haven't written any AI code, and you have a working detector. Later lessons build on this rather than replacing it.

## Try it Yourself

1. Run the YOLOv8 file and hold up three different objects to the camera.
2. Switch to the YOLOX file and compare how smooth it feels.
3. Challenge: switch to the YOLOv5 file, and walk into the frame. Does it find you more reliably?

## Common Issues

**Problem**: No preview window appears.
**Solution**: Check `rpicam-hello` alone works. If you are connected over SSH, use `-n` and `-v 2`, because there is no display to draw on.
**Why**: The preview needs a screen. The text-only mode does not.

**Problem**: The file path is not found.
**Solution**: Check you installed `rpicam-apps` and that the Pi OS is up to date, then list the folder with `ls /usr/share/rpi-camera-assets/`.
**Why**: The JSON files ship with the camera software packages.

## Check your understanding

{% quiz %}
? What does the `--post-process-file` option do?
- [ ] It saves the video to disk
- [x] It points `rpicam-hello` at a JSON file that picks the Hailo model and how to show the results
- [ ] It overclocks the HAT
> The JSON files in `/usr/share/rpi-camera-assets/` describe the model and the output.

? Which flag turns off the viewfinder, handy for a robot with no screen?
- [ ] `-t 0`
- [x] `-n`
- [ ] `-v 2`
> `-n` turns off the viewfinder. `-v 2` shows text output only.
{% endquiz %}
