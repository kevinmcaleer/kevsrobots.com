---
layout: course_overview
title: Building Object Detection Models with Raspberry Pi AI Camera
author: Kevin McAleer
cover: /learn/object_model/assets/cover.jpg
date: 2025-12-04
description: Learn how to build custom object detection models using the Raspberry
  Pi AI Camera. Train your own models with PyTorch, optimize them for edge deployment,
  and run real-time detection on Raspberry Pi hardware. Complete hands-on course from
  dataset creation to production deployment.
level: intermediate
course_duration: 0h 55m
lesson_count: 9
first_lesson: 00_intro.html
quiz_total: 24
video: ARCtzjHfyMg
outcomes:
- Explain what an object detection model does and how a camera sees bounding boxes
  and confidence scores
- Collect and label a small image dataset and lay it out the way YOLO training expects
- Fine-tune a pre-trained YOLO model on your own objects with transfer learning
- Export and quantise the model, then package it for the Raspberry Pi AI Camera (IMX500)
- Run the model on the camera from Python with Picamera2 and react to what it sees
- Measure, diagnose and improve a detector that misses things or sees things that
  are not there
prerequisites:
- Basic Python (variables, loops, functions), as in the Python course
- Comfortable typing commands in a terminal, as in the Linux intro course
you_need:
- A Raspberry Pi 5 or Raspberry Pi 4 running Raspberry Pi OS
- A Raspberry Pi AI Camera (Sony IMX500)
- A computer to train on, with Python 3.10 to 3.12 (Linux, or Windows with WSL, for
  the export step)
- A phone or webcam to photograph your objects, and 30 to 60 minutes per lesson
navigation:
- name: Building Object Detection Models with Raspberry Pi AI Camera
- content:
  - section: Getting Started
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Setting up Your Development Environment
      link: 01_setup.html
  - section: Dataset Preparation
    content:
    - name: Preparing Custom Object Detection Datasets
      link: 02_dataset_prep.html
  - section: Model Training
    content:
    - name: Training Custom Object Detection Models
      link: 03_training.html
    - name: Exporting and Quantising for the IMX500
      link: 04_onnx_conversion.html
  - section: Deployment
    content:
    - name: Deploying to the Raspberry Pi AI Camera
      link: 05_deployment.html
    - name: Real-Time Object Detection
      link: 06_realtime_detection.html
  - section: Advanced Topics
    content:
    - name: Troubleshooting and Improving Your Model
      link: 07_troubleshooting.html
    - name: Course Summary and Next Steps
      link: 08_summary.html
---
