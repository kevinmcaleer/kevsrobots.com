---
layout: lesson
title: Deploying to Raspberry Pi AI Camera
author: Kevin McAleer
type: page
cover: /learn/object_model/assets/cover.jpg
date: 2025-12-04
previous: 04_onnx_conversion.html
next: 06_realtime_detection.html
description: Learn how to deploy your trained object detection model to the Raspberry
  Pi AI Camera for edge inference
percent: 66
duration: 8
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
    - name: Converting Models to ONNX Format
      link: 04_onnx_conversion.html
  - section: Deployment
    content:
    - name: Deploying to Raspberry Pi AI Camera
      link: 05_deployment.html
    - name: Real-Time Object Detection
      link: 06_realtime_detection.html
  - section: Advanced Topics
    content:
    - name: Troubleshooting and Optimization
      link: 07_troubleshooting.html
    - name: Course Summary and Next Steps
      link: 08_summary.html
---


## Overview

Now that you have a trained model in ONNX format, let's deploy it to the Raspberry Pi AI Camera! This lesson covers transferring your model, installing dependencies, converting for IMX500 hardware, and running your first inference on the Pi.

---

## Objectives

By the end of this lesson, you will:

- Transfer your ONNX model to the Raspberry Pi
- Install IMX500 tools and dependencies
- Convert the model for IMX500 hardware acceleration
- Run a test inference on a sample image
- Understand the deployment workflow
- Verify camera and model integration

---

## Prerequisites

Before starting:
- [ ] Raspberry Pi 5 or 4 with Raspberry Pi OS installed (latest version)
- [ ] Raspberry Pi AI Camera connected to CSI port
- [ ] Your ONNX model file from Lesson 04
- [ ] SSH access to your Pi (or keyboard/monitor connected)
- [ ] Internet connection on the Pi

---

## Why Deploy to the Edge?

Running object detection on the Raspberry Pi (edge device) instead of the cloud offers huge advantages:

**Benefits of edge deployment:**
- **Privacy** - Images never leave the device
- **Speed** - No network latency (10-100ms vs 500-2000ms)
- **Reliability** - Works without internet connection
- **Cost** - No cloud API fees
- **Power** - Low power consumption (~5W total)

**Real-world uses:**
- Security cameras that work offline
- Factory quality control on the production line
- Agricultural robots in fields without connectivity
- Privacy-focused home automation

---

## Transfer Your Model

First, get your ONNX model onto the Pi.

### Option 1: Using SCP (Recommended)

From your computer's terminal:

```bash
# Replace 'raspberrypi.local' with your Pi's hostname or IP address
scp custom_detector.onnx pi@raspberrypi.local:~/models/

# You'll be prompted for the Pi's password
```

**Can't find your Pi's address?**
```bash
# On the Pi, run:
hostname -I

# Or use:
ping raspberrypi.local
```

### Option 2: Using USB Drive

1. Copy `custom_detector.onnx` to a USB drive
2. Insert the drive into your Pi
3. On the Pi, copy the file:

```bash
# Find the USB drive
lsblk

# Mount if needed (usually auto-mounts to /media/pi/)
sudo mount /dev/sda1 /mnt

# Copy the model
mkdir -p ~/models
cp /media/pi/YOUR_USB_NAME/custom_detector.onnx ~/models/

# Verify it copied
ls -lh ~/models/custom_detector.onnx
```

### Option 3: Direct Download

If your model is hosted somewhere:

```bash
mkdir -p ~/models
cd ~/models
wget https://your-server.com/custom_detector.onnx
```

---

## Install IMX500 Tools

On your Raspberry Pi, install the required tools:

```bash
# Update package list (important!)
sudo apt update

# Install IMX500 tools and Python bindings
sudo apt install -y imx500-tools python3-imx500-tools

# Install additional Python packages
sudo apt install -y python3-picamera2 python3-opencv python3-numpy

# Verify installation
imx500-package.sh --version
```

**Expected output:**
```
imx500-package version 1.0.2
```

**If installation fails:**
Make sure you're on the latest Raspberry Pi OS:
```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

---

## Convert Model for IMX500

The IMX500 sensor has its own neural processor. Convert your model to its optimized format:

```bash
# Navigate to your models directory
cd ~/models

# Convert ONNX to IMX500 format (.rpk file)
imx500-package.sh -i custom_detector.onnx -o detector_imx500

# This takes 2-5 minutes depending on model size
```

**What's happening:**
- Model is optimized for IMX500 neural processor
- Converted from floating-point to fixed-point (INT8/INT16)
- Packaged with metadata (input/output specs, normalization params)
- Creates `.rpk` (Raspberry Pi Package) file

**Expected output:**
```
Processing custom_detector.onnx...
Analyzing model structure...
Converting layers to IMX500 format...
Quantizing weights...
✓ Conversion successful
✓ Created detector_imx500/network.rpk
✓ Model size: 12.3 MB (reduced from 45.3 MB)
```

**Check the output:**
```bash
ls -lh detector_imx500/
```

You should see:
- `network.rpk` - The converted model
- `labels.txt` - Class names (if you provided them)
- `metadata.json` - Model configuration

---

## Verify Camera Connection

Make sure your AI Camera is properly connected:

```bash
# List available cameras
libcamera-still --list-cameras

# Should show the IMX500 camera
```

**Expected output:**
```
Available cameras:
0 : imx500 [4056x3040] (/base/axi/pcie@120000/rp1/i2c@88000/imx500@1a)
```

**If camera not found:**

1. **Check physical connection:**
   - Cable fully inserted in both camera and Pi
   - Blue side of cable faces USB ports on Pi
   - Camera module LED should light up on boot

2. **Enable camera in config:**
```bash
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable
# Reboot: sudo reboot
```

3. **Check cable:**
```bash
vcgencmd get_camera
# Should return: detected=1
```

---

## Test Camera Capture

Take a test photo to verify everything works:

```bash
# Capture a still image
libcamera-still -o test.jpg --width 640 --height 480

# View the file
ls -lh test.jpg
```

**If successful**, you'll have a `test.jpg` file. Transfer it to your computer to view:

```bash
# On your computer:
scp pi@raspberrypi.local:~/test.jpg ./
```

---

## Test Inference

Now let's run your model on a test image:

```python
# test_deployment.py
from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500
import numpy as np
import time

print("Initializing camera and model...")

# Initialize camera
picam2 = Picamera2()

# Load IMX500 model
imx500 = IMX500(picam2)
imx500.load_network("/home/pi/models/detector_imx500/network.rpk")

# Configure camera
config = picam2.create_still_configuration()
picam2.configure(config)
picam2.start()

print("✓ Camera initialized")
print("✓ Model loaded")

# Capture and process
print("\nCapturing image...")
start_time = time.time()

image = picam2.capture_array()
print(f"✓ Image captured: {image.shape}")

# Run inference
results = imx500.get_inference_result()

inference_time = (time.time() - start_time) * 1000
print(f"✓ Inference complete in {inference_time:.1f} ms")

# Print detections
if results and len(results['boxes']) > 0:
    print(f"\nDetected {len(results['boxes'])} objects:")
    for i, (box, label, score) in enumerate(zip(
        results['boxes'],
        results['labels'],
        results['scores']
    )):
        if score > 0.5:  # Confidence threshold
            print(f"  {i+1}. Class {label}: {score:.2f} - {box}")
else:
    print("\nNo objects detected")

picam2.stop()
print("\n✓ Test complete!")
```

**Run it:**
```bash
python3 test_deployment.py
```

**Expected output:**
```
Initializing camera and model...
✓ Camera initialized
✓ Model loaded

Capturing image...
✓ Image captured: (480, 640, 3)
✓ Inference complete in 45.3 ms

Detected 2 objects:
  1. Class 1: 0.92 - [120, 80, 340, 420]
  2. Class 2: 0.78 - [400, 150, 580, 380]

✓ Test complete!
```

---

## Performance Expectations

On Raspberry Pi 5 with AI Camera:

| Metric | Value | Notes |
|--------|-------|-------|
| **Inference speed** | 10-30 FPS | Depends on model complexity |
| **Latency** | 30-100ms | Per frame processing |
| **Power consumption** | ~5W | Total system power |
| **Accuracy** | Same as validation | Should match your test set results |
| **Startup time** | 2-3 seconds | Model loading time |

**Raspberry Pi 4 will be slightly slower** (~20-50% depending on model).

---

## Try It Yourself

1. **Test different lighting** - Capture images in bright, dim, and mixed lighting
2. **Measure FPS** - Modify script to process 100 frames and calculate average FPS
3. **Compare models** - Deploy different model sizes and compare speed vs accuracy

**Challenge:** Create a script that automatically takes a photo when it detects your target object.

---

## Common Issues

**Problem**: "imx500-tools: command not found"
**Solution**: Update OS: `sudo apt update && sudo apt full-upgrade`, then reinstall
**Why**: IMX500 support is recent, requires latest Raspberry Pi OS (Bookworm or newer)

**Problem**: "Conversion failed: unsupported operator"
**Solution**: Some PyTorch operations aren't supported by IMX500. Simplify your model
**Why**: Edge hardware has limited operator support compared to full PyTorch

**Problem**: "Camera not detected"
**Solution**: Check cable connection, run `sudo raspi-config` and enable camera, reboot
**Why**: Camera interface needs to be enabled in system configuration

**Problem**: Model loads but gets no detections
**Solution**: Check class mappings, verify confidence threshold isn't too high, test on training images first
**Why**: Mismatched class indices or overly strict filtering

**Problem**: "Permission denied" when accessing camera
**Solution**: Add user to video group: `sudo usermod -a -G video $USER`, then logout/login
**Why**: Camera devices require video group membership

**Problem**: Very slow inference (>500ms)
**Solution**: Verify model was converted to .rpk format, check you're using IMX500 not CPU inference
**Why**: ONNX model runs on CPU, .rpk runs on dedicated neural processor

---

## Verify Deployment Checklist

Before moving to the next lesson:

- [ ] Model file transferred to Pi
- [ ] IMX500 tools installed successfully
- [ ] Model converted to .rpk format
- [ ] Camera detected by system
- [ ] Test photo captured successfully
- [ ] Test inference script runs without errors
- [ ] Getting reasonable detection results
- [ ] Inference time <100ms

---

## Summary

You've learned:
- Multiple methods to transfer models to Raspberry Pi
- Installing and using IMX500 tools
- Converting ONNX models for edge deployment with hardware acceleration
- Verifying camera connection and configuration
- Running test inference on the AI Camera
- Expected performance metrics and troubleshooting

Your model is now running on the edge! In the next lesson, you'll build a complete real-time detection application.

---

## Next Lesson

In the next lesson, you'll build a real-time object detection application with live camera feed, bounding box visualization, and performance monitoring.

---
