---
layout: lesson
title: Converting Models to ONNX Format
author: Kevin McAleer
type: page
cover: /learn/object_model/assets/cover.jpg
date: 2025-12-04
previous: 03_training.html
next: 05_deployment.html
description: Convert your trained PyTorch model to ONNX format for deployment on the
  Raspberry Pi AI Camera
percent: 55
duration: 7
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

Your PyTorch model works great on your computer, but the Raspberry Pi AI Camera needs a different format. In this lesson, you'll convert your model to ONNX (Open Neural Network Exchange), a universal format that works across different hardware and frameworks. This is the crucial step before deploying to the Pi.

---

## Objectives

By the end of this lesson, you will be able to:

- Understand what ONNX is and why it matters
- Convert PyTorch models to ONNX format
- Verify ONNX model correctness
- Prepare models for IMX500 hardware
- Troubleshoot conversion issues

---

## Why ONNX?

ONNX is like a universal translator for neural networks:

**The Problem:**
- PyTorch models only work with PyTorch
- Raspberry Pi uses specialized hardware (IMX500 sensor)
- Direct PyTorch on Pi is slow and inefficient

**The Solution:**
ONNX converts your model to a format that:
- Works on any hardware (Pi, mobile, web, embedded devices)
- Runs faster with hardware optimizations
- Reduces file size for edge deployment
- Maintains compatibility across frameworks

**Real-world analogy:**
Like exporting a document from Word to PDF - anyone can open it, regardless of what software they use.

---

## Install ONNX Tools

First, install the required packages:

```bash
# Install ONNX and verification tools
pip install onnx onnxruntime
```

**Verify installation:**

```python
import onnx
import onnxruntime as ort

print(f"✓ ONNX version: {onnx.__version__}")
print(f"✓ ONNX Runtime version: {ort.__version__}")
```

---

## Convert Your Model to ONNX

Here's the complete conversion process:

```python
import torch
import torch.onnx
from torchvision.models.detection import fasterrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

# ===== Load Your Trained Model =====
print("Loading trained model...")

NUM_CLASSES = 4  # Adjust to your number of classes

# Create model architecture (same as training)
model = fasterrcnn_resnet50_fpn(pretrained=False)
in_features = model.roi_heads.box_predictor.cls_score.in_features
model.roi_heads.box_predictor = FastRCNNPredictor(in_features, NUM_CLASSES)

# Load trained weights
model.load_state_dict(torch.load('custom_detector.pth'))
model.eval()  # Important: set to evaluation mode

print("✓ Model loaded successfully")

# ===== Create Dummy Input =====
# ONNX needs an example input to trace the model
dummy_input = torch.randn(1, 3, 640, 640)  # Batch=1, RGB, 640x640

print(f"✓ Dummy input shape: {dummy_input.shape}")

# ===== Export to ONNX =====
print("\nExporting to ONNX format...")

output_file = "custom_detector.onnx"

torch.onnx.export(
    model,                      # Model to export
    dummy_input,                # Example input
    output_file,                # Output filename
    export_params=True,         # Store trained weights
    opset_version=11,           # ONNX version (11 is widely supported)
    do_constant_folding=True,   # Optimize constant operations
    input_names=['input'],      # Input tensor name
    output_names=['boxes', 'labels', 'scores'],  # Output tensor names
    dynamic_axes={              # Allow variable batch sizes
        'input': {0: 'batch_size'},
        'boxes': {0: 'batch_size'},
        'labels': {0: 'batch_size'},
        'scores': {0: 'batch_size'}
    }
)

print(f"✓ Model exported to {output_file}")

# ===== Check File Size =====
import os
file_size_mb = os.path.getsize(output_file) / (1024 * 1024)
print(f"✓ File size: {file_size_mb:.2f} MB")
```

**Expected output:**
```
Loading trained model...
✓ Model loaded successfully
✓ Dummy input shape: torch.Size([1, 3, 640, 640])

Exporting to ONNX format...
✓ Model exported to custom_detector.onnx
✓ File size: 45.32 MB
```

---

## Verify Your ONNX Model

Always verify the conversion worked correctly:

```python
import onnx
import onnxruntime as ort
import numpy as np

# ===== Load and Check ONNX Model =====
print("Verifying ONNX model...")

onnx_model = onnx.load("custom_detector.onnx")

# Check model structure is valid
try:
    onnx.checker.check_model(onnx_model)
    print("✓ ONNX model is valid!")
except Exception as e:
    print(f"✗ Model validation failed: {e}")

# ===== Print Model Info =====
print("\nModel Information:")
print(f"  Inputs: {[input.name for input in onnx_model.graph.input]}")
print(f"  Outputs: {[output.name for output in onnx_model.graph.output]}")

# ===== Test Inference with ONNX Runtime =====
print("\nTesting inference...")

# Create ONNX Runtime session
ort_session = ort.InferenceSession("custom_detector.onnx")

# Create test input
test_input = np.random.randn(1, 3, 640, 640).astype(np.float32)

# Run inference
outputs = ort_session.run(None, {'input': test_input})

print("✓ Inference successful!")
print(f"  Output shapes: {[out.shape for out in outputs]}")
```

**Expected output:**
```
Verifying ONNX model...
✓ ONNX model is valid!

Model Information:
  Inputs: ['input']
  Outputs: ['boxes', 'labels', 'scores']

Testing inference...
✓ Inference successful!
  Output shapes: [(100, 4), (100,), (100,)]
```

---

## Preparing for IMX500 Deployment

The Raspberry Pi AI Camera uses the Sony IMX500 sensor, which requires additional conversion. You'll do this step on the Raspberry Pi itself.

**On your Raspberry Pi, install IMX500 tools:**

```bash
# Update package list
sudo apt update

# Install IMX500 tools
sudo apt install -y imx500-tools python3-imx500-tools

# Verify installation
imx500-package.sh --version
```

**Convert ONNX to IMX500 format:**

```bash
# Transfer your ONNX file to the Pi first
# (Use scp, USB drive, or any file transfer method)

# On the Raspberry Pi:
cd ~/models
imx500-package.sh -i custom_detector.onnx -o custom_detector_imx500

# This creates a .rpk file optimized for IMX500
```

**What this does:**
- Optimizes model for IMX500 hardware
- Converts to fixed-point precision (smaller, faster)
- Packages with metadata for the camera
- Creates `.rpk` (Raspberry Pi Package) file

---

## Try It Yourself

1. **Convert with different input sizes** - Try 320x320 or 800x800 instead of 640x640
2. **Check model compatibility** - Run `onnx.checker.check_model()` on your exported model
3. **Compare file sizes** - Export with and without `do_constant_folding` and compare sizes

**Challenge:** Write a script that automatically converts PyTorch → ONNX → IMX500 format in one command.

---

## Common Issues

**Problem**: "RuntimeError: ONNX export failed"
**Solution**: Make sure model is in eval mode: `model.eval()` before export
**Why**: Training mode has operations (dropout, batch norm) that aren't compatible with ONNX

**Problem**: "OpsetVersion not supported"
**Solution**: Try `opset_version=10` or `opset_version=12`
**Why**: Different hardware supports different ONNX opset versions

**Problem**: ONNX file is very large (>100MB)
**Solution**: Use model quantization or pruning techniques
**Why**: Full precision models are large; edge devices benefit from compression

**Problem**: "Input/output names don't match"
**Solution**: Check your `input_names` and `output_names` match your model's actual tensors
**Why**: ONNX needs explicit tensor naming for compatibility

**Problem**: Inference results differ between PyTorch and ONNX
**Solution**: Check numerical precision settings and use `atol=1e-3` for comparisons
**Why**: Minor numerical differences are normal due to optimization

**Problem**: "imx500-tools: command not found" on Pi
**Solution**: Ensure you're on latest Raspberry Pi OS: `sudo apt update && sudo apt upgrade`
**Why**: IMX500 support is recent, needs updated OS

---

## Optimization Tips

### Reduce Model Size

```python
# Option 1: Quantize to INT8 (4x smaller, slightly less accurate)
import torch.quantization

model_quantized = torch.quantization.quantize_dynamic(
    model, {torch.nn.Linear}, dtype=torch.qint8
)

# Then export quantized model
torch.onnx.export(model_quantized, dummy_input, "detector_quantized.onnx")
```

### Test Inference Speed

```python
import time

# Warm up
for _ in range(10):
    outputs = ort_session.run(None, {'input': test_input})

# Benchmark
start_time = time.time()
num_runs = 100

for _ in range(num_runs):
    outputs = ort_session.run(None, {'input': test_input})

avg_time = (time.time() - start_time) / num_runs
print(f"Average inference time: {avg_time*1000:.2f} ms")
print(f"FPS: {1/avg_time:.1f}")
```

---

## Summary

You've learned:
- What ONNX is and why it's essential for deployment
- Converting PyTorch models to ONNX format
- Verifying model correctness after conversion
- Preparing models for IMX500 hardware
- Troubleshooting common conversion issues
- Optimizing model size and speed

Your model is now ready for deployment! In the next lesson, you'll transfer it to the Raspberry Pi and run it on the AI Camera.

---

## Next Lesson

In the next lesson, you'll deploy your ONNX model to the Raspberry Pi AI Camera and run your first inference on the edge device.

---