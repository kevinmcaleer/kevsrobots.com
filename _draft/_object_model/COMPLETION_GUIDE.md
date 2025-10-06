# Object Detection Course Completion Guide

This guide provides templates and detailed instructions for completing the Raspberry Pi AI Camera Object Detection course.

---

## Current Status

✅ **Completed:**
- `00_intro.md` - Fully rewritten with proper structure
- File renaming and numbering structure

🚧 **In Progress:**
- `01_setup.md` - Needs improvement
- `02_dataset_prep.md` - Needs improvement
- `03_training.md` - Needs improvement
- `04_onnx_conversion.md` - Needs improvement

❌ **To Create:**
- `05_deployment.md` - Deploy to Raspberry Pi
- `06_realtime_detection.md` - Real-time object detection
- `07_troubleshooting.md` - Common issues and optimization
- `08_summary.md` - Wrap up and next steps

---

## Lesson Improvement Checklist

For each existing lesson (01-04), add the following:

### 1. Real-World Context
- [ ] Add "Why This Matters" section explaining real-world use
- [ ] Include specific example use cases
- [ ] Show how this step fits into the bigger picture

### 2. Hands-On Code Examples
- [ ] Add complete, runnable code snippets
- [ ] Include expected output
- [ ] Add comments explaining what each section does

### 3. Try It Yourself Section
```markdown
## Try It Yourself

1. **Modify the code to...** - Specific challenge
2. **Experiment with...** - Exploration task
3. **Challenge:** - Advanced task for confident learners

<details>
<summary>💡 Hint</summary>
Helpful hint without giving away the answer
</details>
```

### 4. Common Issues Section
```markdown
## Common Issues

**Problem**: Specific error message or issue
**Solution**: Step-by-step fix
**Why it happens**: Explanation of root cause
```

### 5. Summary and Next Steps
```markdown
## Summary

You've learned:
- Key point 1
- Key point 2
- Key point 3

## Next Lesson

In the next lesson, you'll learn [preview of next topic].
```

---

## Lesson 01: Setup - Improvement Template

### Add These Sections:

#### Real-World Context
```markdown
## Why Jupyter Lab?

Jupyter Lab provides an interactive environment perfect for:
- **Iterative development** - Test code snippets quickly
- **Visualization** - See your dataset and results inline
- **Documentation** - Mix code, notes, and images
- **Reproducibility** - Share notebooks with your team

Professional ML engineers use Jupyter for prototyping before deploying to production.
```

#### Practical Example - Verify Installation
```markdown
## Verify Your Setup

Let's confirm everything is installed correctly:

```python
# test_setup.py
import sys
import jupyterlab
import ipykernel

print("✓ Python version:", sys.version)
print("✓ Jupyter Lab:", jupyterlab.__version__)
print("✓ IPyKernel:", ipykernel.__version__)
print("\nSetup successful! You're ready to go.")
```

**Expected output:**
```
✓ Python version: 3.10.12
✓ Jupyter Lab: 4.0.0
✓ IPyKernel: 6.25.0

Setup successful! You're ready to go.
```
```

#### Try It Yourself
```markdown
## Try It Yourself

1. **Create a new notebook** - Open Jupyter Lab and create a notebook named `test.ipynb`
2. **Run a cell** - Type `print("Hello, AI Camera!")` and press Shift+Enter
3. **Install a package** - Try installing matplotlib: `!pip install matplotlib`

**Challenge:** Create a simple plot to test matplotlib is working:
```python
import matplotlib.pyplot as plt
plt.plot([1, 2, 3, 4])
plt.ylabel('Sample numbers')
plt.show()
```
```

---

## Lesson 02: Dataset Prep - Improvement Template

### Add These Sections:

#### Real-World Dataset Examples
```markdown
## Example Datasets

Here are some real-world examples:

| Use Case | Objects to Detect | # Images Needed |
|----------|------------------|-----------------|
| Smart doorbell | person, package, pet | 200-500 each |
| Warehouse robot | box, pallet, forklift | 300-600 each |
| Plant monitor | healthy_leaf, diseased_leaf, pest | 400-800 each |

**Rule of thumb:** Start with 200-300 images per class, add more if accuracy is low.
```

#### Practical Annotation Example
```markdown
## Hands-On: Annotate Your First Image

Let's annotate an image of a coffee mug:

1. **Open LabelImg:**
```bash
labelImg
```

2. **Load your image folder** - Click "Open Dir"

3. **Draw a bounding box:**
   - Click "Create RectBox" (or press 'W')
   - Click and drag around the object
   - Label it (e.g., "mug")
   - Press Ctrl+S to save

4. **Check the annotation file:**
```xml
<annotation>
  <object>
    <name>mug</name>
    <bndbox>
      <xmin>120</xmin>
      <ymin>80</ymin>
      <xmax>340</xmax>
      <ymax>420</ymax>
    </bndbox>
  </object>
</annotation>
```

**Your first annotation is complete!** 🎉
```

#### Common Issues
```markdown
## Common Issues

**Problem**: "ModuleNotFoundError: No module named 'PyQt5'"
**Solution**: Install PyQt5: `pip install PyQt5`
**Why**: LabelImg requires PyQt5 for its GUI

**Problem**: Annotations saved in wrong format
**Solution**: In LabelImg, click "View" → "Auto Save Mode" and select "PascalVOC"
**Why**: Different formats exist (YOLO, COCO, etc.). We use PascalVOC for this course.

**Problem**: Can't see bounding boxes clearly
**Solution**: Use 'Ctrl+Plus' to zoom in, adjust box by clicking and dragging corners
**Why**: Precision matters - boxes should tightly fit objects
```

---

## Lesson 03: Training - Improvement Template

### Add Complete Training Script
```markdown
## Complete Training Code

Here's a complete, runnable training script:

```python
import torch
import torchvision
from torchvision.models.detection import fasterrcnn_resnet50_fpn
from torch.utils.data import DataLoader
import time

# 1. Load pre-trained model
model = fasterrcnn_resnet50_fpn(pretrained=True)

# 2. Modify for your number of classes (e.g., 3 classes + background)
num_classes = 4
in_features = model.roi_heads.box_predictor.cls_score.in_features
model.roi_heads.box_predictor = torchvision.models.detection.faster_rcnn.FastRCNNPredictor(
    in_features, num_classes
)

# 3. Setup optimizer
params = [p for p in model.parameters() if p.requires_grad]
optimizer = torch.optim.SGD(params, lr=0.005, momentum=0.9, weight_decay=0.0005)

# 4. Training loop
model.train()
num_epochs = 50

print(f"Starting training for {num_epochs} epochs...")
start_time = time.time()

for epoch in range(num_epochs):
    epoch_loss = 0

    for images, targets in train_loader:
        # Forward pass
        loss_dict = model(images, targets)
        losses = sum(loss for loss in loss_dict.values())

        # Backward pass
        optimizer.zero_grad()
        losses.backward()
        optimizer.step()

        epoch_loss += losses.item()

    # Print progress
    avg_loss = epoch_loss / len(train_loader)
    elapsed = time.time() - start_time
    print(f"Epoch {epoch+1}/{num_epochs} - Loss: {avg_loss:.4f} - Time: {elapsed/60:.1f}min")

# Save the model
torch.save(model.state_dict(), 'custom_detector.pth')
print(f"\n✓ Training complete! Model saved to 'custom_detector.pth'")
```

**Expected training time:**
- MacBook Pro M1 (GPU): ~1 hour for 50 epochs
- Mac/PC (CPU only): ~3-4 hours for 50 epochs
- Linux with NVIDIA GPU: ~30-45 minutes for 50 epochs
```

#### Visualize Training Progress
```markdown
## Monitor Your Training

Add this code to visualize loss over time:

```python
import matplotlib.pyplot as plt

# Track losses
losses_per_epoch = []

# In your training loop, after each epoch:
losses_per_epoch.append(avg_loss)

# Plot the loss curve
plt.figure(figsize=(10, 5))
plt.plot(losses_per_epoch)
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.title('Training Loss Over Time')
plt.grid(True)
plt.savefig('training_loss.png')
plt.show()
```

**What to look for:**
- Loss should **decrease** over time
- If loss stays flat: learning rate might be too low
- If loss jumps around: learning rate might be too high
- Ideal: Smooth downward curve
```

---

## Lesson 04: ONNX Conversion - Improvement Template

### Add Complete Conversion Example
```markdown
## Convert Your Model to ONNX

Here's the complete conversion process:

```python
import torch
import torch.onnx

# 1. Load your trained model
model = fasterrcnn_resnet50_fpn(pretrained=False, num_classes=4)
model.load_state_dict(torch.load('custom_detector.pth'))
model.eval()

# 2. Create dummy input (example image)
dummy_input = torch.randn(1, 3, 640, 640)

# 3. Export to ONNX
torch.onnx.export(
    model,
    dummy_input,
    "detector.onnx",
    export_params=True,
    opset_version=11,
    do_constant_folding=True,
    input_names=['input'],
    output_names=['boxes', 'labels', 'scores'],
    dynamic_axes={
        'input': {0: 'batch_size'},
        'boxes': {0: 'batch_size'},
        'labels': {0: 'batch_size'},
        'scores': {0: 'batch_size'}
    }
)

print("✓ Model exported to detector.onnx")
print(f"File size: {os.path.getsize('detector.onnx') / 1024 / 1024:.2f} MB")
```

**What just happened:**
- Converted PyTorch model to ONNX format
- ONNX is platform-independent (works on Pi, mobile, web)
- Dynamic axes allow variable batch sizes
```

#### Verify ONNX Model
```markdown
## Verify Your ONNX Model

Check the model is valid:

```python
import onnx

# Load and check ONNX model
onnx_model = onnx.load("detector.onnx")
onnx.checker.check_model(onnx_model)
print("✓ ONNX model is valid!")

# Print model info
print(f"\nInputs: {[input.name for input in onnx_model.graph.input]}")
print(f"Outputs: {[output.name for output in onnx_model.graph.output]}")
```

**Common Issues:**

**Problem**: "RuntimeError: ONNX export failed"
**Solution**: Make sure model is in eval mode: `model.eval()`
**Why**: Training mode has operations not supported by ONNX

**Problem**: Large file size (>100MB)
**Solution**: Use model compression or quantization
**Why**: Full precision models are large, quantization reduces size 4x
```

---

## New Lesson 05: Deployment Template

Create file: `05_deployment.md`

```markdown
---
title: Deploying to Raspberry Pi AI Camera
description: >-
    Learn how to deploy your trained object detection model to the Raspberry Pi AI Camera for edge inference
type: page
layout: lesson
---

## Overview

Now that you have a trained model in ONNX format, let's deploy it to the Raspberry Pi AI Camera! This lesson covers transferring your model, installing dependencies, and running your first inference on the Pi.

---

## Objectives

By the end of this lesson, you will:

- Transfer your ONNX model to the Raspberry Pi
- Install IMX500 tools and dependencies
- Convert the model for IMX500 hardware
- Run a test inference on a sample image
- Understand the deployment workflow

---

## Prerequisites

Before starting:
- [ ] Raspberry Pi 5 or 4 with Raspberry Pi OS installed
- [ ] Raspberry Pi AI Camera connected
- [ ] Your ONNX model file from Lesson 04
- [ ] SSH access to your Pi (or keyboard/monitor)

---

## Transfer Your Model

### Option 1: Using SCP (from your computer)

```bash
# Replace 'raspberrypi.local' with your Pi's IP address
scp detector.onnx pi@raspberrypi.local:~/models/
```

### Option 2: Using USB Drive

1. Copy `detector.onnx` to USB drive
2. Insert into Pi
3. Copy to home directory:
```bash
cp /media/pi/USB_DRIVE/detector.onnx ~/models/
```

---

## Install IMX500 Tools

On your Raspberry Pi, install the required tools:

```bash
# Update package list
sudo apt update

# Install IMX500 tools
sudo apt install -y imx500-tools python3-imx500-tools

# Verify installation
imx500-package.sh --version
```

---

## Convert Model for IMX500

The IMX500 sensor requires a specific format:

```bash
# Navigate to your model directory
cd ~/models

# Convert ONNX to IMX500 format
imx500-package.sh -i detector.onnx -o detector_imx500

# This creates a .rpk file ready for the camera
```

**What's happening:**
- Model is optimized for IMX500 hardware
- Converted to fixed-point precision
- Packaged with metadata for the camera

**Expected output:**
```
Processing detector.onnx...
✓ Conversion successful
✓ Created detector_imx500/network.rpk
Size: 12.3 MB
```

---

## Test Inference

Let's test your model on a sample image:

```python
# test_deployment.py
from picamera2 import Picamera2
import numpy as np

# Initialize camera
picam2 = Picamera2()
config = picam2.create_still_configuration()
picam2.configure(config)

# Load model
model_path = "/home/pi/models/detector_imx500/network.rpk"

# Capture and process
print("Capturing image...")
image = picam2.capture_array()

# Run inference (simplified - actual implementation varies)
# results = model.detect(image)

print("✓ Deployment successful!")
print(f"Image shape: {image.shape}")
```

---

## Verify Camera Connection

Make sure your AI Camera is working:

```bash
# List available cameras
libcamera-still --list-cameras

# Take a test photo
libcamera-still -o test.jpg

# View available IMX500 models
ls /usr/share/imx500-models/
```

**Expected output:**
```
Available cameras:
0 : imx500 [4056x3040]
```

---

## Common Issues

**Problem**: "imx500-tools: command not found"
**Solution**: Make sure you're on latest Raspberry Pi OS: `sudo apt update && sudo apt upgrade`
**Why**: IMX500 support is new, needs recent OS version

**Problem**: "Conversion failed: unsupported operator"
**Solution**: Some PyTorch operations aren't supported. Try simplifying your model
**Why**: Edge hardware has limited operator support

**Problem**: "Camera not detected"
**Solution**: Check ribbon cable connection, run `sudo raspi-config` and enable camera
**Why**: Camera needs to be enabled in config

---

## Performance Expectations

On Raspberry Pi 5 with AI Camera:
- **Inference speed**: 10-30 FPS depending on model complexity
- **Latency**: 30-100ms per frame
- **Power**: ~5W total system power
- **Accuracy**: Should match your validation set results

---

## Try It Yourself

1. **Deploy multiple models** - Convert different models and compare performance
2. **Test on different images** - Try various lighting conditions and angles
3. **Measure inference time** - Use `time` command to measure speed

**Challenge:** Modify the test script to display bounding boxes on detected objects

---

## Summary

You've learned:
- How to transfer models to Raspberry Pi
- Installing and using IMX500 tools
- Converting ONNX models for edge deployment
- Running test inference on the AI Camera

## Next Lesson

In the next lesson, you'll build a real-time object detection application with live camera feed and visual feedback!

---
```

---

## New Lesson 06: Real-Time Detection Template

Create file: `06_realtime_detection.md`

```markdown
---
title: Real-Time Object Detection
description: >-
    Build a real-time object detection application using your custom model and the Raspberry Pi AI Camera
type: page
layout: lesson
---

## Overview

Time to bring everything together! In this lesson, you'll build a complete real-time object detection system that processes live camera feed, detects objects, and displays results with bounding boxes.

---

## Objectives

- Process live camera feed in real-time
- Display detected objects with bounding boxes
- Show class labels and confidence scores
- Optimize for smooth performance
- Save detection results

---

## Complete Real-Time Detection Script

Here's a full working example:

```python
# realtime_detector.py
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import cv2
import numpy as np
import time

class RealtimeDetector:
    def __init__(self, model_path):
        self.picam2 = Picamera2()
        self.model_path = model_path
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()

        # Configure camera
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def draw_detection(self, image, box, label, confidence):
        """Draw bounding box and label on image"""
        x1, y1, x2, y2 = box

        # Draw rectangle
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw label background
        label_text = f"{label}: {confidence:.2f}"
        (text_width, text_height), _ = cv2.getTextSize(
            label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
        )
        cv2.rectangle(
            image, (x1, y1 - text_height - 10),
            (x1 + text_width, y1), (0, 255, 0), -1
        )

        # Draw label text
        cv2.putText(
            image, label_text, (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2
        )

    def calculate_fps(self):
        """Calculate frames per second"""
        self.frame_count += 1
        elapsed = time.time() - self.start_time

        if elapsed > 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.start_time = time.time()

        return self.fps

    def run(self):
        """Main detection loop"""
        print("Starting real-time detection... Press 'q' to quit")

        try:
            while True:
                # Capture frame
                frame = self.picam2.capture_array()

                # TODO: Run your model inference here
                # detections = self.model.detect(frame)

                # Example detection (replace with actual model output)
                # detections = [
                #     {'box': (100, 100, 200, 200), 'label': 'mug', 'confidence': 0.95},
                # ]

                # Draw detections
                # for det in detections:
                #     self.draw_detection(frame, det['box'], det['label'], det['confidence'])

                # Calculate and display FPS
                fps = self.calculate_fps()
                cv2.putText(
                    frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2
                )

                # Display frame
                cv2.imshow('Object Detection', frame)

                # Check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.picam2.stop()
            cv2.destroyAllWindows()
            print(f"\nAverage FPS: {fps:.1f}")

# Run detector
if __name__ == "__main__":
    detector = RealtimeDetector("/home/pi/models/detector_imx500/network.rpk")
    detector.run()
```

**Run it:**
```bash
python3 realtime_detector.py
```

---

## Performance Optimization Tips

### 1. Reduce Resolution
```python
# Use smaller image size for faster inference
config = picam2.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}  # Smaller = faster
)
```

### 2. Skip Frames
```python
# Process every Nth frame
frame_skip = 2
if frame_count % frame_skip == 0:
    detections = model.detect(frame)
```

### 3. Use Threading
```python
from threading import Thread

# Run capture in separate thread
capture_thread = Thread(target=capture_frames)
detection_thread = Thread(target=detect_objects)
```

---

## Save Detections

Add this to save important detections:

```python
def save_detection(image, label, confidence):
    """Save high-confidence detections"""
    if confidence > 0.9:  # Only save if very confident
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"detection_{label}_{timestamp}.jpg"
        cv2.imwrite(filename, image)
        print(f"✓ Saved {filename}")
```

---

## Add Audio Alerts

Make your detector speak:

```python
import os

def alert(label):
    """Play audio alert when object detected"""
    os.system(f'espeak "{label} detected"')
```

---

## Try It Yourself

1. **Add confidence threshold** - Only show detections above 70% confidence
2. **Count objects** - Display total number of each object type
3. **Record video** - Save video clips when objects are detected
4. **Remote monitoring** - Stream video over network using Flask

**Challenge:** Create a "tripwire" that alerts when an object enters a specific region of the frame

---

## Common Issues

**Problem**: Low FPS (<5 FPS)
**Solution**: Reduce image resolution, skip frames, or simplify model
**Why**: Real-time needs 10+ FPS for smooth experience

**Problem**: Detections flickering
**Solution**: Add temporal smoothing - only show consistent detections
**Why**: Model confidence varies frame-to-frame

**Problem**: High false positive rate
**Solution**: Increase confidence threshold, retrain with more data
**Why**: Model needs more examples to be certain

---

## Summary

You've built a complete real-time object detection system!

You learned:
- Processing live camera streams
- Drawing bounding boxes and labels
- Calculating and displaying FPS
- Optimizing for real-time performance
- Saving detections

## Next Lesson

In the next lesson, you'll learn troubleshooting techniques and advanced optimization strategies.

---
```

---

## Lesson 07 & 08 - Quick Templates

### Lesson 07: Troubleshooting (create `07_troubleshooting.md`)
**Sections needed:**
- Model not detecting objects
- False positives
- Performance issues
- Memory errors
- Camera connection problems
- Each with Problem/Solution/Why structure

### Lesson 08: Summary (create `08_summary.md`)
**Sections needed:**
- Course recap
- What you've accomplished
- Real-world project ideas
- Next steps and advanced topics
- Resources for continued learning
- Community links

---

## Updated course.yml Structure

```yaml
- name: Building Object Detection Models with Raspberry Pi AI Camera
  author: Kevin McAleer
  date_created: 2024-10-16
  date_published: 2024-10-16
  layout: course
  cover: assets/cover.jpg
  groups:
      - python
      - ai
      - raspberry_pi
      - computer_vision
  description: >-
      Learn how to build custom object detection models using the Raspberry Pi AI Camera.
      Train your own models with PyTorch, optimize them for edge deployment, and run
      real-time detection on Raspberry Pi hardware.
  content:
  - section:
      name: Getting Started
      content:
      - 00_intro.md
      - 01_setup.md
  - section:
      name: Dataset Preparation
      content:
      - 02_dataset_prep.md
  - section:
      name: Model Training
      content:
      - 03_training.md
      - 04_onnx_conversion.md
  - section:
      name: Deployment
      content:
      - 05_deployment.md
      - 06_realtime_detection.md
  - section:
      name: Advanced Topics
      content:
      - 07_troubleshooting.md
      - 08_summary.md
```

---

## Next Steps

1. **Review and improve lessons 01-04** using the templates above
2. **Create lessons 05-08** using the provided templates
3. **Add images** to `assets/` folder (cover.jpg, screenshots, diagrams)
4. **Update course.yml** with the structure above
5. **Test build**: Run `python build.py` from repo root
6. **Move to source**: `mv _draft/_object_model source/object_model`
7. **Deploy**: Build and test on local Jekyll

---

## Quality Checklist

Before moving to source/:

- [ ] All lessons have frontmatter (title, description, layout, type)
- [ ] Every lesson has real code examples that run
- [ ] "Try It Yourself" sections in every lesson
- [ ] "Common Issues" sections for technical lessons
- [ ] Summary section at end of each lesson
- [ ] Images optimized (<300KB each)
- [ ] course.yml matches actual lesson files
- [ ] Code tested and works
- [ ] Links to prerequisite courses work
- [ ] Progression makes sense (beginner → advanced)

---

## Estimated Completion Time

- Improve lessons 01-04: **2-3 hours**
- Create lessons 05-06: **2-3 hours**
- Create lessons 07-08: **1-2 hours**
- Add images and test: **1 hour**
- **Total: 6-9 hours**

---

Good luck completing the course! This will be a valuable resource for the maker community. 🚀
