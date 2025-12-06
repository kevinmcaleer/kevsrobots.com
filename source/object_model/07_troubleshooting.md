---
title: Troubleshooting and Optimization
description: >-
    Master troubleshooting techniques and optimization strategies for production-ready object detection systems
type: page
layout: lesson
---

## Overview

Even the best object detection systems encounter issues in real-world deployment. This lesson equips you with systematic troubleshooting techniques, performance optimization strategies, and solutions to common problems you'll face when deploying models to production.

---

## Objectives

By the end of this lesson, you will:

- Diagnose and fix model accuracy issues
- Resolve deployment and hardware problems
- Optimize inference speed and resource usage
- Handle edge cases and challenging conditions
- Implement robust error handling
- Monitor and maintain production systems

---

## Problem Category 1: Model Not Detecting Objects

### Symptom: Model runs but finds nothing

**Diagnosis Steps:**

1. **Test on training images first:**

    ```python
    # Load a known training image
    test_image = cv2.imread('custom_dataset/train/images/img_001.jpg')
    results = model.detect(test_image)

    if len(results) == 0:
        print("Model doesn't detect training images - model issue")
    else:
        print("Model works on training data - deployment issue")
    ```

2. **Check confidence threshold:**

    ```python
    # Try very low threshold
    results = model.detect(image, confidence_threshold=0.1)

    if results:
        print(f"Got detections at low threshold: {[r['score'] for r in results]}")
        print("Issue: Threshold too high for your use case")
    ```

3. **Verify class mappings:**

    ```python
    # Print model's expected classes
    print("Model classes:", model.class_names)

    # Check they match your dataset
    print("Dataset classes:", dataset.class_names)

    if model.class_names != dataset.class_names:
        print("ERROR: Class mismatch!")
    ```

---

**Common Causes & Solutions:**

| Cause | Solution | Why |
|-------|----------|-----|
| Model undertrained | Train for more epochs (100+) | Needs more iterations to learn features |
| Dataset too small | Collect 2-3x more images | Model needs diverse examples |
| Wrong input size | Verify image preprocessing matches training | Size mismatch breaks learned features |
| Class mapping wrong | Fix label indices in deployment code | Model outputs indices, not names |
| Lighting mismatch | Collect data in target environment | Model trained on different conditions |
{:class="table table-single"}

---

## Problem Category 2: High False Positive Rate

### Symptom: Model detects objects that aren't there

**Diagnosis:**

```python
# Analyze false positives
false_positives = []

for detection in results:
    if detection['score'] < 0.8:  # Likely false positive
        false_positives.append(detection)

print(f"False positives: {len(false_positives)}/{len(results)}")
print("Common classes:", [d['label'] for d in false_positives])
```

**Solutions:**

1. **Increase confidence threshold:**
    ```python
    # Be more selective
    results = model.detect(image, confidence_threshold=0.7)  # Up from 0.5
    ```

2. **Add negative examples to dataset:**
    ```bash
    # Collect "background" images
    # - Empty scenes
    # - Similar but different objects
    # - Common false positive scenarios

    # Label them with no bounding boxes
    ```

3. **Use non-maximum suppression (NMS):**
    ```python
    def apply_nms(detections, iou_threshold=0.5):
        """Remove overlapping duplicate detections"""
        import torch
        from torchvision.ops import nms

        boxes = torch.tensor([d['box'] for d in detections])
        scores = torch.tensor([d['score'] for d in detections])

        keep_indices = nms(boxes, scores, iou_threshold)

        return [detections[i] for i in keep_indices]
    ```

4. **Filter by size:**
    ```python
    def filter_tiny_boxes(detections, min_area=500):
        """Remove unreasonably small detections"""
        filtered = []
        for det in detections:
            x1, y1, x2, y2 = det['box']
            area = (x2 - x1) * (y2 - y1)
            if area >= min_area:
                filtered.append(det)
        return filtered
    ```

---

## Problem Category 3: Performance Issues

### Symptom: Slow inference, low FPS

**Measure current performance:**

```python
import time
import numpy as np

# Benchmark 100 frames
times = []
for _ in range(100):
    start = time.time()
    frame = picam2.capture_array()
    results = model.detect(frame)
    times.append(time.time() - start)

avg_time = np.mean(times)
fps = 1 / avg_time

print(f"Average inference: {avg_time*1000:.1f} ms")
print(f"FPS: {fps:.1f}")
print(f"Min: {min(times)*1000:.1f} ms, Max: {max(times)*1000:.1f} ms")
```

**Optimization Strategies:**

### 1. Reduce Input Resolution

    ```python
    # Before: 640x480 = 307,200 pixels
    config = picam2.create_preview_configuration(
        main={"size": (640, 480)}
    )

    # After: 320x240 = 76,800 pixels (4x fewer!)
    config = picam2.create_preview_configuration(
        main={"size": (320, 240)}
    )
    # Result: ~2x faster inference
    ```

### 2. Model Quantization

    ```python
    # Convert to INT8 (already done in IMX500 conversion)
    # But verify it worked:

    import os
    model_size_mb = os.path.getsize('model.onnx') / 1024 / 1024
    rpk_size_mb = os.path.getsize('model.rpk') / 1024 / 1024

    print(f"ONNX size: {model_size_mb:.1f} MB")
    print(f"RPK size: {rpk_size_mb:.1f} MB")
    print(f"Compression: {model_size_mb/rpk_size_mb:.1f}x")

    # Should see 3-4x compression
    ```

### 3. Frame Skipping

    ```python
    class FrameSkipper:
        def __init__(self, skip_frames=2):
            self.skip_frames = skip_frames
            self.frame_counter = 0
            self.last_results = []

        def process(self, frame):
            self.frame_counter += 1

            if self.frame_counter % (self.skip_frames + 1) == 0:
                # Run inference on this frame
                self.last_results = model.detect(frame)

            # Return cached results for skipped frames
            return self.last_results

    # Usage:
    skipper = FrameSkipper(skip_frames=2)  # Process every 3rd frame
    while True:
        frame = picam2.capture_array()
        results = skipper.process(frame)  # 3x faster!
    ```

### 4. Optimize Drawing

    ```python
    # Slow: Drawing every box every frame
    for detection in results:
        draw_box(frame, detection)  # Expensive!

    # Fast: Only draw if displaying
    if display_enabled:
        for detection in results:
            draw_box(frame, detection)
    ```

---

## Problem Category 4: Memory Issues

### Symptom: System crashes after hours of operation

**Monitor memory usage:**

```python
import psutil
import gc

def log_memory():
    process = psutil.Process()
    mem_mb = process.memory_info().rss / 1024 / 1024
    print(f"Memory usage: {mem_mb:.1f} MB")

# Call every 100 frames
if frame_counter % 100 == 0:
    log_memory()
    gc.collect()  # Force garbage collection
```

**Solutions:**

1. **Release frames properly:**
    ```python
    # Bad:
    frames = []
    while True:
        frame = picam2.capture_array()
        frames.append(frame)  # Memory leak!

    # Good:
    while True:
        frame = picam2.capture_array()
        results = process(frame)
        # frame is automatically released when out of scope
    ```

2. **Limit saved images:**
    ```python
    import glob

    def cleanup_old_detections(max_files=100):
        """Keep only the most recent N detection images"""
        files = sorted(glob.glob('detection_*.jpg'))
        if len(files) > max_files:
            for f in files[:-max_files]:
                os.remove(f)

    # Call periodically
    if frame_counter % 1000 == 0:
        cleanup_old_detections()
    ```

3. **Restart periodically:**
    ```python
    MAX_FRAMES = 10000

    while frame_counter < MAX_FRAMES:
        # Main loop
        frame_counter += 1

    # Restart process
    print("Restarting for memory cleanup...")
    os.execv(sys.executable, ['python3'] + sys.argv)
    ```

---

## Problem Category 5: Environmental Challenges

### Low Light Conditions

**Problem**: Model fails at night or in dim lighting

**Solutions:**

1. **Camera settings:**
    ```python
    # Increase exposure time
    picam2.set_controls({
        "ExposureTime": 50000,  # Microseconds
        "AnalogueGain": 8.0      # Amplify signal
    })
    ```

2. **Add IR illumination:**
    ```bash
    # Use IR LEDs (invisible to humans, visible to camera)
    # Models trained on visible light work on IR with some accuracy loss
    ```

3. **Collect night training data:**
    ```bash
    # Take 100-200 images in target lighting conditions
    # Retrain or fine-tune model
    ```

### Glare and Reflections

**Problem**: Windows, monitors cause false detections

**Solutions:**

1. **Filter by detection stability:**
    ```python
    class StableDetector:
        def __init__(self, min_frames=3):
            self.min_frames = min_frames
            self.detection_history = []

        def filter(self, detections):
            # Only return detections seen in multiple frames
            stable = []
            for det in detections:
                if self.appears_in_history(det):
                    stable.append(det)
            return stable
    ```

2. **Adjust camera angle:**
    ```bash
    # Mount camera to minimize direct reflections
    # Use polarizing filter if necessary
    ```

### Motion Blur

**Problem**: Fast-moving objects are blurry, not detected

**Solutions:**

1. **Increase frame rate:**
    ```python
    config = picam2.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        controls={"FrameRate": 60}  # Up from 30
    )
    ```

2. **Reduce exposure time:**
    ```python
    picam2.set_controls({
        "ExposureTime": 10000  # Shorter exposure = less blur
    })
    ```

---

## Production Best Practices

### 1. Robust Error Handling

    ```python
    def safe_detection_loop():
        consecutive_errors = 0
        MAX_ERRORS = 5

        while True:
            try:
                frame = picam2.capture_array()
                results = model.detect(frame)
                process_results(results)

                consecutive_errors = 0  # Reset on success

            except Exception as e:
                consecutive_errors += 1
                print(f"Error {consecutive_errors}/{MAX_ERRORS}: {e}")

                if consecutive_errors >= MAX_ERRORS:
                    print("Too many errors, restarting...")
                    restart_system()

                time.sleep(1)  # Backoff before retry
    ```

### 2. Logging and Monitoring

    ```python
    import logging
    from datetime import datetime

    # Setup logging
    logging.basicConfig(
        filename=f'detector_{datetime.now():%Y%m%d}.log',
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    # Log key events
    logging.info(f"System started")
    logging.info(f"Model loaded: {model_path}")

    # Log detections
    logging.info(f"Detected {len(results)} objects: {class_counts}")

    # Log errors
    logging.error(f"Failed to process frame: {e}")
    ```

### 3. Health Monitoring

    ```python
    class HealthMonitor:
        def __init__(self):
            self.last_detection_time = time.time()
            self.total_detections = 0

        def check_health(self):
            # No detections for 5 minutes?
            if time.time() - self.last_detection_time > 300:
                logging.warning("No detections in 5 minutes")

            # Check memory
            if psutil.virtual_memory().percent > 90:
                logging.warning("High memory usage!")

            # Check temperature (Pi-specific)
            temp = os.popen("vcgencmd measure_temp").readline()
            if "temp=" in temp:
                temp_c = float(temp.replace("temp=","").replace("'C\n",""))
                if temp_c > 70:
                    logging.warning(f"High temperature: {temp_c}°C")

        def record_detection(self):
            self.last_detection_time = time.time()
            self.total_detections += 1
    ```

---

## Advanced Debugging

### Visualize Model Outputs

```python
import matplotlib.pyplot as plt

def visualize_confidence_distribution(results):
    """See what confidence levels your model outputs"""
    scores = [r['score'] for r in results]

    plt.figure(figsize=(10, 5))
    plt.hist(scores, bins=20, edgecolor='black')
    plt.xlabel('Confidence Score')
    plt.ylabel('Count')
    plt.title('Detection Confidence Distribution')
    plt.axvline(x=0.5, color='r', linestyle='--', label='Current Threshold')
    plt.legend()
    plt.savefig('confidence_dist.png')
    plt.show()
```

### Compare Predictions to Ground Truth

```python
def calculate_accuracy(predictions, ground_truth, iou_threshold=0.5):
    """Measure model performance on test set"""
    true_positives = 0
    false_positives = 0
    false_negatives = 0

    for pred_box in predictions:
        best_iou = 0
        for gt_box in ground_truth:
            iou = calculate_iou(pred_box, gt_box)
            best_iou = max(best_iou, iou)

        if best_iou >= iou_threshold:
            true_positives += 1
        else:
            false_positives += 1

    false_negatives = len(ground_truth) - true_positives

    precision = true_positives / (true_positives + false_positives)
    recall = true_positives / (true_positives + false_negatives)

    return precision, recall
```

---

## Summary

You've mastered:
- Systematic troubleshooting for accuracy, performance, and stability
- Optimizing inference speed with multiple techniques
- Handling environmental challenges (lighting, glare, motion)
- Implementing production-ready error handling
- Monitoring system health and performance
- Advanced debugging and validation techniques

You're now equipped to deploy and maintain robust object detection systems in production!

---

## Next Lesson

In the final lesson, you'll review everything you've learned and discover next steps for advancing your computer vision skills.

---
