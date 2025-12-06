---
title: Real-Time Object Detection
description: >-
    Build a real-time object detection application using your custom model and the Raspberry Pi AI Camera with live video feed
type: page
layout: lesson
---

## Overview

Time to bring everything together! In this lesson, you'll build a complete real-time object detection system that processes live camera feed, detects objects, draws bounding boxes, and displays results with smooth performance. This is where your model comes to life!

---

## Objectives

By the end of this lesson, you will:

- Process live camera feed in real-time
- Display detected objects with bounding boxes and labels
- Show class names and confidence scores
- Calculate and display FPS (frames per second)
- Optimize for smooth performance
- Save detection results and logs
- Add audio or visual alerts for detections

---

## Why Real-Time Matters

Real-time object detection opens up countless applications:

**Practical uses:**
- **Security** - Alert when unknown person approaches
- **Manufacturing** - Detect defects on production line instantly
- **Robotics** - Allow robots to navigate and manipulate objects
- **Agriculture** - Count ripe fruits, identify pests in real-time
- **Accessibility** - Help visually impaired navigate environments

**Real-time requirements:**
- **Latency** - <100ms from capture to detection
- **FPS** - 10+ frames per second for smooth experience
- **Reliability** - Consistent performance over hours of operation

---

## Complete Real-Time Detection Script

Here's a full, production-ready application:

```python
# realtime_detector.py
from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500
import cv2
import numpy as np
import time
from datetime import datetime

class RealtimeDetector:
    """
    Real-time object detection system using Raspberry Pi AI Camera
    """

    def __init__(self, model_path, class_names, confidence_threshold=0.5):
        """
        Initialize detector with model and settings

        Args:
            model_path: Path to .rpk model file
            class_names: Dictionary mapping class IDs to names
            confidence_threshold: Minimum confidence to show detection
        """
        self.model_path = model_path
        self.class_names = class_names
        self.confidence_threshold = confidence_threshold

        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()

        # Detection counts
        self.detection_counts = {name: 0 for name in class_names.values()}

        # Initialize camera and model
        print("Initializing camera...")
        self.picam2 = Picamera2()
        self.imx500 = IMX500(self.picam2)

        # Load model
        print(f"Loading model: {model_path}")
        self.imx500.load_network(model_path)

        # Configure camera for preview
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

        print("✓ System ready!")

    def draw_detection(self, image, box, label, confidence):
        """
        Draw bounding box and label on image

        Args:
            image: Image array to draw on
            box: Bounding box [x1, y1, x2, y2]
            label: Class name
            confidence: Detection confidence
        """
        x1, y1, x2, y2 = map(int, box)

        # Choose color based on class
        color = self.get_class_color(label)

        # Draw rectangle
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        # Prepare label text
        label_text = f"{label}: {confidence:.2f}"

        # Calculate text size for background
        (text_width, text_height), baseline = cv2.getTextSize(
            label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
        )

        # Draw label background
        cv2.rectangle(
            image,
            (x1, y1 - text_height - 10),
            (x1 + text_width + 10, y1),
            color,
            -1
        )

        # Draw label text
        cv2.putText(
            image, label_text,
            (x1 + 5, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 255), 2
        )

    def get_class_color(self, label):
        """Get consistent color for each class"""
        colors = {
            'person': (0, 255, 0),      # Green
            'mug': (255, 0, 0),          # Blue
            'laptop': (0, 0, 255),       # Red
            'keyboard': (255, 255, 0),   # Cyan
        }
        return colors.get(label, (128, 128, 128))  # Default gray

    def calculate_fps(self):
        """Calculate current FPS"""
        self.frame_count += 1
        elapsed = time.time() - self.start_time

        if elapsed > 1.0:  # Update FPS every second
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.start_time = time.time()

        return self.fps

    def draw_info_panel(self, image):
        """Draw information panel with stats"""
        height, width = image.shape[:2]

        # Draw semi-transparent panel
        panel = image.copy()
        cv2.rectangle(panel, (10, 10), (350, 150), (0, 0, 0), -1)
        cv2.addWeighted(panel, 0.6, image, 0.4, 0, image)

        # FPS
        cv2.putText(
            image, f"FPS: {self.fps:.1f}",
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 255, 0), 2
        )

        # Detection counts
        y_offset = 70
        cv2.putText(
            image, "Detections:",
            (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 255), 1
        )

        for class_name, count in self.detection_counts.items():
            y_offset += 25
            cv2.putText(
                image, f"  {class_name}: {count}",
                (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1
            )

    def save_detection(self, image, detections):
        """Save image with detections"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"detection_{timestamp}.jpg"
        cv2.imwrite(filename, image)
        print(f"✓ Saved {filename} ({len(detections)} objects)")

    def run(self, display=True, save_detections=False):
        """
        Main detection loop

        Args:
            display: Show live video window
            save_detections: Save images with high-confidence detections
        """
        print("\nStarting real-time detection...")
        print("Press 'q' to quit, 's' to save current frame")
        print("-" * 50)

        try:
            while True:
                # Capture frame
                frame = self.picam2.capture_array()

                # Get inference results
                results = self.imx500.get_inference_result()

                # Process detections
                detections = []
                if results and 'boxes' in results:
                    for box, label_id, score in zip(
                        results['boxes'],
                        results['labels'],
                        results['scores']
                    ):
                        if score >= self.confidence_threshold:
                            label = self.class_names.get(label_id, f"Class {label_id}")
                            detections.append((box, label, score))

                            # Draw detection
                            self.draw_detection(frame, box, label, score)

                            # Update count
                            if label in self.detection_counts:
                                self.detection_counts[label] += 1

                # Calculate and display FPS
                fps = self.calculate_fps()

                # Draw info panel
                self.draw_info_panel(frame)

                # Save high-confidence detections
                if save_detections and detections:
                    max_conf = max(d[2] for d in detections)
                    if max_conf > 0.9:
                        self.save_detection(frame, detections)

                # Display frame
                if display:
                    cv2.imshow('Object Detection - Press Q to quit', frame)

                    # Check for keypress
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('s'):
                        self.save_detection(frame, detections)

        finally:
            self.picam2.stop()
            if display:
                cv2.destroyAllWindows()

            print("\n" + "=" * 50)
            print("Session Summary:")
            print(f"  Average FPS: {fps:.1f}")
            print(f"  Total detections by class:")
            for class_name, count in self.detection_counts.items():
                if count > 0:
                    print(f"    {class_name}: {count}")
            print("=" * 50)


# ===== Usage =====
if __name__ == "__main__":
    # Define your class names (must match training)
    class_names = {
        0: 'background',
        1: 'mug',
        2: 'laptop',
        3: 'keyboard',
        # Add your classes here
    }

    # Create detector
    detector = RealtimeDetector(
        model_path="/home/pi/models/detector_imx500/network.rpk",
        class_names=class_names,
        confidence_threshold=0.5  # Show detections above 50% confidence
    )

    # Run detector
    detector.run(display=True, save_detections=True)
```

**Run it:**
```bash
python3 realtime_detector.py
```

---

## Performance Optimization Tips

### 1. Reduce Resolution for Speed

```python
# Lower resolution = faster processing
config = picam2.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}  # Smaller = faster
)
```

**Trade-off:**
- 640x480: Better accuracy, ~15-20 FPS
- 320x240: Lower accuracy, ~25-30 FPS

### 2. Skip Frames

Process every Nth frame to increase speed:

```python
frame_skip = 2  # Process every 2nd frame
frame_counter = 0

while True:
    frame = picam2.capture_array()
    frame_counter += 1

    if frame_counter % frame_skip == 0:
        results = imx500.get_inference_result()
        # Process results...
```

### 3. Adjust Confidence Threshold

```python
# Higher threshold = fewer false positives, faster drawing
detector = RealtimeDetector(
    model_path=model_path,
    class_names=class_names,
    confidence_threshold=0.7  # Only show very confident detections
)
```

### 4. Use Threading

Separate capture and processing:

```python
from threading import Thread
import queue

frame_queue = queue.Queue(maxsize=2)

def capture_thread():
    while running:
        frame = picam2.capture_array()
        if not frame_queue.full():
            frame_queue.put(frame)

def detection_thread():
    while running:
        if not frame_queue.empty():
            frame = frame_queue.get()
            # Process frame...

# Start both threads
Thread(target=capture_thread).start()
Thread(target=detection_thread).start()
```

---

## Add Audio Alerts

Make your detector speak when it finds objects:

```python
import os

def alert(label, confidence):
    """Play audio alert for detection"""
    if confidence > 0.8:
        os.system(f'espeak "{label} detected with {int(confidence*100)}% confidence"')

# In your detection loop:
if score > 0.8:
    alert(label, score)
```

**Install espeak if needed:**
```bash
sudo apt install espeak
```

---

## Remote Monitoring with Flask

Stream detections over your network:

```python
from flask import Flask, Response
import io

app = Flask(__name__)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    def generate():
        while True:
            frame = detector.get_latest_frame()
            _, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '''
    <html>
    <body>
    <h1>Object Detection Stream</h1>
    <img src="/video_feed" width="640" height="480">
    </body>
    </html>
    '''

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
```

Access from any device: `http://raspberrypi.local:5000`

---

## Try It Yourself

1. **Add confidence threshold slider** - Adjust threshold in real-time with keyboard
2. **Count objects** - Track total detections per session
3. **Record video** - Save video clips when objects are detected
4. **Zone monitoring** - Alert only when object enters specific region
5. **Multi-camera** - Use multiple AI cameras simultaneously

**Challenge:** Create a "tripwire" that sends an email when an object crosses a virtual line in the frame.

---

## Common Issues

**Problem**: Low FPS (<5 FPS)
**Solution**: Reduce resolution to 320x240, skip every other frame, increase confidence threshold
**Why**: Real-time needs 10+ FPS for smooth experience, high resolution or drawing too many boxes slows it down

**Problem**: Detections flickering on and off
**Solution**: Add temporal smoothing - only show detections that appear in 3+ consecutive frames
**Why**: Model confidence varies frame-to-frame, smoothing reduces jitter

**Problem**: High false positive rate
**Solution**: Increase confidence threshold to 0.7 or 0.8, retrain with more diverse data
**Why**: Model needs more examples or is being too lenient

**Problem**: Memory error after running for long time
**Solution**: Restart detection loop every 1000 frames, ensure frames are released properly
**Why**: Memory leak from accumulating frames or opencv resources

**Problem**: Camera connection lost during operation
**Solution**: Add try/except around capture, reconnect camera if error occurs
**Why**: Hardware/driver issues can cause intermittent failures

**Problem**: Lag between detection and display
**Solution**: Reduce resolution, skip frames, or use separate threads for capture/processing
**Why**: Processing can't keep up with capture rate

---

## Benchmark Your System

Add performance logging:

```python
import statistics

inference_times = []

# In your loop:
start = time.time()
results = imx500.get_inference_result()
inference_time = (time.time() - start) * 1000
inference_times.append(inference_time)

# After 100 frames:
if len(inference_times) >= 100:
    print(f"\nPerformance Stats (last 100 frames):")
    print(f"  Average: {statistics.mean(inference_times):.1f} ms")
    print(f"  Min: {min(inference_times):.1f} ms")
    print(f"  Max: {max(inference_times):.1f} ms")
    print(f"  Std Dev: {statistics.stdev(inference_times):.1f} ms")
    inference_times.clear()
```

---

## Summary

You've built a complete real-time object detection system!

You learned:
- Processing live camera streams with the AI Camera
- Drawing bounding boxes and labels in real-time
- Calculating and displaying FPS
- Optimizing for real-time performance
- Saving detection results
- Adding audio alerts and remote monitoring
- Troubleshooting common real-time issues

Your detector is now running at full speed on the edge!

---

## Next Lesson

In the next lesson, you'll learn advanced troubleshooting techniques, optimization strategies, and how to handle edge cases that arise in production deployments.

---
