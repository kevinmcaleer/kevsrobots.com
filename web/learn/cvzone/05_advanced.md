---
layout: lesson
title: Advanced Visualization with CVZone
author: Kevin McAleer
type: page
cover: /learn/cvzone/assets/computer_vision.jpg
previous: 04_posture.html
description: Dive deeper into CVZone's capabilities and learn how to annotate images
  for better pose analysis insights.
percent: 100
duration: 2
navigation:
- name: Computer Vision on Raspberry Pi with CVZone
- content:
  - section: Overview
    content:
    - name: Computer Vision with CVZone
      link: 00_intro.html
    - name: Watch the demonstration
      link: 00_video.html
  - section: Getting Started
    content:
    - name: Installing CVZone
      link: 01_install.html
  - section: Face, Hand and Posture Detection
    content:
    - name: Face Detection
      link: 02_face.html
    - name: Hand Detection
      link: 03_hand.html
    - name: Posture Detection
      link: 04_posture.html
    - name: Advanced Visualization with CVZone
      link: 05_advanced.html
---


## Advanced Visualization with CVZone

Enhancing visual feedback is essential, especially when working with posture detection. With CVZone's utilities like `putText`, you can overlay important information onto your video stream, providing real-time feedback and insights.

---

## Annotating with Text

CVZone offers a `putText` function similar to OpenCV's, but with an easier interface. Let's explore how to use it:

1. **Basic Text Annotation:**

   With CVZone, you can effortlessly overlay text on images or video frames:

   ```python
   from cvzone.Utils import putText
   img = putText(img, "Detected Posture", [50, 50])
   ```

   This adds the text "Detected Posture" at the coordinate (50, 50) on the image.

2. **Displaying Dynamic Data:**

   Annotate the image with real-time, dynamic data, such as the angle between specific landmarks:

   ```python
   angle, img = detector.findAngle(lmList[11][0:2], lmList[13][0:2], lmList[15][0:2], img=img)
   img = putText(img, f"Angle: {angle:.2f}°", [50, 80])
   ```

---

## Enhancing Visualization

Now, let's build on our previous code to make it more sophisticated with added visualizations:

```python
from cvzone.PoseModule import PoseDetector
from cvzone.Utils import putText
import cv2

cap = cv2.VideoCapture(0)
detector = PoseDetector()

while True:
    success, img = cap.read()
    img = detector.findPose(img)
    lmList, _ = detector.findPosition(img, draw=False)

    if lmList:
        angle, img = detector.findAngle(lmList[11][0:2], lmList[13][0:2], lmList[15][0:2], img=img)
        img = putText(img, f"Angle: {angle:.2f}°", [50, 80], fontSize=1, color=(255, 0, 255))

        # Check posture and provide real-time feedback
        if angle > 50:
            feedback = "Poor Posture!"
            img = putText(img, feedback, [50, 120], fontSize=1.2, color=(0, 0, 255))
        else:
            feedback = "Good Posture!"
            img = putText(img, feedback, [50, 120], fontSize=1.2, color=(0, 255, 0))

    cv2.imshow("Image", img)
    cv2.waitKey(1)
```

---

## Further Exploration

- **Custom Annotations:** Experiment with different text positions, sizes, and colors to make the feedback more intuitive.

- **Interactive Feedback:** Use CVZone's functionalities to draw shapes, lines, or even custom animations to make your application more interactive.

- **Integration:** Think about integrating the posture detection system with other applications or even using it as an input method (e.g., a game controlled by your posture).

---

By leveraging CVZone's comprehensive utilities, you can transform basic detection code into an informative, interactive, and user-friendly application.

---
