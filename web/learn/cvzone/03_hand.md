---
layout: lesson
title: Hand Detection
author: Kevin McAleer
type: page
cover: assets/hand.png
previous: 02_face.html
next: 04_posture.html
description: This lesson will guide you through capturing and analyzing video feed
  to highlight and track hands effectively
percent: 70
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


![Cover Image]({{page.cover}}){:class="cover"}

## Hand Detection

Hand detection is the computer vision technique of identifying and locating hands in images or videos. Recognizing hands can serve as a basis for gesture recognition, touchless control systems, and more. In this lesson, we'll utilize CVZone to perform hand detection in real time.

1. **Capture video from the camera:**

   As with the face detection lesson, we'll be using the OpenCV library to capture live video feed from the camera. If you've followed the face detection lesson, this step should be familiar.

   ```python
   import cv2
   cap = cv2.VideoCapture(0)
   ```

2. **Use CVZone for hand detection:**

   ```python
   hand_detector = cvzone.HandDetector(detectionCon=0.8)

   while True:
       success, img = cap.read()
       img, list_hands = hand_detector.findHands(img)
       cv2.imshow("Hand Detection", img)
       if cv2.waitKey(1) & 0xFF == ord('q'):
           break
   ```

   Here's a breakdown of the code:
   - We initialize the hand detection module from CVZone with a detection confidence threshold of `0.8`. This means the algorithm will only consider detections with a confidence of 80% or higher.
   - We then continuously read frames from our camera.
   - Process each frame to detect hands using `hand_detector.findHands(img)`.
   - The detected hands are highlighted in the frame which is then displayed using `cv2.imshow()`.
   - As before, pressing the 'q' key will exit the program.

---

## Expanding Your Horizons

- **Gesture Recognition:** Build upon hand detection to recognize specific hand gestures which can be used to control applications, games, or devices without physical contact.

- **Customize Visuals:** Play with the visual feedback. Adjust the color, size, and annotations displayed around detected hands.

- **Landmark Detection:** Many hand detection systems also provide hand landmarks – specific points on a hand, like fingertip or wrist. Explore how these can be used for more detailed analysis or interaction.

- **Performance Tweaks:** If running on a Raspberry Pi, consider optimizations such as frame resizing or adjusting detection parameters to ensure real-time performance.

---

Through this lesson, you'll have the foundational knowledge needed to explore more complex applications of hand detection. The sky's the limit when it comes to building upon these basics!

---
