---
title: Stream ESP32CAM video
description: Capture ESP32CAM Video Stream in Python on a Raspberry Pi 5
layout: project
date: 2023-12-29
cover: /assets/img/blog/esp32cam/esp32cam.jpg
excerpt: >-
    How to capture a video stream from an ESP32CAM in Python on a Raspberry Pi 5
author: Kevin McAleer
difficulty: intermediate
groups:
    - robots
    - raspberrypi
tags:
    - OpenCV
    - Computer Vision
    - esp32cam
    - Python
    - Raspberry Pi
---

![ESP32CAM](/assets/img/blog/esp32cam/cam01.jpg){:class="cover"}

## ESP32CAM Module

![ESP32CAM](/assets/img/blog/esp32cam/cam04.jpg){:class="img-fluid w-100 shadow-lg"}

The ESP32Cam is a tiny module that allows you to stream video from the camera to a web browser. It's a great way to get easily add a POV capability to your robotics project. The firmware that comes loaded on the ESP32Cam has a web server, self hosted wifi hotspot as well as a built-in a video streamer.

We'll use the video streamer to capture the video stream in Python on a Raspberry Pi 5, and then process that video in realtime.

---

## What is RTSP?

The ESP32Cam video streamer uses [`RTSP`](/resources/glossary#rtsp) to stream the video. 

`RTSP` stands for Real-Time Streaming Protocol. It's a network protocol, a set of rules, used for controlling the streaming of audio and video data over the Internet in real-time. Think of it as a 'remote control' for live video feeds.

---

### How Does RTSP Work?

1. **Connection**: First, your device (like your computer or smartphone) contacts the server (where the video is stored or being broadcast from) using RTSP. It's like dialing a phone number to start a call.

1. **Control Commands**: Once connected, RTSP allows you to send control commands to the server. You can tell the server to do things like 'play the video', 'pause', 'rewind', or 'fast forward' – similar to how you use a remote control with your TV.

1. **Streaming**: Unlike downloading a file, where you wait for the entire file to download before viewing, RTSP allows the video or audio to be played as it's being transmitted. This is known as `streaming`.

1. **Separate Data Transport**: RTSP itself doesn't send the video or audio data. Instead, it works alongside other protocols (like RTP - Real-Time Transport Protocol) that handle the actual transmission of the audio and video data.

---

### Why Use RTSP?

- **Live Control**: RTSP is great for situations where you need real-time control over streaming, like in security camera feeds, live broadcasts, or video conferencing.

- **Efficiency**: It's efficient for streaming live content because it reduces delay and allows for interactive control over the stream.

- **Flexibility**: RTSP supports various media types and can be used with different kinds of networks and devices.

---

We can use the `cv2` library in Python to capture the video stream from the ESP32Cam. We'll use the `cv2.VideoCapture()` function to capture the video stream. We'll pass the URL of the ESP32Cam video stream to the `cv2.VideoCapture()` function, and it will return a video stream object that we can use to capture the video frames.

```python
rtsp_url = 'http://192.168.4.1:81/stream'

# Capture the video stream
cap = cv2.VideoCapture(rtsp_url)
```

---

## Detecting Objects in the Video Stream

Before we write a simple program to capture the RTSP stream and process it, we need to setup a new Python environment on the Raspberry Pi 5. We'll use the `cvzone` module to detect objects in the video stream. The `cvzone` module is a wrapper around the `cv2` library, and makes it easier to detect objects in the video stream.

---

### Creating a virtual environment

```bash
python3 -m venv venv
source venv/bin/activate
```

---

### Install CVZone

```bash
pip3 install cvzone, mediapipe, opencv-python
```

---

### Face Detection

We can use the `cv2` library to detect objects in the video stream. To detect faces, we can use the CVZone module and the `FaceDetector` class. The `FaceDetector` class will detect faces in the video stream, and return a list of faces that it has detected. The `cv2.imshow()` function will display the video stream in a window on the screen, with a greenbox around the detected faces, along with a percentage confidence score.



```bash

```python
while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Process the frame with OpenCV here
    frame, list_faces = face_detector.findFaces(frame)
    cv2.imshow("Face Detection", frame)
```

---

## Why this is cool

The ESP32Cam is low power, simple to setup and configure and pretty low cost too (£0.57 each on Aliexpress - plus shipping). By offloading the video processing to the Raspberry Pi 5, we don't need to change the ESP32Cam firmware and can build on the image processing capabilities on the Pi 5.

---

## What else can we do?

We can use the image data to make decision on how to control the robot remotely, making it move towards objects or look at a face.

![ESP32CAM](/assets/img/blog/esp32cam/cam02.jpg){:class="img-fluid w-100 shadow-lg"}

![ESP32CAM](/assets/img/blog/esp32cam/cam03.jpg){:class="img-fluid w-100 shadow-lg"}

---

## Bill of Materials

Item                                                     | Description     | Price per item | Qty |  Cost
---------------------------------------------------------|-----------------|---------------:|:---:|-----:
[ESP32CAM](https://www.aliexpress.com/item/1005006096159627.html?channel=twinner ) | ESP32CAM Module |          £0.57 |  1  | £0.57
{:class="table table-striped"}

---

## Python code

```python
import cv2
import cvzone
from cvzone import FaceDetectionModule
face_detector = FaceDetectionModule.FaceDetector()


# Replace with your RTSP stream URL
rtsp_url = 'http://192.168.4.1:81/stream'

# Capture the video stream
cap = cv2.VideoCapture(rtsp_url)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Process the frame with OpenCV here
    frame, list_faces = face_detector.findFaces(frame)
    cv2.imshow("Face Detection", frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

# Release everything if job is finished
cap.release()
cv2.destroyAllWindows()
```
