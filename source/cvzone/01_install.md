---
title: Installing CVZone
description: >-
   Kickstart your computer vision journey on Raspberry Pi using CVZone
layout: lesson
type: page
cover: /learn/cvzone/assets/install.png
date_updated: 2026-11-24
---

![Cover Image]({{page.cover}}){:class="cover"}

## Overview

By the end of this lesson you will be able to create a virtual environment with OpenCV, MediaPipe and CVZone, and show a live camera picture in a window.

In this lesson you'll set up your Raspberry Pi for computer vision: a Python virtual environment with OpenCV, MediaPipe and CVZone, plus a quick camera test.

---

## What each library does

- **OpenCV** (`cv2`) - reads frames from the camera, draws on them and shows them in a window
- **MediaPipe** - Google's machine learning models that actually find the faces, hands and bodies
- **CVZone** - a thin layer over both that turns each detector into a couple of lines of code

---

## Requirements

- **Raspberry Pi 4 or 5** running **64-bit** Raspberry Pi OS. MediaPipe only provides packages for 64-bit systems
- **Camera:** a USB webcam or a Raspberry Pi camera module
- **Python 3** (already installed on Raspberry Pi OS)

---

## Installation steps

1. **Update your Pi:**

   ```bash
   sudo apt update && sudo apt full-upgrade -y
   ```

2. **Create a virtual environment:**

   Recent versions of Raspberry Pi OS don't let `pip` install packages system-wide, so we create a virtual environment for the course. The `--system-site-packages` option lets it see the camera library that comes with Raspberry Pi OS.

   ```bash
   python3 -m venv --system-site-packages ~/cvzone-env
   source ~/cvzone-env/bin/activate
   ```

   Your prompt now starts with `(cvzone-env)`. Run the `source` line again each time you open a new terminal.

3. **Install CVZone and MediaPipe:**

   ```bash
   pip install cvzone mediapipe
   ```

   This also installs OpenCV (`opencv-python`) and NumPy. It can take a few minutes.

> **Versions**: the code in this course was checked against the source of CVZone 2.0.0 with MediaPipe's current "Tasks" interface. CVZone has changed between versions, so if you have an older copy, run `pip install --upgrade cvzone mediapipe` inside the virtual environment. Run `pip list` to see what you have.

> ## The first run downloads a model
>
> The first time you create a `FaceDetector`, `HandDetector` or `PoseDetector`, CVZone downloads that detector's model file from Google's servers and saves it in the folder you ran the program from. That needs an internet connection once. After that the file is reused, so keep running the programs from the same folder (for example `~/cvzone-course`) and you won't download it again.

---

## Test your camera

Save this as `camera_test.py` and run it with `python camera_test.py`:

```python
import cv2

cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    if not success:
        print("No camera image")
        break
    cv2.imshow("Camera test", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

- `cv2.VideoCapture(0)` opens the first camera
- `cap.read()` grabs one frame. `success` is `False` if it failed
- `cv2.imshow()` shows the frame in a window, and `cv2.waitKey(1)` gives the window time to draw and checks for a key press
- Press `q` to quit

> ## Using a Raspberry Pi camera module
>
> `cv2.VideoCapture(0)` works with USB webcams. Camera modules on the ribbon cable use the `picamera2` library instead. Swap the capture lines for:
>
> ```python
> from picamera2 import Picamera2
>
> picam2 = Picamera2()
> picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
> picam2.start()
>
> img = picam2.capture_array()   # use this in place of cap.read()
> ```
>
> The `RGB888` format gives you frames in the same colour order OpenCV expects. The rest of the code in this course works the same way.

---

## Try it Yourself

1. Run `python camera_test.py` and wave at the camera. Is the picture mirrored or the right way round?
2. Add `print(img.shape)` inside the loop, before `cv2.imshow()`. What do the three numbers mean? (Height, width and colour channels.)
3. Add `cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)` and `cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)` after `cv2.VideoCapture(0)` and check what `img.shape` says now.
4. Challenge: flip the picture like a mirror with `img = cv2.flip(img, 1)` and decide whether you prefer it.

---

## Common Issues

- **Problem**: `pip install` says `error: externally-managed-environment`.
- **Solution**: Activate the virtual environment first with `source ~/cvzone-env/bin/activate`.
- **Why**: Raspberry Pi OS protects the system Python, so packages go in a virtual environment.

- **Problem**: `pip` can't find a version of `mediapipe`.
- **Solution**: Check you're on 64-bit Raspberry Pi OS with `uname -m`, which should print `aarch64`. If it does, check [MediaPipe on PyPI](https://pypi.org/project/mediapipe/) supports your Python version (`python --version`).
- **Why**: MediaPipe only publishes packages for 64-bit Arm and particular Python versions.

- **Problem**: The test program prints "No camera image".
- **Solution**: Check the webcam is plugged in and try `cv2.VideoCapture(1)`, or use the `picamera2` code for a camera module.
- **Why**: OpenCV couldn't open a camera at that index.

- **Problem**: A detector program stops with an error about downloading a model.
- **Solution**: Check the Pi is online, then run it again. If you are on a school or work network, ask whether `storage.googleapis.com` is blocked.
- **Why**: CVZone fetches the model file from Google's servers the first time, and can't start without it.

> **Teachers**: Plan the install before the lesson. It downloads several packages and later a model file for each detector, so a whole class doing it at once can be slow on a shared connection. Setting up one Pi, or running the install ahead of time, saves a lot of waiting. Check the school network lets the Pi reach PyPI and Google's download servers. Decide in advance where the camera will point, as the picture is of whoever sits in front of it.

---

{% quiz %}
? What does MediaPipe do in this course?
- [ ] It shows the video window
- [x] It provides the machine learning models that find faces, hands and bodies
- [ ] It installs Python packages
> OpenCV handles the video, MediaPipe does the detecting, and CVZone makes both easy to use.

? Why do we use a virtual environment?
- [x] Raspberry Pi OS doesn't let pip install packages system-wide
- [ ] It makes the camera faster
- [ ] CVZone only works in virtual environments
> A virtual environment keeps the course's packages separate from the system Python.

? What does `success` tell you after `success, img = cap.read()`?
- [ ] How sharp the picture is
- [x] Whether a frame was grabbed from the camera
- [ ] Whether a face was found
> `success` is `False` if the camera could not give a frame, which is why the test program prints "No camera image".

? Which library do you use for a camera module on the ribbon cable?
- [ ] `cv2.VideoCapture(0)`
- [x] `picamera2`
- [ ] MediaPipe
> The lesson's note shows `Picamera2`, with the `RGB888` format so OpenCV gets the colours it expects.
{% endquiz %}

---
