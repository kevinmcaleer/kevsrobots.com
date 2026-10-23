---
title: Installing CVZone
description: >-
   Kickstart your computer vision journey on Raspberry Pi using CVZone
layout: lesson
type: page
cover: /learn/cvzone/assets/install.png
---

![Cover Image]({{page.cover}}){:class="cover"}

## Overview

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

## Common Issues

**Problem:** `pip install` says `error: externally-managed-environment`.
**Solution:** Activate the virtual environment first with `source ~/cvzone-env/bin/activate`.
**Why:** Raspberry Pi OS protects the system Python, so packages go in a virtual environment.

**Problem:** `pip` can't find a version of `mediapipe`.
**Solution:** Check you're on 64-bit Raspberry Pi OS with `uname -m`, which should print `aarch64`. If it does, check [MediaPipe on PyPI](https://pypi.org/project/mediapipe/) supports your Python version (`python --version`).
**Why:** MediaPipe only publishes packages for 64-bit Arm and particular Python versions.

**Problem:** The test program prints "No camera image".
**Solution:** Check the webcam is plugged in and try `cv2.VideoCapture(1)`, or use the `picamera2` code for a camera module.
**Why:** OpenCV couldn't open a camera at that index.

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
{% endquiz %}

---
