---
title: Face Detection
description: >-
    Detect faces in a live video feed with CVZone, and use their position and size in your own code
layout: lesson
type: page
cover: /learn/cvzone/assets/face.jpg
date_updated: 2026-11-24
---

![Cover Image]({{page.cover}}){:class="cover"}

## Face detection

By the end of this lesson you will be able to find faces in a live camera picture and use each face's position and size in your own code.

Face detection is the process of finding faces in images or video and working out where they are. (It doesn't tell you *who* someone is: that's face *recognition*.) In this lesson we'll detect faces in real time with CVZone.

---

## The code

```python
import cv2
from cvzone.FaceDetectionModule import FaceDetector

cap = cv2.VideoCapture(0)
detector = FaceDetector(minDetectionCon=0.5)

while True:
    success, img = cap.read()
    if not success:
        break

    img, faces = detector.findFaces(img)

    if faces:
        face = faces[0]
        print("Centre:", face["center"], "Size:", face["bbox"][2], "x", face["bbox"][3])

    cv2.imshow("Face Detection", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## How it works

- `FaceDetector(minDetectionCon=0.5)` loads MediaPipe's face model. Faces it's less than 50% sure about are ignored
- `detector.findFaces(img)` finds every face, draws a box and a confidence score on the image, and returns the image and a list of faces
- Each face in the list is a dictionary:
  - `bbox` - the box around the face as `(x, y, width, height)`
  - `center` - the middle of the box as `(x, y)`
  - `score` - how confident the model is. In CVZone 2.0.0 this is a list holding one number between 0 and 1, so use `face["score"][0]`
  - `id` - the face's number in this frame
- Pass `draw=False` to `findFaces()` if you want the data without the drawings
- `FaceDetector` also takes `modelSelection`: `0` (the default) is the short-range model, for faces fairly close to the camera, and `1` is a full-range model that suits people further away. It downloads its own model file the first time you use it

---

## Going further

- **Multiple faces:** `faces` holds every face in the frame, so `len(faces)` counts people
- **Follow a face:** compare `face["center"][0]` with the middle of the image to tell whether someone is to the left or right. A robot or a pan-tilt camera could turn to follow them
- **How close is it?** A bigger box means the face is closer to the camera
- **Speed things up:** a smaller frame is much quicker to process. Add `cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)` and `cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)` after opening the camera

---

## Try it Yourself

1. Print how many faces are in view with `len(faces)`. Ask a friend to join you in the picture.
2. Print "Left", "Right" or "Centre" depending on where the face is in the picture. The image width is `img.shape[1]`.
3. Print "Close" or "Far" by checking whether the box width, `face["bbox"][2]`, is bigger than a number you choose. Find a good number by testing.
4. Challenge: use `cvzone.putTextRect()` (you'll meet it in the last lesson) to show the direction on the video instead of printing it.

---

## Common Issues

- **Problem**: The program stops with an error the first time you run it, mentioning a download.
- **Solution**: Make sure the Pi is online and run it again from the same folder.
- **Why**: CVZone downloads the face model file once and saves it beside your program.

- **Problem**: No faces are found, or the box flickers on and off.
- **Solution**: Face the camera, put a light in front of you rather than behind you, and move a little closer. You can also lower `minDetectionCon`.
- **Why**: The model struggles with faces that are small, turned away or in silhouette.

- **Problem**: `TypeError: unsupported operand type(s)` when you use `face["score"]` in a sum or comparison.
- **Solution**: Use `face["score"][0]`.
- **Why**: In CVZone 2.0.0 the score is stored inside a list.

> **Teachers**: This lesson points a camera at people. Agree your school's rules before the lesson, and let learners opt out of being in the picture. The program finds faces but doesn't save images or work out who anyone is, and nothing here records video, so keep it that way unless you have permission. Pi speed matters too: a Pi 4 may feel slower than a Pi 5, so shrinking the frame as suggested above helps.

{% quiz %}
? What's the difference between face detection and face recognition?
- [x] Detection finds where faces are, recognition works out whose face it is
- [ ] They're the same thing
- [ ] Recognition only works on photos
> This lesson finds faces. Telling people apart needs a recognition model.

? In CVZone, what does `face["bbox"]` contain?
- [ ] The face's name
- [x] x, y, width and height of the box around the face
- [ ] A list of facial landmarks
> `bbox` is the bounding box. `center` gives you its middle point.

? How would you tell whether a face is on the left or right of the picture?
- [ ] Check `face["score"]`
- [x] Compare `face["center"][0]` with the middle of the image width
- [ ] Count the faces with `len(faces)`
> The first number of `center` is the x position, and `img.shape[1]` is the image width, so half of it is the middle.
{% endquiz %}

---
