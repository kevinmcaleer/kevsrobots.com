---
title: Face Detection
description: >-
    Detect faces in a live video feed with CVZone, and use their position and size in your own code
layout: lesson
type: page
cover: /learn/cvzone/assets/face.jpg
---

![Cover Image]({{page.cover}}){:class="cover"}

## Face detection

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
  - `score` - how confident the model is
- Pass `draw=False` to `findFaces()` if you want the data without the drawings

---

## Going further

- **Multiple faces:** `faces` holds every face in the frame, so `len(faces)` counts people
- **Follow a face:** compare `face["center"][0]` with the middle of the image to tell whether someone is to the left or right. A robot or a pan-tilt camera could turn to follow them
- **How close is it?** A bigger box means the face is closer to the camera
- **Speed things up:** a smaller frame is much quicker to process. Add `cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)` and `cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)` after opening the camera

---

## Try it yourself

Print "Left", "Right" or "Centre" depending on where the face is in the picture. The image width is `img.shape[1]`.

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
{% endquiz %}

---
