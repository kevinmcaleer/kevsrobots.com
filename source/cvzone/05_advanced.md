---
title: Annotating Your Video
description: >-
    Overlay live text and feedback on your video with CVZone's putTextRect, and build a curl counter
layout: lesson
type: page
cover: /learn/cvzone/assets/banners/05_advanced.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to draw live text on your video, and build a bicep curl counter that uses a joint angle and two thresholds.

## Annotating your video

Printing numbers to the terminal is fine for testing, but it's much nicer to show feedback right on the video. CVZone's `putTextRect()` draws text on a coloured background, so it's easy to read whatever is behind it.

---

## Adding text

```python
import cvzone

img, bbox = cvzone.putTextRect(img, "Hello!", (50, 50))
```

- The position is where the text starts, in pixels from the top left corner
- It returns the image and the box around the text

You can change the look with a few options:

```python
img, _ = cvzone.putTextRect(img, "Hello!", (50, 50),
                            scale=2, thickness=2,
                            colorT=(255, 255, 255),   # text colour
                            colorR=(255, 0, 255))     # background colour
```

> ## Colours are BGR
>
> OpenCV colours are `(blue, green, red)`, not `(red, green, blue)`. So `(0, 0, 255)` is red and `(255, 0, 0)` is blue.

---

## A bicep curl counter

Let's put the last lesson to work. This program watches your left elbow and counts a curl each time you bend your arm fully and straighten it again.

```python
import cv2
import cvzone
from cvzone.PoseModule import PoseDetector

cap = cv2.VideoCapture(0)
detector = PoseDetector()

count = 0
arm_bent = False

while True:
    success, img = cap.read()
    if not success:
        break

    img = detector.findPose(img)
    lmList, _ = detector.findPosition(img, draw=False)

    if lmList:
        angle, img = detector.findAngle(lmList[11][0:2], lmList[13][0:2], lmList[15][0:2], img=img)
        angle = min(angle, 360 - angle)   # 0 to 180, whichever way the arm swings

        if angle < 50 and not arm_bent:
            arm_bent = True
        elif angle > 150 and arm_bent:
            arm_bent = False
            count += 1

        status = "Bent" if arm_bent else "Straight"
        colour = (0, 165, 255) if arm_bent else (0, 200, 0)
        img, _ = cvzone.putTextRect(img, f"Angle: {int(angle)}", (50, 60), scale=2, thickness=2)
        img, _ = cvzone.putTextRect(img, status, (50, 120), scale=2, thickness=2, colorR=colour)

    img, _ = cvzone.putTextRect(img, f"Curls: {count}", (50, 200), scale=3, thickness=3, colorR=(255, 0, 0))

    cv2.imshow("Curl Counter", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## How it works

- `findAngle()` can return 20 or 340 for the same bend, so `min(angle, 360 - angle)` folds the answer into 0 to 180. A straight arm is still about 180
- `arm_bent` remembers whether the arm was last seen bent or straight
- A curl only counts when the arm goes from bent (below 50 degrees) back to straight (above 150 degrees). The gap between the two numbers stops one wobbly movement being counted several times
- The status label changes colour so you can see the state at a glance
- The count is drawn outside the `if lmList:` block, so it stays on screen even when you step out of view

> ## No degree symbol
>
> OpenCV's built-in fonts can't draw `°`, so it would show up as `??`. Stick to plain text in your labels.

---

## Try it Yourself

1. Change the text of the status label, and its colours, to something you like.
2. Change the 50 and 150 thresholds. Does a looser curl still count? Does a wobbly one get counted twice?
3. Add a second counter for your right arm, using landmarks 12, 14 and 16.
4. Challenge: count squats with the hip, knee and ankle (23, 25, 27), standing side-on to the camera.

---

## Common Issues

- **Problem**: The count goes up when you hardly move, or never goes up.
- **Solution**: Print the angle and watch its range as you curl, then set the two thresholds a little inside that range.
- **Why**: Every camera position and body gives different angles, and a curl is only counted when the angle crosses both thresholds.

- **Problem**: The text shows `??` where a degree sign should be.
- **Solution**: Leave the symbol out and write `Angle: 45`.
- **Why**: OpenCV's built-in fonts can't draw that character.

- **Problem**: The text is hard to read, or runs off the edge of the picture.
- **Solution**: Use a smaller `scale`, or move the position.
- **Why**: `putTextRect()` draws from the position you give, and doesn't check it fits.

> **Teachers**: This is the course's project lesson, so give it a full session. Keep the exercise gentle and optional, as curls and squats in front of a class camera can embarrass some pupils. Pairs can take turns, with one doing the exercise and one holding the keyboard, and learners can test the counter by moving an arm slowly without any weights.

---

## Further exploration

- **Both arms:** count curls on the right arm too, using landmarks 12, 14 and 16
- **Other exercises:** squats use the hip, knee and ankle (23, 25, 27)
- **Other drawing tools:** `cvzone.cornerRect()` draws a stylish box around a `bbox`, and `cvzone.stackImages()` puts several images side by side

---

{% quiz %}
? Which colour is `(0, 0, 255)` in OpenCV?
- [ ] Blue
- [x] Red
- [ ] Green
> OpenCV uses BGR order, so the last number is red.

? Why does the curl counter use two thresholds (50 and 150) instead of one?
- [ ] To make the program faster
- [x] So small wobbles around one angle don't count as several curls
- [ ] Because `findAngle()` returns two values
> The gap means the arm has to fully bend and fully straighten before a curl is counted.

? Why is the count drawn outside the `if lmList:` block?
- [x] So it stays on screen even when you step out of view
- [ ] Because it can't be drawn inside an `if`
- [ ] To make the detector faster
> The lesson draws it outside so the total remains visible even when nobody is detected.
{% endquiz %}

---

## What's next?

- [Building Object Detection Models with Raspberry Pi AI Camera](/learn/object_model/) - train and run your own object detection model
- Combine hand or face tracking with a robot from [Build a SMARS Robot](/learn/smars/) or [BurgerBot](/learn/burgerbot/)

---
