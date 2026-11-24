---
title: Hand Detection
description: >-
    Track hands in a live video feed with CVZone and count how many fingers are raised
layout: lesson
type: page
cover: /learn/cvzone/assets/hand.png
date_updated: 2026-11-24
---

![Cover Image]({{page.cover}}){:class="cover"}

## Hand detection

By the end of this lesson you will be able to find a hand in the video, count the fingers that are raised and measure a pinch.

Hand detection finds hands in images or video. MediaPipe goes further and finds 21 **landmarks** on each hand: the wrist and the joints and tips of every finger. That's enough to recognise gestures, which makes hands a great way to control robots and games without touching anything.

---

## The code

```python
import cv2
from cvzone.HandTrackingModule import HandDetector

cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.8, maxHands=2)

while True:
    success, img = cap.read()
    if not success:
        break

    hands, img = detector.findHands(img)

    if hands:
        hand = hands[0]
        fingers = detector.fingersUp(hand)
        print("Fingers up:", fingers, "count:", fingers.count(1))

    cv2.imshow("Hand Detection", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

> ## Watch the order
>
> `findHands()` returns `hands, img`, the other way round from `findFaces()`. Mixing them up gives a confusing error the first time you use the result.

---

## How it works

- `HandDetector(detectionCon=0.8, maxHands=2)` tracks up to two hands. In CVZone 2.0.0 `maxHands` is used, but the `detectionCon` value is stored and not passed on to MediaPipe, so changing it makes no difference
- `detector.findHands(img)` draws a box and a dot at the centre of each hand, and returns a list of hands plus the image
- Each hand is a dictionary with:
  - `lmList` - the 21 landmarks, each `[x, y, z]`. Landmark 0 is the wrist and 8 is the tip of the index finger. In CVZone 2.0.0 the `z` value is always `0`, so use `x` and `y`
  - `bbox` and `center` - the box around the hand
  - `type` - `"Left"` or `"Right"` in older CVZone versions. In 2.0.0 it is always `"Unknown"`, so don't rely on it
- `detector.fingersUp(hand)` returns five numbers, thumb first, with `1` for a raised finger and `0` for a folded one. So `[0, 1, 1, 0, 0]` is a peace sign. Each finger is judged by whether its tip is above the joint two landmarks back, so keep your hand upright. The thumb is judged by a simple left and right comparison, so it can read the wrong way round depending on which hand you use and which way your palm faces

---

## Measuring a pinch

The distance between two landmarks makes a handy control. Add this inside the `if hands:` block:

```python
        thumb_tip = hand["lmList"][4][0:2]
        index_tip = hand["lmList"][8][0:2]
        length, info, img = detector.findDistance(thumb_tip, index_tip, img)
        print("Pinch distance:", int(length))
```

`findDistance()` draws a line between the two points and returns the distance in pixels. Pinch your fingers together and apart to watch it change. You could use this as a volume knob or to set a robot's speed.

---

## Try it Yourself

1. Print only the number of raised fingers with `fingers.count(1)`. Hold up one to five fingers and check it.
2. Make the program print "Rock" when no fingers are up, "Paper" when all five are, and "Scissors" for `[0, 1, 1, 0, 0]`.
3. Use the pinch distance to set a number from 0 to 100, for example `volume = min(100, int(length / 3))`, and print it as you pinch.
4. Challenge: show the finger count as big text on the video once you have read the next lesson's `putTextRect()`.

---

## Common Issues

- **Problem**: A confusing error, or nothing drawn, after you write `img, hands = detector.findHands(img)`.
- **Solution**: Write `hands, img = detector.findHands(img)`.
- **Why**: `findHands()` returns the hands first, the opposite way round from `findFaces()`.

- **Problem**: The thumb shows `1` when it's folded, or `0` when it's out.
- **Solution**: Try the other hand, or turn your palm to face the camera. Or compare the result yourself using the landmark positions.
- **Why**: `fingersUp()` judges the thumb with a left and right test that assumes one hand and one palm direction.

- **Problem**: `hand["type"]` is always `"Unknown"`.
- **Solution**: Don't use it to tell left from right on CVZone 2.0.0. Compare `hand["center"][0]` against the middle of the picture instead.
- **Why**: CVZone 2.0.0 doesn't fill in the handedness, so every hand gets the same placeholder.

> **Teachers**: Hands in a camera picture are less personal than faces, but pupils are still on camera, so keep the same ground rules. The detector is hungry for processing power. Tracking fewer hands is less work for the Pi, so try `maxHands=1` if the picture is slow, and compare a Pi 4 with a Pi 5 if you have both.

{% quiz %}
? What does `fingersUp()` return for an open hand with all fingers spread?
- [ ] `5`
- [x] `[1, 1, 1, 1, 1]`
- [ ] `[0, 0, 0, 0, 0]`
> It returns one value per finger, thumb first, with 1 for each raised finger.

? How many landmarks does MediaPipe find on each hand?
- [ ] 5
- [x] 21
- [ ] 33
> 21 points: the wrist plus four on each finger. (33 is the number of body landmarks in the next lesson.)

? What does `findDistance()` return in the pinch example?
- [ ] Only the distance in centimetres
- [x] The distance in pixels, some information about the points, and the image
- [ ] A list of fingers
> `length, info, img = detector.findDistance(...)` gives the distance in pixels along with the points and the image.
{% endquiz %}

---
