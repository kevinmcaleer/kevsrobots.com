---
title: Hand Detection
description: >-
    Track hands in a live video feed with CVZone and count how many fingers are raised
layout: lesson
type: page
cover: /learn/cvzone/assets/hand.png
---

![Cover Image]({{page.cover}}){:class="cover"}

## Hand detection

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
        print(hand["type"], "hand, fingers up:", fingers, "count:", fingers.count(1))

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

- `HandDetector(detectionCon=0.8, maxHands=2)` only accepts hands it's at least 80% sure about, and tracks up to two
- `detector.findHands(img)` draws the hand skeleton and returns a list of hands plus the image
- Each hand is a dictionary with:
  - `lmList` - the 21 landmarks, each `[x, y, z]`. Landmark 0 is the wrist and 8 is the tip of the index finger
  - `bbox` and `center` - the box around the hand
  - `type` - `"Left"` or `"Right"`
- `detector.fingersUp(hand)` returns five numbers, thumb first, with `1` for a raised finger and `0` for a folded one. So `[0, 1, 1, 0, 0]` is a peace sign

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

## Try it yourself

Show a number from 0 to 5 with your fingers and print it in big letters. Then make the program print "Rock", "Paper" or "Scissors" from your hand shape.

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
{% endquiz %}

---
