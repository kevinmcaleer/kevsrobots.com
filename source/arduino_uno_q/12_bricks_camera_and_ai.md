---
title: Bricks, a camera and AI without the setup
description: Build Q Board stage 7, a people counter that uses a USB webcam and a ready-made AI Brick, then shows the count on the LED matrix
layout: lesson
type: page
cover: /learn/arduino_uno_q/assets/banners/12_bricks_camera_and_ai.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to use the Video Object Detection Brick to count people with a camera and show the count on the matrix.

Normally, teaching a computer to spot people in a camera picture means installing a model, a runtime and a stack of libraries before you write a line of your own code. On the Uno Q, a Brick does that setup for you. You ask it what it can see, and it tells you.

This is **Q Board stage 7**. The camera and the AI model are Linux jobs. The matrix and the LED stay with the microcontroller.

---

## What you'll build

- A **people counter**. A webcam looks at the room, an AI model finds the people, and Python counts them.
- The count is drawn as big digits on the **LED matrix**.
- **LED 4** is red when someone is seen and green when nobody is.

**What you'll need.** A USB webcam, and a USB-C hub with power delivery. The board has one USB-C port, so the hub carries both the power and the camera. Use a hub that is not Apple's multiport adapter, which is not supported.

No camera? Read the lesson and use the confidence demo further down. The rest of the course does not depend on this lesson.

---

## What a Brick gives you

The Brick for this lesson is **Video Object Detection** (ID `arduino:video_object_detection`). Add it with **Add Brick**, as you did for the web page in [lesson 11](/learn/arduino_uno_q/11_a_web_page_for_your_project.html).

It runs a pre-trained model on the live camera stream. For each thing it finds, it reports three details:

- a **label**, such as "person" or "bicycle",
- a **confidence** score, and
- a **bounding box**, the rectangle around the thing.

It can also use custom models trained with Edge Impulse, if you ever outgrow the pre-trained one.

AI Bricks run in their own container. Their logs get their own tab in the console, next to **App launch**, **Python** and **Serial Monitor**. The first **Run** after adding one takes longer, so give it time.

To check the board can see your camera, type this in a terminal **on the board** (an ADB shell or SSH session, as in [lesson 10](/learn/arduino_uno_q/10_linux_survival_kit.html)).

```bash
ls /dev/video*
```

If it lists one or more `/dev/video` entries, the camera is seen.

---

## Which brain does what

Here is the nice surprise. The sensor is now a camera, and the camera belongs to Python, so the sketch no longer reads the potentiometer at all. It goes back to the lesson 7 display sketch, with only its first comment changed.

**sketch/sketch.ino**

```cpp
#include <Arduino_RouterBridge.h>
#include <Arduino_LED_Matrix.h>
#include <vector>

// Q Board, stage 7: the same display sketch as stage 4. The camera is Python's job

Arduino_LED_Matrix matrix;
uint8_t frame[104] = {0};   // the picture on the matrix right now

// Python calls this with 104 brightness values (0 to 7)
void draw(std::vector<uint8_t> newFrame) {
  size_t len = min(newFrame.size(), (size_t)104);
  memcpy(frame, newFrame.data(), len);
}

// Python calls this to set the colour of LED 4
void set_status(bool red, bool green, bool blue) {
  digitalWrite(LED4_R, red ? LOW : HIGH);    // active low: LOW is on
  digitalWrite(LED4_G, green ? LOW : HIGH);
  digitalWrite(LED4_B, blue ? LOW : HIGH);
}

void setup() {
  pinMode(LED4_R, OUTPUT);
  pinMode(LED4_G, OUTPUT);
  pinMode(LED4_B, OUTPUT);
  set_status(false, false, false);

  matrix.begin();
  matrix.setGrayscaleBits(3);
  matrix.clear();

  Bridge.begin();
  Bridge.provide("draw", draw);                    // only copies memory
  Bridge.provide_safe("set_status", set_status);   // touches pins: run it in loop()
}

void loop() {
  matrix.draw(frame);
  delay(10);
}
```

You have seen every line of this before. It offers `draw` and `set_status`, and it does what Python asks.

That is the point of the lesson. **Adding AI to the project did not change the sketch.** The camera and the model run on Linux, and the microcontroller carries on drawing and switching LEDs, exactly as in lesson 7. When you split a project well, a new feature usually lands on one side only.

---

## The Python

`FONT` and `number_frame()` are unchanged from lesson 7, and shown again to keep the file complete.

**python/main.py**

```python
import time

from arduino.app_utils import App, Bridge
from arduino.app_bricks.video_objectdetection import VideoObjectDetection

people = 0        # how many people the camera saw last time
last_seen = 0.0   # when it last saw anything at all

FONT = {
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    "6": ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
}


def number_frame(number, level=7):
    """Return 104 brightness values that show a number from 0 to 99."""
    frame = [0] * 104
    text = f"{number:2d}"
    for position, digit in enumerate(text):
        if digit == " ":
            continue
        left = 1 + position * 6
        for row, line in enumerate(FONT[digit]):
            for col, pixel in enumerate(line):
                if pixel == "1":
                    frame[row * 13 + left + col] = level
    return frame


detector = VideoObjectDetection(confidence=0.5, debounce_sec=1.0)


def on_detections(detections: dict):
    """The Brick calls this with everything it can see, grouped by label."""
    global people, last_seen
    people = len(detections.get("person", []))
    last_seen = time.time()
    print("I can see:", ", ".join(detections.keys()))


detector.on_detect_all(on_detections)


def loop():
    # No news for 3 seconds means the view is empty
    count = people if time.time() - last_seen < 3 else 0
    Bridge.call("draw", bytes(number_frame(count)))
    Bridge.notify("set_status", count > 0, count == 0, False)
    time.sleep(0.2)


App.run(user_loop=loop)
```

### Walking through the Python

**Creating the detector.** `VideoObjectDetection(confidence=0.5, debounce_sec=1.0)` sets up the Brick with two settings. We will look at both below.

**The callback.** `detector.on_detect_all(on_detections)` says "call this function when you see things". The Brick passes in one dict. Each key is a label, and each value is a list of detections, like this.

```python
{"person": [{"confidence": 0.87, "bounding_box_xyxy": (10, 20, 110, 220)}]}
```
{: .no-snakie }

So `detections.get("person", [])` is the list of people, or an empty list if there are none, and `len(...)` counts them. If you only care about one label, `detector.on_detect("person", function)` reacts to just that label.

The callback follows the rule from the last two lessons: it **stores** the count in `people`, plus the time in `last_seen`, and leaves the Bridge calls to `loop()`.

**Handling no news.** This is the part that catches people out. The Brick only reports when it sees something. When the room empties, it does not call you with "zero people". It goes quiet.

So the code remembers **when** it last heard anything (`last_seen`). In `loop()`, `time.time() - last_seen < 3` asks "did we hear in the last 3 seconds?". If not, the count is 0. **No news has to be handled by you**, and a timeout is a simple way to do it.

**The loop.** It sends the count to the matrix with `Bridge.call("draw", ...)`, and sets LED 4 with `Bridge.notify("set_status", count > 0, count == 0, False)`: red when someone is there, green when nobody is.

The `print` line writes which labels the camera can see to the **Python** tab, which is handy for checking what the model thinks it sees.

---

## What is confidence?

Every detection comes with a **confidence** score between 0 and 1. It is the model's own estimate of how sure it is that the label is right. A score of 0.87 means "pretty sure". A score of 0.35 means "something like a person, perhaps".

The `confidence` setting is the **lowest score to accept**. Detections below it are ignored. The Brick's own default is 0.3, and our code asks for 0.5.

- Set it **low**, and you catch more real people, but you also get false alarms from coat stands and shadows.
- Set it **high**, and the false alarms go away, but you start missing real people.

There is no perfect value. It is a trade-off, and you tune it for your room. Try it below.

{% include confidence_demo.html %}

Drag the threshold slider up and down. Watch which detections pass, and how the people count changes.

The other setting, `debounce_sec`, is the minimum time between repeat reports of the same thing. The Brick's default is 0. We use 1.0, so one person does not cause a flood of reports.

---

## Run it

1. Plug the hub into the board's USB-C port, and the webcam into the hub.
2. Power the hub with a suitable supply.
3. Copy the sketch and `main.py` into your Q Board App, and add the **Video Object Detection** Brick.
4. Press **Run**. The first run is slow while the Brick's container starts.
5. Stand in front of the camera. The matrix should show 1, and LED 4 should turn red.
6. Walk away. After about 3 seconds the matrix shows 0 and LED 4 turns green.

You have reached **Q Board stage 7**.

---

> **Teachers**: Allow about 60 minutes (our estimate). This lesson needs a USB webcam and a powered USB-C hub with power delivery for each board, which is extra cost and usually shared kit. Apple USB-C multiport adapters are not supported. A camera pointed at a room captures people, so agree who may be in view before you run it. With no camera, learners can follow the confidence demo, and the rest of the course does not depend on this lesson.

## Try it Yourself

1. **Tune the confidence.** Change `confidence=0.5` to `0.3`, then to `0.7`, and **Run** each time. Which value works best in your room?
2. **Change the timeout.** Change the 3 in `loop()` to 1, then to 6. What happens to the LED when you leave the room?
3. **Challenge: count something else.** Change `"person"` to another label the model reports, such as `"bicycle"`. Print `detections.keys()` first to see what your camera finds.

---

## Common Issues

**Problem**: No camera is found.
**Solution**: Check the hub is powered, then type `ls /dev/video*` on the board and see if the camera is listed.
**Why**: The board has one USB-C port, so a hub with no power, or a camera it cannot see, leaves the Brick with no picture.

**Problem**: It counts 0 with someone in view.
**Solution**: Lower `confidence` a little, and improve the light.
**Why**: In poor light the model's scores are lower, and anything under `confidence` is ignored.

**Problem**: The count flickers.
**Solution**: Raise `confidence` a little, or raise `debounce_sec`.
**Why**: Scores that hover near the threshold pass and fail from one frame to the next.

**Problem**: The first run is slow.
**Solution**: Wait, and watch the Brick's own tab in the console.
**Why**: The Brick runs in its own container, and it has to start before it can see anything.

---

## Check your understanding

{% quiz %}
? Why did the sketch not need to change when we added AI?
- [ ] The sketch already contained the AI model
- [x] The camera and model are Linux jobs, and the sketch only draws and sets LEDs
- [ ] The AI Brick replaces the Bridge
> Split well, the new feature lands on one side. Python does the camera and AI, and the sketch keeps the matrix and the LED.

? What does the `confidence` setting do?
- [ ] It sets how many people can be counted
- [x] It sets the lowest score the Brick will accept for a detection
- [ ] It sets how often the camera takes a picture
> Detections scoring below `confidence` are ignored. Higher means fewer false alarms but more missed people.

? Why does the code keep `last_seen` and treat 3 seconds of silence as 0 people?
- [ ] The camera needs 3 seconds to warm up
- [x] The Brick only reports when it sees something, so an empty room means no news
- [ ] `debounce_sec` makes the Brick stop after 3 seconds
> "No news" has to be handled by you. A timeout turns silence into a count of 0.
{% endquiz %}
