# Course widgets

Small interactive pieces for course lessons (`source/<course>/*.md`). Each is one `{% include %}`. They need no backend and store nothing; without JavaScript the reader sees a short plain-text fallback.

Code: `web/assets/js/course-widgets.js`, `web/assets/css/course-widgets.css`, includes in `web/_includes/`. The first widget on a page loads the script and stylesheet (`course_widget_assets.html`).

## LED matrix painter

For the Arduino UNO Q's 8 x 13 LED matrix. The reader clicks or drags to paint, picks one of 7 brightness levels, and copies a complete sketch that uses `Arduino_LED_Matrix`.

```liquid
{% include matrix_painter.html %}
{% include matrix_painter.html preset="Heart" name="heart" %}
```

| Option | Meaning |
| --- | --- |
| `preset` | Starting picture: `Heart`, `Smile`, `Arrow` or `Fade` |
| `name` | Name of the C array in the generated sketch (default `frame`) |

Unticking "8 brightness levels" switches the sketch to `setGrayscaleBits(1)` and an array of 0s and 1s.

## Bridge step-through

Walks one Bridge (RPC) message between Python, the Arduino Router and the sketch, one step per press of **Next**, showing the code that runs and the message on the wire.

```liquid
{% include bridge_steps.html %}
{% include bridge_steps.html scenario="deadlock" %}
```

| Scenario | Shows |
| --- | --- |
| `call` (default) | `Bridge.call("set_led_state", True)` from Python to the sketch and back |
| `deadlock` | Why printing inside a `Bridge.provide()` function hangs, and the `provide_safe()` fix |

The steps are in `SCENARIOS` in `course-widgets.js`. The facts in them come from Arduino's Bridge API reference and the UNO Q user manual: check both before changing a step.

## Sorter

The reader sorts items into two buckets, one at a time, and gets the reason after each answer. Not tied to any board: use it wherever "which of these two?" is the lesson.

```liquid
{% capture items %}
Read a wheel encoder 10,000 times a second | right | It needs exact timing, and Linux can pause your program at any moment.
Serve a web page | left | Networking and web servers are what Linux is for.
{% endcapture %}
{% include sorter.html title="Which brain?" left="Linux side" right="Microcontroller" items=items %}
```

One item per line: `text | left or right | why`. Don't use `|` inside the text. Six to eight items works well. The sorter doesn't count towards the lesson's quiz score, so keep a `{% quiz %}` at the end of the lesson as well.

## Uno Q simulator

A stand-in Arduino UNO Q for lessons that use the Bridge. The reader edits `python/main.py` and presses **Run**. The Python runs in the browser (Pyodide, in a worker, loaded only when Run is pressed) against a stand-in `arduino.app_utils`, and a small JavaScript "sketch" plays the lesson's `sketch.ino`. The box draws the LED matrix, the four LEDs, the Python output and a count of every message that crossed the Bridge.

```liquid
{% include uno_q_sim.html sketch="clock" %}
```

With no `code`, the box starts with the nearest Python code block above it, so the lesson shows its code once and the checker still sees it. Pass `code=` (from a `{% capture %}`) to start with something else.

| Sketch | Plays |
| --- | --- |
| `blink` | Blinks LED 3 green every half second. Provides nothing |
| `clock` (default) | Provides `draw` (104 values) and `set_status` (red, green, blue for LED 4) |
| `sensor` | As `clock`, and once a second calls `get_limit`, notifies `on_reading` with the slider value and lights LED 3 red above the limit |
| `webpage` | As `sensor`, and provides `set_limit`. Adds a pretend phone page that asks `GET /status` each second and sends `POST /limit` from its slider, through a stand-in for the WebUI Brick's `expose_api` |

The stand-in keeps the rules that matter: an unknown name raises `ValueError`, `Bridge.call()` inside a provided function raises `RuntimeError`, and a `main.py` that never reaches `App.run()` stops. It does not run C++ and its timing is loose, so every lesson that uses it should say so and end with a run on the real board. To add a sketch, add it to `SIM_SKETCHES` and to `provides` or `sketchTick` in `simWorker` in `course-widgets.js`.

## Confidence demo

A threshold slider over six made-up object detections. Shows which ones the Video Object Detection Brick would report at that `confidence`, and how many people the lesson's code would count.

```liquid
{% include confidence_demo.html %}
```

## Cost break-even

Two number boxes: a one-off cost and a monthly cost. Shows the months until the one-off purchase has paid for itself.

```liquid
{% include cost_breakeven.html board="53" monthly="5" %}
```

## Move detector

A chess board in the opening position. The reader makes a move, sees how much each square "changed", and sees the 20 legal moves scored until one is left. A switch adds camera noise and a brushed piece. It is a simplified picture of the detector in Kev's chess camera: the idea, not his code.

```liquid
{% include move_detector.html %}
```

## Tips

- Put a widget straight after the paragraph that explains the idea, and follow it with one sentence saying what to try.
- A widget is not the real hardware. Say so, and end the section with the step to do on the board.
