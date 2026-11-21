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

## Tips

- Put a widget straight after the paragraph that explains the idea, and follow it with one sentence saying what to try.
- A widget is not the real hardware. Say so, and end the section with the step to do on the board.
