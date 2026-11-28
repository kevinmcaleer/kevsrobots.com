/* Interactive widgets for course lessons. See docs/course_widgets.md.
 *
 *   {% include matrix_painter.html %}     8 x 13 LED matrix painter (Arduino UNO Q)
 *   {% include bridge_steps.html %}       step through a Bridge RPC call
 *   {% include sorter.html %}             sort items into two buckets
 *
 * Each include writes a <div data-kr-widget="..."> and loads this file once.
 * No dependencies, no network, nothing stored: a widget that fails to start
 * leaves its plain-HTML fallback on the page.
 */
(function () {
  'use strict';
  if (window.krCourseWidgets) return;

  function el(tag, attrs, children) {
    var node = document.createElement(tag);
    Object.keys(attrs || {}).forEach(function (k) {
      if (k === 'text') node.textContent = attrs[k];
      else if (k === 'class') node.className = attrs[k];
      else if (k.slice(0, 2) === 'on') node.addEventListener(k.slice(2), attrs[k]);
      else if (attrs[k] !== false && attrs[k] != null) node.setAttribute(k, attrs[k]);
    });
    (children || []).forEach(function (c) { if (c) node.appendChild(c); });
    return node;
  }

  function copyText(text, button) {
    var done = function () {
      var old = button.textContent;
      button.textContent = 'Copied';
      setTimeout(function () { button.textContent = old; }, 1500);
    };
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(text).then(done, function () {});
    }
  }

  // ---------------------------------------------------------------- matrix
  // The UNO Q's LED matrix is 8 rows by 13 columns, drawn from a flat array of
  // 104 values, row by row. With setGrayscaleBits(3) each value is 0 to 7;
  // with setGrayscaleBits(1) it is 0 or 1. (UNO Q user manual, "LED Matrix".)
  var ROWS = 8, COLS = 13;

  var PRESETS = {
    Heart: [
      '.............',
      '...11...11...',
      '..1111.1111..',
      '..111111111..',
      '...1111111...',
      '....11111....',
      '.....111.....',
      '......1......'],
    Smile: [
      '.............',
      '...11...11...',
      '...11...11...',
      '.............',
      '..1.......1..',
      '...1.....1...',
      '....11111....',
      '.............'],
    Arrow: [
      '......1......',
      '.....111.....',
      '....11111....',
      '...1111111...',
      '.....111.....',
      '.....111.....',
      '.....111.....',
      '.....111.....'],
    Fade: [
      '0000000000000',
      '1111111111111',
      '2222222222222',
      '3333333333333',
      '4444444444444',
      '5555555555555',
      '6666666666666',
      '7777777777777']
  };

  function matrixPainter(root) {
    var cells = [], values = [], level = 7, painting = false, grey = true;
    var name = (root.getAttribute('data-name') || 'frame').replace(/[^A-Za-z0-9_]/g, '') || 'frame';
    for (var i = 0; i < ROWS * COLS; i++) values.push(0);

    var grid = el('div', { class: 'kr-mx-grid', role: 'grid', 'aria-label': 'LED matrix, 8 rows by 13 columns' });
    var out = el('textarea', { class: 'kr-mx-code', readonly: 'readonly', rows: '14', spellcheck: 'false',
      'aria-label': 'Sketch code for this frame' });
    var status = el('p', { class: 'kr-mx-status', 'aria-live': 'polite' });

    function shown(v) { return grey ? v : (v ? 1 : 0); }

    function paint(i, v) {
      values[i] = v;
      var s = shown(v), max = grey ? 7 : 1;
      cells[i].style.setProperty('--kr-led', String(s / max));
      cells[i].setAttribute('aria-label', 'Row ' + (Math.floor(i / COLS) + 1) + ', column ' + (i % COLS + 1) +
        ', brightness ' + s);
      cells[i].classList.toggle('is-on', s > 0);
    }

    function code() {
      var lines = [];
      for (var r = 0; r < ROWS; r++) {
        var row = [];
        for (var c = 0; c < COLS; c++) row.push(shown(values[r * COLS + c]));
        lines.push('    ' + row.join(',') + (r < ROWS - 1 ? ',' : ''));
      }
      return '#include <Arduino_LED_Matrix.h>\n\n' +
        'uint8_t ' + name + '[104] = {\n' + lines.join('\n') + '\n};\n\n' +
        'Arduino_LED_Matrix matrix;\n\n' +
        'void setup() {\n  matrix.begin();\n' +
        '  matrix.setGrayscaleBits(' + (grey ? 3 : 1) + ');  // ' +
        (grey ? '3 bits: brightness 0 to 7' : '1 bit: on or off') + '\n' +
        '  matrix.draw(' + name + ');\n}\n\nvoid loop() {\n}\n';
    }

    function refresh() {
      out.value = code();
      var lit = values.filter(function (v) { return shown(v) > 0; }).length;
      status.textContent = lit + ' of 104 LEDs lit';
    }

    function setAll(fn) {
      for (var i = 0; i < values.length; i++) paint(i, fn(values[i], i));
      refresh();
    }

    function apply(i) {
      paint(i, values[i] === level && !painting ? 0 : level);
      refresh();
    }

    for (var r = 0; r < ROWS; r++) {
      var rowEl = el('div', { class: 'kr-mx-row', role: 'row' });
      for (var c = 0; c < COLS; c++) {
        (function (i) {
          var b = el('button', { type: 'button', class: 'kr-mx-led', role: 'gridcell' });
          b.addEventListener('pointerdown', function (e) {
            e.preventDefault();
            painting = false;
            apply(i);
            painting = true;
            if (b.releasePointerCapture && e.pointerId != null) {
              try { b.releasePointerCapture(e.pointerId); } catch (err) { /* not captured */ }
            }
          });
          b.addEventListener('pointerenter', function () { if (painting) { paint(i, level); refresh(); } });
          b.addEventListener('keydown', function (e) {
            var to = { ArrowRight: i + 1, ArrowLeft: i - 1, ArrowDown: i + COLS, ArrowUp: i - COLS }[e.key];
            if (to != null && to >= 0 && to < cells.length) { e.preventDefault(); cells[to].focus(); }
            if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); painting = false; apply(i); }
          });
          cells.push(b);
          rowEl.appendChild(b);
        })(r * COLS + c);
      }
      grid.appendChild(rowEl);
    }
    document.addEventListener('pointerup', function () { painting = false; });
    grid.addEventListener('pointerleave', function () { painting = false; });

    var levels = el('div', { class: 'kr-mx-levels', role: 'radiogroup', 'aria-label': 'Brush brightness' });
    var levelButtons = [];
    function pickLevel(v) {
      level = v;
      levelButtons.forEach(function (b, n) {
        b.classList.toggle('is-picked', n + 1 === v);
        b.setAttribute('aria-checked', n + 1 === v ? 'true' : 'false');
      });
    }
    for (var v = 1; v <= 7; v++) {
      (function (v) {
        var b = el('button', { type: 'button', class: 'kr-mx-level', role: 'radio', text: String(v),
          title: 'Brightness ' + v, onclick: function () { pickLevel(v); } });
        b.style.setProperty('--kr-led', String(v / 7));
        levelButtons.push(b);
        levels.appendChild(b);
      })(v);
    }

    var greyBox = el('input', { type: 'checkbox', id: 'kr-mx-grey-' + Math.random().toString(36).slice(2), checked: 'checked' });
    greyBox.addEventListener('change', function () {
      grey = greyBox.checked;
      levels.hidden = !grey;
      if (!grey) pickLevel(7);
      setAll(function (v) { return v; });
    });
    var greyLabel = el('label', { for: greyBox.id, text: ' 8 brightness levels' });

    var presetBar = el('div', { class: 'kr-mx-buttons' });
    Object.keys(PRESETS).forEach(function (key) {
      presetBar.appendChild(el('button', { type: 'button', class: 'kr-w-btn', text: key, onclick: function () {
        var art = PRESETS[key].join('');
        setAll(function (old, i) {
          var ch = art.charAt(i);
          if (ch === '.') return 0;
          var n = parseInt(ch, 10);
          return key === 'Fade' ? n : (n ? level : 0);
        });
      } }));
    });
    var tools = el('div', { class: 'kr-mx-buttons' }, [
      el('button', { type: 'button', class: 'kr-w-btn', text: 'Clear', onclick: function () { setAll(function () { return 0; }); } }),
      el('button', { type: 'button', class: 'kr-w-btn', text: 'Fill', onclick: function () { setAll(function () { return level; }); } }),
      el('button', { type: 'button', class: 'kr-w-btn', text: 'Invert', onclick: function () {
        setAll(function (v) { return grey ? 7 - v : (v ? 0 : 7); });
      } })
    ]);
    var copy = el('button', { type: 'button', class: 'kr-w-btn kr-w-btn-main', text: 'Copy sketch' });
    copy.addEventListener('click', function () { copyText(out.value, copy); });

    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: 'Try it: LED matrix painter' }),
      el('span', { class: 'kr-w-hint', text: 'Click or drag to paint. Click a lit LED again to turn it off.' })
    ]));
    root.appendChild(el('div', { class: 'kr-mx-layout' }, [
      el('div', { class: 'kr-mx-board' }, [grid, status]),
      el('div', { class: 'kr-mx-side' }, [
        el('div', { class: 'kr-mx-control' }, [el('span', { class: 'kr-mx-caption', text: 'Brush' }), levels]),
        el('div', { class: 'kr-mx-control' }, [greyBox, greyLabel]),
        el('div', { class: 'kr-mx-control' }, [el('span', { class: 'kr-mx-caption', text: 'Start from' }), presetBar]),
        el('div', { class: 'kr-mx-control' }, [tools])
      ])
    ]));
    root.appendChild(el('div', { class: 'kr-mx-out' }, [
      el('div', { class: 'kr-mx-out-head' }, [
        el('span', { class: 'kr-mx-caption', text: 'sketch/sketch.ino' }), copy
      ]),
      out
    ]));
    pickLevel(7);
    var start = root.getAttribute('data-preset');
    if (start && PRESETS[start]) {
      var art = PRESETS[start].join('');
      setAll(function (old, i) { var ch = art.charAt(i); return ch === '.' ? 0 : (start === 'Fade' ? +ch : 7); });
    } else {
      setAll(function () { return 0; });
    }
  }

  // ---------------------------------------------------------------- bridge
  // What happens to one Bridge message. Facts from Arduino's Bridge API
  // reference and UNO Q user manual: the arduino-router service, MessagePack
  // RPC, /var/run/arduino-router.sock on Linux, /dev/ttyHS1 at 115200 to the
  // microcontroller, request [0, id, method, params], response [1, id, error, result].
  var LANES = [
    { id: 'py', title: 'Python', sub: 'python/main.py on Linux' },
    { id: 'router', title: 'Arduino Router', sub: 'arduino-router service on Linux' },
    { id: 'mcu', title: 'Sketch', sub: 'sketch.ino on the microcontroller' }
  ];

  var SCENARIOS = {
    call: {
      title: 'Follow one call: Python turns on an LED',
      steps: [
        { at: 'mcu', to: 'router', code: 'Bridge.begin();\nBridge.provide("set_led_state", set_led_state);',
          wire: 'register "set_led_state"',
          text: 'The sketch starts first. It tells the router "I have a function called set_led_state". The router adds it to its list.' },
        { at: 'py', code: 'Bridge.call("set_led_state", True)',
          text: 'Your Python script calls the function by name, as if it were a normal Python function. The script now waits for an answer.' },
        { at: 'py', to: 'router', wire: '[0, 1, "set_led_state", [true]]',
          text: 'The Bridge packs the call into a small MessagePack message: 0 means "request", 1 is its id, then the name and the arguments. It goes to the router through a socket file, /var/run/arduino-router.sock.' },
        { at: 'router', to: 'mcu', wire: '[0, 1, "set_led_state", [true]]',
          text: 'The router looks the name up, sees the sketch owns it, and sends the message down the serial link between the two chips (/dev/ttyHS1, 115200 baud).' },
        { at: 'mcu', code: 'void set_led_state(bool state) {\n  digitalWrite(LED_BUILTIN, state ? LOW : HIGH);\n}', led: true,
          text: 'The sketch unpacks the message and runs your function with state = true. The LED turns on.' },
        { at: 'mcu', to: 'py', wire: '[1, 1, null, null]', led: true,
          text: 'A response goes back the same way: 1 means "response", the same id 1, no error, and no result because the function returns void.' },
        { at: 'py', code: 'Bridge.call("set_led_state", True)  # returns', led: true,
          text: 'Bridge.call() returns and your script carries on with its next line.' }
      ]
    },
    deadlock: {
      title: 'Follow the bug: a print inside provide()',
      steps: [
        { at: 'mcu', code: 'Bridge.provide("set_led_state", set_led_state);\n\nvoid set_led_state(bool state) {\n  digitalWrite(LED_BUILTIN, state ? LOW : HIGH);\n  Serial.println("LED changed");  // the bug\n}',
          text: 'This sketch looks harmless. It prints a message every time the LED changes. The function is registered with provide().' },
        { at: 'py', to: 'mcu', wire: '[0, 1, "set_led_state", [true]]',
          text: 'Python calls it. The request travels through the router to the sketch, exactly as before.' },
        { at: 'mcu', code: 'set_led_state(true)   // running in the Bridge thread', led: true,
          text: 'A provide() function runs inside the Bridge\'s own background thread, the one that handles messages. The LED turns on. So far so good.' },
        { at: 'mcu', code: 'Serial.println("LED changed");', led: true, stuck: true,
          text: 'Now the print. On the UNO Q, Serial output travels over the Bridge too. So the function tries to start a new message while the Bridge is still busy answering the first one. Arduino\'s documentation warns that this can deadlock: each side waits for the other.' },
        { at: 'py', code: 'Bridge.call("set_led_state", True)  # still waiting...', led: true, stuck: true,
          text: 'When that happens, no response comes back. After 10 seconds Bridge.call() gives up and raises TimeoutError, and the Bridge on the sketch side stays hung.' },
        { at: 'mcu', code: 'Bridge.provide_safe("set_led_state", set_led_state);', fixed: true,
          text: 'The fix is one word. provide_safe() runs your function inside loop() instead of inside the Bridge thread, so it is free to print or call the Bridge. Use provide_safe() whenever a function does more than set a variable.' }
      ]
    }
  };

  function bridgeSteps(root) {
    var scenario = SCENARIOS[root.getAttribute('data-scenario')] || SCENARIOS.call;
    var n = 0;
    var laneEls = {}, codeEls = {};
    var lanes = el('div', { class: 'kr-br-lanes' });
    LANES.forEach(function (lane, i) {
      var code = el('pre', { class: 'kr-br-code' });
      var box = el('div', { class: 'kr-br-lane', 'data-lane': lane.id }, [
        el('div', { class: 'kr-br-lane-title', text: lane.title }),
        el('div', { class: 'kr-br-lane-sub', text: lane.sub }),
        code,
        lane.id === 'mcu' ? el('div', { class: 'kr-br-led', title: 'The LED' }) : null
      ]);
      laneEls[lane.id] = box;
      codeEls[lane.id] = code;
      lanes.appendChild(box);
      if (i < LANES.length - 1) lanes.appendChild(el('div', { class: 'kr-br-gap', 'data-gap': String(i) }));
    });
    var wire = el('div', { class: 'kr-br-wire' });
    var text = el('p', { class: 'kr-br-text', 'aria-live': 'polite' });
    var count = el('span', { class: 'kr-br-count' });
    var back = el('button', { type: 'button', class: 'kr-w-btn', text: 'Back' });
    var next = el('button', { type: 'button', class: 'kr-w-btn kr-w-btn-main', text: 'Next' });

    function show() {
      var s = scenario.steps[n];
      var order = ['py', 'router', 'mcu'];
      LANES.forEach(function (lane) {
        laneEls[lane.id].classList.toggle('is-active', lane.id === s.at || lane.id === s.to);
        laneEls[lane.id].classList.toggle('is-stuck', !!s.stuck && (lane.id === s.at));
        laneEls[lane.id].classList.toggle('is-fixed', !!s.fixed && lane.id === s.at);
        codeEls[lane.id].textContent = lane.id === s.at && s.code ? s.code : '';
        codeEls[lane.id].hidden = !(lane.id === s.at && s.code);
      });
      root.querySelector('.kr-br-led').classList.toggle('is-on', !!s.led);
      var a = order.indexOf(s.at), b = s.to ? order.indexOf(s.to) : -1;
      root.querySelectorAll('.kr-br-gap').forEach(function (gap, i) {
        var crossing = b >= 0 && i >= Math.min(a, b) && i < Math.max(a, b);
        gap.classList.toggle('is-crossing', crossing);
        gap.textContent = crossing ? (b > a ? '→' : '←') : '';
      });
      wire.textContent = s.wire ? 'On the wire: ' + s.wire : '';
      wire.hidden = !s.wire;
      text.textContent = s.text;
      count.textContent = 'Step ' + (n + 1) + ' of ' + scenario.steps.length;
      back.disabled = n === 0;
      next.textContent = n === scenario.steps.length - 1 ? 'Start again' : 'Next';
    }
    back.addEventListener('click', function () { if (n > 0) { n--; show(); } });
    next.addEventListener('click', function () { n = n === scenario.steps.length - 1 ? 0 : n + 1; show(); });

    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: scenario.title })
    ]));
    root.appendChild(lanes);
    root.appendChild(wire);
    root.appendChild(text);
    root.appendChild(el('div', { class: 'kr-br-nav' }, [back, next, count]));
    show();
  }

  // ---------------------------------------------------------------- sorter
  // Items come from the include as <li data-answer="left|right" data-why="...">.
  function sorter(root) {
    var left = root.getAttribute('data-left') || 'Left';
    var right = root.getAttribute('data-right') || 'Right';
    var title = root.getAttribute('data-title') || 'Sort it';
    var items = [];
    root.querySelectorAll('li[data-answer]').forEach(function (li) {
      items.push({ text: li.textContent.trim(), answer: li.getAttribute('data-answer'), why: li.getAttribute('data-why') || '' });
    });
    if (!items.length) return;
    var n = 0, score = 0;
    var card = el('div', { class: 'kr-so-card' });
    var why = el('p', { class: 'kr-so-why', 'aria-live': 'polite' });
    var progress = el('span', { class: 'kr-so-progress' });
    var lists = { left: el('ul', { class: 'kr-so-list' }), right: el('ul', { class: 'kr-so-list' }) };
    var buttons = {
      left: el('button', { type: 'button', class: 'kr-w-btn kr-so-pick', text: left }),
      right: el('button', { type: 'button', class: 'kr-w-btn kr-so-pick', text: right })
    };
    var nextBtn = el('button', { type: 'button', class: 'kr-w-btn kr-w-btn-main', text: 'Next', hidden: 'hidden' });

    function show() {
      why.textContent = '';
      why.className = 'kr-so-why';
      nextBtn.hidden = true;
      if (n >= items.length) {
        card.textContent = 'All sorted: ' + score + ' of ' + items.length + ' right first time.';
        card.classList.add('is-done');
        buttons.left.hidden = buttons.right.hidden = true;
        nextBtn.hidden = false;
        nextBtn.textContent = 'Start again';
        progress.textContent = '';
        return;
      }
      card.classList.remove('is-done');
      card.textContent = items[n].text;
      buttons.left.hidden = buttons.right.hidden = false;
      buttons.left.disabled = buttons.right.disabled = false;
      progress.textContent = (n + 1) + ' of ' + items.length;
    }

    function pick(side) {
      var item = items[n];
      var ok = side === item.answer;
      if (ok) score++;
      buttons.left.disabled = buttons.right.disabled = true;
      why.textContent = (ok ? 'Yes. ' : 'Not quite: this one belongs on the ' +
        (item.answer === 'left' ? left : right) + '. ') + item.why;
      why.classList.add(ok ? 'is-right' : 'is-wrong');
      lists[item.answer].appendChild(el('li', { text: item.text, class: ok ? '' : 'was-wrong' }));
      nextBtn.hidden = false;
      nextBtn.textContent = n === items.length - 1 ? 'See my score' : 'Next';
      nextBtn.focus();
    }
    buttons.left.addEventListener('click', function () { pick('left'); });
    buttons.right.addEventListener('click', function () { pick('right'); });
    nextBtn.addEventListener('click', function () {
      if (n >= items.length) {
        n = 0; score = 0;
        lists.left.textContent = ''; lists.right.textContent = '';
        nextBtn.textContent = 'Next';
      } else {
        n++;
      }
      show();
    });

    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: title }), progress
    ]));
    root.appendChild(card);
    root.appendChild(el('div', { class: 'kr-so-picks' }, [buttons.left, buttons.right, nextBtn]));
    root.appendChild(why);
    root.appendChild(el('div', { class: 'kr-so-buckets' }, [
      el('div', { class: 'kr-so-bucket' }, [el('div', { class: 'kr-so-bucket-title', text: left }), lists.left]),
      el('div', { class: 'kr-so-bucket' }, [el('div', { class: 'kr-so-bucket-title', text: right }), lists.right])
    ]));
    show();
  }

  // ------------------------------------------------------------- simulator
  // A stand-in UNO Q for lessons that use the Bridge. The reader's main.py
  // runs in the browser (Pyodide, in a worker) against a stand-in
  // arduino.app_utils, and a small JavaScript "sketch" plays the lesson's
  // sketch.ino: it provides the same Bridge names and drives a drawn LED
  // matrix and LEDs. It is not the board: no C++ runs and timing is loose.
  var PYODIDE_BASE = 'https://cdn.jsdelivr.net/pyodide/v0.26.4/full/';

  var SIM_SKETCHES = {
    blink: { text: 'The sketch blinks LED 3 green every half second. It provides nothing over the Bridge.', pot: false },
    clock: { text: 'The sketch provides draw (104 values) and set_status (red, green, blue for LED 4).', pot: false },
    sensor: { text: 'The sketch provides draw and set_status. Once a second it calls get_limit (until it gets an answer), notifies on_reading with the knob value, and lights LED 3 red above the limit.', pot: true },
    webpage: { text: 'The sketch does what the sensor sketch does, and also provides set_limit. The phone stands in for assets/index.html: it asks GET /status once a second and sends POST /limit when you move its slider.', pot: true, page: true }
  };

  // Stand-in for the WebUI - HTML Brick: only the REST side (expose_api).
  var SIM_WEBUI_STUB = [
    'import json',
    '',
    '_routes = {}',
    '',
    'class WebUI:',
    '    def __init__(self, **options):',
    '        pass',
    '',
    '    def expose_api(self, method, path, function):',
    '        if not callable(function):',
    '            raise TypeError("expose_api(method, path, function)")',
    '        _routes[(method.upper(), path)] = function',
    '',
    '    def on_message(self, message_type, callback):',
    '        pass',
    '',
    '    def on_connect(self, callback):',
    '        pass',
    '',
    '    def on_disconnect(self, callback):',
    '        pass',
    '',
    '    def send_message(self, message_type, message=None, room=None):',
    '        pass',
    '',
    'def _request(method, path, value):',
    '    function = _routes.get((method, path))',
    '    if function is None:',
    '        return ""',
    '    result = function() if value is None else function(int(value))',
    '    return json.dumps(result)',
    ''
  ].join('\n');

  var SIM_STUB = [
    'import _krsim',
    'from pyodide.ffi import to_js as _to_js',
    '',
    '__all__ = ["App", "Bridge", "Leds"]',
    '',
    'class AppError(Exception):',
    '    pass',
    '',
    'class _Started(BaseException):',
    '    pass',
    '',
    'class _Bridge:',
    '    def __init__(self):',
    '        self._handlers = {}',
    '        self._in_handler = False',
    '',
    '    def provide(self, method_name, handler):',
    '        if not callable(handler):',
    '            raise ValueError("handler is not callable")',
    '        self._handlers[method_name] = handler',
    '',
    '    def _send(self, kind, method_name, args):',
    '        error = _krsim.send(kind, method_name, _to_js(list(args)))',
    '        if error and kind == "call":',
    '            raise ValueError(error)',
    '',
    '    def call(self, method_name, *args, timeout=10):',
    '        if self._in_handler:',
    '            raise RuntimeError("Bridge.call() was used inside a function provided to the Bridge. Nested calls are not supported: store the value and call from loop() instead.")',
    '        self._send("call", method_name, args)',
    '',
    '    def notify(self, method_name, *args):',
    '        self._send("notify", method_name, args)',
    '',
    '    def _has(self, method_name):',
    '        return method_name in self._handlers',
    '',
    '    def _dispatch(self, method_name, *args):',
    '        self._in_handler = True',
    '        try:',
    '            return self._handlers[method_name](*args)',
    '        finally:',
    '            self._in_handler = False',
    '',
    'Bridge = _Bridge()',
    '',
    'class _App:',
    '    def run(self, user_loop=None):',
    '        if user_loop is not None and not callable(user_loop):',
    '            raise TypeError("user_loop must be a function")',
    '        _krsim.started(user_loop)',
    '        raise _Started()',
    '',
    'App = _App()',
    '',
    'class _Leds:',
    '    def _set(self, number, r, g, b):',
    '        for value in (r, g, b):',
    '            if value not in (0, 1, True, False):',
    '                raise ValueError("LED colour values are 0 or 1")',
    '        _krsim.led(number, bool(r), bool(g), bool(b))',
    '',
    '    def set_led1_color(self, r, g, b):',
    '        self._set(1, r, g, b)',
    '',
    '    def set_led2_color(self, r, g, b):',
    '        self._set(2, r, g, b)',
    '',
    'Leds = _Leds()',
    ''
  ].join('\n');

  // Runs inside the worker. Self-contained: it is turned into source text.
  function simWorker(cfg) {
    var py = null, bridge = null, webui = null, loopFn = null, started = false, stopped = false;
    var sensing = cfg.sketch === 'sensor' || cfg.sketch === 'webpage';
    var state = { matrix: [], led1: [0, 0, 0], led2: [0, 0, 0], led3: [0, 0, 0], led4: [0, 0, 0] };
    for (var i = 0; i < 104; i++) state.matrix.push(0);
    var dirty = true, out = [], traffic = {}, trafficDirty = false, pot = 50;
    var sk = { limit: 50, haveLimit: false, last: 0 };

    function say(text) { out.push(String(text)); if (out.length > 200) out.splice(0, out.length - 200); }
    function count(dir, kind, name, detail) {
      var key = dir + kind + name;
      var row = traffic[key] || (traffic[key] = { dir: dir, kind: kind, name: name, count: 0, detail: '' });
      row.count++; row.detail = detail; trafficDirty = true;
    }
    function flush() {
      if (!dirty && !out.length && !trafficDirty) return;
      var rows = null;
      if (trafficDirty) { rows = []; for (var k in traffic) rows.push(traffic[k]); }
      self.postMessage({ type: 'update', state: dirty ? state : null, out: out, traffic: rows });
      dirty = false; out = []; trafficDirty = false;
    }
    function describe(args) {
      var parts = [];
      for (var i = 0; i < args.length; i++) {
        var a = args[i];
        if (a && typeof a === 'object' && typeof a.length === 'number') parts.push(a.length + (a instanceof Uint8Array ? ' bytes' : ' values'));
        else if (a === true) parts.push('True');
        else if (a === false) parts.push('False');
        else parts.push(String(a));
      }
      return parts.join(', ');
    }
    // The reader's part of a Python traceback: from their file down, without
    // the frames inside Pyodide or the stand-in library.
    function lastLine(err) {
      var lines = String(err && err.message ? err.message : err).split('\n').filter(function (l) { return l.trim(); });
      var from = -1;
      for (var i = 0; i < lines.length; i++) { if (lines[i].indexOf('File "<exec>"') !== -1) { from = i; break; } }
      if (from === -1) return lines.slice(-1).join('\n');
      var keep = [];
      for (var j = from; j < lines.length; j++) {
        if (lines[j].indexOf('File "/') !== -1) { j++; continue; }
        keep.push(lines[j].replace('File "<exec>"', 'File "main.py"'));
      }
      return keep.slice(-8).join('\n');
    }

    // What the lesson's sketch provides, played in JavaScript.
    var provides = {};
    if (cfg.sketch === 'clock' || sensing) {
      provides.draw = function (args) {
        if (args.length !== 1) return 'draw takes 1 argument, the frame';
        var frame = args[0];
        if (!frame || typeof frame !== 'object' || typeof frame.length !== 'number') return 'draw expects the frame as bytes';
        var n = Math.min(frame.length, 104);
        for (var i = 0; i < n; i++) state.matrix[i] = Math.max(0, Math.min(7, frame[i] | 0));
        dirty = true;
        return null;
      };
      provides.set_status = function (args) {
        if (args.length !== 3) return 'set_status takes 3 arguments: red, green, blue';
        state.led4 = [args[0] ? 1 : 0, args[1] ? 1 : 0, args[2] ? 1 : 0];
        dirty = true;
        return null;
      };
    }
    if (cfg.sketch === 'webpage') {
      provides.set_limit = function (args) {
        if (args.length !== 1 || typeof args[0] !== 'number') return 'set_limit takes 1 argument, a whole number';
        sk.limit = Math.round(args[0]);
        sk.haveLimit = true;
        return null;
      };
    }

    function sketchTick() {
      var now = Date.now();
      if (cfg.sketch === 'blink') {
        var on = Math.floor(now / 500) % 2 === 0 ? 1 : 0;
        if (state.led3[1] !== on) { state.led3 = [0, on, 0]; dirty = true; }
        return;
      }
      if (!sensing || now - sk.last < 1000) return;
      sk.last = now;
      if (!sk.haveLimit) {
        count('up', 'call', 'get_limit', '');
        if (bridge && bridge._has('get_limit')) {
          try {
            var value = bridge._dispatch('get_limit');
            if (typeof value === 'number') { sk.limit = Math.round(value); sk.haveLimit = true; }
          } catch (err) { say('Error in get_limit:\n' + lastLine(err)); }
        }
      }
      count('up', 'notify', 'on_reading', String(pot));
      if (bridge && bridge._has('on_reading')) {
        try { bridge._dispatch('on_reading', pot); } catch (err) { say('Error in on_reading:\n' + lastLine(err)); }
      }
      var red = pot > sk.limit ? 1 : 0;
      if (state.led3[0] !== red) { state.led3 = [red, 0, 0]; dirty = true; }
    }

    var api = {
      send: function (kind, name, args) {
        var list = [];
        for (var i = 0; i < args.length; i++) list.push(args[i]);
        count('down', kind, name, describe(list));
        var fn = provides[name];
        if (!fn) return 'The simulated sketch does not provide "' + name + '". Check the spelling.';
        return fn(list);
      },
      started: function (loop) { started = true; loopFn = loop && loop.copy ? loop.copy() : (loop || null); },
      led: function (number, r, g, b) { state['led' + number] = [r ? 1 : 0, g ? 1 : 0, b ? 1 : 0]; dirty = true; }
    };

    function finish(message) {
      stopped = true;
      if (message) say(message);
      flush();
      self.postMessage({ type: 'done' });
    }
    function tick() {
      if (stopped) return;
      sketchTick();
      if (loopFn) {
        try { loopFn(); } catch (err) { finish(lastLine(err)); return; }
      }
      setTimeout(tick, loopFn ? 5 : 50);
    }
    function run(code) {
      try {
        py.runPython(code);
      } catch (err) {
        if (!started) { finish(lastLine(err)); return; }
      }
      if (!started) { finish('main.py reached the end without calling App.run(), so the App stopped.'); return; }
      bridge = py.pyimport('arduino.app_utils').Bridge;
      if (py.runPython('import sys; "arduino.app_bricks.web_ui" in sys.modules')) webui = py.pyimport('arduino.app_bricks.web_ui');
      tick();
    }

    self.onmessage = function (event) {
      var msg = event.data;
      if (msg.type === 'pot') { pot = msg.value; return; }
      if (msg.type === 'api') {
        // The pretend phone asking the App's web addresses.
        var body = null;
        if (webui && !stopped) {
          try { body = webui._request(msg.method, msg.path, msg.value); } catch (err) { say('Error in ' + msg.method + ' ' + msg.path + ':\n' + lastLine(err)); }
        }
        self.postMessage({ type: 'api', path: msg.path, body: body });
        return;
      }
      if (msg.type !== 'run') return;
      try {
        self.importScripts(cfg.base + 'pyodide.js');
      } catch (err) {
        self.postMessage({ type: 'failed' });
        return;
      }
      self.loadPyodide({ indexURL: cfg.base, stdout: say, stderr: say }).then(function (p) {
        py = p;
        py.registerJsModule('_krsim', api);
        py.FS.mkdirTree('/home/pyodide/arduino');
        py.FS.writeFile('/home/pyodide/arduino/__init__.py', '');
        py.FS.writeFile('/home/pyodide/arduino/app_utils.py', cfg.stub);
        py.FS.mkdirTree('/home/pyodide/arduino/app_bricks');
        py.FS.writeFile('/home/pyodide/arduino/app_bricks/__init__.py', '');
        py.FS.writeFile('/home/pyodide/arduino/app_bricks/web_ui.py', cfg.webui);
        self.postMessage({ type: 'ready' });
        setInterval(flush, 60);
        run(msg.code);
      }, function () { self.postMessage({ type: 'failed' }); });
    };
  }

  function unoQSim(root) {
    var kind = root.getAttribute('data-sketch');
    var sketch = SIM_SKETCHES[kind] || SIM_SKETCHES.clock;
    if (!SIM_SKETCHES[kind]) kind = 'clock';
    // The program: the include's own code, or else the nearest Python block
    // above the box, so a lesson shows its code once.
    var srcNode = root.querySelector('pre');
    var given = srcNode ? srcNode.textContent.replace(/^\n+|\s+$/g, '') : '';
    if (!given) {
      var blocks = document.querySelectorAll('.language-python pre, pre code.language-python');
      for (var b = blocks.length - 1; b >= 0; b--) {
        if (root.compareDocumentPosition(blocks[b]) & Node.DOCUMENT_POSITION_PRECEDING) {
          given = blocks[b].textContent.replace(/^\n+|\s+$/g, '');
          break;
        }
      }
    }
    var original = given + '\n';
    var worker = null;

    var editor = el('textarea', { class: 'kr-sim-editor', spellcheck: 'false', autocapitalize: 'off', autocomplete: 'off', 'aria-label': 'python/main.py', rows: Math.min(26, Math.max(8, original.split('\n').length)) });
    editor.value = original;
    editor.addEventListener('keydown', function (event) {
      if (event.key !== 'Tab' || event.shiftKey || event.altKey || event.ctrlKey || event.metaKey) return;
      event.preventDefault();
      var at = editor.selectionStart;
      editor.value = editor.value.slice(0, at) + '    ' + editor.value.slice(editor.selectionEnd);
      editor.selectionStart = editor.selectionEnd = at + 4;
    });

    var cells = [];
    var grid = el('div', { class: 'kr-mx-grid kr-sim-grid', role: 'img', 'aria-label': 'Simulated LED matrix' });
    for (var r = 0; r < ROWS; r++) {
      var row = el('div', { class: 'kr-mx-row' });
      for (var c = 0; c < COLS; c++) {
        var cell = el('span', { class: 'kr-mx-led' });
        cells.push(cell);
        row.appendChild(cell);
      }
      grid.appendChild(row);
    }
    var leds = {};
    var ledRow = el('div', { class: 'kr-sim-leds' });
    [['led1', 'LED 1', 'Linux'], ['led2', 'LED 2', 'Linux'], ['led3', 'LED 3', 'sketch'], ['led4', 'LED 4', 'sketch']].forEach(function (d) {
      leds[d[0]] = el('span', { class: 'kr-sim-led' });
      ledRow.appendChild(el('span', { class: 'kr-sim-led-box', title: d[1] + ' belongs to ' + (d[2] === 'Linux' ? 'Linux' : 'the sketch') }, [leds[d[0]], el('span', { text: d[1] })]));
    });

    var potValue = el('span', { text: '50' });
    var pot = el('input', { type: 'range', min: '0', max: '99', value: '50', class: 'kr-sim-pot', 'aria-label': 'Potentiometer on A0' });
    pot.addEventListener('input', function () {
      potValue.textContent = pot.value;
      if (worker) worker.postMessage({ type: 'pot', value: Number(pot.value) });
    });
    var potBox = sketch.pot ? el('label', { class: 'kr-sim-potbox' }, [el('span', { class: 'kr-mx-caption', text: 'Potentiometer on A0' }), pot, potValue]) : null;

    // The pretend phone (sketch="webpage"): what assets/index.html does.
    var phone = null, phoneReading, phoneLimit, phoneSlider, phoneState, phoneTimer = null;
    if (sketch.page) {
      phoneReading = el('div', { class: 'kr-sim-phone-reading', text: '--' });
      phoneLimit = el('span', { text: '50' });
      phoneSlider = el('input', { type: 'range', min: '0', max: '99', value: '50', 'aria-label': 'Alarm limit on the phone page' });
      phoneState = el('div', { class: 'kr-sim-phone-state', text: 'Not running' });
      phoneSlider.addEventListener('input', function () { phoneLimit.textContent = phoneSlider.value; });
      phoneSlider.addEventListener('change', function () {
        if (worker) worker.postMessage({ type: 'api', method: 'POST', path: '/limit', value: Number(phoneSlider.value) });
      });
      phone = el('div', { class: 'kr-sim-phone' }, [
        el('span', { class: 'kr-mx-caption', text: 'Phone: http://qboard.local:7000' }),
        el('div', { class: 'kr-sim-phone-title', text: 'Q Board' }),
        phoneReading,
        el('div', {}, [el('span', { text: 'Alarm limit: ' }), phoneLimit]),
        phoneSlider,
        phoneState
      ]);
    }
    function phoneAnswer(msg) {
      if (!phone || msg.path !== '/status') return;
      if (msg.body === null) { phoneState.textContent = 'Lost contact with the board'; return; }
      if (msg.body === '') { phoneState.textContent = 'The App has no /status address yet'; return; }
      var data = JSON.parse(msg.body);
      phoneReading.textContent = data.reading === null || data.reading === undefined ? '--' : data.reading;
      phone.classList.toggle('is-over', typeof data.reading === 'number' && data.reading > data.limit);
      if (document.activeElement !== phoneSlider && typeof data.limit === 'number') { phoneSlider.value = data.limit; phoneLimit.textContent = data.limit; }
      phoneState.textContent = 'Connected';
    }

    var status = el('span', { class: 'kr-w-hint', role: 'status', text: 'Press Run to start.' });
    var output = el('pre', { class: 'kr-sim-out', 'aria-label': 'Python output', tabindex: '0' });
    var traffic = el('div', { class: 'kr-sim-traffic' });
    var lines = [];

    function paint(state) {
      for (var i = 0; i < 104; i++) {
        var v = state.matrix[i] / 7;
        cells[i].style.setProperty('--kr-led', String(v));
        cells[i].classList.toggle('is-on', v > 0);
      }
      Object.keys(leds).forEach(function (name) {
        var rgb = state[name];
        var lit = rgb[0] || rgb[1] || rgb[2];
        leds[name].style.background = lit ? 'rgb(' + (rgb[0] ? 255 : 30) + ',' + (rgb[1] ? 220 : 30) + ',' + (rgb[2] ? 255 : 30) + ')' : '';
        leds[name].classList.toggle('is-on', !!lit);
      });
    }
    function blank() {
      var matrix = [];
      for (var i = 0; i < 104; i++) matrix.push(0);
      paint({ matrix: matrix, led1: [0, 0, 0], led2: [0, 0, 0], led3: [0, 0, 0], led4: [0, 0, 0] });
    }
    function showTraffic(rows) {
      traffic.textContent = '';
      if (!rows.length) return;
      traffic.appendChild(el('span', { class: 'kr-mx-caption', text: 'Messages on the Bridge' }));
      rows.forEach(function (row) {
        var who = row.dir === 'down' ? 'Python to sketch' : 'Sketch to Python';
        traffic.appendChild(el('div', { class: 'kr-sim-msg kr-sim-' + row.dir }, [
          el('span', { class: 'kr-sim-who', text: who }),
          el('code', { text: row.kind + ' ' + row.name + (row.detail ? ' (' + row.detail + ')' : '') }),
          el('span', { class: 'kr-sim-count', text: 'x ' + row.count })
        ]));
      });
    }
    function print(newLines) {
      newLines.forEach(function (text) { String(text).split('\n').forEach(function (l) { lines.push(l); }); });
      if (lines.length > 200) lines = lines.slice(-200);
      output.textContent = lines.join('\n');
      output.scrollTop = output.scrollHeight;
    }

    var runBtn = el('button', { type: 'button', class: 'kr-w-btn kr-w-btn-main', text: 'Run', onclick: run });
    var stopBtn = el('button', { type: 'button', class: 'kr-w-btn', text: 'Stop', disabled: 'disabled', onclick: function () { stop('Stopped.'); } });
    var resetBtn = el('button', { type: 'button', class: 'kr-w-btn', text: 'Reset code', onclick: function () { editor.value = original; } });

    function stop(message) {
      if (worker) { worker.terminate(); worker = null; }
      if (phoneTimer) { clearInterval(phoneTimer); phoneTimer = null; }
      if (phone) { phoneState.textContent = 'Lost contact with the board'; phone.classList.remove('is-over'); }
      runBtn.disabled = false;
      stopBtn.disabled = true;
      status.textContent = message;
    }
    function run() {
      if (worker) return;
      if (!window.Worker || !window.Blob || !window.URL) { status.textContent = 'This browser cannot run the simulator.'; return; }
      lines = [];
      output.textContent = '';
      showTraffic([]);
      blank();
      runBtn.disabled = true;
      stopBtn.disabled = false;
      status.textContent = 'Loading Python. The first run takes a few seconds.';
      var source = '(' + simWorker.toString() + ')(' + JSON.stringify({ base: PYODIDE_BASE, sketch: kind, stub: SIM_STUB, webui: SIM_WEBUI_STUB }) + ');';
      var url = URL.createObjectURL(new Blob([source], { type: 'text/javascript' }));
      worker = new Worker(url);
      URL.revokeObjectURL(url);
      worker.onmessage = function (event) {
        var msg = event.data;
        if (msg.type === 'ready') {
          status.textContent = 'Running. Press Stop to end it.';
          if (phone) phoneTimer = setInterval(function () { if (worker) worker.postMessage({ type: 'api', method: 'GET', path: '/status', value: null }); }, 1000);
        }
        else if (msg.type === 'api') phoneAnswer(msg);
        else if (msg.type === 'failed') stop('Python could not be loaded. Check your connection and try again.');
        else if (msg.type === 'done') stop('The App stopped. See the output below.');
        else if (msg.type === 'update') {
          if (msg.state) paint(msg.state);
          if (msg.out && msg.out.length) print(msg.out);
          if (msg.traffic) showTraffic(msg.traffic);
        }
      };
      worker.onerror = function () { stop('The simulator hit a problem. Press Run to try again.'); };
      worker.postMessage({ type: 'run', code: editor.value });
      if (sketch.pot) worker.postMessage({ type: 'pot', value: Number(pot.value) });
    }

    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: 'Uno Q simulator' }),
      el('span', { class: 'kr-w-hint', text: 'A stand-in for the board. Edit the Python and press Run.' })
    ]));
    root.appendChild(el('div', { class: 'kr-sim-layout' }, [
      el('div', { class: 'kr-sim-code' }, [
        el('span', { class: 'kr-mx-caption', text: 'python/main.py' }),
        editor,
        el('div', { class: 'kr-sim-buttons' }, [runBtn, stopBtn, resetBtn, status])
      ]),
      el('div', { class: 'kr-sim-board' }, [
        el('span', { class: 'kr-mx-caption', text: 'The board' }),
        grid, ledRow, potBox, phone,
        el('p', { class: 'kr-sim-sketch', text: sketch.text }),
        traffic
      ])
    ]));
    root.appendChild(el('span', { class: 'kr-mx-caption', text: 'Python output' }));
    root.appendChild(output);
    blank();
  }

  // ------------------------------------------------------ confidence demo
  // Made-up detections from one camera frame, and the threshold that decides
  // which ones the App hears about (the Video Object Detection Brick's
  // `confidence` setting). Shows what on_detect_all() would be given.
  var SEEN = [['person', 0.91], ['chair', 0.62], ['cup', 0.55], ['person', 0.48], ['dog', 0.33], ['person', 0.27]];

  function confidenceDemo(root) {
    var slider = el('input', { type: 'range', min: '0.1', max: '0.95', step: '0.05', value: '0.5', class: 'kr-cf-slider', 'aria-label': 'Confidence threshold' });
    var value = el('strong', { text: '0.50' });
    var list = el('div', { class: 'kr-cf-list' });
    var result = el('p', { class: 'kr-cf-result', role: 'status' });
    var rows = SEEN.map(function (d) {
      var bar = el('span', { class: 'kr-cf-bar' });
      bar.style.width = Math.round(d[1] * 100) + '%';
      var row = el('div', { class: 'kr-cf-row' }, [
        el('span', { class: 'kr-cf-label', text: d[0] }),
        el('span', { class: 'kr-cf-track' }, [bar]),
        el('span', { class: 'kr-cf-score', text: d[1].toFixed(2) }),
        el('span', { class: 'kr-cf-verdict' })
      ]);
      list.appendChild(row);
      return row;
    });
    function show() {
      var limit = Number(slider.value);
      value.textContent = limit.toFixed(2);
      var counts = {}, people = 0;
      SEEN.forEach(function (d, i) {
        var pass = d[1] >= limit;
        rows[i].classList.toggle('is-out', !pass);
        rows[i].lastChild.textContent = pass ? 'reported' : 'ignored';
        if (pass) { counts[d[0]] = (counts[d[0]] || 0) + 1; if (d[0] === 'person') people++; }
      });
      var labels = Object.keys(counts).map(function (k) { return counts[k] + ' x ' + k; });
      result.textContent = (labels.length ? 'Your function is given: ' + labels.join(', ') + '.' : 'Nothing passes, so your function is not called.') + ' People counted: ' + people + '.';
    }
    slider.addEventListener('input', show);
    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: 'Confidence threshold' }),
      el('span', { class: 'kr-w-hint', text: 'Made-up scores for one camera frame. Drag the slider.' })
    ]));
    root.appendChild(el('label', { class: 'kr-cf-control' }, [el('span', { text: 'confidence = ' }), value, slider]));
    root.appendChild(list);
    root.appendChild(result);
    show();
  }

  // -------------------------------------------------------- cost break-even
  function costBreakeven(root) {
    function field(label, start) {
      var input = el('input', { type: 'number', min: '0', step: '1', value: start, class: 'kr-cb-input' });
      return { input: input, box: el('label', { class: 'kr-cb-field' }, [el('span', { class: 'kr-mx-caption', text: label }), input]) };
    }
    var board = field('One-off cost of the board', root.getAttribute('data-board') || '53');
    var monthly = field('Monthly cost of renting a server', root.getAttribute('data-monthly') || '5');
    var result = el('p', { class: 'kr-cb-result', role: 'status' });
    function show() {
      var b = Number(board.input.value), m = Number(monthly.input.value);
      if (!(b > 0) || !(m > 0)) { result.textContent = 'Enter both costs to see the break-even point.'; return; }
      var months = Math.ceil(b / m);
      result.textContent = 'The board has paid for itself after ' + months + (months === 1 ? ' month' : ' months') +
        '. Over two years, renting costs ' + (m * 24).toFixed(0) + ' and the board still cost ' + b.toFixed(0) + '.';
    }
    board.input.addEventListener('input', show);
    monthly.input.addEventListener('input', show);
    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: 'Buy or rent?' }),
      el('span', { class: 'kr-w-hint', text: 'Use your own prices, in your own currency.' })
    ]));
    root.appendChild(el('div', { class: 'kr-cb-fields' }, [board.box, monthly.box]));
    root.appendChild(result);
    root.appendChild(el('p', { class: 'kr-w-hint', text: 'This leaves out electricity and the cost of the AI model itself, which you pay either way.' }));
    show();
  }

  // --------------------------------------------------------- move detector
  // A simplified picture of how Kev's chess camera reads a move without
  // recognising pieces (kevsrobots.com/blog/uno-q-chess-camera.html): score
  // how much each square changed, then ask the rules which legal move fits.
  // Opening position, white to move: exactly 20 legal moves.
  var FILES = 'abcdefgh';
  function squareName(index) { return FILES[index % 8] + (8 - Math.floor(index / 8)); }
  function startPosition() {
    var back = ['R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'], board = [];
    for (var i = 0; i < 64; i++) board.push(null);
    for (var f = 0; f < 8; f++) {
      board[f] = { piece: back[f], white: false };
      board[8 + f] = { piece: 'P', white: false };
      board[48 + f] = { piece: 'P', white: true };
      board[56 + f] = { piece: back[f], white: true };
    }
    return board;
  }
  function openingMoves() {
    var moves = [];
    for (var f = 0; f < 8; f++) { moves.push([48 + f, 40 + f]); moves.push([48 + f, 32 + f]); }
    [[57, 40], [57, 42], [62, 45], [62, 47]].forEach(function (m) { moves.push(m); });
    return moves;
  }

  function moveDetector(root) {
    var board, change, picked, done, moves = openingMoves();
    var noise = el('input', { type: 'checkbox' });
    var boardEl = el('div', { class: 'kr-md-board', role: 'group', 'aria-label': 'Chess board. Choose a white pawn or knight, then where it goes.' });
    var heatEl = el('div', { class: 'kr-md-board kr-md-heat', role: 'img', 'aria-label': 'How much each square changed' });
    var text = el('p', { class: 'kr-md-text', role: 'status' });
    var table = el('div', { class: 'kr-md-table' });
    var squares = [], heats = [];
    for (var i = 0; i < 64; i++) {
      var dark = (Math.floor(i / 8) + i % 8) % 2 === 1;
      var sq = el('button', { type: 'button', class: 'kr-md-sq' + (dark ? ' is-dark' : ''), 'data-index': i });
      sq.addEventListener('click', onSquare);
      squares.push(sq);
      boardEl.appendChild(sq);
      var heat = el('span', { class: 'kr-md-sq' + (dark ? ' is-dark' : '') }, [el('span', { class: 'kr-md-glow' })]);
      heats.push(heat);
      heatEl.appendChild(heat);
    }
    function targets(from) { return moves.filter(function (m) { return m[0] === from; }).map(function (m) { return m[1]; }); }
    function draw() {
      for (var i = 0; i < 64; i++) {
        var p = board[i], sq = squares[i];
        sq.textContent = '';
        if (p) sq.appendChild(el('span', { class: 'kr-md-piece' + (p.white ? ' is-white' : ''), text: p.piece }));
        var can = !done && (picked === null ? targets(i).length > 0 : targets(picked).indexOf(i) !== -1);
        sq.classList.toggle('is-picked', picked === i);
        sq.classList.toggle('is-target', !done && picked !== null && targets(picked).indexOf(i) !== -1);
        sq.disabled = !(can || picked === i);
        sq.setAttribute('aria-label', squareName(i) + (p ? ', ' + (p.white ? 'white ' : 'black ') + p.piece : ', empty'));
        heats[i].firstChild.style.opacity = change ? String(Math.min(1, change[i])) : '0';
      }
    }
    function onSquare(event) {
      var i = Number(event.currentTarget.getAttribute('data-index'));
      if (done) return;
      if (picked === null) { if (targets(i).length) picked = i; }
      else if (i === picked) picked = null;
      else if (targets(picked).indexOf(i) !== -1) { play(picked, i); return; }
      text.textContent = picked === null ? 'Choose a white pawn or knight.' : 'Now choose where it goes.';
      draw();
    }
    function play(from, to) {
      board[to] = board[from];
      board[from] = null;
      picked = null;
      done = true;
      change = [];
      for (var i = 0; i < 64; i++) change.push(noise.checked ? Math.random() * 0.15 : 0);
      change[from] = 0.85 + Math.random() * 0.15;
      change[to] = 0.85 + Math.random() * 0.15;
      var brushed = null;
      if (noise.checked) {
        // A sleeve brushes a neighbouring piece on the way past.
        brushed = from + (from % 8 === 7 ? -1 : 1);
        change[brushed] = 0.35;
      }
      var scored = moves.map(function (m) {
        var elsewhere = 0;
        for (var i = 0; i < 64; i++) if (i !== m[0] && i !== m[1]) elsewhere = Math.max(elsewhere, change[i]);
        return { from: m[0], to: m[1], score: change[m[0]] + change[m[1]] - elsewhere };
      }).sort(function (a, b) { return b.score - a.score; });
      table.textContent = '';
      table.appendChild(el('span', { class: 'kr-mx-caption', text: 'The 20 legal moves, best fit first (top 5 shown)' }));
      scored.slice(0, 5).forEach(function (s, n) {
        var bar = el('span', { class: 'kr-cf-bar' });
        bar.style.width = Math.max(0, Math.round(s.score / 2 * 100)) + '%';
        table.appendChild(el('div', { class: 'kr-cf-row' + (n === 0 ? ' is-best' : ' is-out') }, [
          el('span', { class: 'kr-cf-label', text: squareName(s.from) + ' to ' + squareName(s.to) }),
          el('span', { class: 'kr-cf-track' }, [bar]),
          el('span', { class: 'kr-cf-score', text: s.score.toFixed(2) })
        ]));
      });
      var best = scored[0];
      text.textContent = 'The camera saw two squares change a lot: ' + squareName(from) + ' and ' + squareName(to) + '. ' +
        (brushed !== null ? 'It also saw ' + squareName(brushed) + ' change a little, where a piece was brushed. ' : '') +
        'Only one legal move fits, so the App reads ' + squareName(best.from) + ' to ' + squareName(best.to) + '. It never needed to know which piece it was.';
      draw();
    }
    function reset() {
      board = startPosition(); change = null; picked = null; done = false;
      table.textContent = '';
      text.textContent = 'You are white. Choose a pawn or a knight, then where it goes.';
      draw();
    }
    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: 'Read a move without knowing the pieces' }),
      el('span', { class: 'kr-w-hint', text: 'A simplified picture of the chess camera.' })
    ]));
    root.appendChild(el('div', { class: 'kr-md-layout' }, [
      el('div', { class: 'kr-md-col' }, [el('span', { class: 'kr-mx-caption', text: 'The board' }), boardEl]),
      el('div', { class: 'kr-md-col' }, [el('span', { class: 'kr-mx-caption', text: 'What the camera scores: how much each square changed' }), heatEl])
    ]));
    root.appendChild(text);
    root.appendChild(table);
    root.appendChild(el('div', { class: 'kr-sim-buttons' }, [
      el('button', { type: 'button', class: 'kr-w-btn kr-w-btn-main', text: 'New move', onclick: reset }),
      el('label', { class: 'kr-md-noise' }, [noise, el('span', { text: ' Add camera noise and a brushed piece' })])
    ]));
    reset();
  }

  // ------------------------------------------------------------ POV builder
  // Design thinking course, lesson 4: user + need + insight -> a point of view,
  // three How Might We questions, and a short brief to paste into an AI.
  function povBuilder(root) {
    function field(label, key, hint) {
      var input = el('textarea', { rows: '2', class: 'kr-pv-input', placeholder: hint });
      input.value = root.getAttribute('data-' + key) || '';
      return { input: input, box: el('label', { class: 'kr-pv-field' }, [el('span', { class: 'kr-mx-caption', text: label }), input]) };
    }
    var user = field('User: who, specifically?', 'user', 'e.g. Leo and Priya, the pupils on plant duty');
    var need = field('Need: a verb, not a gadget', 'need', 'e.g. a way to tell which plant needs water');
    var insight = field('Insight: what you learned that surprised you', 'insight', 'e.g. they are afraid of getting it wrong');
    var pov = el('p', { class: 'kr-pv-pov', role: 'status' });
    var hmw = el('ul', { class: 'kr-pv-hmw' });
    var warn = el('p', { class: 'kr-w-hint kr-pv-warn' });
    var brief = el('pre', { class: 'kr-pv-brief' });
    var copy = el('button', { type: 'button', class: 'kr-w-btn', text: 'Copy brief' });
    copy.addEventListener('click', function () { copyText(brief.textContent, copy); });
    var GADGETS = /\b(app|robot|website|web page|screen|sensor|button|pump|camera|ai|chatbot|dashboard|device|machine|alexa)\b/i;
    function clean(t) { return t.trim().replace(/[.\s]+$/, ''); }
    function lowerFirst(t) { return t ? t.charAt(0).toLowerCase() + t.slice(1) : t; }
    function show() {
      var u = clean(user.input.value), n = clean(need.input.value), i = clean(insight.input.value);
      hmw.textContent = '';
      if (!u || !n || !i) {
        pov.textContent = 'Fill in all three boxes to see your point of view.';
        brief.textContent = ''; warn.textContent = '';
        return;
      }
      var needCore = lowerFirst(n.replace(/^(a way|some way|a means)\s+(to|of)\s+/i, ''));
      var uc = u + (u.indexOf(',') >= 0 ? ',' : '');
      pov.textContent = uc + ' need' + (/\band\b|,/.test(u) ? '' : 's') + ' ' + lowerFirst(n) + ', because ' + lowerFirst(i) + '.';
      [
        'How might we help ' + uc + ' ' + needCore.replace(/^to\s+/i, '') + '?',
        'How might we make it impossible to ' + (/afraid|worr|scared|nervous/i.test(i) ? 'get it wrong' : 'miss') + '?',
        'How might we make it easy even when ' + lowerFirst(i) + '?'
      ].forEach(function (q) { hmw.appendChild(el('li', { text: q })); });
      warn.textContent = GADGETS.test(n)
        ? 'Your need names a thing (' + n.match(GADGETS)[0] + '). That is a solution sneaking in. Try describing what they need to do instead.'
        : '';
      brief.textContent = [
        'Before writing any code, help me design this.',
        '',
        'Who it is for: ' + u + '.',
        'Point of view: ' + pov.textContent,
        'What we found out: ' + i + '.',
        '',
        'Please do three things, and wait for me after each:',
        '1. Ask me up to five questions about anything in this brief that is unclear.',
        '2. Suggest at least eight quite different ideas, including some with no electronics at all.',
        '3. For the idea I pick, list what we would need to test with real people before building it.',
        '',
        'Do not write code yet.'
      ].join('\n');
    }
    [user, need, insight].forEach(function (f) { f.input.addEventListener('input', show); });
    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: 'Point of view builder' }),
      el('span', { class: 'kr-w-hint', text: 'Edit the boxes with your own project.' })
    ]));
    root.appendChild(el('div', { class: 'kr-pv-fields' }, [user.box, need.box, insight.box]));
    root.appendChild(el('span', { class: 'kr-mx-caption', text: 'Your point of view' }));
    root.appendChild(pov);
    root.appendChild(warn);
    root.appendChild(el('span', { class: 'kr-mx-caption', text: 'How might we...' }));
    root.appendChild(hmw);
    root.appendChild(el('div', { class: 'kr-pv-briefhead' }, [el('span', { class: 'kr-mx-caption', text: 'A brief to paste into your AI assistant' }), copy]));
    root.appendChild(brief);
    show();
  }

  // ------------------------------------------------------------- five users
  // Jakob Nielsen (2000), after Nielsen and Landauer (1993): share of usability
  // problems found by n testers = 1 - (1 - L)^n, with L about 0.31.
  function fiveUsers(root) {
    var users = el('input', { type: 'range', min: '1', max: '15', step: '1', class: 'kr-cf-slider', 'aria-label': 'Number of testers' });
    users.value = root.getAttribute('data-users') || '5';
    var rate = el('input', { type: 'range', min: '10', max: '60', step: '1', class: 'kr-cf-slider', 'aria-label': 'Share of problems one tester finds' });
    rate.value = root.getAttribute('data-rate') || '31';
    var usersOut = el('strong'), rateOut = el('strong');
    var bars = el('div', { class: 'kr-fu-bars', 'aria-hidden': 'true' });
    var result = el('p', { class: 'kr-cf-result', role: 'status' });
    var cols = [];
    for (var k = 1; k <= 15; k++) {
      var bar = el('span', { class: 'kr-fu-bar' });
      var col = el('span', { class: 'kr-fu-col' }, [bar, el('span', { class: 'kr-fu-n', text: String(k) })]);
      cols.push({ col: col, bar: bar });
      bars.appendChild(col);
    }
    function found(n, L) { return 1 - Math.pow(1 - L, n); }
    function show() {
      var n = Number(users.value), L = Number(rate.value) / 100;
      usersOut.textContent = String(n);
      rateOut.textContent = rate.value + '%';
      cols.forEach(function (c, i) {
        c.bar.style.height = (found(i + 1, L) * 100).toFixed(1) + '%';
        c.col.classList.toggle('is-on', i + 1 <= n);
      });
      var share = found(n, L), extra = found(n + 1, L) - share;
      result.textContent = n + (n === 1 ? ' tester finds' : ' testers find') + ' about ' + Math.round(share * 100) +
        '% of the problems. One more tester would add ' + (extra * 100).toFixed(1) + ' percentage points.';
    }
    users.addEventListener('input', show);
    rate.addEventListener('input', show);
    root.textContent = '';
    root.appendChild(el('div', { class: 'kr-w-head' }, [
      el('span', { class: 'kr-w-label', text: 'How many testers?' }),
      el('span', { class: 'kr-w-hint', text: 'Nielsen’s model. Bars show 1 to 15 testers.' })
    ]));
    root.appendChild(el('label', { class: 'kr-cf-control' }, [el('span', { text: 'Testers: ' }), usersOut, users]));
    root.appendChild(el('label', { class: 'kr-cf-control' }, [el('span', { text: 'One tester finds: ' }), rateOut, rate]));
    root.appendChild(bars);
    root.appendChild(result);
    root.appendChild(el('p', { class: 'kr-w-hint', text: 'A model, not a law: it assumes your testers are like your real users and all try the same tasks.' }));
    show();
  }

  var WIDGETS = { 'matrix-painter': matrixPainter, 'bridge-steps': bridgeSteps, sorter: sorter, 'uno-q-sim': unoQSim,
    'confidence-demo': confidenceDemo, 'cost-breakeven': costBreakeven, 'move-detector': moveDetector,
    'pov-builder': povBuilder, 'five-users': fiveUsers };

  function start() {
    document.querySelectorAll('[data-kr-widget]').forEach(function (root) {
      if (root.getAttribute('data-kr-ready')) return;
      var make = WIDGETS[root.getAttribute('data-kr-widget')];
      if (!make) return;
      root.setAttribute('data-kr-ready', '1');
      var fallback = root.innerHTML;
      try {
        make(root);
        root.classList.add('is-live');
      } catch (err) {
        root.innerHTML = fallback;
        if (window.console) console.error('course widget failed', err);
      }
    });
  }

  window.krCourseWidgets = { start: start };
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
