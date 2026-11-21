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
    sensor: { text: 'The sketch provides draw and set_status. Once a second it calls get_limit (until it gets an answer), notifies on_reading with the knob value, and lights LED 3 red above the limit.', pot: true }
  };

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
    var py = null, bridge = null, loopFn = null, started = false, stopped = false;
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
    if (cfg.sketch === 'clock' || cfg.sketch === 'sensor') {
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

    function sketchTick() {
      var now = Date.now();
      if (cfg.sketch === 'blink') {
        var on = Math.floor(now / 500) % 2 === 0 ? 1 : 0;
        if (state.led3[1] !== on) { state.led3 = [0, on, 0]; dirty = true; }
        return;
      }
      if (cfg.sketch !== 'sensor' || now - sk.last < 1000) return;
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
      tick();
    }

    self.onmessage = function (event) {
      var msg = event.data;
      if (msg.type === 'pot') { pot = msg.value; return; }
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
      var source = '(' + simWorker.toString() + ')(' + JSON.stringify({ base: PYODIDE_BASE, sketch: kind, stub: SIM_STUB }) + ');';
      var url = URL.createObjectURL(new Blob([source], { type: 'text/javascript' }));
      worker = new Worker(url);
      URL.revokeObjectURL(url);
      worker.onmessage = function (event) {
        var msg = event.data;
        if (msg.type === 'ready') status.textContent = 'Running. Press Stop to end it.';
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
        grid, ledRow, potBox,
        el('p', { class: 'kr-sim-sketch', text: sketch.text }),
        traffic
      ])
    ]));
    root.appendChild(el('span', { class: 'kr-mx-caption', text: 'Python output' }));
    root.appendChild(output);
    blank();
  }

  var WIDGETS = { 'matrix-painter': matrixPainter, 'bridge-steps': bridgeSteps, sorter: sorter, 'uno-q-sim': unoQSim };

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
