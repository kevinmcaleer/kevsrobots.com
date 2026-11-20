/*
 * "Open in Snakie" on lesson code (kevsrobots.com#581, Snakie by kevsrobots).
 *
 * Every Python code block in a lesson gets a small "Open in Snakie" link at
 * its top right, beside the copy button. It opens app.snakie.org in a new tab
 * with the block's code in a new, unsaved tab named after the lesson
 * (e.g. 06_our_first_program.py). links() also builds the snakie://open
 * link for the desktop app; the block no longer shows it, to keep the block
 * to three buttons.
 *
 * The code travels in the URL's hash, compressed the way Snakie's own code
 * embeds do it (raw DEFLATE, then base64url): `#src=…`. A hash never reaches
 * a server, so nothing is stored on kevsrobots or logged by Snakie. Browsers
 * without CompressionStream send `?code=<plain text>` instead, which Snakie
 * also reads.
 *
 * {% include snakie.html %} boxes are skipped: Snakie's embed has its own
 * Open in Snakie link. To leave a block out, add {: .no-snakie } under its
 * fence; to leave a whole lesson out, set `snakie: false` in its front matter
 * (the layout then doesn't load this script). REPL transcripts (`>>>`) are
 * skipped too, since they aren't a program.
 *
 * The play button (the lesson runner) sits beside it, in Snakie's green. The
 * block stays a plain code block; pressing play pops an output panel up under
 * it that runs the code in the reader's browser (MicroPython, or Python 3 for
 * numpy and friends): the output in a live REPL, and the board with its pins
 * lit when the code drives them. While it runs, play becomes stop; the panel
 * has Run again and a close button. Nothing is loaded until play is pressed.
 * The board comes from `board:` in the lesson's front matter (Snakie's default
 * is the Pico 2 W).
 *
 * Every program gets play, so nothing looks switched off. Code a browser can't
 * stand in for (input(), Wi-Fi, a desktop library) opens a short note under
 * the block instead, saying why and offering Open in Snakie.
 *
 * So a block carries three things and no more: play, copy (code-copy.js) and
 * Open in Snakie.
 *
 * Pages that render Markdown later can call OpenInSnakie.attach(root).
 */
(function () {
  'use strict';

  var APP = 'https://app.snakie.org/';
  var DESKTOP = 'snakie://open';
  var LANGS = ['python', 'py', 'python3', 'micropython', 'upython'];
  var EMBED_JS = APP + 'snakie-embed.js';
  // The lesson's board, from the script tag the layout writes.
  var BOARD = (typeof document !== 'undefined' && document.currentScript && document.currentScript.getAttribute('data-board')) || '';
  // Full Python libraries: these run in Snakie's Pyodide engine, not MicroPython.
  var PYODIDE_MODULES = ['numpy', 'pandas', 'matplotlib', 'scipy', 'sklearn', 'sympy'];
  // Nothing in a browser tab can stand in for these.
  var NO_RUN_MODULES = ['network', 'socket', 'usocket', 'bluetooth', 'aioble', 'espnow', 'urequests', 'requests',
    'umqtt', 'serial', 'tkinter', 'pygame', 'cv2', 'picamera', 'picamera2', 'RPi', 'gpiozero', 'flask', 'fastapi',
    'subprocess', 'rclpy'];

  var SNAKIE_ICON = '<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true" focusable="false"><path d="M3 4.5h10M3 8h6.5M3 11.5h4.5M11 10l2.5 1.5L11 13"/></svg>';
  var PLAY_ICON = '<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true" focusable="false"><path d="M5 3.2v9.6l7.6-4.8z"/></svg>';
  var STOP_ICON = '<svg viewBox="0 0 16 16" width="12" height="12" aria-hidden="true" focusable="false"><rect x="3.5" y="3.5" width="9" height="9" rx="1.5"/></svg>';

  // --- Pure helpers (also used by scripts/tests/open-in-snakie.test.js) ---

  function toBase64Url(bytes) {
    var bin = '';
    for (var i = 0; i < bytes.length; i += 0x8000) {
      bin += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000));
    }
    return btoa(bin).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
  }

  /** Snakie's code payload: raw DEFLATE, base64url. Null if the browser can't. */
  function encodeCode(code) {
    if (typeof CompressionStream === 'undefined' || typeof TextEncoder === 'undefined') {
      return Promise.resolve(null);
    }
    try {
      var stream = new Blob([new TextEncoder().encode(code)]).stream()
        .pipeThrough(new CompressionStream('deflate-raw'));
      return new Response(stream).arrayBuffer().then(function (buf) {
        return toBase64Url(new Uint8Array(buf));
      }, function () { return null; });
    } catch (e) {
      return Promise.resolve(null);
    }
  }

  /** A file name for the n-th (1-based) Python block of a lesson at `pathname`. */
  function fileName(pathname, n) {
    var last = String(pathname || '').replace(/\/+$/, '').split('/').pop() || '';
    var slug = decodeURIComponent(last).replace(/\.html?$/i, '')
      .replace(/[^A-Za-z0-9_-]+/g, '_').replace(/^_+|_+$/g, '').slice(0, 60);
    if (!slug || slug === 'index') slug = 'main';
    return slug + (n > 1 ? '_' + n : '') + '.py';
  }

  /** Both links, from the code and its (possibly null) compressed payload. */
  function links(code, name, payload) {
    var q = 'name=' + encodeURIComponent(name);
    if (payload !== null) {
      return { web: APP + '?' + q + '#src=' + payload, desktop: DESKTOP + '?' + q + '#src=' + payload };
    }
    var plain = q + '&code=' + encodeURIComponent(code);
    return { web: APP + '?' + plain, desktop: DESKTOP + '?' + plain };
  }

  /** Worth opening as a program: not empty, not a `>>>` REPL session. */
  function isProgram(code) {
    var first = String(code).split('\n').filter(function (l) { return l.trim() !== ''; })[0];
    return !!first && !/^\s*(>>>|\$ )/.test(first);
  }

  /** The top-level module names a program imports. */
  function imports(code) {
    var out = [];
    String(code).split('\n').forEach(function (line) {
      var m = /^\s*(?:from\s+([A-Za-z_][\w]*)|import\s+(.+))/.exec(line);
      if (!m) return;
      if (m[1]) { out.push(m[1]); return; }
      m[2].split('#')[0].split(',').forEach(function (part) {
        var name = part.trim().split(/\s+/)[0].split('.')[0];
        if (name) out.push(name);
      });
    });
    return out;
  }

  /**
   * Can this block run in the page, and in which of Snakie's engines?
   * Null when it can't: it waits for typing, or needs hardware a browser
   * can't stand in for. Otherwise { engine: 'micropython' | 'pyodide' }.
   */
  function runPlan(code) {
    if (!isProgram(code)) return null;
    if (/\binput\s*\(/.test(code)) return null;
    var mods = imports(code);
    for (var i = 0; i < mods.length; i++) {
      if (NO_RUN_MODULES.indexOf(mods[i]) !== -1) return null;
    }
    for (var j = 0; j < mods.length; j++) {
      if (PYODIDE_MODULES.indexOf(mods[j]) !== -1) return { engine: 'pyodide' };
    }
    return { engine: 'micropython' };
  }

  /**
   * Why a program can't run in the page, in words for the reader, or null if
   * it can. Matches runPlan(): null here exactly when runPlan() gives a plan.
   */
  function whyNot(code) {
    if (!isProgram(code)) return 'This isn’t a whole program, so there’s nothing to run.';
    if (/\binput\s*\(/.test(code)) return 'This program waits for you to type, so it runs best in Snakie.';
    var mods = imports(code);
    for (var i = 0; i < mods.length; i++) {
      if (NO_RUN_MODULES.indexOf(mods[i]) !== -1) {
        return 'This program uses ' + mods[i] + ', which needs a real board or computer, not a browser tab.';
      }
    }
    return null;
  }

  function isPythonBlock(code) {
    var holder = code.closest('[class*="language-"]') || code;
    for (var i = 0; i < LANGS.length; i++) {
      if (holder.classList.contains('language-' + LANGS[i]) || code.classList.contains('language-' + LANGS[i])) {
        return !holder.closest('.no-snakie, .snakie-box');
      }
    }
    return false;
  }

  // --- The DOM side ---

  function link(cls, label, icon, text) {
    var a = document.createElement('a');
    a.className = cls;
    a.title = label;
    a.setAttribute('aria-label', label);
    a.innerHTML = icon + (text ? '<span>' + text + '</span>' : '');
    return a;
  }

  // Loads Snakie's embed helper once, on the first press of play.
  var embedLoading = null;
  function loadEmbed() {
    if (window.SnakieEmbed) return Promise.resolve(window.SnakieEmbed);
    if (!embedLoading) {
      embedLoading = new Promise(function (resolve, reject) {
        var el = document.createElement('script');
        el.src = EMBED_JS;
        el.onload = function () { resolve(window.SnakieEmbed); };
        el.onerror = function () { embedLoading = null; reject(new Error('snakie-embed.js failed to load')); };
        document.head.appendChild(el);
      });
    }
    return embedLoading;
  }

  function setPlay(button, running) {
    button.classList.toggle('is-running', running);
    button.innerHTML = running ? STOP_ICON : PLAY_ICON;
    var label = running ? 'Stop the program' : 'Run this code here';
    button.title = label;
    button.setAttribute('aria-label', label);
  }

  /**
   * A short note under the block (not a dead button): why this code can't run
   * in the page, and the next step, Open in Snakie. Pressing play again closes it.
   */
  function toggleNote(wrap, reason, web) {
    var open = wrap.nextElementSibling;
    if (open && open.classList.contains('snakie-run-note')) {
      open.remove();
      return;
    }
    var note = document.createElement('div');
    note.className = 'snakie-run-note';
    note.setAttribute('role', 'status');
    var text = document.createElement('span');
    text.textContent = reason;
    var go = document.createElement('a');
    go.className = 'snakie-run-note-open';
    go.href = web.href;
    go.target = '_blank';
    go.rel = 'noopener';
    go.innerHTML = SNAKIE_ICON + '<span>Open in Snakie</span>';
    var close = document.createElement('button');
    close.type = 'button';
    close.className = 'snakie-run-note-close';
    close.setAttribute('aria-label', 'Close');
    close.title = 'Close';
    close.innerHTML = '<svg viewBox="0 0 16 16" width="12" height="12" aria-hidden="true"><path d="M4 4l8 8M12 4l-8 8"/></svg>';
    close.addEventListener('click', function () { note.remove(); });
    note.appendChild(text);
    note.appendChild(go);
    note.appendChild(close);
    wrap.insertAdjacentElement('afterend', note);
  }

  /**
   * Play: pop the output panel up under the block and run the code there.
   * Pressed again while it runs, stop; once it has finished, run it again.
   * An older Snakie without the runner gets its full live box instead (the
   * earlier Run here), and code too long for a link opens in Snakie.
   */
  function play(wrap, button, name, plan, web) {
    var open = wrap.__snakieRunner;
    if (open) {
      if (open.running) open.stop(); else open.run();
      return;
    }
    if (button.dataset.busy) return;
    button.dataset.busy = '1';
    button.classList.add('is-loading');
    loadEmbed().then(function (embed) {
      var opts = { name: name, engine: plan.engine };
      if (BOARD) opts.board = BOARD;
      if (embed && typeof embed.runner === 'function') {
        opts.onState = function (running) { setPlay(button, running); };
        opts.onClose = function () { setPlay(button, false); button.focus(); };
        var handle = embed.runner(wrap, opts);
        if (handle) { setPlay(button, true); return; }
      } else if (embed && typeof embed.upgrade === 'function' && embed.upgrade(wrap, opts)) {
        return;
      }
      window.open(web.href, '_blank', 'noopener');
    }, function () {
      window.open(web.href, '_blank', 'noopener');
    }).then(function () {
      delete button.dataset.busy;
      button.classList.remove('is-loading');
    });
  }

  function addLinks(code, n) {
    var pre = code.parentElement;
    if (!pre || pre.dataset.openInSnakie) return false;
    var text = code.textContent.replace(/\n$/, '');
    if (!isProgram(text)) return false;
    pre.dataset.openInSnakie = '1';

    // code-copy.js (loaded in <head>) has usually wrapped the <pre> already.
    var wrap = pre.parentElement && pre.parentElement.classList.contains('code-copy-wrap')
      ? pre.parentElement : null;
    if (!wrap) {
      wrap = document.createElement('div');
      wrap.className = 'code-copy-wrap';
      pre.parentNode.insertBefore(wrap, pre);
      wrap.appendChild(pre);
    }
    wrap.classList.add('has-open-in-snakie');

    var name = fileName(window.location.pathname, n);
    var group = document.createElement('div');
    group.className = 'open-in-snakie';
    var web = link('open-in-snakie-web', 'Open ' + name + ' in Snakie (opens app.snakie.org)', SNAKIE_ICON, 'Open in Snakie');
    web.target = '_blank';
    web.rel = 'noopener';

    var run = document.createElement('button');
    run.type = 'button';
    run.className = 'snakie-play';
    setPlay(run, false);
    var plan = runPlan(text);
    run.addEventListener('click', function () {
      if (plan) play(wrap, run, name, plan, web);
      else toggleNote(wrap, whyNot(text) || 'This code runs best in Snakie.', web);
    });
    group.appendChild(run);
    group.appendChild(web);
    wrap.appendChild(group);

    var plain = links(text, name, null);
    web.href = plain.web;
    encodeCode(text).then(function (payload) {
      if (payload === null) return;
      web.href = links(text, name, payload).web;
    });
    return true;
  }

  function attach(root) {
    var n = 0;
    (root || document).querySelectorAll('pre > code').forEach(function (code) {
      if (!isPythonBlock(code)) return;
      if (addLinks(code, n + 1) || code.parentElement.dataset.openInSnakie) n += 1;
    });
  }

  var api = { attach: attach, fileName: fileName, links: links, isProgram: isProgram, encodeCode: encodeCode, runPlan: runPlan, whyNot: whyNot, imports: imports };

  if (typeof module === 'object' && module.exports) {
    module.exports = api;
    return;
  }

  window.OpenInSnakie = api;
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function () { attach(document); });
  } else {
    attach(document);
  }
})();
