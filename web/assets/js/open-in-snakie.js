/*
 * "Open in Snakie" on lesson code (kevsrobots.com#581, Snakie by kevsrobots).
 *
 * Every Python code block in a lesson gets a small "Open in Snakie" link at
 * its top right, beside the copy button. It opens app.snakie.org in a new tab
 * with the block's code in a new, unsaved tab named after the lesson
 * (e.g. 06_our_first_program.py). Next to it, a desktop icon hands the same
 * code to the installed Snakie app through a snakie://open link.
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
 * "Run here" (#589) sits beside it on code that can run in a browser. It
 * turns that one block into Snakie's live code box, in place, and runs it:
 * the editor, the board with the pins the code uses, and a REPL. Nothing is
 * loaded until it is pressed. The box's "Plain code" link puts the block back.
 * The board comes from `board:` in the lesson's front matter (Snakie's default
 * is the Pico 2 W). Code that needs input(), the network or a desktop library
 * gets no Run here, only Open in Snakie.
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
  var RUN_ICON = '<svg viewBox="0 0 16 16" width="12" height="12" aria-hidden="true" focusable="false"><path d="M4.5 3l8 5-8 5z"/></svg>';
  var DESKTOP_ICON = '<svg viewBox="0 0 16 16" width="16" height="16" aria-hidden="true" focusable="false"><path d="M2.5 3.5h11v7h-11zM6 13.5h4M8 10.5v3"/></svg>';

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

  // Loads Snakie's embed helper once, on the first Run here.
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

  /** Swap the block for Snakie's live box. False if this Snakie can't, so the caller opens the app instead. */
  function runHere(wrap, name, plan) {
    return loadEmbed().then(function (embed) {
      if (!embed || typeof embed.upgrade !== 'function') return false;
      var opts = { name: name, engine: plan.engine };
      if (BOARD) opts.board = BOARD;
      var live = embed.upgrade(wrap, opts);
      if (!live) return false;
      var back = document.createElement('button');
      back.type = 'button';
      back.className = 'run-here-back';
      back.textContent = 'Plain code';
      back.title = 'Close the Snakie box and show the plain code again';
      back.addEventListener('click', function () {
        live.unmount();
        back.remove();
      });
      live.box.insertAdjacentElement('afterend', back);
      return true;
    }, function () { return false; });
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
    var desk = link('open-in-snakie-desktop', 'Open ' + name + ' in the Snakie desktop app', DESKTOP_ICON, '');
    var plan = runPlan(text);
    if (plan) {
      var run = document.createElement('button');
      run.type = 'button';
      run.className = 'open-in-snakie-run';
      run.title = 'Run this code here, in the page';
      run.innerHTML = RUN_ICON + '<span>Run here</span>';
      run.addEventListener('click', function () {
        run.disabled = true;
        runHere(wrap, name, plan).then(function (ok) {
          run.disabled = false;
          // An older Snakie, or code too long for a link: the full app can still run it.
          if (!ok) {
            run.remove();
            window.open(web.href, '_blank', 'noopener');
          }
        });
      });
      group.appendChild(run);
    }
    group.appendChild(web);
    group.appendChild(desk);
    wrap.appendChild(group);

    var plain = links(text, name, null);
    web.href = plain.web;
    desk.href = plain.desktop;
    encodeCode(text).then(function (payload) {
      if (payload === null) return;
      var l = links(text, name, payload);
      web.href = l.web;
      desk.href = l.desktop;
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

  var api = { attach: attach, fileName: fileName, links: links, isProgram: isProgram, encodeCode: encodeCode, runPlan: runPlan, imports: imports };

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
