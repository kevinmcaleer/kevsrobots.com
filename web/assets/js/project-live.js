// Live project pages (#590): the code and the wiring on a project page know
// about each other.
//
//  - The wiring picture becomes a viewer: zoom, drag, and the name of a wire
//    when you point at it. The flat wiring.svg image stays in the page for
//    print, PDF and readers without JavaScript.
//  - Point at a pin in a Snakie code box and its wire lights; point at a wire
//    and the code that uses its pin lights. Snakie's helper (snakie-embed.js)
//    carries the pin names between its frames and this page.
//  - The project's Code repository shows as a Snakie box with a tab per file,
//    under a Code heading. The code still lives on GitHub: nothing is stored
//    here.
//  - In the editor, a bar says when the code and the wiring disagree
//    ("main.py uses GP14, but nothing is wired to GP14").
//
// Wires can only be told apart in pictures saved since Snakie began writing
// their ids into the SVG (Snakie#1320). An older picture still zooms; saving
// the wiring again adds the ids.
(function () {
  'use strict';

  // ------------------------------------------------------------ pure helpers
  // (also used by scripts/tests/project-live.test.js)

  function splitEnd(endpoint) {
    var s = String(endpoint || '');
    var dot = s.indexOf('.');
    if (dot <= 0 || dot === s.length - 1) return null;
    return { part: s.slice(0, dot), pin: s.slice(dot + 1) };
  }

  /**
   * What each board pin is wired to, from a wiring (Snakie's robot definition).
   * Keep in step with Snakie's src/shared/wiring-pins.ts. Returns
   * { GP15: { parts: ['Servo 1'], ends: ['Servo 1 SIG'], wires: ['w1'] } }.
   */
  function boardPins(robot) {
    var out = {};
    if (!robot || !Array.isArray(robot.connections)) return out;
    var parts = {};
    (robot.parts || []).forEach(function (p) { if (p && p.id) parts[p.id] = p; });
    var carrier = robot.boardMountedOn || null;

    function canonical(endpoint) {
      var ep = splitEnd(endpoint);
      if (!ep) return String(endpoint);
      var owner = ep.part;
      for (var hops = 0; hops < 8; hops++) {
        var next = owner === 'board' ? carrier : (parts[owner] && parts[owner].mountedOn);
        if (!next || next === owner) break;
        owner = next;
      }
      return owner + '.' + ep.pin;
    }

    var parent = {};
    function find(k) {
      var root = k;
      while (parent[root] !== undefined && parent[root] !== root) root = parent[root];
      parent[k] = root;
      return root;
    }
    var wires = robot.connections.filter(function (c) {
      return c && typeof c.from === 'string' && typeof c.to === 'string';
    });
    wires.forEach(function (c) {
      var a = canonical(c.from), b = canonical(c.to);
      if (parent[a] === undefined) parent[a] = a;
      if (parent[b] === undefined) parent[b] = b;
      var ra = find(a), rb = find(b);
      if (ra !== rb) parent[ra] = rb;
    });
    var groups = {};
    function group(endpoint) {
      var root = find(canonical(endpoint));
      if (!groups[root]) groups[root] = { board: [], ends: [], wires: [] };
      return groups[root];
    }
    wires.forEach(function (c) {
      var g = group(c.from);
      if (c.id) g.wires.push(c.id);
      [c.from, c.to].forEach(function (endpoint) {
        var ep = splitEnd(endpoint);
        if (!ep) return;
        if (ep.part === 'board' || ep.part === carrier) {
          if (g.board.indexOf(ep.pin) === -1) g.board.push(ep.pin);
        } else if (g.ends.indexOf(endpoint) === -1) {
          g.ends.push(endpoint);
        }
      });
    });
    Object.keys(groups).forEach(function (root) {
      var g = groups[root];
      if (!g.board.length || !g.ends.length) return;
      var names = [], ends = [];
      g.ends.forEach(function (endpoint) {
        var ep = splitEnd(endpoint);
        var name = partLabel(parts[ep.part], ep.part);
        if (names.indexOf(name) === -1) names.push(name);
        ends.push(name + ' ' + ep.pin);
      });
      g.board.forEach(function (pin) {
        out[pin] = { parts: names, ends: ends, wires: g.wires.slice() };
      });
    });
    return out;
  }

  function partLabel(part, fallback) {
    return String((part && (part.label || part.part)) || fallback || '').trim();
  }

  /** `GP15`, `gp15`, `GPIO15`, `IO15` and `15` are one pin; `LED` is itself. */
  function pinKey(name) {
    var s = String(name || '').trim().toUpperCase();
    var m = /^(?:GP|GPIO|IO|D)?(\d+)$/.exec(s);
    return m ? '#' + Number(m[1]) : s;
  }

  /** The power and ground names a program never sets up. */
  function isSupply(name) {
    return /^(GND|AGND|VBUS|VSYS|VIN|VCC|VDD|3V3?(_EN|_OUT)?|5V|RUN|EN|RESET|ADC_VREF|VREF)$/i.test(String(name || '').trim());
  }

  /** A wire, in words: "GP15 to Servo 1 SIG". */
  function wireLabel(robot, wire) {
    var parts = {};
    ((robot && robot.parts) || []).forEach(function (p) { if (p && p.id) parts[p.id] = p; });
    function end(endpoint) {
      var ep = splitEnd(endpoint);
      if (!ep) return String(endpoint || '');
      if (ep.part === 'board') return ep.pin;
      return partLabel(parts[ep.part], ep.part) + ' ' + ep.pin;
    }
    var a = end(wire.from), b = end(wire.to);
    // The board's end first, as a reader would say it.
    if (splitEnd(wire.to) && splitEnd(wire.to).part === 'board') { var t = a; a = b; b = t; }
    return a + ' to ' + b;
  }

  /**
   * Where a file's pins and the wiring disagree, as sentences. `pins` is what
   * Snakie reports ([{ id, onBoard }]); `wired` is boardPins(robot).
   */
  function mismatches(file, pins, wired) {
    var wiredKeys = {};
    Object.keys(wired).forEach(function (pin) { wiredKeys[pinKey(pin)] = pin; });
    var name = file || 'The code';
    var out = [];
    (pins || []).forEach(function (p) {
      if (!p || !p.id || p.onBoard === false || /^LED$/i.test(p.id)) return;
      if (!wiredKeys[pinKey(p.id)]) out.push(name + ' uses ' + p.id + ', but nothing is wired to ' + p.id + '.');
    });
    return out;
  }

  /** Wired signal pins that none of the page's code uses. */
  function unusedPins(allPins, wired) {
    var used = {};
    (allPins || []).forEach(function (p) { if (p && p.id) used[pinKey(p.id)] = true; });
    return Object.keys(wired).filter(function (pin) { return !isSupply(pin) && !used[pinKey(pin)]; });
  }

  /** `https://github.com/o/n[/tree/<ref>[/<dir>]]` → { owner, name, ref, dir }. */
  function parseRepo(url) {
    var m = /^https:\/\/github\.com\/([A-Za-z0-9-]+)\/([A-Za-z0-9._-]+?)(?:\.git)?(?:\/(?:tree|blob)\/([^/\s]+)(?:\/([^\s?#]*))?)?\/?(?:[?#].*)?$/.exec(String(url || '').trim());
    if (!m) return null;
    var dir = (m[4] || '').replace(/\/+$/, '');
    if (dir.split('/').some(function (s) { return s === '..' || s === '.'; })) return null;
    return { owner: m[1], name: m[2], ref: m[3] || null, dir: dir };
  }

  /**
   * The files to show as tabs, from GitHub's listing of a folder: Python only,
   * the program a board runs first (main.py, code.py), then the rest by name.
   */
  function pickFiles(listing, max) {
    var first = ['main.py', 'code.py', 'boot.py'];
    var files = (Array.isArray(listing) ? listing : []).filter(function (f) {
      return f && f.type === 'file' && typeof f.path === 'string' && /\.py$/i.test(f.name || f.path);
    });
    files.sort(function (a, b) {
      var ia = first.indexOf(String(a.name).toLowerCase()), ib = first.indexOf(String(b.name).toLowerCase());
      if (ia !== ib) return (ia === -1 ? 99 : ia) - (ib === -1 ? 99 : ib);
      return String(a.name).localeCompare(String(b.name));
    });
    return files.slice(0, max || 5).map(function (f) { return f.path; });
  }

  // ------------------------------------------------------------------- SVG

  var SVG_BLOCKED = ['script', 'foreignobject', 'iframe', 'object', 'embed', 'audio', 'video', 'handler', 'listener', 'set', 'animate'];

  /**
   * The API already cleans a wiring SVG before it stores it (svg_safety.py).
   * It is about to become part of this page rather than an <img>, so clean it
   * again here: drawing only.
   */
  function cleanSvg(text) {
    var doc = new DOMParser().parseFromString(String(text), 'image/svg+xml');
    var svg = doc.documentElement;
    if (!svg || svg.nodeName.toLowerCase() !== 'svg' || doc.getElementsByTagName('parsererror').length) return null;
    var all = [svg].concat(Array.prototype.slice.call(svg.getElementsByTagName('*')));
    all.forEach(function (el) {
      if (SVG_BLOCKED.indexOf(el.localName.toLowerCase()) !== -1) { el.remove(); return; }
      Array.prototype.slice.call(el.attributes).forEach(function (attr) {
        var name = attr.localName.toLowerCase(), value = String(attr.value);
        if (name.indexOf('on') === 0) el.removeAttributeNode(attr);
        else if (name === 'href' && !/^(#|data:image\/(png|jpe?g|gif|webp);base64,)/i.test(value.trim())) el.removeAttributeNode(attr);
        else if (/^\s*javascript:/i.test(value)) el.removeAttributeNode(attr);
        else if (name === 'style' && /javascript:|expression\s*\(|url\(\s*(?!['"]?\s*(?:#|data:image\/))/i.test(value)) el.removeAttributeNode(attr);
      });
      if (el.localName.toLowerCase() === 'style' && /javascript:|@import|url\(\s*(?!['"]?\s*(?:#|data:image\/))/i.test(el.textContent || '')) el.textContent = '';
    });
    return document.importNode(svg, true);
  }

  // ---------------------------------------------------------------- viewer

  var viewers = [];

  function button(label, text, onClick) {
    var b = document.createElement('button');
    b.type = 'button';
    b.className = 'wiring-live-btn';
    b.setAttribute('aria-label', label);
    b.title = label;
    b.textContent = text;
    b.addEventListener('click', onClick);
    return b;
  }

  /** Turn one wiring <img> into the viewer. `robot` may be null (no wire names). */
  function buildViewer(img, svg, robot) {
    var wired = boardPins(robot);
    var byWire = {};
    ((robot && robot.connections) || []).forEach(function (c) { if (c && c.id) byWire[c.id] = c; });
    // wire id → the board pins of its net
    var pinsOfWire = {};
    Object.keys(wired).forEach(function (pin) {
      wired[pin].wires.forEach(function (id) { (pinsOfWire[id] = pinsOfWire[id] || []).push(pin); });
    });

    var fig = document.createElement('figure');
    fig.className = 'wiring-live';
    var stage = document.createElement('div');
    stage.className = 'wiring-live-stage';
    svg.removeAttribute('width');
    svg.removeAttribute('height');
    svg.setAttribute('role', 'img');
    svg.setAttribute('aria-label', img.getAttribute('alt') || 'Wiring diagram');
    stage.appendChild(svg);
    var tip = document.createElement('div');
    tip.className = 'wiring-live-tip';
    tip.setAttribute('role', 'status');
    var groups = Array.prototype.slice.call(svg.querySelectorAll('[data-wire]'));
    var hint = groups.length ? 'Point at a wire to see what it joins.' : '';
    tip.textContent = hint;

    // Zoom and drag: a transform on the picture inside a clipped stage.
    var scale = 1, tx = 0, ty = 0;
    function apply() {
      svg.style.transform = 'translate(' + tx + 'px,' + ty + 'px) scale(' + scale + ')';
      fig.classList.toggle('is-zoomed', scale > 1);
    }
    function zoom(factor) {
      var next = Math.max(1, Math.min(6, scale * factor));
      // Keep the middle of the stage where it is.
      tx = tx * (next / scale);
      ty = ty * (next / scale);
      scale = next;
      if (scale === 1) { tx = 0; ty = 0; }
      apply();
    }
    var drag = null;
    stage.addEventListener('pointerdown', function (e) {
      if (scale <= 1 || e.button !== 0) return;
      drag = { x: e.clientX - tx, y: e.clientY - ty };
      stage.setPointerCapture(e.pointerId);
    });
    stage.addEventListener('pointermove', function (e) {
      if (!drag) return;
      tx = e.clientX - drag.x;
      ty = e.clientY - drag.y;
      apply();
    });
    ['pointerup', 'pointercancel'].forEach(function (name) {
      stage.addEventListener(name, function () { drag = null; });
    });
    var tools = document.createElement('div');
    tools.className = 'wiring-live-tools';
    tools.appendChild(button('Zoom in', '+', function () { zoom(1.5); }));
    tools.appendChild(button('Zoom out', '−', function () { zoom(1 / 1.5); }));
    tools.appendChild(button('Fit the whole diagram', 'Fit', function () { scale = 1; tx = 0; ty = 0; apply(); }));

    // Lighting wires. While the reader points at a wire here, a code box
    // saying "nothing hovered" (the pointer just left it) must not clear it.
    var pointing = false;
    function light(ids, text) {
      var on = {};
      ids.forEach(function (id) { on[id] = true; });
      groups.forEach(function (g) { g.classList.toggle('is-lit', !!on[g.getAttribute('data-wire')]); });
      fig.classList.toggle('has-lit', ids.length > 0);
      tip.textContent = text || hint;
    }
    function tellCode(pins) {
      if (window.SnakieEmbed && typeof window.SnakieEmbed.hover === 'function') window.SnakieEmbed.hover(pins);
    }
    groups.forEach(function (g) {
      var id = g.getAttribute('data-wire');
      var wire = byWire[id] || { from: g.getAttribute('data-from'), to: g.getAttribute('data-to') };
      var label = wireLabel(robot, wire);
      var title = document.createElementNS('http://www.w3.org/2000/svg', 'title');
      title.textContent = label;
      g.appendChild(title);
      g.addEventListener('pointerenter', function () {
        var pins = pinsOfWire[id] || [];
        // The whole net lights: every wire that carries this pin.
        var ids = pins.length ? [].concat.apply([], pins.map(function (p) { return wired[p].wires; })) : [id];
        pointing = true;
        light(ids, label);
        tellCode(pins);
      });
      g.addEventListener('pointerleave', function () { pointing = false; light([]); tellCode([]); });
    });

    fig.appendChild(stage);
    fig.appendChild(tools);
    if (hint) fig.appendChild(tip);
    img.classList.add('wiring-live-print');
    img.insertAdjacentElement('afterend', fig);

    var viewer = {
      /** A code box is pointing at these pins. */
      hover: function (pins) {
        if (pointing) return;
        var ids = [], names = [];
        var keys = (pins || []).map(pinKey);
        Object.keys(wired).forEach(function (pin) {
          if (keys.indexOf(pinKey(pin)) === -1) return;
          ids = ids.concat(wired[pin].wires);
          names.push(pin + ' to ' + wired[pin].ends.join(', '));
        });
        light(ids, names.join('; '));
      },
      el: fig
    };
    viewers.push(viewer);
    return viewer;
  }

  var hoverBound = false;
  function bindHover() {
    if (hoverBound) return;
    hoverBound = true;
    document.addEventListener('snakie:pin-hover', function (e) {
      var pins = (e.detail && e.detail.pins) || [];
      viewers.forEach(function (v) { if (v.el.isConnected) v.hover(pins); });
    });
  }

  /**
   * Turn each wiring picture under `root` into the viewer. `opts`:
   * { api, projectId, withVersion?(url) }. Returns a promise of how many.
   */
  function wiringViewers(root, opts) {
    var imgs = Array.prototype.slice.call(root.querySelectorAll('img[src]')).filter(function (img) {
      return /\/api\/projects\/\d+\/wiring\.svg(\?|$)/.test(img.getAttribute('src')) && !img.classList.contains('wiring-live-print');
    });
    if (!imgs.length || typeof DOMParser === 'undefined') return Promise.resolve(0);
    var v = opts.withVersion || function (u) { return u; };
    var robot = fetch(v(opts.api + '/api/projects/' + opts.projectId + '/wiring'), { credentials: 'include' })
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (body) { return body && body.data ? body.data : null; })
      .catch(function () { return null; });
    bindHover();
    return Promise.all(imgs.map(function (img) {
      return Promise.all([
        fetch(img.getAttribute('src'), { credentials: 'include' }).then(function (r) { return r.ok ? r.text() : null; }).catch(function () { return null; }),
        robot
      ]).then(function (got) {
        var svg = got[0] ? cleanSvg(got[0]) : null;
        if (!svg || !img.isConnected) return 0;
        buildViewer(img, svg, got[1]);
        return 1;
      });
    })).then(function (ns) { return ns.reduce(function (a, b) { return a + b; }, 0); });
  }

  // ----------------------------------------------------------- Code section

  /** GitHub's listing of the repo folder, remembered for the session (60 asks an hour per reader). */
  function listRepo(repo) {
    var key = 'kr-repo-files:' + repo.owner + '/' + repo.name + '@' + (repo.ref || '') + ':' + repo.dir;
    try {
      var saved = sessionStorage.getItem(key);
      if (saved) return Promise.resolve(JSON.parse(saved));
    } catch (_) { /* no storage: just ask */ }
    var url = 'https://api.github.com/repos/' + repo.owner + '/' + repo.name + '/contents/' +
      repo.dir.split('/').map(encodeURIComponent).join('/') + (repo.ref ? '?ref=' + encodeURIComponent(repo.ref) : '');
    return fetch(url, { headers: { Accept: 'application/vnd.github+json' } })
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (listing) {
        var files = pickFiles(listing, 5);
        try { sessionStorage.setItem(key, JSON.stringify(files)); } catch (_) { /* fine */ }
        return files;
      })
      .catch(function () { return []; });
  }

  /**
   * Show the project's repository as a Snakie box in `section` (which holds a
   * heading and a `.view-code-box`). Nothing is shown when the repo has no
   * Python at that folder, or GitHub won't say. `opts`: { repoUrl, projectId }.
   */
  function codeSection(section, opts) {
    var repo = parseRepo(opts.repoUrl);
    if (!repo || !section || !window.ProjectEmbeds) return Promise.resolve(false);
    return listRepo(repo).then(function (files) {
      if (!files.length) return false;
      var box = section.querySelector('.view-code-box');
      box.replaceChildren();
      var holder = document.createElement('div');
      holder.className = 'project-code-embed snakie-box';
      holder.style.borderRadius = '12px';
      holder.style.overflow = 'hidden';
      var link = document.createElement('a');
      link.href = opts.repoUrl;
      link.rel = 'noopener';
      link.target = '_blank';
      link.textContent = 'See the code on GitHub';
      holder.appendChild(link);
      box.appendChild(holder);
      section.classList.remove('d-none');
      var mount = { repo: repo.owner + '/' + repo.name, path: files[0] };
      if (files.length > 1) mount.paths = files.join(',');
      if (repo.ref) mount.ref = repo.ref;
      if (opts.projectId) mount.project = String(opts.projectId);
      return window.ProjectEmbeds.loadSnakieEmbed()
        .then(function (api) { if (api) api.mount(holder, mount); return true; })
        .catch(function () { return true; });
    });
  }

  // ------------------------------------------------------------ robot frame

  /** Does this list of project files hold a robot model (a URDF)? */
  function hasRobot(files) {
    return (files || []).some(function (f) { return f && /\.urdf$/i.test(String(f.filename || '')); });
  }

  /**
   * The project's robot as a Snakie frame: it draws the URDF and meshes among
   * the project's files, and Run in a code box on the page turns the joints
   * the wiring binds to servo pins. Shown only when the project has a URDF and
   * the Snakie script on hand knows robot frames. Resolves to whether it showed.
   */
  function robotSection(section, opts) {
    if (!section || !hasRobot(opts.files) || !window.ProjectEmbeds) return Promise.resolve(false);
    return window.ProjectEmbeds.loadSnakieEmbed().then(function (api) {
      if (!api || !api.robot) return false;
      var box = section.querySelector('.view-robot-box');
      box.replaceChildren();
      var holder = document.createElement('div');
      holder.className = 'project-robot-embed snakie-box';
      holder.style.borderRadius = '12px';
      holder.style.overflow = 'hidden';
      box.appendChild(holder);
      section.classList.remove('d-none');
      api.mount(holder, { robot: '1', project: String(opts.projectId), title: 'This project’s robot in Snakie' });
      return true;
    }).catch(function () { return false; });
  }

  // ------------------------------------------------------- editor warnings

  /**
   * In the editor: keep `bar` saying where the code boxes and the wiring
   * disagree. `loadRobot()` resolves to the project's wiring (or null). The
   * wiring is read again when a code box reports, at most every few seconds,
   * so a wire added in the workspace clears its warning.
   */
  function watchWarnings(bar, loadRobot) {
    var byFrame = {};
    var robot = null, fetchedAt = 0, pending = null;
    function fresh() {
      if (pending) return pending;
      if (Date.now() - fetchedAt < 4000) return Promise.resolve(robot);
      pending = Promise.resolve().then(loadRobot).catch(function () { return null; }).then(function (r) {
        robot = r; fetchedAt = Date.now(); pending = null;
        return r;
      });
      return pending;
    }
    function draw() {
      fresh().then(function (r) {
        var lines = [];
        if (r && Array.isArray(r.connections) && r.connections.length) {
          var wired = boardPins(r);
          Object.keys(byFrame).forEach(function (id) {
            mismatches(byFrame[id].file, byFrame[id].pins, wired).forEach(function (line) {
              if (lines.indexOf(line) === -1) lines.push(line);
            });
          });
        }
        bar.replaceChildren();
        bar.hidden = lines.length === 0;
        lines.slice(0, 4).forEach(function (line) {
          var p = document.createElement('div');
          p.textContent = line;
          bar.appendChild(p);
        });
        if (lines.length > 4) {
          var more = document.createElement('div');
          more.textContent = 'And ' + (lines.length - 4) + ' more.';
          bar.appendChild(more);
        }
      });
    }
    document.addEventListener('snakie:pins', function (e) {
      var d = e.detail;
      if (!d || !d.id) return;
      byFrame[d.id] = { file: String(d.file || '').split('/').pop(), pins: d.pins || [] };
      draw();
    });
    return { refresh: function () { fetchedAt = 0; draw(); } };
  }

  var api = {
    boardPins: boardPins, pinKey: pinKey, isSupply: isSupply, wireLabel: wireLabel, mismatches: mismatches,
    unusedPins: unusedPins, parseRepo: parseRepo, pickFiles: pickFiles, cleanSvg: cleanSvg,
    wiringViewers: wiringViewers, codeSection: codeSection, watchWarnings: watchWarnings,
    hasRobot: hasRobot, robotSection: robotSection
  };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else window.ProjectLive = api;
})();
