/**
 * Schematic Editor — issue #178 Phase E2.
 *
 * Full-screen editor for project schematics. One schematic per project
 * for v1; the graph (symbol instances + nets + labels) is held entirely
 * client-side and serialised to a single JSON blob in
 * ``project_schematics.schematic_data`` via the backend's
 * /api/projects/{id}/schematic endpoint.
 *
 * Renderer: Fabric.js v6.4.0 (CDN UMD, exposes ``window.fabric``).
 *
 * Coordinate system: a fixed 1600 × 1200 logical scene scaled to fit the
 * panel. The dot grid is rendered as a Fabric pattern (16px display
 * pitch == 2.54mm conceptual pitch) so snap math is just
 * ``Math.round(p / 16) * 16``.
 *
 * State persistence: the entire STATE.schematic document is PUT to
 * /api/projects/{id}/schematic via a debounced (800ms) autosave; the
 * server stores and returns it verbatim.
 *
 * Out of scope (will land in follow-up PRs):
 *   - Symbol Designer (the library is hard-coded JS data here).
 *   - ERC / pin-type checking.
 *   - Net auto-routing — segments are computed via a single L-bend.
 *   - Multiple schematics per project.
 *   - Region cropping for embedding into instruction steps.
 */
(function () {
  'use strict';

  // ====================================================================
  // 1. CONSTANTS
  // ====================================================================

  var API = 'https://projects.kevsrobots.com';
  var SCENE_W = 1600;
  var SCENE_H = 1200;
  var GRID = 16;                  // display pitch (px) — represents 2.54mm
  var AUTOSAVE_MS = 800;

  // Brand colours
  var INK = '#222222';            // default symbol + net colour
  var BRAND_RED = '#c8312a';      // selected net highlight + selection border
  var BG = '#ffffff';
  var GRID_DOT = '#cfd3d8';       // colour of the background dot grid
  var JUNCTION_DOT_R = 4;         // px

  // Junction-merge tolerance — any two segment endpoints within this
  // distance (px) count as "meeting at the same point" for junction-dot
  // rendering and net-merge detection. With the snap forcing all
  // endpoints to GRID multiples this should never be more than ~0px in
  // practice, but a small tolerance protects against rounding noise from
  // the orthogonal-routing computation.
  var JUNCTION_TOLERANCE = 2;

  // Selected net highlight stroke width. Slightly thicker than the
  // default so the selection is obvious without re-laying-out.
  var NET_STROKE = 2;
  var NET_STROKE_SELECTED = 3.5;

  // ====================================================================
  // 2. SYMBOL LIBRARY (stub — Symbol Designer ships next)
  // ====================================================================
  //
  // Pin offsets are in scene units (px). ``side`` controls which edge
  // the pin attaches to. The hard-coded set covers a credible minimal
  // circuit (Pico + R + C + LED + GND + V+) and one generic 14-pin IC
  // for representing other ICs the user doesn't have a dedicated symbol
  // for. The full library lives in the Symbol Designer (next PR).

  var SYMBOL_LIBRARY = [
    {
      id: 'rpi-pico',
      name: 'Raspberry Pi Pico',
      refDesPrefix: 'U',
      bodyWidth: 160,
      bodyHeight: 320,
      // Subset of the Pico's pinout — enough to demonstrate net drawing.
      // Each entry maps a pin number to its symbol-relative position.
      pins: [
        // Left edge — GPIOs
        { number: '1',  name: 'GP0',  side: 'left',  offset: 32 },
        { number: '2',  name: 'GP1',  side: 'left',  offset: 64 },
        { number: '3',  name: 'GND',  side: 'left',  offset: 96 },
        { number: '4',  name: 'GP2',  side: 'left',  offset: 128 },
        { number: '5',  name: 'GP3',  side: 'left',  offset: 160 },
        { number: '6',  name: 'GP4',  side: 'left',  offset: 192 },
        { number: '7',  name: 'GP5',  side: 'left',  offset: 224 },
        { number: '8',  name: 'GND',  side: 'left',  offset: 256 },
        { number: '9',  name: 'GP6',  side: 'left',  offset: 288 },
        // Right edge — power + extras
        { number: '36', name: '3V3',  side: 'right', offset: 32 },
        { number: '37', name: '3V3_EN', side: 'right', offset: 64 },
        { number: '38', name: 'GND',  side: 'right', offset: 96 },
        { number: '39', name: 'VSYS', side: 'right', offset: 128 },
        { number: '40', name: 'VBUS', side: 'right', offset: 160 },
        { number: '34', name: 'GP28', side: 'right', offset: 192 },
        { number: '33', name: 'GND',  side: 'right', offset: 224 },
        { number: '32', name: 'GP27', side: 'right', offset: 256 },
        { number: '31', name: 'GP26', side: 'right', offset: 288 },
      ],
    },
    {
      id: 'ic-14',
      name: 'Generic 14-pin IC',
      refDesPrefix: 'U',
      bodyWidth: 128,
      bodyHeight: 224,
      pins: [
        { number: '1',  name: '1',  side: 'left',  offset: 32 },
        { number: '2',  name: '2',  side: 'left',  offset: 64 },
        { number: '3',  name: '3',  side: 'left',  offset: 96 },
        { number: '4',  name: '4',  side: 'left',  offset: 128 },
        { number: '5',  name: '5',  side: 'left',  offset: 160 },
        { number: '6',  name: '6',  side: 'left',  offset: 192 },
        { number: '7',  name: 'GND', side: 'left', offset: 224 - 32 },
        { number: '14', name: 'VCC', side: 'right', offset: 32 },
        { number: '13', name: '13', side: 'right', offset: 64 },
        { number: '12', name: '12', side: 'right', offset: 96 },
        { number: '11', name: '11', side: 'right', offset: 128 },
        { number: '10', name: '10', side: 'right', offset: 160 },
        { number: '9',  name: '9',  side: 'right', offset: 192 },
        { number: '8',  name: '8',  side: 'right', offset: 224 - 32 },
      ],
    },
    {
      id: 'resistor',
      name: 'Resistor',
      refDesPrefix: 'R',
      bodyWidth: 64,
      bodyHeight: 32,
      pins: [
        { number: '1', name: '1', side: 'left',  offset: 16 },
        { number: '2', name: '2', side: 'right', offset: 16 },
      ],
    },
    {
      id: 'capacitor',
      name: 'Capacitor',
      refDesPrefix: 'C',
      bodyWidth: 64,
      bodyHeight: 32,
      pins: [
        { number: '1', name: '+', side: 'left',  offset: 16 },
        { number: '2', name: '-', side: 'right', offset: 16 },
      ],
    },
    {
      id: 'led',
      name: 'LED',
      refDesPrefix: 'D',
      bodyWidth: 64,
      bodyHeight: 48,
      pins: [
        { number: '1', name: 'A', side: 'left',  offset: 24 },
        { number: '2', name: 'K', side: 'right', offset: 24 },
      ],
    },
    {
      id: 'gnd',
      name: 'GND',
      refDesPrefix: 'GND',
      bodyWidth: 48,
      bodyHeight: 32,
      // GND has one pin attached at the top — wires drop into the
      // GND symbol from above.
      pins: [
        { number: '1', name: 'GND', side: 'top', offset: 24 },
      ],
    },
    {
      id: 'vplus',
      name: 'V+ rail',
      refDesPrefix: 'V',
      bodyWidth: 48,
      bodyHeight: 32,
      pins: [
        { number: '1', name: 'V+', side: 'bottom', offset: 24 },
      ],
    },
  ];

  function symbolDefById(id) {
    for (var i = 0; i < SYMBOL_LIBRARY.length; i++) {
      if (SYMBOL_LIBRARY[i].id === id) return SYMBOL_LIBRARY[i];
    }
    return null;
  }

  // Convert a Symbol Designer document (``{ bodyShapes, pins }`` in
  // symbol-space coords centred on (0,0)) into the editor's stub-library
  // shape (``{ bodyWidth, bodyHeight, pins: [{ side, offset, ... }] }``).
  // The pin x/y coords from the designer come in centred-at-origin
  // space; the editor's ``pinLocalPos`` expects pins to live on the
  // body's edge with an ``offset`` from the top-left corner along the
  // relevant axis. The bounding box is computed from the union of pin
  // coords + body shapes so each user-designed symbol stays compact.
  function symbolDefFromCustom(row) {
    if (!row || !row.symbol_data) return null;
    var doc;
    try { doc = JSON.parse(row.symbol_data); }
    catch (_) { return null; }
    if (!doc || typeof doc !== 'object') return null;
    var pins = Array.isArray(doc.pins) ? doc.pins : [];
    var shapes = Array.isArray(doc.bodyShapes) ? doc.bodyShapes : [];

    // Bounding box from pins + shapes (centred coords).
    var xs = [], ys = [];
    pins.forEach(function (p) { xs.push(p.x || 0); ys.push(p.y || 0); });
    shapes.forEach(function (s) {
      if (s.kind === 'rect') {
        xs.push(s.x); xs.push(s.x + s.w);
        ys.push(s.y); ys.push(s.y + s.h);
      } else if (s.kind === 'line') {
        xs.push(s.x1); xs.push(s.x2);
        ys.push(s.y1); ys.push(s.y2);
      }
    });
    if (xs.length === 0) { xs = [-32, 32]; ys = [-24, 24]; }
    var minX = Math.min.apply(null, xs);
    var maxX = Math.max.apply(null, xs);
    var minY = Math.min.apply(null, ys);
    var maxY = Math.max.apply(null, ys);

    // Pad slightly so pins stick out of the body rather than sitting on
    // its corners — feels like a real symbol.
    var pad = 8;
    minX -= pad; minY -= pad; maxX += pad; maxY += pad;
    var bodyWidth = Math.max(maxX - minX, 32);
    var bodyHeight = Math.max(maxY - minY, 32);

    // Convert each pin's centred coord into a (side, offset) the editor
    // can render. ``offset`` is measured from the top-left of the body
    // box, along the axis that runs parallel to that side.
    var convertedPins = pins.map(function (p, i) {
      var offset = (p.side === 'left' || p.side === 'right')
        ? (p.y - minY)
        : (p.x - minX);
      return {
        number: p.number || String(i + 1),
        name: p.name || (p.number || String(i + 1)),
        side: p.side || 'left',
        offset: Math.max(0, Math.round(offset)),
        type: p.type || 'I/O',
      };
    });

    return {
      id: 'custom-' + row.id,
      name: row.name || ('Custom #' + row.id),
      refDesPrefix: row.ref_des_prefix || 'U',
      bodyWidth: bodyWidth,
      bodyHeight: bodyHeight,
      pins: convertedPins,
      isCustom: true,
      customId: row.id,
      // Optional body shapes the designer drew — kept so the renderer
      // could overlay them later. For v1 the editor uses the bounding
      // box only (mirrors the stub library look).
      _bodyShapes: shapes,
      _minX: minX,
      _minY: minY,
    };
  }

  // ====================================================================
  // 3. STATE
  // ====================================================================

  var STATE = {
    projectId: null,
    project: null,
    me: null,
    isOwner: false,
    schematic: null,           // ProjectSchematicResponse — id + graph wrapper
    graph: emptyGraph(),       // mutable in-memory copy of the JSON document
    canvas: null,              // fabric.Canvas instance

    activeTool: 'select',
    pendingSymbolId: null,     // when the user picks a symbol; null in select mode
    selectedInstanceId: null,
    selectedNetId: null,

    // While drawing a net: from-endpoint + preview Fabric object.
    netDrawingFrom: null,      // { instanceId, pinNumber, x, y }
    netPreviewObj: null,

    refDesCounters: {},        // { U: 1, R: 1, ... } — auto-incrementing per prefix
    netCounter: 0,             // for auto-naming Net-1, Net-2, …

    saveStatus: 'saved',
    autosaveTimer: null,

    // Viewport state: multiplier on top of the auto-fit scale. 1 = "fit
    // the whole scene into the wrap". > 1 zooms in, < 1 zooms out.
    // Pan offsets accumulate in CSS pixel units relative to the
    // auto-centred default.
    userZoom: 1,
    panX: 0,
    panY: 0,
  };

  function emptyGraph() {
    return {
      id: 0,
      name: '',
      description: '',
      instances: [],
      nets: [],
      labels: [],
    };
  }

  var dom = {};

  // ====================================================================
  // 4. UTILITIES
  // ====================================================================

  function nextId(prefix) {
    // Lightweight uid generator — graph-local, doesn't need to be unique
    // across schematics, just within the current one.
    return prefix + '-' + Math.random().toString(36).slice(2, 10);
  }

  function snap(v) {
    return Math.round(v / GRID) * GRID;
  }

  function escapeHtml(s) {
    return String(s == null ? '' : s)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;');
  }

  function apiFetch(url, opts) {
    opts = opts || {};
    if (window.ProjectAuth && typeof window.ProjectAuth.apiFetch === 'function') {
      return window.ProjectAuth.apiFetch(url, opts);
    }
    opts.credentials = opts.credentials || 'include';
    return fetch(url, opts);
  }

  async function apiFetchWithTermsRetry(url, opts) {
    var resp = await apiFetch(url, opts);
    if (resp.status === 403 && window.TermsGate &&
        typeof window.TermsGate.handleResponse === 'function') {
      try {
        var retry = await window.TermsGate.handleResponse(resp);
        if (retry) return apiFetch(url, opts);
      } catch (_) { /* fall through with original 403 */ }
    }
    return resp;
  }

  function showOnly(el) {
    [dom.loading, dom.notOwner, dom.error, dom.main].forEach(function (n) {
      if (!n) return;
      if (n === el) n.classList.remove('d-none');
      else n.classList.add('d-none');
    });
  }

  function setSaveStatus(status, label) {
    STATE.saveStatus = status;
    if (!dom.saveStatus) return;
    var icon, text;
    switch (status) {
      case 'saving': icon = 'fa-circle-notch fa-spin'; text = 'Saving…'; break;
      case 'error':  icon = 'fa-triangle-exclamation'; text = label || 'Save failed'; break;
      case 'saved':
      default:       icon = 'fa-check'; text = 'Saved'; break;
    }
    dom.saveStatus.innerHTML =
      '<i class="fas ' + icon + ' me-1"></i>' + escapeHtml(text);
  }

  // ====================================================================
  // 5. SYMBOL INSTANCE MATH
  // ====================================================================
  //
  // A SymbolInstance has an origin point ``(x, y)`` (centre of the
  // symbol body). Pin positions are computed relative to that origin
  // accounting for the symbol's rotation + flips. ``pinScenePos`` is
  // the single source of truth used by both the renderer and the
  // net-drawing logic.

  function symbolWidth(symbolDef, rotation) {
    return (rotation === 90 || rotation === 270)
      ? symbolDef.bodyHeight : symbolDef.bodyWidth;
  }
  function symbolHeight(symbolDef, rotation) {
    return (rotation === 90 || rotation === 270)
      ? symbolDef.bodyWidth : symbolDef.bodyHeight;
  }

  // Compute a pin's position in the symbol's *unrotated*, *unflipped*
  // local frame (origin = top-left of bodyWidth × bodyHeight rect).
  function pinLocalPos(symbolDef, pin) {
    switch (pin.side) {
      case 'left':   return { x: 0,                       y: pin.offset };
      case 'right':  return { x: symbolDef.bodyWidth,     y: pin.offset };
      case 'top':    return { x: pin.offset,              y: 0 };
      case 'bottom': return { x: pin.offset,              y: symbolDef.bodyHeight };
      default:       return { x: 0, y: 0 };
    }
  }

  // Convert a local-frame point to the symbol's *instance* frame
  // (origin = the symbol's instance.x / instance.y, i.e. its centre).
  // Applies rotation (0/90/180/270) + horizontal/vertical flip.
  function localToScene(instance, symbolDef, lx, ly) {
    var cx = symbolDef.bodyWidth / 2;
    var cy = symbolDef.bodyHeight / 2;
    var dx = lx - cx;
    var dy = ly - cy;
    if (instance.flipH) dx = -dx;
    if (instance.flipV) dy = -dy;
    var rot = ((instance.rotation || 0) % 360 + 360) % 360;
    var rx, ry;
    switch (rot) {
      case 0:   rx = dx;  ry = dy;  break;
      case 90:  rx = -dy; ry = dx;  break;
      case 180: rx = -dx; ry = -dy; break;
      case 270: rx = dy;  ry = -dx; break;
      default:  rx = dx;  ry = dy;
    }
    return { x: instance.x + rx, y: instance.y + ry };
  }

  function pinScenePos(instance, pin) {
    var symbolDef = symbolDefById(instance.symbolId);
    if (!symbolDef) return { x: instance.x, y: instance.y };
    var local = pinLocalPos(symbolDef, pin);
    return localToScene(instance, symbolDef, local.x, local.y);
  }

  function findInstance(id) {
    for (var i = 0; i < STATE.graph.instances.length; i++) {
      if (STATE.graph.instances[i].id === id) return STATE.graph.instances[i];
    }
    return null;
  }

  function findPin(instance, pinNumber) {
    var symbolDef = symbolDefById(instance.symbolId);
    if (!symbolDef) return null;
    for (var i = 0; i < symbolDef.pins.length; i++) {
      if (symbolDef.pins[i].number === pinNumber) return symbolDef.pins[i];
    }
    return null;
  }

  // ====================================================================
  // 6. SYMBOL PLACEMENT + REF-DES ASSIGNMENT
  // ====================================================================

  function bumpRefDes(prefix) {
    var n = (STATE.refDesCounters[prefix] || 0) + 1;
    STATE.refDesCounters[prefix] = n;
    return prefix + n;
  }

  // Recompute the refDes counters by scanning instances. Called when a
  // saved graph is loaded so subsequent placements continue the
  // sequence rather than colliding.
  function rebuildRefDesCounters() {
    STATE.refDesCounters = {};
    STATE.graph.instances.forEach(function (inst) {
      var m = (inst.refDes || '').match(/^([A-Za-z]+)(\d+)$/);
      if (!m) return;
      var prefix = m[1];
      var n = parseInt(m[2], 10);
      if (n > (STATE.refDesCounters[prefix] || 0)) {
        STATE.refDesCounters[prefix] = n;
      }
    });
  }

  function rebuildNetCounter() {
    var max = 0;
    STATE.graph.nets.forEach(function (net) {
      var m = (net.name || '').match(/^Net-(\d+)$/);
      if (m) {
        var n = parseInt(m[1], 10);
        if (n > max) max = n;
      }
    });
    STATE.netCounter = max;
  }

  function placeSymbolAt(symbolId, sceneX, sceneY) {
    var symbolDef = symbolDefById(symbolId);
    if (!symbolDef) return null;
    var instance = {
      id: nextId('inst'),
      symbolId: symbolId,
      refDes: bumpRefDes(symbolDef.refDesPrefix),
      x: snap(sceneX),
      y: snap(sceneY),
      rotation: 0,
      flipH: false,
      flipV: false,
    };
    STATE.graph.instances.push(instance);
    renderGraph();
    scheduleAutosave();
    return instance;
  }

  // ====================================================================
  // 7. NET DRAWING
  // ====================================================================
  //
  // Net topology:
  //   - A Net groups endpoints (pin references) and an ordered list of
  //     wire segments (each a 4-tuple: x1,y1,x2,y2).
  //   - When the user clicks two pins, we compute an L-shaped polyline
  //     between them (single 90° corner) and:
  //       * if both pins are unconnected → create a new net,
  //       * if one is on an existing net → join into that net,
  //       * if both are on different nets → MERGE the two nets into
  //         one. The merge call is documented in the wrap-up.

  function endpointsMatch(a, b) {
    return a.instanceId === b.instanceId && a.pinNumber === b.pinNumber;
  }

  function findNetByEndpoint(endpoint) {
    for (var i = 0; i < STATE.graph.nets.length; i++) {
      var net = STATE.graph.nets[i];
      for (var j = 0; j < net.endpoints.length; j++) {
        if (endpointsMatch(net.endpoints[j], endpoint)) return net;
      }
    }
    return null;
  }

  // Auto-name nets touching named power pins (GND / VBUS / VCC / 3V3 / V+)
  // — the auto-name dramatically improves the connections-table read.
  function autoNetNameForPin(instance, pin) {
    var name = (pin.name || '').toUpperCase();
    if (name === 'GND') return 'GND';
    if (name === 'VBUS' || name === 'VCC' || name === 'V+' ||
        name === '3V3' || name === '5V')   return name;
    var symbolDef = symbolDefById(instance.symbolId);
    if (symbolDef && symbolDef.id === 'gnd') return 'GND';
    if (symbolDef && symbolDef.id === 'vplus') return 'VBUS';
    return null;
  }

  function newNet(name) {
    if (!name) {
      STATE.netCounter += 1;
      name = 'Net-' + STATE.netCounter;
    }
    var net = {
      id: nextId('net'),
      name: name,
      color: INK,
      description: '',
      endpoints: [],
      segments: [],
    };
    STATE.graph.nets.push(net);
    return net;
  }

  // Compute an L-shaped polyline between (x1,y1) and (x2,y2). One 90°
  // corner; horizontal-then-vertical is chosen so the resulting wire
  // visually exits the source pin along the symbol's pin axis (which
  // is left/right for the bulk of our library) — the connection feels
  // right for most layouts. A future router could choose corner
  // direction dynamically.
  function lShapedSegments(x1, y1, x2, y2) {
    x1 = snap(x1); y1 = snap(y1);
    x2 = snap(x2); y2 = snap(y2);
    if (x1 === x2 && y1 === y2) return [];
    if (x1 === x2 || y1 === y2) {
      // Straight line — no corner.
      return [{ x1: x1, y1: y1, x2: x2, y2: y2 }];
    }
    var cornerX = x2;
    var cornerY = y1;
    return [
      { x1: x1, y1: y1, x2: cornerX, y2: cornerY },
      { x1: cornerX, y1: cornerY, x2: x2, y2: y2 },
    ];
  }

  function mergeNets(target, source) {
    // Append source's endpoints (deduped) and segments into target,
    // then drop source from STATE.graph.nets.
    source.endpoints.forEach(function (ep) {
      var dup = target.endpoints.some(function (t) { return endpointsMatch(t, ep); });
      if (!dup) target.endpoints.push(ep);
    });
    source.segments.forEach(function (seg) { target.segments.push(seg); });
    if (source.description && !target.description) {
      target.description = source.description;
    }
    var idx = STATE.graph.nets.indexOf(source);
    if (idx >= 0) STATE.graph.nets.splice(idx, 1);
    return target;
  }

  // Connect two pin endpoints. Decides on a target net (creating /
  // merging as needed), appends the L-shaped segments + endpoints, and
  // re-renders. Returns the resulting net.
  function commitNetSegment(fromEp, toEp) {
    if (endpointsMatch(fromEp, toEp)) return null;

    var fromInst = findInstance(fromEp.instanceId);
    var toInst   = findInstance(toEp.instanceId);
    if (!fromInst || !toInst) return null;
    var fromPin = findPin(fromInst, fromEp.pinNumber);
    var toPin   = findPin(toInst,   toEp.pinNumber);
    if (!fromPin || !toPin) return null;

    var fromXY = pinScenePos(fromInst, fromPin);
    var toXY   = pinScenePos(toInst,   toPin);

    var fromNet = findNetByEndpoint(fromEp);
    var toNet   = findNetByEndpoint(toEp);
    var target;
    if (fromNet && toNet && fromNet !== toNet) {
      // Merge into the larger net (favours the named-power net if either
      // is named GND/VBUS/etc., otherwise just the longer one).
      var preferred = preferTargetNet(fromNet, toNet);
      var other     = (preferred === fromNet) ? toNet : fromNet;
      target = mergeNets(preferred, other);
    } else if (fromNet) {
      target = fromNet;
    } else if (toNet) {
      target = toNet;
    } else {
      var auto = autoNetNameForPin(fromInst, fromPin) ||
                 autoNetNameForPin(toInst,   toPin);
      target = newNet(auto);
    }

    [fromEp, toEp].forEach(function (ep) {
      var dup = target.endpoints.some(function (t) { return endpointsMatch(t, ep); });
      if (!dup) target.endpoints.push(ep);
    });
    lShapedSegments(fromXY.x, fromXY.y, toXY.x, toXY.y).forEach(function (s) {
      target.segments.push(s);
    });

    return target;
  }

  function preferTargetNet(a, b) {
    var aPower = isPowerNetName(a.name);
    var bPower = isPowerNetName(b.name);
    if (aPower && !bPower) return a;
    if (bPower && !aPower) return b;
    // Prefer the one with more endpoints — losing a smaller name is
    // less surprising than losing GND.
    return (a.endpoints.length >= b.endpoints.length) ? a : b;
  }
  function isPowerNetName(name) {
    return /^(GND|VBUS|VCC|3V3|5V|V\+)$/i.test(name || '');
  }

  // ====================================================================
  // 8. JUNCTION DOTS
  // ====================================================================
  //
  // A junction is a point where 3+ wire segment endpoints meet. We
  // compute it by counting how often each (x, y) appears as a segment
  // endpoint *across all nets*; any point with count >= 3 gets a dot.
  // Points within JUNCTION_TOLERANCE px are treated as identical (snap
  // makes this almost always exact, but the tolerance covers rounding).

  function computeJunctions() {
    var buckets = [];
    function bump(x, y) {
      for (var i = 0; i < buckets.length; i++) {
        var b = buckets[i];
        if (Math.abs(b.x - x) <= JUNCTION_TOLERANCE &&
            Math.abs(b.y - y) <= JUNCTION_TOLERANCE) {
          b.count += 1;
          return;
        }
      }
      buckets.push({ x: x, y: y, count: 1 });
    }
    STATE.graph.nets.forEach(function (net) {
      net.segments.forEach(function (s) {
        bump(s.x1, s.y1);
        bump(s.x2, s.y2);
      });
    });
    return buckets.filter(function (b) { return b.count >= 3; });
  }

  // ====================================================================
  // 9. RENDERING — FABRIC OBJECTS
  // ====================================================================

  function buildGridPattern() {
    // Render the dot grid as a small offscreen canvas and use it as the
    // background pattern. Cheap, scales with zoom.
    var off = document.createElement('canvas');
    off.width = GRID;
    off.height = GRID;
    var ctx = off.getContext('2d');
    ctx.fillStyle = BG;
    ctx.fillRect(0, 0, GRID, GRID);
    ctx.fillStyle = GRID_DOT;
    ctx.beginPath();
    ctx.arc(0, 0, 1.3, 0, Math.PI * 2);
    ctx.fill();
    return off;
  }

  function applyGridBackground() {
    if (!STATE.canvas) return;
    var pattern = new fabric.Pattern({
      source: buildGridPattern(),
      repeat: 'repeat',
    });
    STATE.canvas.set('backgroundColor', pattern);
    STATE.canvas.requestRenderAll();
  }

  // Build a Fabric.Group representing one symbol instance. Pins,
  // pin labels, body, refDes label and origin cross are all included.
  function buildInstanceFabric(instance) {
    var symbolDef = symbolDefById(instance.symbolId);
    if (!symbolDef) return null;
    var w = symbolDef.bodyWidth;
    var h = symbolDef.bodyHeight;
    var pieces = [];

    // Body
    var body = new fabric.Rect({
      left: -w / 2,
      top: -h / 2,
      width: w,
      height: h,
      fill: '#ffffff',
      stroke: INK,
      strokeWidth: 1.5,
      rx: 4,
      ry: 4,
      originX: 'left',
      originY: 'top',
    });
    pieces.push(body);

    // refDes + name label, anchored above the body. When the user has
    // set a friendly name on the instance, prefer that over the
    // generic symbol-type name (e.g. "U1 · Main controller" rather
    // than "U1 · Raspberry Pi 5").
    var friendly = (instance.label && instance.label.trim()) ? instance.label.trim() : null;
    var displayName = friendly || symbolDef.name;
    var refLabel = new fabric.Text(
      (instance.refDes || '') + ' · ' + displayName,
      {
        left: 0,
        top: -h / 2 - 22,
        fontSize: 12,
        fontFamily: 'system-ui, -apple-system, "Segoe UI", sans-serif',
        fill: INK,
        originX: 'center',
        originY: 'top',
      }
    );
    pieces.push(refLabel);

    // Pins: a short stick + a pin-name label.
    symbolDef.pins.forEach(function (pin) {
      var local = pinLocalPos(symbolDef, pin);
      // Translate from "0..bodyWidth" coords to body-centred coords.
      var px = local.x - w / 2;
      var py = local.y - h / 2;
      var stickLen = 8;
      var sx2 = px, sy2 = py;
      switch (pin.side) {
        case 'left':   sx2 = px - stickLen; sy2 = py; break;
        case 'right':  sx2 = px + stickLen; sy2 = py; break;
        case 'top':    sx2 = px; sy2 = py - stickLen; break;
        case 'bottom': sx2 = px; sy2 = py + stickLen; break;
      }
      var stick = new fabric.Line([px, py, sx2, sy2], {
        stroke: INK,
        strokeWidth: 1.5,
      });
      pieces.push(stick);

      // Pin label — anchored just inside the body, away from the stick.
      var lx = px, ly = py;
      var anchorX = 'left', anchorY = 'center';
      var pad = 4;
      switch (pin.side) {
        case 'left':   lx = px + pad;  ly = py - 6; anchorX = 'left';   anchorY = 'top'; break;
        case 'right':  lx = px - pad;  ly = py - 6; anchorX = 'right';  anchorY = 'top'; break;
        case 'top':    lx = px - 6;    ly = py + pad; anchorX = 'right'; anchorY = 'top'; break;
        case 'bottom': lx = px - 6;    ly = py - pad; anchorX = 'right'; anchorY = 'bottom'; break;
      }
      pieces.push(new fabric.Text(pin.name || pin.number, {
        left: lx,
        top: ly,
        fontSize: 9,
        fill: INK,
        fontFamily: 'system-ui, -apple-system, "Segoe UI", sans-serif',
        originX: anchorX,
        originY: anchorY,
      }));
    });

    // Origin cross (small red "+")
    pieces.push(new fabric.Line([-6, 0, 6, 0], { stroke: BRAND_RED, strokeWidth: 1 }));
    pieces.push(new fabric.Line([0, -6, 0, 6], { stroke: BRAND_RED, strokeWidth: 1 }));

    var group = new fabric.Group(pieces, {
      left: instance.x,
      top: instance.y,
      originX: 'center',
      originY: 'center',
      angle: instance.rotation || 0,
      flipX: !!instance.flipH,
      flipY: !!instance.flipV,
      lockScalingFlip: true,
      hasControls: false,         // no scale/skew handles — symbols are fixed size
      hasBorders: true,
      borderColor: BRAND_RED,
      borderScaleFactor: 1.2,
      objectCaching: false,
    });
    group.data = { kind: 'instance', instanceId: instance.id };
    return group;
  }

  function buildNetFabric(net, selected) {
    // One Polyline per segment — cheaper than building a single
    // multi-segment Path when we want individual segments to be
    // hit-testable.
    var objs = [];
    net.segments.forEach(function (seg) {
      var line = new fabric.Line([seg.x1, seg.y1, seg.x2, seg.y2], {
        stroke: selected ? BRAND_RED : (net.color || INK),
        strokeWidth: selected ? NET_STROKE_SELECTED : NET_STROKE,
        selectable: true,
        hasControls: false,
        hasBorders: false,
        hoverCursor: 'pointer',
        perPixelTargetFind: true,
        objectCaching: false,
      });
      line.data = { kind: 'net', netId: net.id };
      objs.push(line);
    });
    return objs;
  }

  function buildJunctionFabric(j) {
    return new fabric.Circle({
      left: j.x,
      top: j.y,
      radius: JUNCTION_DOT_R,
      fill: INK,
      originX: 'center',
      originY: 'center',
      selectable: false,
      evented: false,
    });
  }

  function buildPinHotspot(instance, pin) {
    // Invisible (but evented) hit target on each pin — only active in
    // Net-drawing mode so it doesn't compete with the symbol Group for
    // hover / click. Sized generously (12px square) so pin clicks are
    // forgiving.
    var pos = pinScenePos(instance, pin);
    var hot = new fabric.Circle({
      left: pos.x,
      top: pos.y,
      radius: 6,
      fill: 'rgba(0,0,0,0.001)',
      stroke: null,
      originX: 'center',
      originY: 'center',
      selectable: false,
      hasControls: false,
      hasBorders: false,
      hoverCursor: 'crosshair',
      evented: true,
    });
    hot.data = {
      kind: 'pin-hotspot',
      instanceId: instance.id,
      pinNumber: pin.number,
      x: pos.x,
      y: pos.y,
    };
    return hot;
  }

  // ====================================================================
  // 10. RENDER LOOP
  // ====================================================================
  //
  // ``renderGraph`` is the single rebuild entry point: it wipes the
  // Fabric canvas and reconstructs every visible object from
  // STATE.graph + selection. Coarse but simple — the canvas rarely
  // holds more than ~50 symbols + ~100 segments in v1.

  function renderGraph() {
    if (!STATE.canvas) return;
    STATE.canvas.discardActiveObject();
    STATE.canvas.clear();
    applyGridBackground();

    // Nets first so symbols sit on top.
    STATE.graph.nets.forEach(function (net) {
      var selected = (net.id === STATE.selectedNetId);
      buildNetFabric(net, selected).forEach(function (line) {
        STATE.canvas.add(line);
      });
    });

    // Symbols
    STATE.graph.instances.forEach(function (inst) {
      var g = buildInstanceFabric(inst);
      if (g) STATE.canvas.add(g);
    });

    // Junctions
    computeJunctions().forEach(function (j) {
      STATE.canvas.add(buildJunctionFabric(j));
    });

    // Pin hotspots — only when the Net tool is active.
    if (STATE.activeTool === 'net') {
      STATE.graph.instances.forEach(function (inst) {
        var symbolDef = symbolDefById(inst.symbolId);
        if (!symbolDef) return;
        symbolDef.pins.forEach(function (pin) {
          STATE.canvas.add(buildPinHotspot(inst, pin));
        });
      });
    }

    // Re-attach net preview overlay if drawing.
    if (STATE.netPreviewObj) {
      STATE.canvas.add(STATE.netPreviewObj);
    }

    STATE.canvas.requestRenderAll();
    renderConnectionsTable();
    renderProps();
    updateSymbolHud();
  }

  // ====================================================================
  // 11. CONNECTIONS TABLE
  // ====================================================================

  function refLabel(endpoint) {
    var inst = findInstance(endpoint.instanceId);
    if (!inst) return '?';
    var pin = findPin(inst, endpoint.pinNumber);
    var pinName = pin ? (pin.name || pin.number) : endpoint.pinNumber;
    return (inst.refDes || '?') + '.' + pinName;
  }

  // Endpoint-pair listing: one row per *unique* pair of endpoints
  // within a net. For a 2-endpoint net that's 1 row. For a 5-endpoint
  // net that's 10 rows — fine for typical Pico-class schematics. If
  // this gets too noisy in practice we can switch to the simpler
  // "one row per endpoint" listing documented in the wrap-up.
  function deriveConnections() {
    var rows = [];
    STATE.graph.nets.forEach(function (net) {
      var eps = net.endpoints;
      if (eps.length === 0) return;
      if (eps.length === 1) {
        rows.push({
          netId: net.id,
          netName: net.name,
          from: refLabel(eps[0]),
          to: '—',
          description: net.description || '',
        });
        return;
      }
      for (var i = 0; i < eps.length; i++) {
        for (var j = i + 1; j < eps.length; j++) {
          rows.push({
            netId: net.id,
            netName: net.name,
            from: refLabel(eps[i]),
            to:   refLabel(eps[j]),
            description: net.description || '',
          });
        }
      }
    });
    return rows;
  }

  // ====================================================================
  // 10b. PROPERTIES PANEL (right pane, above the connections table)
  // ====================================================================

  function renderProps() {
    if (!dom.propsBody) return;
    var inst = findInstance(STATE.selectedInstanceId);
    if (!inst) {
      dom.propsBody.innerHTML =
        '<div class="se-props-empty text-muted small">' +
        'Select a component on the canvas to see its properties.' +
        '</div>';
      return;
    }
    var symbolDef = symbolDefById(inst.symbolId);
    var symbolTypeName = symbolDef ? symbolDef.name : '(unknown)';
    var refDes = inst.refDes || '';
    var label = inst.label || '';
    dom.propsBody.innerHTML =
      '<div class="se-props-field">' +
      '  <span class="se-props-field-label">Symbol type</span>' +
      '  <span class="se-props-field-value">' + escapeHtml(symbolTypeName) + '</span>' +
      '</div>' +
      '<div class="se-props-field">' +
      '  <label class="se-props-field-label" for="se-props-refdes">Reference</label>' +
      '  <input class="se-props-input" id="se-props-refdes" type="text"' +
      '         value="' + escapeHtml(refDes) + '"' +
      '         placeholder="U1" maxlength="20"' +
      '         data-instance-id="' + inst.id + '">' +
      '</div>' +
      '<div class="se-props-field">' +
      '  <label class="se-props-field-label" for="se-props-label">Friendly name</label>' +
      '  <input class="se-props-input" id="se-props-label" type="text"' +
      '         value="' + escapeHtml(label) + '"' +
      '         placeholder="e.g. Main controller" maxlength="60"' +
      '         data-instance-id="' + inst.id + '">' +
      '</div>';

    var refInput = document.getElementById('se-props-refdes');
    var labelInput = document.getElementById('se-props-label');
    if (refInput) {
      refInput.addEventListener('change', function () { onPropsRefDesChange(inst.id, refInput.value); });
      refInput.addEventListener('blur',   function () { onPropsRefDesChange(inst.id, refInput.value); });
      refInput.addEventListener('keydown', function (e) {
        if (e.key === 'Enter') { e.preventDefault(); refInput.blur(); }
      });
    }
    if (labelInput) {
      labelInput.addEventListener('change', function () { onPropsLabelChange(inst.id, labelInput.value); });
      labelInput.addEventListener('blur',   function () { onPropsLabelChange(inst.id, labelInput.value); });
      labelInput.addEventListener('keydown', function (e) {
        if (e.key === 'Enter') { e.preventDefault(); labelInput.blur(); }
      });
    }
  }

  function onPropsRefDesChange(instanceId, value) {
    var inst = findInstance(instanceId);
    if (!inst) return;
    var next = (value || '').trim();
    if (next === (inst.refDes || '')) return;
    inst.refDes = next;
    renderGraph();
    scheduleAutosave();
  }

  function onPropsLabelChange(instanceId, value) {
    var inst = findInstance(instanceId);
    if (!inst) return;
    var next = (value || '').trim();
    if (next === (inst.label || '')) return;
    inst.label = next;
    renderGraph();
    scheduleAutosave();
  }

  // ====================================================================
  // 11. CONNECTIONS TABLE
  // ====================================================================

  function renderConnectionsTable() {
    if (!dom.tableBody) return;
    var rows = deriveConnections();
    if (dom.tableCount) {
      dom.tableCount.textContent = rows.length
        ? '(' + rows.length + ')' : '';
    }
    if (rows.length === 0) {
      dom.tableBody.innerHTML =
        '<tr class="se-table-empty"><td colspan="4" class="text-muted text-center py-3">' +
        'No connections yet &mdash; draw a wire between two pins to start.' +
        '</td></tr>';
      return;
    }
    var html = '';
    rows.forEach(function (r, idx) {
      var sel = (r.netId === STATE.selectedNetId) ? ' is-selected' : '';
      html +=
        '<tr class="se-table-row' + sel + '" data-net-id="' + escapeHtml(r.netId) + '">' +
          '<td class="se-td-net">' + escapeHtml(r.netName) + '</td>' +
          '<td class="se-td-from">' + escapeHtml(r.from) + '</td>' +
          '<td class="se-td-to">' + escapeHtml(r.to) + '</td>' +
          '<td class="se-td-desc">' +
            '<input type="text" class="form-control form-control-sm se-desc-input" ' +
                   'data-net-id="' + escapeHtml(r.netId) + '" ' +
                   'value="' + escapeHtml(r.description) + '" ' +
                   'placeholder="(none)" maxlength="500">' +
          '</td>' +
        '</tr>';
    });
    dom.tableBody.innerHTML = html;
  }

  function onTableRowClick(ev) {
    var input = ev.target.closest('.se-desc-input');
    if (input) return;       // let the input handle its own focus / clicks
    var row = ev.target.closest('.se-table-row');
    if (!row) return;
    var netId = row.dataset.netId;
    if (!netId) return;
    selectNet(netId, { centreOnCanvas: true });
  }

  function onTableInputChange(ev) {
    var input = ev.target.closest('.se-desc-input');
    if (!input) return;
    var netId = input.dataset.netId;
    if (!netId) return;
    var net = STATE.graph.nets.find(function (n) { return n.id === netId; });
    if (!net) return;
    if (net.description === input.value) return;
    net.description = input.value;
    scheduleAutosave();
    // Don't re-render the whole table — that would steal focus mid-edit.
    // The other rows reading the same net will pick up the new value on
    // the next coarse re-render (next save / canvas update).
  }

  // ====================================================================
  // 12. SELECTION + HIGHLIGHT
  // ====================================================================

  function selectNet(netId, opts) {
    opts = opts || {};
    STATE.selectedNetId = netId;
    STATE.selectedInstanceId = null;
    renderGraph();
    // Scroll the connections table to the first matching row.
    if (dom.tableBody) {
      var row = dom.tableBody.querySelector('.se-table-row.is-selected');
      if (row && row.scrollIntoView) {
        row.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
      }
    }
    if (opts.centreOnCanvas) {
      var net = STATE.graph.nets.find(function (n) { return n.id === netId; });
      if (net && net.segments.length > 0) {
        // No "pan-to-point" in our minimal canvas (fit-to-screen) —
        // the visual highlight already provides the cue.
      }
    }
  }

  function clearSelection() {
    STATE.selectedInstanceId = null;
    STATE.selectedNetId = null;
    renderGraph();
  }

  // ====================================================================
  // 13. AUTOSAVE
  // ====================================================================

  function serialiseGraph() {
    // The schematic id is server-side; everything else is the document.
    return JSON.stringify({
      instances: STATE.graph.instances,
      nets: STATE.graph.nets,
      labels: STATE.graph.labels,
    });
  }

  function scheduleAutosave() {
    if (STATE.autosaveTimer) clearTimeout(STATE.autosaveTimer);
    setSaveStatus('saving');
    STATE.autosaveTimer = setTimeout(function () {
      STATE.autosaveTimer = null;
      doAutosave();
    }, AUTOSAVE_MS);
  }

  async function doAutosave() {
    if (!STATE.schematic) return;
    try {
      var resp = await apiFetchWithTermsRetry(
        API + '/api/projects/' + STATE.projectId + '/schematic',
        {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ schematic_data: serialiseGraph() }),
        }
      );
      if (!resp.ok) throw new Error('PUT schematic HTTP ' + resp.status);
      setSaveStatus('saved');
    } catch (e) {
      setSaveStatus('error', 'Save failed');
    }
  }

  // ====================================================================
  // 14. SYMBOL HUD (rotate / flip / delete above selection)
  // ====================================================================

  function updateSymbolHud() {
    if (!dom.symbolHud) return;
    if (!STATE.selectedInstanceId) {
      dom.symbolHud.hidden = true;
      return;
    }
    var inst = findInstance(STATE.selectedInstanceId);
    if (!inst) { dom.symbolHud.hidden = true; return; }
    var symbolDef = symbolDefById(inst.symbolId);
    if (!symbolDef) { dom.symbolHud.hidden = true; return; }
    dom.symbolHud.hidden = false;
    // Position above the symbol. The HUD is positioned absolute inside
    // .se-canvas-wrap. Convert scene → CSS by running the scene point
    // through Fabric's viewport transform (which gives canvas-element
    // pixel coords), then add the canvas element's offset within the
    // wrap (it's centered in the flex layout, plus padding).
    var vt = STATE.canvas.viewportTransform;
    var canvasPxX = inst.x * vt[0] + vt[4];
    var canvasPxY = (inst.y - symbolHeight(symbolDef, inst.rotation) / 2) * vt[3] + vt[5];
    var canvasEl = STATE.canvas.lowerCanvasEl;
    var canvasRect = canvasEl.getBoundingClientRect();
    var wrapRect = dom.canvasWrap.getBoundingClientRect();
    var offsetX = canvasRect.left - wrapRect.left;
    var offsetY = canvasRect.top - wrapRect.top;
    dom.symbolHud.style.left = (offsetX + canvasPxX) + 'px';
    dom.symbolHud.style.top = (offsetY + canvasPxY - 36) + 'px';
  }

  function rotateSelected(deltaDeg) {
    var inst = findInstance(STATE.selectedInstanceId);
    if (!inst) return;
    inst.rotation = (((inst.rotation || 0) + deltaDeg) % 360 + 360) % 360;
    rerouteNetsFor(inst.id);
    renderGraph();
    scheduleAutosave();
  }

  function flipSelected(axis) {
    var inst = findInstance(STATE.selectedInstanceId);
    if (!inst) return;
    if (axis === 'h') inst.flipH = !inst.flipH;
    if (axis === 'v') inst.flipV = !inst.flipV;
    rerouteNetsFor(inst.id);
    renderGraph();
    scheduleAutosave();
  }

  function deleteSelected() {
    if (STATE.selectedInstanceId) {
      var id = STATE.selectedInstanceId;
      STATE.graph.instances = STATE.graph.instances.filter(function (i) {
        return i.id !== id;
      });
      // Strip net endpoints that referenced this instance. Nets that
      // become endpoint-less are removed entirely.
      STATE.graph.nets = STATE.graph.nets.map(function (n) {
        n.endpoints = n.endpoints.filter(function (ep) {
          return ep.instanceId !== id;
        });
        return n;
      }).filter(function (n) {
        return n.endpoints.length > 0 || n.segments.length > 0;
      });
      STATE.selectedInstanceId = null;
    } else if (STATE.selectedNetId) {
      STATE.graph.nets = STATE.graph.nets.filter(function (n) {
        return n.id !== STATE.selectedNetId;
      });
      STATE.selectedNetId = null;
    }
    renderGraph();
    scheduleAutosave();
  }

  // When a symbol moves or rotates, the pin scene-positions shift —
  // the stored segments don't auto-follow. For v1 we rebuild each
  // affected net's segments from scratch via the recorded endpoints,
  // re-running the L-bend router for each pair. Imperfect (we lose any
  // hand-edited bends the user might have made) but the v1 editor
  // doesn't expose hand-editing, so the loss is hypothetical.
  function rerouteNetsFor(instanceId) {
    STATE.graph.nets.forEach(function (net) {
      var touchesThis = net.endpoints.some(function (ep) {
        return ep.instanceId === instanceId;
      });
      if (!touchesThis) return;
      // Re-route as a fan from endpoints[0] to the others. Topology is
      // preserved (same endpoints, same net id); only the segment
      // geometry refreshes.
      if (net.endpoints.length < 2) { net.segments = []; return; }
      var newSegs = [];
      var anchor = net.endpoints[0];
      var anchorInst = findInstance(anchor.instanceId);
      var anchorPin = anchorInst && findPin(anchorInst, anchor.pinNumber);
      if (!anchorInst || !anchorPin) { net.segments = []; return; }
      var anchorXY = pinScenePos(anchorInst, anchorPin);
      for (var i = 1; i < net.endpoints.length; i++) {
        var ep = net.endpoints[i];
        var inst = findInstance(ep.instanceId);
        var pin = inst && findPin(inst, ep.pinNumber);
        if (!inst || !pin) continue;
        var xy = pinScenePos(inst, pin);
        lShapedSegments(anchorXY.x, anchorXY.y, xy.x, xy.y).forEach(function (s) {
          newSegs.push(s);
        });
      }
      net.segments = newSegs;
    });
  }

  // ====================================================================
  // 15. CANVAS EVENTS
  // ====================================================================

  function pointerScene(opt) {
    var p = STATE.canvas.getPointer(opt.e);
    return { x: p.x, y: p.y };
  }

  function onCanvasMouseDown(opt) {
    var obj = opt.target;
    var p = pointerScene(opt);

    // ----- Place-symbol mode -----
    if (STATE.activeTool === 'symbol' && STATE.pendingSymbolId) {
      placeSymbolAt(STATE.pendingSymbolId, p.x, p.y);
      return;
    }
    if (STATE.activeTool === 'gnd') {
      placeSymbolAt('gnd', p.x, p.y);
      setActiveTool('select');
      return;
    }
    if (STATE.activeTool === 'vplus') {
      placeSymbolAt('vplus', p.x, p.y);
      setActiveTool('select');
      return;
    }

    // ----- Net-drawing mode -----
    if (STATE.activeTool === 'net') {
      if (obj && obj.data && obj.data.kind === 'pin-hotspot') {
        var ep = {
          instanceId: obj.data.instanceId,
          pinNumber: obj.data.pinNumber,
        };
        if (!STATE.netDrawingFrom) {
          STATE.netDrawingFrom = {
            instanceId: ep.instanceId,
            pinNumber: ep.pinNumber,
            x: obj.data.x,
            y: obj.data.y,
          };
        } else {
          // Commit the net.
          var fromEp = {
            instanceId: STATE.netDrawingFrom.instanceId,
            pinNumber: STATE.netDrawingFrom.pinNumber,
          };
          var net = commitNetSegment(fromEp, ep);
          STATE.netDrawingFrom = null;
          if (STATE.netPreviewObj) {
            STATE.canvas.remove(STATE.netPreviewObj);
            STATE.netPreviewObj = null;
          }
          renderGraph();
          if (net) selectNet(net.id);
          scheduleAutosave();
        }
        return;
      }
      // Click empty space mid-draw: abort.
      if (STATE.netDrawingFrom) {
        STATE.netDrawingFrom = null;
        if (STATE.netPreviewObj) {
          STATE.canvas.remove(STATE.netPreviewObj);
          STATE.netPreviewObj = null;
          STATE.canvas.requestRenderAll();
        }
      }
      return;
    }

    // ----- Selection (default) -----
    if (obj && obj.data) {
      if (obj.data.kind === 'instance') {
        var hadNet = STATE.selectedNetId;
        STATE.selectedInstanceId = obj.data.instanceId;
        STATE.selectedNetId = null;
        // Don't re-render the whole graph on instance selection — that
        // would wipe `obj` (the very group the user just mouse-down'd
        // on) and replace it with a fresh group, killing Fabric's drag
        // initiation mid-click. The instance's red border already
        // comes from Fabric's selection state. Only re-render if a
        // *net* was previously selected, so its highlight clears.
        if (hadNet) {
          renderGraph();
        } else {
          renderConnectionsTable();
          renderProps();
          updateSymbolHud();
        }
      } else if (obj.data.kind === 'net') {
        STATE.selectedNetId = obj.data.netId;
        STATE.selectedInstanceId = null;
        renderGraph();
      } else {
        clearSelection();
      }
      return;
    }
    clearSelection();
  }

  function onCanvasMouseMove(opt) {
    if (STATE.activeTool !== 'net' || !STATE.netDrawingFrom) return;
    var p = pointerScene(opt);
    var from = STATE.netDrawingFrom;
    var segs = lShapedSegments(from.x, from.y, p.x, p.y);
    if (STATE.netPreviewObj) {
      STATE.canvas.remove(STATE.netPreviewObj);
      STATE.netPreviewObj = null;
    }
    if (segs.length === 0) { STATE.canvas.requestRenderAll(); return; }
    var pts = [{ x: segs[0].x1, y: segs[0].y1 }];
    segs.forEach(function (s) { pts.push({ x: s.x2, y: s.y2 }); });
    STATE.netPreviewObj = new fabric.Polyline(pts, {
      stroke: BRAND_RED,
      strokeWidth: NET_STROKE,
      fill: '',
      strokeDashArray: [4, 4],
      selectable: false,
      evented: false,
    });
    STATE.canvas.add(STATE.netPreviewObj);
    STATE.canvas.requestRenderAll();
  }

  function onCanvasObjectMoving(opt) {
    var obj = opt.target;
    if (!obj || !obj.data || obj.data.kind !== 'instance') return;
    var inst = findInstance(obj.data.instanceId);
    if (!inst) return;
    // Snap the moving group to the grid as it drags.
    var snappedLeft = snap(obj.left);
    var snappedTop = snap(obj.top);
    obj.set({ left: snappedLeft, top: snappedTop });
    inst.x = snappedLeft;
    inst.y = snappedTop;
    rerouteNetsFor(inst.id);
    updateSymbolHud();
  }

  function onCanvasObjectModified(opt) {
    var obj = opt.target;
    if (!obj || !obj.data || obj.data.kind !== 'instance') return;
    var inst = findInstance(obj.data.instanceId);
    if (!inst) return;
    inst.x = obj.left;
    inst.y = obj.top;
    rerouteNetsFor(inst.id);
    // Defer renderGraph one tick. renderGraph wipes the canvas and
    // re-adds objects; if we do that synchronously inside the
    // mouse:up → object:modified pipeline, Fabric's drag-cleanup
    // (which runs AFTER our handler) still holds a reference to the
    // now-deleted Fabric group as the active "transformed" target,
    // and the next mousemove keeps "dragging" the ghost — the symbol
    // ends up velcroed to the cursor. Letting Fabric finish first
    // and re-rendering on the next tick fixes it.
    requestAnimationFrame(function () { renderGraph(); });
    scheduleAutosave();
  }

  // ====================================================================
  // 16. TOOLS PANE
  // ====================================================================

  function setActiveTool(tool) {
    STATE.activeTool = tool;
    // When entering net mode, clear any partial draw.
    if (tool !== 'net' && STATE.netDrawingFrom) {
      STATE.netDrawingFrom = null;
      if (STATE.netPreviewObj) {
        STATE.canvas.remove(STATE.netPreviewObj);
        STATE.netPreviewObj = null;
      }
    }
    // The Rotate tool is a one-shot — apply to the selection then revert.
    if (tool === 'rotate') {
      rotateSelected(90);
      STATE.activeTool = 'select';
      tool = 'select';
    }
    Array.prototype.forEach.call(
      document.querySelectorAll('.se-tool-btn'),
      function (btn) {
        btn.classList.toggle('is-active', btn.dataset.tool === tool);
      }
    );
    if (tool !== 'symbol') {
      STATE.pendingSymbolId = null;
      Array.prototype.forEach.call(
        document.querySelectorAll('.se-symbol-item'),
        function (li) { li.classList.remove('is-pending'); }
      );
    }
    renderGraph();
  }

  function renderSymbolList() {
    if (!dom.symbolList) return;
    var html = '';
    SYMBOL_LIBRARY.forEach(function (sym) {
      var customBadge = sym.isCustom
        ? '<span class="se-symbol-custom-badge" title="Designed in the Symbol Designer">Custom</span>'
        : '';
      html +=
        '<li class="se-symbol-item' + (sym.isCustom ? ' is-custom' : '') +
            '" data-symbol-id="' + escapeHtml(sym.id) + '" ' +
            'role="option" tabindex="0">' +
          '<span class="se-symbol-thumb">' +
            (sym.isCustom
              ? '<i class="fas fa-shapes"></i>'
              : '<i class="fas fa-microchip"></i>') +
          '</span>' +
          '<span class="se-symbol-meta">' +
            '<span class="se-symbol-name">' + escapeHtml(sym.name) +
              customBadge + '</span>' +
            '<span class="se-symbol-sub small text-muted">' +
              escapeHtml(sym.refDesPrefix) + ' &middot; ' +
              sym.pins.length + ' pins' +
            '</span>' +
          '</span>' +
        '</li>';
    });
    dom.symbolList.innerHTML = html;
  }

  // ------------------------------------------------------------------
  // Symbol Designer integration
  // ------------------------------------------------------------------
  //
  // The schematic editor ships with the SYMBOL_LIBRARY array at the top
  // of this file (Pico + R + C + LED + GND + V+ + generic IC). The
  // Symbol Designer page (/projects/symbol/edit.html) grows that
  // library project-by-project: each row in /api/projects/{id}/symbols
  // is converted into a stub-shaped def via symbolDefFromCustom() and
  // appended here. Failures are silent (the editor still works with
  // just the built-in symbols).

  async function loadCustomSymbolsIntoLibrary() {
    try {
      var r = await apiFetch(API + '/api/projects/' + STATE.projectId + '/symbols');
      if (!r.ok) return;
      var rows = await r.json();
      if (!Array.isArray(rows)) return;
      rows.forEach(function (row) {
        var def = symbolDefFromCustom(row);
        if (def) SYMBOL_LIBRARY.push(def);
      });
    } catch (_) { /* silent — built-in library still works */ }
  }

  function onSymbolItemClick(ev) {
    var li = ev.target.closest('.se-symbol-item');
    if (!li) return;
    var symId = li.dataset.symbolId;
    if (!symId) return;
    STATE.pendingSymbolId = symId;
    setActiveTool('symbol');
    Array.prototype.forEach.call(
      document.querySelectorAll('.se-symbol-item'),
      function (other) { other.classList.toggle('is-pending', other === li); }
    );
  }

  // ====================================================================
  // 17. EXPORTS
  // ====================================================================

  function downloadBlob(blob, filename) {
    var url = URL.createObjectURL(blob);
    var a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    setTimeout(function () { URL.revokeObjectURL(url); }, 200);
  }

  function exportCSV() {
    var rows = deriveConnections();
    var lines = ['Net,From,To,Description'];
    rows.forEach(function (r) {
      lines.push([
        csvCell(r.netName),
        csvCell(r.from),
        csvCell(r.to),
        csvCell(r.description),
      ].join(','));
    });
    var blob = new Blob([lines.join('\n') + '\n'], { type: 'text/csv;charset=utf-8' });
    downloadBlob(blob, 'schematic-connections.csv');
  }
  function csvCell(v) {
    v = String(v == null ? '' : v);
    if (/[",\n]/.test(v)) return '"' + v.replace(/"/g, '""') + '"';
    return v;
  }

  function exportPNG() {
    if (!STATE.canvas) return;
    // Use Fabric's toDataURL with 2x supersampling for a crisp export.
    var dataUrl = STATE.canvas.toDataURL({
      format: 'png',
      multiplier: 2,
      enableRetinaScaling: false,
    });
    var a = document.createElement('a');
    a.href = dataUrl;
    a.download = 'schematic.png';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
  }

  // ====================================================================
  // 18. SYMBOL-HUD WIRING
  // ====================================================================

  function wireSymbolHud() {
    if (!dom.symbolHud) return;
    dom.symbolHud.addEventListener('click', function (ev) {
      var btn = ev.target.closest('[data-hud-action]');
      if (!btn) return;
      switch (btn.dataset.hudAction) {
        case 'rotate-ccw': rotateSelected(-90); break;
        case 'rotate-cw':  rotateSelected(90);  break;
        case 'flip-h':     flipSelected('h');   break;
        case 'flip-v':     flipSelected('v');   break;
        case 'delete':     deleteSelected();    break;
      }
    });
  }

  // ====================================================================
  // 19. CANVAS INIT
  // ====================================================================

  function initCanvas() {
    if (typeof fabric === 'undefined' || !fabric.Canvas) {
      throw new Error('Fabric.js failed to load');
    }
    STATE.canvas = new fabric.Canvas(dom.canvasEl, {
      width: SCENE_W,
      height: SCENE_H,
      backgroundColor: BG,
      preserveObjectStacking: true,
      selection: false,
      // Make the canvas a defined size relative to the container.
    });

    var c = STATE.canvas;
    c.on('mouse:down', onCanvasMouseDown);
    c.on('mouse:move', onCanvasMouseMove);
    c.on('object:moving',   onCanvasObjectMoving);
    c.on('object:modified', onCanvasObjectModified);

    syncCanvasDisplaySize();
    window.addEventListener('resize', syncCanvasDisplaySize);

    // Esc aborts a net-in-progress.
    document.addEventListener('keydown', function (e) {
      if (e.key === 'Escape') {
        if (STATE.netDrawingFrom) {
          STATE.netDrawingFrom = null;
          if (STATE.netPreviewObj) {
            STATE.canvas.remove(STATE.netPreviewObj);
            STATE.netPreviewObj = null;
            STATE.canvas.requestRenderAll();
          }
        } else {
          clearSelection();
        }
      } else if (e.key === 'Delete' || e.key === 'Backspace') {
        if (document.activeElement && /input|textarea/i.test(document.activeElement.tagName)) {
          return;
        }
        if (STATE.selectedInstanceId || STATE.selectedNetId) {
          e.preventDefault();
          deleteSelected();
        }
      }
    });
  }

  // Re-fit the canvas to the wrap. Strategy:
  //   1. Set the Fabric canvas's internal AND CSS dimensions to the
  //      wrap's pixel size (no cssOnly trick). Internal = CSS = wrap.
  //   2. Compute the auto-fit scale that lands the whole scene inside
  //      the canvas while preserving aspect ratio.
  //   3. Apply effectiveScale = fitScale * userZoom to the viewport
  //      transform, centring the scene + applying any pan offset.
  // The browser does no extra scaling — what Fabric renders is what
  // you see. User zoom buttons / Ctrl+wheel adjust STATE.userZoom and
  // call applyViewport().
  function syncCanvasDisplaySize() {
    if (!STATE.canvas || !dom.canvasWrap) return;
    var rect = dom.canvasWrap.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    var w = Math.max(1, Math.floor(rect.width));
    var h = Math.max(1, Math.floor(rect.height));
    STATE.canvas.setDimensions({ width: w, height: h });
    applyViewport();
  }

  function fitScale() {
    if (!STATE.canvas) return 1;
    var w = STATE.canvas.getWidth();
    var h = STATE.canvas.getHeight();
    var s = Math.min(w / SCENE_W, h / SCENE_H);
    return s > 0 ? s : 1;
  }

  function effectiveScale() {
    return fitScale() * (STATE.userZoom || 1);
  }

  function applyViewport() {
    if (!STATE.canvas) return;
    var w = STATE.canvas.getWidth();
    var h = STATE.canvas.getHeight();
    var s = effectiveScale();
    // Centre the scene at its midpoint, plus the user's pan offset.
    var tx = (w - SCENE_W * s) / 2 + STATE.panX;
    var ty = (h - SCENE_H * s) / 2 + STATE.panY;
    STATE.canvas.setViewportTransform([s, 0, 0, s, tx, ty]);
    updateSymbolHud();
  }

  function zoomBy(factor, focal) {
    // Multiplicative zoom around an optional focal point (CSS coords
    // relative to the canvas top-left). The focal point stays under
    // the cursor across the zoom step.
    var prev = STATE.userZoom || 1;
    var next = Math.max(0.2, Math.min(6, prev * factor));
    if (next === prev) return;
    if (focal && STATE.canvas) {
      var vt = STATE.canvas.viewportTransform;
      var sceneX = (focal.x - vt[4]) / vt[0];
      var sceneY = (focal.y - vt[5]) / vt[3];
      STATE.userZoom = next;
      var s = effectiveScale();
      var w = STATE.canvas.getWidth();
      var h = STATE.canvas.getHeight();
      // Solve for pan that keeps (sceneX, sceneY) at the same screen
      // (focal.x, focal.y):
      //   focal.x = sceneX * s + (w - SCENE_W * s)/2 + panX
      STATE.panX = focal.x - sceneX * s - (w - SCENE_W * s) / 2;
      STATE.panY = focal.y - sceneY * s - (h - SCENE_H * s) / 2;
    } else {
      STATE.userZoom = next;
    }
    applyViewport();
  }

  function resetZoom() {
    STATE.userZoom = 1;
    STATE.panX = 0;
    STATE.panY = 0;
    applyViewport();
  }

  // Match the instruction builder's runtime workspace sizing — the CSS
  // fallback can't know the real chrome height, so measure and lock to
  // viewport.
  function adjustSchematicWorkspaceHeight() {
    var ws = dom.workspace || document.getElementById('se-workspace');
    if (!ws) return;
    var top = ws.getBoundingClientRect().top;
    var avail = Math.max(320, window.innerHeight - top);
    ws.style.height = avail + 'px';
    // The wrap inside took its size from the workspace flex layout;
    // re-sync the Fabric viewport so the canvas matches the new wrap.
    syncCanvasDisplaySize();
  }

  // ====================================================================
  // 20. BOOTSTRAP — fetch project, schematic, render
  // ====================================================================

  async function fetchMe() {
    try {
      var r = await apiFetch(API + '/api/auth/me');
      if (!r.ok) return null;
      return await r.json();
    } catch (_) { return null; }
  }
  async function fetchProject() {
    var r = await apiFetch(API + '/api/projects/' + STATE.projectId);
    if (!r.ok) {
      var err = new Error('Project HTTP ' + r.status);
      err.status = r.status;
      throw err;
    }
    return r.json();
  }
  async function fetchSchematic() {
    var r = await apiFetch(API + '/api/projects/' + STATE.projectId + '/schematic');
    if (r.status === 404) return null;
    if (!r.ok) throw new Error('Schematic HTTP ' + r.status);
    return r.json();
  }
  async function ensureSchematic() {
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + STATE.projectId + '/schematic',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({}),
      }
    );
    if (!resp.ok) throw new Error('Create schematic HTTP ' + resp.status);
    return resp.json();
  }

  function loadGraphFromSchematic(schematic) {
    var doc = emptyGraph();
    if (schematic && schematic.schematic_data) {
      try {
        var parsed = JSON.parse(schematic.schematic_data);
        if (parsed && typeof parsed === 'object') {
          if (Array.isArray(parsed.instances)) doc.instances = parsed.instances;
          if (Array.isArray(parsed.nets))      doc.nets = parsed.nets;
          if (Array.isArray(parsed.labels))    doc.labels = parsed.labels;
        }
      } catch (_) {
        // Corrupt blob — start fresh and let the next save overwrite it.
      }
    }
    if (schematic) {
      doc.id = schematic.id;
      doc.name = schematic.name || '';
      doc.description = schematic.description || '';
    }
    STATE.graph = doc;
    rebuildRefDesCounters();
    rebuildNetCounter();
  }

  function cacheDom() {
    dom.workspace      = document.getElementById('se-workspace');
    dom.titlebar       = document.getElementById('se-titlebar');
    dom.backLink       = document.getElementById('se-back-link');
    dom.projectTitle   = document.getElementById('se-project-title');
    dom.saveStatus     = document.getElementById('se-save-status');
    dom.exportCsvBtn   = document.getElementById('se-export-csv');
    dom.exportPngBtn   = document.getElementById('se-export-png');
    dom.zoomIn         = document.getElementById('se-zoom-in');
    dom.zoomOut        = document.getElementById('se-zoom-out');
    dom.zoomReset      = document.getElementById('se-zoom-reset');
    dom.zoomPct        = document.getElementById('se-zoom-pct');
    dom.tableToggle    = document.getElementById('se-table-toggle');
    dom.propsSection   = document.getElementById('se-props-section');
    dom.propsBody      = document.getElementById('se-props-body');
    dom.loading        = document.getElementById('se-loading');
    dom.notOwner       = document.getElementById('se-not-owner');
    dom.error          = document.getElementById('se-error');
    dom.errorDetail    = document.getElementById('se-error-detail');
    dom.main           = document.getElementById('se-main');
    dom.tools          = document.getElementById('se-tools');
    dom.symbolList     = document.getElementById('se-symbol-list');
    dom.openSymDesigner = document.getElementById('se-open-symbol-designer');
    dom.canvasArea     = document.getElementById('se-canvas-area');
    dom.canvasWrap     = document.getElementById('se-canvas-wrap');
    dom.canvasEl       = document.getElementById('se-canvas');
    dom.symbolHud      = document.getElementById('se-symbol-hud');
    dom.tableWrap      = document.getElementById('se-table-wrap');
    dom.tableCount     = document.getElementById('se-table-count');
    dom.tableBody      = document.getElementById('se-table-body');
  }

  function wireUi() {
    // Tool buttons
    Array.prototype.forEach.call(
      document.querySelectorAll('.se-tool-btn'),
      function (btn) {
        btn.addEventListener('click', function () {
          setActiveTool(btn.dataset.tool);
        });
      }
    );

    // Symbol list
    if (dom.symbolList) {
      dom.symbolList.addEventListener('click', onSymbolItemClick);
    }

    // Table interactions (delegated)
    if (dom.tableBody) {
      dom.tableBody.addEventListener('click', onTableRowClick);
      dom.tableBody.addEventListener('change', onTableInputChange);
    }

    // Exports
    if (dom.exportCsvBtn) dom.exportCsvBtn.addEventListener('click', exportCSV);
    if (dom.exportPngBtn) dom.exportPngBtn.addEventListener('click', exportPNG);

    // Zoom controls
    if (dom.zoomIn)    dom.zoomIn.addEventListener('click', function () { zoomBy(1.25, null); updateZoomPctLabel(); });
    if (dom.zoomOut)   dom.zoomOut.addEventListener('click', function () { zoomBy(1 / 1.25, null); updateZoomPctLabel(); });
    if (dom.zoomReset) dom.zoomReset.addEventListener('click', function () { resetZoom(); updateZoomPctLabel(); });
    // Ctrl/Cmd + wheel zoom — focal point follows the cursor so what's
    // under the mouse stays under the mouse across the zoom step.
    if (dom.canvasWrap) {
      dom.canvasWrap.addEventListener('wheel', function (e) {
        if (!(e.ctrlKey || e.metaKey)) return;
        e.preventDefault();
        var rect = STATE.canvas.lowerCanvasEl.getBoundingClientRect();
        var focal = { x: e.clientX - rect.left, y: e.clientY - rect.top };
        var factor = e.deltaY < 0 ? 1.1 : 1 / 1.1;
        zoomBy(factor, focal);
        updateZoomPctLabel();
      }, { passive: false });
    }
    // Keyboard shortcuts: Ctrl/Cmd + / - / 0
    document.addEventListener('keydown', function (e) {
      if (!(e.ctrlKey || e.metaKey)) return;
      // Ignore when typing in inputs.
      if (document.activeElement &&
          /input|textarea/i.test(document.activeElement.tagName)) return;
      var k = e.key;
      if (k === '+' || k === '=') { e.preventDefault(); zoomBy(1.25, null); updateZoomPctLabel(); }
      else if (k === '-' || k === '_') { e.preventDefault(); zoomBy(1 / 1.25, null); updateZoomPctLabel(); }
      else if (k === '0') { e.preventDefault(); resetZoom(); updateZoomPctLabel(); }
    });

    // Connections-panel collapse toggle. Persist the collapsed state in
    // localStorage so each user sees the panel they last left.
    var TABLE_COLLAPSE_KEY = 'kr-schematic-table-collapsed';
    if (dom.tableToggle && dom.main) {
      var initiallyCollapsed = false;
      try {
        initiallyCollapsed = localStorage.getItem(TABLE_COLLAPSE_KEY) === '1';
      } catch (_) {}
      if (initiallyCollapsed) dom.main.classList.add('is-table-collapsed');
      updateTableToggleAria();
      dom.tableToggle.addEventListener('click', function () {
        dom.main.classList.toggle('is-table-collapsed');
        var collapsed = dom.main.classList.contains('is-table-collapsed');
        try { localStorage.setItem(TABLE_COLLAPSE_KEY, collapsed ? '1' : '0'); } catch (_) {}
        updateTableToggleAria();
        // Grid column widths changed — re-fit the canvas to fill the
        // new width.
        syncCanvasDisplaySize();
      });
    }

    wireSymbolHud();
  }

  function updateTableToggleAria() {
    if (!dom.tableToggle || !dom.main) return;
    var collapsed = dom.main.classList.contains('is-table-collapsed');
    dom.tableToggle.setAttribute('aria-expanded', collapsed ? 'false' : 'true');
    dom.tableToggle.title = collapsed
      ? 'Expand connections panel'
      : 'Collapse connections panel';
    dom.tableToggle.setAttribute('aria-label', dom.tableToggle.title);
  }

  function updateZoomPctLabel() {
    if (!dom.zoomPct) return;
    dom.zoomPct.textContent = Math.round((STATE.userZoom || 1) * 100) + '%';
  }

  // Embedded-mode postMessage bridge: the instruction builder hides the
  // schematic editor's title bar entirely when embedding the editor in a
  // schematic-type step (?embedded=1). Zoom / export / save-status are
  // proxied through postMessage so the builder's own title bar acts as
  // the single header for all step types.
  function isEmbedded() {
    return document.body.classList.contains('is-embedded');
  }

  function postToParent(payload) {
    if (!isEmbedded() || !window.parent || window.parent === window) return;
    try { window.parent.postMessage(payload, '*'); } catch (_) {}
  }

  function wireParentBridge() {
    if (!isEmbedded()) return;
    window.addEventListener('message', function (e) {
      var msg = e && e.data;
      if (!msg || msg.kr_se_action == null) return;
      switch (msg.kr_se_action) {
        case 'zoom-in':    zoomBy(1.25, null); postZoom(); break;
        case 'zoom-out':   zoomBy(1 / 1.25, null); postZoom(); break;
        case 'zoom-reset': resetZoom(); postZoom(); break;
        case 'export-csv': exportCSV(); break;
        case 'export-png': exportPNG(); break;
      }
    });
    // Initial handshake — let the parent know we're ready + current
    // zoom %. Re-emit on subsequent zoom changes so the parent's
    // percent label stays in sync.
    postZoom();
  }

  function postZoom() {
    postToParent({ kr_se_event: 'zoom', percent: Math.round((STATE.userZoom || 1) * 100) });
  }

  async function init() {
    cacheDom();
    var params = new URLSearchParams(window.location.search);
    STATE.projectId = params.get('id');
    if (!STATE.projectId) {
      if (dom.errorDetail) dom.errorDetail.textContent = 'Missing ?id=… project id in the URL.';
      showOnly(dom.error);
      return;
    }
    if (dom.backLink) dom.backLink.href =
      '/projects/instructions/edit.html?id=' + encodeURIComponent(STATE.projectId);

    try {
      var me = await fetchMe();
      STATE.me = me;
      var project = await fetchProject();
      STATE.project = project;
      STATE.isOwner = !!(me && project && project.author_username === me.username);

      if (dom.projectTitle) {
        dom.projectTitle.textContent =
          'Schematic · ' + (project.title || 'Untitled');
      }

      if (!STATE.isOwner) {
        showOnly(dom.notOwner);
        return;
      }

      // Create the row if it doesn't exist (idempotent POST on the
      // backend). This is also where the B3 step-type-switch handler
      // gets the schematic id from — having the row already in place
      // means the step-link write is just an InstructionStep PUT.
      var schematic = await fetchSchematic();
      if (!schematic) {
        schematic = await ensureSchematic();
      }
      STATE.schematic = schematic;
      loadGraphFromSchematic(schematic);

      // Symbol Designer integration: pull the project's user-designed
      // symbols and append them to SYMBOL_LIBRARY so they show up in
      // the tools-pane symbol list alongside the built-in stubs.
      await loadCustomSymbolsIntoLibrary();

      wireUi();
      initCanvas();
      renderSymbolList();
      setActiveTool('select');
      renderGraph();
      setSaveStatus('saved');
      wireParentBridge();

      // Enable the "+ open Symbol Designer" button now that the page
      // ships (was disabled in the E2 merge).
      if (dom.openSymDesigner) {
        dom.openSymDesigner.disabled = false;
        dom.openSymDesigner.removeAttribute('title');
        dom.openSymDesigner.addEventListener('click', function () {
          window.location.href = '/projects/symbol/edit.html?project_id=' +
            encodeURIComponent(STATE.projectId) + '&new=true';
        });
      }

      showOnly(dom.main);
      // Re-fit after the layout settles. Two-stage:
      //   1. Immediate adjustWorkspaceHeight so the workspace fits the
      //      viewport (it's the wrong size from CSS fallback alone).
      //   2. A microtask later, re-render the graph in case symbols
      //      were placed at scene coords that fell off-screen when the
      //      wrap was at fallback dimensions.
      adjustSchematicWorkspaceHeight();
      window.addEventListener('resize', adjustSchematicWorkspaceHeight);
      if (document.readyState !== 'complete') {
        window.addEventListener('load', adjustSchematicWorkspaceHeight, { once: true });
      }
      requestAnimationFrame(function () {
        adjustSchematicWorkspaceHeight();
        renderGraph();
      });
      setTimeout(function () {
        adjustSchematicWorkspaceHeight();
        renderGraph();
      }, 100);
    } catch (e) {
      if (e && e.status === 404) {
        if (dom.errorDetail) dom.errorDetail.textContent = 'Project not found.';
      }
      showOnly(dom.error);
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
