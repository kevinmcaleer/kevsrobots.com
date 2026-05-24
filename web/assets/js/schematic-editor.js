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
  var INK = '#222222';            // default symbol body + pin labels
  // Nets render in a saturated blue so they're clearly distinguishable
  // from pin name text (which stays INK / near-black). Without this,
  // a net crossing a pin name made the name unreadable. Matches the
  // convention in EAGLE / KiCad schematic exports.
  var NET_INK = '#1d4ed8';
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
  var NET_STROKE = 3;
  var NET_STROKE_SELECTED = 5;

  // Electrical-role overrides for net colour resolution. GND nets
  // are always black, power nets are always brand-red — the
  // auto-palette below DOESN'T include these two so they stay
  // unambiguous in dense schematics where the user is scanning for
  // power rails specifically.
  var NET_INK_GND = '#000000';
  var NET_INK_PWR = BRAND_RED;

  // Auto-assigned colours for signal nets. Every net gets a stable
  // colour from this palette, picked from a hash of net.id so the
  // colours don't shuffle on every reload. Red and black are
  // deliberately absent so the GND / PWR overrides above remain
  // visually distinct from any signal net the auto-palette might
  // assign. Selection state still wins → BRAND_RED is drawn on top.
  var NET_AUTO_PALETTE = [
    '#1d4ed8', // blue
    '#15803d', // green
    '#7c3aed', // purple
    '#d97706', // amber / orange
    '#0e7490', // teal
    '#be185d', // magenta
    '#4338ca', // indigo
    '#0f766e', // dark teal
    '#a16207', // dark amber
    '#86198f', // dark magenta
  ];

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
      category: 'Microcontrollers',
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
      category: 'Microcontrollers',
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
      category: 'Passive',
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
      category: 'Passive',
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
      category: 'Discrete',
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
      category: 'Power',
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
      category: 'Power',
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

    // Convert each pin's centred coord into the editor's local-frame
    // (top-left origin) coords AND a (side, offset) fallback. The
    // designer now allows pins anywhere on the canvas, so we preserve
    // the exact (localX, localY) rather than forcing each pin onto a
    // body edge by side — which was the source of the symbol looking
    // horizontally flipped when pins didn't sit precisely on the
    // computed body bbox.
    //
    // ``side`` is still computed because the stick (and pin label
    // anchor) extend outward in that direction. We prefer the
    // designer's stored ``side``; fall back to picking the closest
    // edge of the bbox to the pin's anchor.
    function sideFromAnchor(p) {
      if (p.side === 'left' || p.side === 'right' ||
          p.side === 'top'  || p.side === 'bottom') return p.side;
      // Pick whichever bbox edge the pin sits closest to.
      var px = p.x || 0, py = p.y || 0;
      var dL = Math.abs(px - minX);
      var dR = Math.abs(maxX - px);
      var dT = Math.abs(py - minY);
      var dB = Math.abs(maxY - py);
      var m = Math.min(dL, dR, dT, dB);
      if (m === dR) return 'right';
      if (m === dL) return 'left';
      if (m === dT) return 'top';
      return 'bottom';
    }
    var convertedPins = pins.map(function (p, i) {
      var px = (typeof p.x === 'number') ? p.x : 0;
      var py = (typeof p.y === 'number') ? p.y : 0;
      var side = sideFromAnchor(p);
      var offset = (side === 'left' || side === 'right')
        ? (py - minY)
        : (px - minX);
      return {
        number: p.number || String(i + 1),
        name: p.name || (p.number || String(i + 1)),
        side: side,
        offset: Math.max(0, Math.round(offset)),
        type: p.type || 'I/O',
        // Body-relative (0,0 = top-left) coords for the renderer.
        // pinLocalPos prefers these when present so designer pins
        // land at the exact spot the user placed them.
        localX: Math.round(px - minX),
        localY: Math.round(py - minY),
      };
    });

    return {
      id: 'custom-' + row.id,
      name: row.name || ('Custom #' + row.id),
      category: 'Custom',
      refDesPrefix: row.ref_des_prefix || 'U',
      bodyWidth: bodyWidth,
      bodyHeight: bodyHeight,
      pins: convertedPins,
      isCustom: true,
      customId: row.id,
      // Pass-through of the symbol row's BOM link so placement code
      // can auto-add the matching BOM item (qty +1) when the user
      // drops the symbol on the schematic.
      bomItemId: row.bom_item_id || null,
      // Optional body shapes the designer drew — kept so the renderer
      // could overlay them later. For v1 the editor uses the bounding
      // box only (mirrors the stub library look).
      _bodyShapes: shapes,
      _minX: minX,
      _minY: minY,
    };
  }

  // Same shape conversion as symbolDefFromCustom, but tagged for the
  // global library (admin-curated, no project_id). The id is
  // ``lib-<id>`` so it never collides with project symbol ids
  // (``custom-<id>``) or built-ins (no prefix). Category comes from
  // the row's curated value rather than being forced to ``Custom``.
  function symbolDefFromLibrary(row) {
    if (!row) return null;
    // Reuse the per-pin / shape geometry calc by faking a custom row;
    // we just override id / category / name at the end.
    var def = symbolDefFromCustom({
      id: row.id,
      name: row.name,
      ref_des_prefix: row.ref_des_prefix,
      symbol_data: row.symbol_data,
      bom_item_id: null, // library symbols don't carry a per-project BOM link
    });
    if (!def) return null;
    def.id = 'lib-' + row.id;
    def.category = row.category || 'Custom';
    def.isLibrary = true;
    def.libraryId = row.id;
    // Keep ``isCustom: true`` so the renderer still treats this as a
    // user-designed symbol (custom body shapes, pin stick length, pin
    // terminator dot). Without this, every render-branch fall-through
    // skipped _bodyShapes / stickLen / terminatorR and library
    // symbols came out as a featureless rounded-corner rectangle on
    // the schematic — even though their geometry was perfectly fine.
    // The library-vs-project-custom distinction lives in ``isLibrary``
    // / ``libraryId``; ``isCustom`` simply means "renders from
    // _bodyShapes" and applies to both.
    delete def.customId;
    return def;
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
  // Custom symbols built by the Symbol Designer pass through the
  // exact ``localX``/``localY`` the user designed (so pins land
  // anywhere — not just on the four body edges). Built-in stub
  // symbols still use the older ``(side, offset)`` form.
  function pinLocalPos(symbolDef, pin) {
    if (typeof pin.localX === 'number' && typeof pin.localY === 'number') {
      return { x: pin.localX, y: pin.localY };
    }
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

  // The pin's wire-attachment point in scene coords — i.e. the outer
  // end of the stick where the visible terminator sits, NOT the body
  // edge. This is what users see and click on, so highlights, hotspots
  // and net endpoints all anchor here. Built-in stub symbols use the
  // same 8 px stick as buildInstanceFabric; custom symbols use 32 px to
  // match the symbol designer.
  function pinScenePos(instance, pin) {
    var symbolDef = symbolDefById(instance.symbolId);
    if (!symbolDef) return { x: instance.x, y: instance.y };
    var local = pinLocalPos(symbolDef, pin);
    var stickLen = symbolDef.isCustom ? 32 : 8;
    var lx = local.x, ly = local.y;
    switch (pin.side) {
      case 'left':   lx -= stickLen; break;
      case 'right':  lx += stickLen; break;
      case 'top':    ly -= stickLen; break;
      case 'bottom': ly += stickLen; break;
    }
    return localToScene(instance, symbolDef, lx, ly);
  }

  // Effective wire-exit direction for a pin. Uses pin.side to pick the
  // axis (horizontal for left/right, vertical for top/bottom), then
  // picks the SIGN by looking at where the pin's wire-attachment sits
  // relative to the body centre. For "reverse" designs (anchor outside
  // the body, terminator at the body edge — like the Pico's left-
  // column pins where pin.side='right' but the body sits to the
  // right of the circle), the wire should exit OPPOSITE to pin.side,
  // not along it.
  //
  // When ``instance`` is provided, the at-rest side is further
  // transformed by the instance's flipH/flipV/rotation so the
  // returned side reflects the pin's actual SCREEN direction after
  // any user-applied rotation or flip. Without this, rotating a GND
  // 180° (so the glyph points up and the pin sits below the body)
  // would still tell the router "wire enters from above going down"
  // and the wire would route to the wrong spot.
  function effectivePinSide(symbolDef, pin, instance) {
    var local = pinLocalPos(symbolDef, pin);
    var stickLen = symbolDef.isCustom ? 32 : 8;
    var lx = local.x, ly = local.y;
    switch (pin.side) {
      case 'left':   lx -= stickLen; break;
      case 'right':  lx += stickLen; break;
      case 'top':    ly -= stickLen; break;
      case 'bottom': ly += stickLen; break;
    }
    // Wire-attachment in body-CENTRED coords (body centre = 0,0).
    var tx = lx - symbolDef.bodyWidth / 2;
    var ty = ly - symbolDef.bodyHeight / 2;
    var horizontal = (pin.side === 'left' || pin.side === 'right');
    var side;
    if (horizontal) side = tx >= 0 ? 'right' : 'left';
    else            side = ty >= 0 ? 'bottom' : 'top';
    if (!instance) return side;
    // Apply flip first, then rotation — same order Fabric uses on the
    // group's render transform.
    if (instance.flipH) {
      if (side === 'left') side = 'right';
      else if (side === 'right') side = 'left';
    }
    if (instance.flipV) {
      if (side === 'top') side = 'bottom';
      else if (side === 'bottom') side = 'top';
    }
    var rot = ((instance.rotation || 0) % 360 + 360) % 360;
    var steps = Math.floor(rot / 90);
    if (steps > 0) {
      // 90° clockwise progression in y-down screen coords:
      // top → right → bottom → left → top
      var cwOrder = ['top', 'right', 'bottom', 'left'];
      var idx = cwOrder.indexOf(side);
      if (idx >= 0) side = cwOrder[(idx + steps) % 4];
    }
    return side;
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
    // Side effect: ensure a matching row in the project BOM. Power
    // glyphs (GND / V+) and wires aren't real components so skip them.
    autoAddSymbolToBom(symbolDef);
    return instance;
  }

  // BOM ids of items already incremented by the auto-add helper this
  // session — used to coalesce rapid placements before the GET-after-
  // POST can pick up the row. Keys are either the BOM row id (after
  // we've POSTed once) or "name:" + lowercase name (before the row
  // exists on the server).
  var _autoBomPending = {};

  function autoAddSymbolToBom(symbolDef) {
    if (!symbolDef) return;
    if (symbolDef.id === 'gnd' || symbolDef.id === 'vplus') return;
    if (!STATE.projectId) return;
    var displayName = (symbolDef.name || '').trim();
    if (!displayName) return;
    // Fire-and-forget — failures shouldn't block the user's placement
    // and the indicator stays "saved" since this is a side effect of
    // the schematic save (which has its own status).
    (async function () {
      try {
        var resp = await apiFetch(
          API + '/api/projects/' + STATE.projectId + '/bom',
          { credentials: 'include' }
        );
        if (!resp.ok) return;
        var rows = await resp.json();
        if (!Array.isArray(rows)) return;
        var lowerName = displayName.toLowerCase();
        // Match priority: explicit bom_item_id link → exact name (case-
        // insensitive). Skip rows with mismatched names just to be
        // safe even when bom_item_id matches.
        var match = null;
        if (symbolDef.bomItemId) {
          for (var i = 0; i < rows.length; i++) {
            if (rows[i].id === symbolDef.bomItemId) { match = rows[i]; break; }
          }
        }
        if (!match) {
          for (var j = 0; j < rows.length; j++) {
            if ((rows[j].name || '').trim().toLowerCase() === lowerName) {
              match = rows[j]; break;
            }
          }
        }
        if (match) {
          // Coalesce: if a placement is already in flight for this
          // row, just inc the local cached qty so back-to-back drops
          // don't lose qty.
          var key = 'id:' + match.id;
          var pendingDelta = (_autoBomPending[key] || 0) + 1;
          _autoBomPending[key] = pendingDelta;
          var nextQty = (match.quantity || 1) + 1;
          try {
            await apiFetch(
              API + '/api/projects/' + STATE.projectId + '/bom/' + match.id,
              {
                method: 'PUT',
                credentials: 'include',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                  name: match.name,
                  quantity: nextQty,
                  unit: match.unit || 'qty',
                  unit_cost: match.unit_cost,
                  currency_code: match.currency_code,
                  supplier_url: match.supplier_url,
                  sort_order: match.sort_order || 0,
                  part_id: match.part_id,
                  supplier_id: match.supplier_id,
                }),
              }
            );
          } finally {
            delete _autoBomPending[key];
          }
        } else {
          var nameKey = 'name:' + lowerName;
          if (_autoBomPending[nameKey]) {
            // Another POST is in flight for the same name — skip to
            // avoid duplicates; that POST will create the row with
            // qty 1 and future placements will hit the existing-row
            // path.
            return;
          }
          _autoBomPending[nameKey] = true;
          try {
            await apiFetch(
              API + '/api/projects/' + STATE.projectId + '/bom',
              {
                method: 'POST',
                credentials: 'include',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                  name: displayName,
                  quantity: 1,
                  unit: 'qty',
                  sort_order: rows.length,
                }),
              }
            );
          } finally {
            delete _autoBomPending[nameKey];
          }
        }
      } catch (_) { /* silent — auto-BOM is best-effort */ }
    })();
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
      // Leave colour unset so the renderer's default (NET_INK = blue)
      // applies. Storing an explicit hex here would pin every new
      // net to that hex regardless of future palette changes — and
      // when we changed the default from black to blue, every
      // pre-existing schematic kept the old black because the colour
      // had been baked into the saved net data.
      color: null,
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
  // Orthogonal wire router between two scene points. Honours the
  // outgoing direction of each endpoint's pin (``fromSide`` /
  // ``toSide``) so the wire exits the source pin in the direction
  // its stick points and enters the destination pin in the direction
  // ITS stick points. Endpoints are kept exact (no snap-to-grid) so
  // wires meet the visible terminator pixel-for-pixel.
  //
  // Cases:
  //   1. Aligned (same x or same y) → straight line.
  //   2. Perpendicular axes (one H pin, one V pin) → single L-bend
  //      with the corner placed so both ends arrive in the correct
  //      direction (corner = (toX, fromY) for from-H/to-V, or
  //      (fromX, toY) for from-V/to-H). When the geometry forces the
  //      wrong arrival direction (the corner sits on the wrong side
  //      of the destination), upgrade to a Z-bend that loops past so
  //      the wire still enters the destination from its natural side.
  //   3. Same axis (both H or both V) → Z-bend with the
  //      perpendicular middle leg at the midpoint.
  function lShapedSegments(x1, y1, x2, y2, fromSide, toSide) {
    if (x1 === x2 && y1 === y2) return [];
    if (x1 === x2 || y1 === y2) {
      return [{ x1: x1, y1: y1, x2: x2, y2: y2 }];
    }
    var fromH = (fromSide === 'left' || fromSide === 'right');
    var toH   = (toSide   === 'left' || toSide   === 'right');

    // Outward direction along each pin's stick (the way the wire
    // leaves the source / approaches the destination from outside).
    var SIDE_DX = { left: -1, right: 1, top: 0, bottom: 0 };
    var SIDE_DY = { left: 0,  right: 0, top: -1, bottom: 1 };
    var fromDx = SIDE_DX[fromSide] || 0;
    var fromDy = SIDE_DY[fromSide] || 0;
    var toDx   = SIDE_DX[toSide]   || 0;
    var toDy   = SIDE_DY[toSide]   || 0;

    function single(cornerX, cornerY) {
      return [
        { x1: x1, y1: y1, x2: cornerX, y2: cornerY },
        { x1: cornerX, y1: cornerY, x2: x2, y2: y2 },
      ];
    }

    // Helper: does the single-L corner respect a given endpoint's
    // outward direction? "Respect" = the wire leaves / arrives in the
    // direction the pin's stick points.
    function legGoesInDir(fromX, fromY, toX, toY, dx, dy) {
      // The leg is axis-aligned (one of dx/dy is 0). Check that the
      // movement matches the requested direction.
      if (dx !== 0) return Math.sign(toX - fromX) === dx;
      if (dy !== 0) return Math.sign(toY - fromY) === dy;
      return true;
    }

    if (fromH !== toH && fromSide && toSide) {
      // Perpendicular axes.
      var cornerX = fromH ? x2 : x1;
      var cornerY = fromH ? y1 : y2;
      var leg1Ok = legGoesInDir(x1, y1, cornerX, cornerY, fromDx, fromDy);
      var leg2Ok = legGoesInDir(cornerX, cornerY, x2, y2, -toDx, -toDy);
      if (leg1Ok && leg2Ok) return single(cornerX, cornerY);
      // Geometry forces wrong direction at one end → loop around with
      // a Z-bend so each end arrives in its natural direction.
      var STUB = 16;
      var sx = x1 + fromDx * STUB;
      var sy = y1 + fromDy * STUB;
      var tx = x2 + toDx * STUB;
      var ty = y2 + toDy * STUB;
      if (fromH) {
        // From-H → To-V: stub-H, mid-V, stub-H, stub-V.
        // After merging collinear: 3 segments.
        return [
          { x1: x1, y1: y1, x2: sx, y2: y1 },   // out from source
          { x1: sx, y1: y1, x2: sx, y2: ty },   // vertical bridge
          { x1: sx, y1: ty, x2: tx, y2: ty },   // across to target's column
          { x1: tx, y1: ty, x2: x2, y2: y2 },   // into target
        ];
      }
      return [
        { x1: x1, y1: y1, x2: x1, y2: sy },
        { x1: x1, y1: sy, x2: tx, y2: sy },
        { x1: tx, y1: sy, x2: tx, y2: ty },
        { x1: tx, y1: ty, x2: x2, y2: y2 },
      ];
    }

    // Same-axis OR no side info → Z-bend with midpoint perpendicular.
    if (fromH || (!fromSide && !toSide && Math.abs(x2 - x1) >= Math.abs(y2 - y1))) {
      var midX = (x1 + x2) / 2;
      return [
        { x1: x1,   y1: y1, x2: midX, y2: y1 },
        { x1: midX, y1: y1, x2: midX, y2: y2 },
        { x1: midX, y1: y2, x2: x2,   y2: y2 },
      ];
    }
    var midY = (y1 + y2) / 2;
    return [
      { x1: x1, y1: y1,   x2: x1, y2: midY },
      { x1: x1, y1: midY, x2: x2, y2: midY },
      { x1: x2, y1: midY, x2: x2, y2: y2 },
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
    var fromDef = symbolDefById(fromInst.symbolId);
    var toDef   = symbolDefById(toInst.symbolId);
    var fromSide = fromPin && fromDef ? effectivePinSide(fromDef, fromPin, fromInst) : (fromPin && fromPin.side);
    var toSide   = toPin   && toDef   ? effectivePinSide(toDef,   toPin,   toInst)   : (toPin   && toPin.side);
    lShapedSegments(
      fromXY.x, fromXY.y, toXY.x, toXY.y, fromSide, toSide
    ).forEach(function (s) {
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

    // Body. Most symbols render as a rounded rectangle; specific
    // power symbols (GND, V+) get their conventional schematic glyph
    // instead so the schematic reads at a glance.
    if (symbolDef.id === 'gnd') {
      // Standard GND symbol: a short vertical stub dropping from the
      // top-centre pin, then three horizontal bars decreasing in width
      // (widest at top, narrowest at bottom).
      var topY  = -h / 2;            // pin attaches here (top centre)
      var stubBottomY = topY + 6;    // short connector line ends here
      var bar1Y = stubBottomY;       // widest bar
      var bar2Y = bar1Y + 6;
      var bar3Y = bar2Y + 6;
      var halfWide = w * 0.40;       // bar widths
      var halfMid  = w * 0.24;
      var halfNarrow = w * 0.10;
      pieces.push(new fabric.Line([0, topY, 0, stubBottomY], {
        stroke: INK, strokeWidth: 1.5,
      }));
      pieces.push(new fabric.Line(
        [-halfWide, bar1Y, halfWide, bar1Y],
        { stroke: INK, strokeWidth: 1.8 }));
      pieces.push(new fabric.Line(
        [-halfMid, bar2Y, halfMid, bar2Y],
        { stroke: INK, strokeWidth: 1.8 }));
      pieces.push(new fabric.Line(
        [-halfNarrow, bar3Y, halfNarrow, bar3Y],
        { stroke: INK, strokeWidth: 1.8 }));
    } else if (symbolDef.id === 'vplus') {
      // V+ rail: an upward-pointing arrow ('^') at the top of the
      // symbol with a vertical connector line dropping straight down
      // to the bottom-centre pin (which extends further down via its
      // own pin stick to where wires attach).
      var arrowTipY = -h / 2;         // tip of the ^ at the symbol's top
      var arrowBaseY = arrowTipY + 8; // where the wings end / vertical begins
      var bodyBottomY = h / 2;        // pin anchor (stick continues below)
      var armHalf = w * 0.30;         // wing half-width
      // Left wing: tip down-and-left
      pieces.push(new fabric.Line(
        [0, arrowTipY, -armHalf, arrowBaseY],
        { stroke: INK, strokeWidth: 1.8 }));
      // Right wing: tip down-and-right
      pieces.push(new fabric.Line(
        [0, arrowTipY, armHalf, arrowBaseY],
        { stroke: INK, strokeWidth: 1.8 }));
      // Vertical connector line from the arrow's centre down to the
      // pin anchor at the body bottom.
      pieces.push(new fabric.Line(
        [0, arrowTipY, 0, bodyBottomY],
        { stroke: INK, strokeWidth: 1.5 }));
    } else if (symbolDef.isCustom && Array.isArray(symbolDef._bodyShapes) &&
               symbolDef._bodyShapes.length > 0) {
      // Custom symbol — render the user-designed body shapes (rects,
      // lines, text) at their actual designer positions instead of an
      // auto-generated bounding rectangle. _minX / _minY were captured
      // by symbolDefFromCustom AFTER the bbox-pad, so subtracting them
      // gives body-local (top-left = 0,0) coords; then shift to
      // body-centred coords for the group children.
      var sMinX = symbolDef._minX;
      var sMinY = symbolDef._minY;
      var halfW = w / 2;
      var halfH = h / 2;
      function toLocalX(x) { return (x - sMinX) - halfW; }
      function toLocalY(y) { return (y - sMinY) - halfH; }
      symbolDef._bodyShapes.forEach(function (s) {
        if (!s || !s.kind) return;
        if (s.kind === 'rect') {
          pieces.push(new fabric.Rect({
            left: toLocalX(s.x),
            top: toLocalY(s.y),
            width: s.w,
            height: s.h,
            fill: 'transparent',
            stroke: s.stroke || INK,
            strokeWidth: 1.5,
            originX: 'left',
            originY: 'top',
          }));
        } else if (s.kind === 'line') {
          pieces.push(new fabric.Line(
            [toLocalX(s.x1), toLocalY(s.y1),
             toLocalX(s.x2), toLocalY(s.y2)],
            { stroke: s.stroke || INK, strokeWidth: 1.5 }
          ));
        } else if (s.kind === 'text') {
          // Text shapes from the designer — preserve the role-based
          // styling so name/label/text variants read the same as in
          // the designer.
          var monoStack = '"Courier New", "Source Code Pro", ui-monospace, monospace';
          pieces.push(new fabric.Text(s.text || '', {
            left: toLocalX(s.x),
            top: toLocalY(s.y),
            fontSize: s.fontSize || 12,
            fontFamily: monoStack,
            fontWeight: (s.role === 'name' || s.bold) ? 'bold' : 'normal',
            fill: s.stroke || INK,
            originX: 'left',
            originY: 'top',
          }));
        }
      });
    } else {
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
    }

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

    // Pins: a stick + a pin-name label. Custom symbols (designed in
    // the Symbol Designer) mirror the designer's pin geometry exactly
    // — a longer 2-grid stick plus a circle terminator at the outer
    // end where wires attach — so the placed instance reads as the
    // same drawing the user just designed. Built-in stub symbols
    // keep the shorter v1 stick.
    var isCustom = !!symbolDef.isCustom;
    var stickLen = isCustom ? 32 : 8;          // 2 * symbol-designer GRID
    var terminatorR = isCustom ? 4 : 0;        // 0 → no terminator drawn
    symbolDef.pins.forEach(function (pin) {
      var local = pinLocalPos(symbolDef, pin);
      // Translate from "0..bodyWidth" coords to body-centred coords.
      var px = local.x - w / 2;
      var py = local.y - h / 2;
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
      // Tag with the pin number so the net-snap highlighter can find
      // this exact line + its terminator + label and recolour them.
      stick.data = { kind: 'pin-piece', pinNumber: pin.number, role: 'stick' };
      pieces.push(stick);
      // Circle terminator at the outer end — the wire-attachment
      // point. Mirrors the symbol designer's Fusion-360-style pin.
      if (terminatorR > 0) {
        var term = new fabric.Circle({
          left: sx2,
          top: sy2,
          radius: terminatorR,
          fill: '#fff',
          stroke: INK,
          strokeWidth: 1.4,
          originX: 'center',
          originY: 'center',
        });
        term.data = { kind: 'pin-piece', pinNumber: pin.number, role: 'terminator' };
        pieces.push(term);
      }

      // Pin label — placed past the INNER end of the pin's line, in
      // the direction TOWARD the symbol's centre. This puts the
      // label inside the body, away from where wires attach, so a
      // wire entering the pin can't visually run over the label.
      //
      // For standard pins (anchor at body edge, terminator outside)
      // the inner end is the anchor (px) — label goes inside body.
      // For "reverse" designs (anchor outside, terminator at body
      // edge — e.g. a Pico where the body rect sits to the right of
      // the pin anchors), the inner end is the terminator (sx2) —
      // again, label lands inside the body.
      //
      // The trick: pick whichever endpoint has the SMALLER absolute
      // value (closer to the symbol centre = "inside"). Then the
      // pad direction is opposite of that endpoint's sign — moving
      // toward centre.
      //
      // Skip for power symbols (GND / V+): their glyph already
      // identifies them and an extra label would be visual noise.
      if (symbolDef.id !== 'gnd' && symbolDef.id !== 'vplus') {
        var lx, ly, anchorX, anchorY;
        // Generous gap so the label doesn't visually touch the
        // pin's terminator circle or the body edge it sits next
        // to. Matches the Symbol Designer's labelPad so the two
        // views render identically.
        var pad = 12;
        var horizontal = (pin.side === 'left' || pin.side === 'right');
        if (horizontal) {
          // Endpoint closer to centre = inner.
          var innerX = (Math.abs(px) <= Math.abs(sx2)) ? px : sx2;
          // Move TOWARD centre: if inner is negative (left of centre),
          // step right (+1); if positive (right of centre), step left.
          var dirX = (innerX < 0) ? 1 : -1;
          lx = innerX + dirX * pad;
          ly = py;
          // Text extends toward the body centre; anchor on the side
          // facing away from centre so growth runs centreward.
          anchorX = (dirX > 0) ? 'left' : 'right';
          anchorY = 'center';
        } else {
          var innerY = (Math.abs(py) <= Math.abs(sy2)) ? py : sy2;
          var dirY = (innerY < 0) ? 1 : -1;
          lx = px;
          ly = innerY + dirY * pad;
          anchorX = 'center';
          anchorY = (dirY > 0) ? 'top' : 'bottom';
        }
        var pinLabel = new fabric.Text(pin.name || pin.number, {
          left: lx,
          top: ly,
          fontSize: 9,
          fill: INK,
          fontFamily: 'system-ui, -apple-system, "Segoe UI", sans-serif',
          originX: anchorX,
          originY: anchorY,
        });
        pinLabel.data = { kind: 'pin-piece', pinNumber: pin.number, role: 'label' };
        pieces.push(pinLabel);
      }
    });

    // Origin cross (small red "+")
    pieces.push(new fabric.Line([-6, 0, 6, 0], { stroke: BRAND_RED, strokeWidth: 1 }));
    pieces.push(new fabric.Line([0, -6, 0, 6], { stroke: BRAND_RED, strokeWidth: 1 }));

    // Transparent balancing rectangle. fabric.Group with
    // originX/Y='center' positions itself by its CHILDREN'S bbox
    // centre — any asymmetry (refLabel above the body, designer
    // text/shapes outside the bbox, wide pin labels extending past
    // a terminator) shifts every child to compensate, so the wire
    // endpoint (computed via pinScenePos from body coords) lands
    // offset from the visible terminator.
    //
    // To cancel this, compute the actual bounding extents of every
    // piece we just added, then push a transparent rect that makes
    // the bbox symmetric around (0, 0). This works regardless of
    // what the designer drew — text/shapes/labels can extend in any
    // direction and we'll catch them.
    var minPX = Infinity, maxPX = -Infinity;
    var minPY = Infinity, maxPY = -Infinity;
    function _bumpBounds(b) {
      if (!b) return;
      if (b.left   < minPX) minPX = b.left;
      if (b.right  > maxPX) maxPX = b.right;
      if (b.top    < minPY) minPY = b.top;
      if (b.bottom > maxPY) maxPY = b.bottom;
    }
    function _pieceBounds(p) {
      var l = p.left || 0;
      var t = p.top  || 0;
      var w = 0, h = 0;
      if (p.type === 'rect') {
        w = p.width  || 0;
        h = p.height || 0;
      } else if (p.type === 'line') {
        var a = Math.min(p.x1, p.x2);
        var b = Math.min(p.y1, p.y2);
        var c = Math.max(p.x1, p.x2);
        var d = Math.max(p.y1, p.y2);
        return { left: a, top: b, right: c, bottom: d };
      } else if (p.type === 'text' || p.type === 'i-text' || p.type === 'IText') {
        // Approximate: char width ≈ 0.6 × fontSize, height ≈ 1.4 ×.
        // Fabric also sets p.width/p.height after construction, prefer
        // them when present.
        w = p.width  || ((p.text || '').length * (p.fontSize || 12) * 0.6);
        h = p.height || ((p.fontSize || 12) * 1.4);
      } else if (p.type === 'circle') {
        var r = p.radius || 0;
        w = h = 2 * r;
      } else {
        return null;
      }
      // Adjust for origin (default is 'left'/'top').
      if (p.originX === 'center') l -= w / 2;
      else if (p.originX === 'right') l -= w;
      if (p.originY === 'center') t -= h / 2;
      else if (p.originY === 'bottom') t -= h;
      return { left: l, top: t, right: l + w, bottom: t + h };
    }
    pieces.forEach(function (p) { _bumpBounds(_pieceBounds(p)); });
    if (isFinite(minPX) && isFinite(minPY)) {
      var symX = Math.max(Math.abs(minPX), Math.abs(maxPX));
      var symY = Math.max(Math.abs(minPY), Math.abs(maxPY));
      // Add a tiny margin so anti-aliasing doesn't shave a pixel.
      symX += 1; symY += 1;
      pieces.push(new fabric.Rect({
        left: -symX, top: -symY,
        width: symX * 2, height: symY * 2,
        fill: 'transparent', stroke: null,
        selectable: false, evented: false,
      }));
    }

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

  // ----- Net colour resolution -----------------------------------
  // Precedence:
  //   1. Selection state → BRAND_RED (always wins so the user can
  //      see what's selected)
  //   2. GND pin attached to the net → black
  //   3. Power pin attached to the net → red
  //   4. User-set custom colour (anything not the legacy black)
  //   5. Auto-palette colour, stable per net.id
  //
  // Pin role inference checks pin.type first (set explicitly via the
  // Symbol Designer's pin-type radio), then falls back to a name
  // pattern so legacy / built-in symbols whose pins don't carry a
  // type field still get the right behaviour.
  function inferPinRole(pin) {
    if (!pin) return null;
    if (pin.type === 'GND') return 'GND';
    if (pin.type === 'PWR') return 'PWR';
    var n = String(pin.name || '').trim().toUpperCase();
    if (n === 'GND' || n === 'VSS' || n === '0V') return 'GND';
    if (/^(VCC|VDD|V\+|VEE|3V3|3\.3V|5V|VSYS|VBUS|VIN)\b/.test(n)) return 'PWR';
    return null;
  }

  function getNetElectricalRole(net) {
    if (!net || !Array.isArray(net.endpoints)) return null;
    var sawPwr = false;
    for (var i = 0; i < net.endpoints.length; i++) {
      var ep = net.endpoints[i];
      var inst = findInstance(ep.instanceId);
      if (!inst) continue;
      var pin = findPin(inst, ep.pinNumber);
      var role = inferPinRole(pin);
      // Treat the GND / V+ rail symbols themselves as a GND / PWR
      // signal — their refDes prefix encodes the intent even when
      // the pin name doesn't (e.g. a Symbol-Designer-authored GND
      // symbol whose pin is just labelled "1").
      if (!role && inst.refDes) {
        var rd = String(inst.refDes).toUpperCase();
        if (rd.indexOf('GND') === 0) role = 'GND';
        // V+ rail refDes is "V<digit>" / "V+" — match exactly that
        // so genuine power rails light up red but voltage regulators
        // (VR1), via stitches (VIA), etc. don't get mis-classified.
        else if (/^V[\d+]/.test(rd)) role = 'PWR';
      }
      if (role === 'GND') return 'GND'; // GND wins immediately
      if (role === 'PWR') sawPwr = true;
    }
    return sawPwr ? 'PWR' : null;
  }

  // Stable per-net auto-colour. Hashes the net's id (or name as
  // fallback) into a palette index — the same net gets the same
  // colour on every reload, and adding/removing other nets doesn't
  // disturb existing ones.
  function autoNetColour(net) {
    var key = String((net && (net.id || net.name)) || '');
    var h = 0;
    for (var i = 0; i < key.length; i++) {
      h = ((h << 5) - h) + key.charCodeAt(i);
      h |= 0; // force 32-bit
    }
    return NET_AUTO_PALETTE[Math.abs(h) % NET_AUTO_PALETTE.length];
  }

  function resolveNetStroke(net, selected) {
    if (selected) return BRAND_RED;
    var role = getNetElectricalRole(net);
    if (role === 'GND') return NET_INK_GND;
    if (role === 'PWR') return NET_INK_PWR;
    // Legacy auto-upgrade: '#222222' (old INK) was hardcoded into
    // every saved net by newNet() pre-palette-change. Treat those
    // as "no override" so existing schematics pick up the new
    // auto-colour. Real user-set colours pass through unchanged.
    var hasCustom = net.color &&
      net.color !== INK && net.color !== '#222' && net.color !== 'black' &&
      net.color !== NET_INK;
    if (hasCustom) return net.color;
    return autoNetColour(net);
  }

  // Compute a perpendicular offset for every segment in the scene so
  // segments that share a "lane" (same orientation + same axis-
  // perp position + overlapping range) get rendered as parallel
  // rails instead of a single overlapping line. Returns a Map keyed
  // by segment ref → {dx, dy}.
  //
  // Algorithm:
  //   1. Group segments by (orientation, axis-perp value rounded
  //      to a few px so near-coincident lanes share a bucket).
  //   2. Within each bucket, find segments whose overlap range is
  //      non-empty and assign offsets [-LANE_GAP, 0, +LANE_GAP,
  //      -2*LANE_GAP, +2*LANE_GAP, ...] in a stable order.
  //   3. Segments that don't overlap any sibling get offset 0.
  //
  // Deterministic ordering (by net.id + segment index) so a given
  // schematic always renders the same way across reloads.
  var LANE_GAP = 4;
  function computeSegmentOffsets() {
    var offsets = new Map();
    if (!STATE.graph || !Array.isArray(STATE.graph.nets)) return offsets;
    // (1) bucket every segment by lane.
    var buckets = {}; // key 'h:200' or 'v:300' → [{ seg, net }, ...]
    STATE.graph.nets.forEach(function (net) {
      if (!Array.isArray(net.segments)) return;
      net.segments.forEach(function (seg) {
        var key;
        if (seg.y1 === seg.y2) {
          key = 'h:' + seg.y1;
        } else if (seg.x1 === seg.x2) {
          key = 'v:' + seg.x1;
        } else {
          return; // diagonals don't share lanes with rectilinear runs
        }
        (buckets[key] = buckets[key] || []).push({ seg: seg, net: net });
      });
    });
    // (2) per-lane overlap check + offset assignment.
    Object.keys(buckets).forEach(function (k) {
      var entries = buckets[k];
      if (entries.length < 2) return;
      var horizontal = (k[0] === 'h');
      // For each pair, decide if they overlap.
      function rangeOf(seg) {
        return horizontal
          ? [Math.min(seg.x1, seg.x2), Math.max(seg.x1, seg.x2)]
          : [Math.min(seg.y1, seg.y2), Math.max(seg.y1, seg.y2)];
      }
      // Group entries that all share at least some range with each
      // other (transitive closure). Each cluster gets distinct
      // offsets; singletons stay at 0.
      var clusters = [];
      entries.forEach(function (entry) {
        var r = rangeOf(entry.seg);
        var joined = null;
        clusters.forEach(function (cl) {
          // Overlap = ranges intersect (open ends count as touching).
          if (cl.range[0] <= r[1] && cl.range[1] >= r[0]) {
            joined = cl;
          }
        });
        if (joined) {
          joined.members.push(entry);
          joined.range[0] = Math.min(joined.range[0], r[0]);
          joined.range[1] = Math.max(joined.range[1], r[1]);
        } else {
          clusters.push({ members: [entry], range: [r[0], r[1]] });
        }
      });
      clusters.forEach(function (cl) {
        if (cl.members.length < 2) return;
        // Stable order by net.id + segment index so colour
        // assignments are deterministic across reloads.
        cl.members.sort(function (a, b) {
          var an = String(a.net.id || ''); var bn = String(b.net.id || '');
          return an < bn ? -1 : an > bn ? 1 : 0;
        });
        // Offsets centred around 0: -1, +1, -2, +2, ... so the
        // bundle of N parallel lines is symmetric about the lane.
        cl.members.forEach(function (entry, i) {
          var rank = Math.floor((i + 1) / 2);
          var sign = (i % 2 === 0) ? -1 : 1;
          var off = rank * LANE_GAP * sign;
          // Skip writing 0-offsets — saves the Map lookup later for
          // segments that don't need any shift.
          if (off === 0) return;
          if (horizontal) {
            offsets.set(entry.seg, { dx: 0, dy: off });
          } else {
            offsets.set(entry.seg, { dx: off, dy: 0 });
          }
        });
      });
    });
    return offsets;
  }

  function buildNetFabric(net, selected, segOffsets) {
    // One Polyline per segment — cheaper than building a single
    // multi-segment Path when we want individual segments to be
    // hit-testable.
    var objs = [];
    var resolvedStroke = resolveNetStroke(net, selected);
    net.segments.forEach(function (seg) {
      // Apply the precomputed lane-offset so two nets that share a
      // run render as parallel rails rather than one overlapping
      // line. Map lookup → {dx, dy}; default to 0 if not in the
      // map (segment doesn't share its lane with anything).
      var off = (segOffsets && segOffsets.get(seg)) || null;
      var dx = off ? off.dx : 0;
      var dy = off ? off.dy : 0;
      var line = new fabric.Line(
        [seg.x1 + dx, seg.y1 + dy, seg.x2 + dx, seg.y2 + dy],
        {
        stroke: resolvedStroke,
        strokeWidth: selected ? NET_STROKE_SELECTED : NET_STROKE,
        // ``strokeLineCap: 'square'`` extends the line by half its
        // stroke-width past each endpoint. At an L-bend, two
        // perpendicular segments share a corner point; with squared
        // caps each line's end fills the corner from both sides so
        // the join reads as a clean sharp angle rather than a
        // hairline gap. The cap also extends past the net's outer
        // endpoints by the same amount, but those endpoints sit at
        // pin terminators (radius-4 circles) which visually mask
        // the small overshoot.
        strokeLineCap: 'square',
        // selectable: false keeps the wire out of rubber-band group
        // selections; the existing onCanvasMouseDown handler still
        // catches a direct click via evented: true and toggles
        // STATE.selectedNetId itself.
        selectable: false,
        evented: true,
        hasControls: false,
        hasBorders: false,
        hoverCursor: 'pointer',
        perPixelTargetFind: true,
        objectCaching: false,
      });
      line.data = { kind: 'net', netId: net.id };
      objs.push(line);
    });
    // Net label — anchored at the midpoint of the longest segment so
    // it sits in a readable spot regardless of which segment was
    // Net names are intentionally NOT drawn on the canvas. The
    // information is still available — every net's name shows up
    // in the right-rail Properties panel when its wire is selected,
    // and in the Connections table below. Floating "Net-1 / Net-2
    // / GND" labels in the middle of the diagram add clutter without
    // adding value (especially now that GND/PWR colour-codes black
    // and red, and signal nets each get their own auto-palette
    // colour). If a future use-case wants a visible net tag on the
    // canvas, re-introduce as an opt-in toggle rather than
    // unconditionally.
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

    // Symbols first, then nets on top — so wires don't disappear
    // behind a body when their route gets close to a symbol. Wires
    // are the user's primary edit target so they need to be visible
    // even where they overlap a symbol's body.
    STATE.graph.instances.forEach(function (inst) {
      var g = buildInstanceFabric(inst);
      if (g) STATE.canvas.add(g);
    });

    // Nets above symbols. Pre-compute lane offsets once for the
    // whole scene so segments that overlap on a shared run get
    // rendered as parallel rails. Passed through to buildNetFabric.
    var segOffsets = computeSegmentOffsets();

    STATE.graph.nets.forEach(function (net) {
      var selected = (net.id === STATE.selectedNetId);
      buildNetFabric(net, selected, segOffsets).forEach(function (line) {
        STATE.canvas.add(line);
      });
    });

    // Junctions above nets so the dot is always visible.
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
    // Net selected → show the net name editor so users can rename
    // wires (e.g. SDA, SCL, 3V3) without using the Label tool prompt.
    if (STATE.selectedNetId) {
      var selNet = STATE.graph.nets.find(function (n) { return n.id === STATE.selectedNetId; });
      if (selNet) {
        dom.propsBody.innerHTML =
          '<div class="se-props-field">' +
          '  <span class="se-props-field-label">Net</span>' +
          '  <span class="se-props-field-value">' + escapeHtml(selNet.name || '(unnamed)') + '</span>' +
          '</div>' +
          '<div class="se-props-field">' +
          '  <label class="se-props-field-label" for="se-props-net-name">Name</label>' +
          '  <input class="se-props-input" id="se-props-net-name" type="text"' +
          '         value="' + escapeHtml(selNet.name || '') + '"' +
          '         placeholder="e.g. SDA, SCL, 3V3" maxlength="40"' +
          '         data-net-id="' + escapeHtml(selNet.id) + '">' +
          '</div>';
        var nameInput = document.getElementById('se-props-net-name');
        if (nameInput) {
          var commit = function () {
            var next = (nameInput.value || '').trim();
            if (next === (selNet.name || '')) return;
            selNet.name = next || null;
            renderGraph();
            scheduleAutosave();
          };
          nameInput.addEventListener('change', commit);
          nameInput.addEventListener('blur', commit);
          nameInput.addEventListener('keydown', function (e) {
            if (e.key === 'Enter') { e.preventDefault(); nameInput.blur(); }
          });
        }
        return;
      }
    }
    var inst = findInstance(STATE.selectedInstanceId);
    if (!inst) {
      dom.propsBody.innerHTML =
        '<div class="se-props-empty text-muted small">' +
        'Select a component or net on the canvas to see its properties.' +
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
    var hadNet = !!STATE.selectedNetId;
    STATE.selectedInstanceId = null;
    STATE.selectedNetId = null;
    // Only renderGraph if a net's highlight needs to clear. Otherwise
    // a click-on-empty just to start a rubber-band drag would wipe
    // the canvas inside Fabric's mouse:down — same family of bug as
    // the schematic editor's "click selects but kills drag" issue.
    if (hadNet) {
      renderGraph();
    } else {
      renderConnectionsTable();
      renderProps();
      updateSymbolHud();
    }
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
      // Let the embedding instruction-builder know the schematic
      // content just changed so it can re-snapshot the thumbnail
      // without polling.
      postToParent({ kr_se_event: 'content-changed' });
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

  // Collect the instance IDs that are currently selected. Single-pick
  // → [selectedInstanceId]. Multi-pick (Fabric ActiveSelection) →
  // every child group's .data.instanceId. This drives the rotate /
  // flip handlers so they operate on every selected instance.
  function selectedInstanceIds() {
    var active = STATE.canvas && STATE.canvas.getActiveObject
      ? STATE.canvas.getActiveObject()
      : null;
    if (active &&
        (active.type === 'activeselection' || active.type === 'activeSelection') &&
        active.getObjects) {
      var ids = [];
      active.getObjects().forEach(function (child) {
        if (child && child.data && child.data.kind === 'instance') {
          ids.push(child.data.instanceId);
        }
      });
      if (ids.length > 0) return ids;
    }
    return STATE.selectedInstanceId ? [STATE.selectedInstanceId] : [];
  }

  // Rebuild an ActiveSelection over the Fabric instance-groups whose
  // `data.instanceId` matches any of the supplied ids. Used after a
  // rotate / flip so the user's multi-selection survives the
  // renderGraph() rebuild and they can chain transforms (e.g. press
  // rotate four times to get 360°).
  function restoreInstanceSelection(ids) {
    if (!STATE.canvas || !ids || ids.length === 0) return;
    var byId = {};
    STATE.canvas.getObjects().forEach(function (o) {
      if (o && o.data && o.data.kind === 'instance') {
        byId[o.data.instanceId] = o;
      }
    });
    var fresh = [];
    ids.forEach(function (id) {
      if (byId[id]) fresh.push(byId[id]);
    });
    if (fresh.length === 0) return;
    if (fresh.length === 1) {
      STATE.canvas.setActiveObject(fresh[0]);
    } else if (typeof fabric !== 'undefined' && fabric.ActiveSelection) {
      var sel = new fabric.ActiveSelection(fresh, { canvas: STATE.canvas });
      STATE.canvas.setActiveObject(sel);
    }
    STATE.canvas.requestRenderAll();
  }

  function rotateSelected(deltaDeg) {
    var ids = selectedInstanceIds();
    if (ids.length === 0) return;
    // Per-instance rotation: each instance rotates IN PLACE about its
    // own origin (instance.x / instance.y). The user gets every
    // selected symbol spinning around its own anchor — what
    // "rotate the selection in place" actually means here.
    ids.forEach(function (id) {
      var inst = findInstance(id);
      if (!inst) return;
      inst.rotation = (((inst.rotation || 0) + deltaDeg) % 360 + 360) % 360;
      rerouteNetsFor(inst.id);
    });
    // Drop the stale AS, re-render with the new rotations, then
    // re-build the AS over the fresh group objects so the user's
    // selection survives the operation (and they can keep clicking
    // rotate to spin the selection further).
    if (STATE.canvas && STATE.canvas.discardActiveObject) {
      STATE.canvas.discardActiveObject();
    }
    renderGraph();
    restoreInstanceSelection(ids);
    scheduleAutosave();
  }

  function flipSelected(axis) {
    var ids = selectedInstanceIds();
    if (ids.length === 0) return;
    ids.forEach(function (id) {
      var inst = findInstance(id);
      if (!inst) return;
      if (axis === 'h') inst.flipH = !inst.flipH;
      if (axis === 'v') inst.flipV = !inst.flipV;
      rerouteNetsFor(inst.id);
    });
    if (STATE.canvas && STATE.canvas.discardActiveObject) {
      STATE.canvas.discardActiveObject();
    }
    renderGraph();
    restoreInstanceSelection(ids);
    scheduleAutosave();
  }

  function deleteSelected() {
    // Gather instance ids from single-select OR Fabric's multi-select
    // (ActiveSelection). selectedInstanceIds() returns [] when nothing
    // is selected.
    var instanceIds = selectedInstanceIds();
    if (instanceIds.length > 0) {
      var idSet = {};
      instanceIds.forEach(function (id) { idSet[id] = true; });
      // Drop the instances themselves.
      STATE.graph.instances = STATE.graph.instances.filter(function (i) {
        return !idSet[i.id];
      });
      // For every net: drop endpoints that pointed at any removed
      // instance, then DROP THE NET ENTIRELY if it no longer has at
      // least two endpoints — a net with 0 or 1 endpoints is an
      // orphan (segments hanging off nothing). This is what the user
      // saw before: nets staying behind as ghost wires after deleting
      // the symbols they connected.
      STATE.graph.nets = STATE.graph.nets
        .map(function (n) {
          n.endpoints = n.endpoints.filter(function (ep) {
            return !idSet[ep.instanceId];
          });
          return n;
        })
        .filter(function (n) {
          return n.endpoints.length >= 2;
        });
      STATE.selectedInstanceId = null;
      if (STATE.canvas && STATE.canvas.discardActiveObject) {
        STATE.canvas.discardActiveObject();
      }
    } else if (STATE.selectedNetId) {
      STATE.graph.nets = STATE.graph.nets.filter(function (n) {
        return n.id !== STATE.selectedNetId;
      });
      STATE.selectedNetId = null;
    } else {
      return;
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
        var anchorDef = symbolDefById(anchorInst.symbolId);
        var pinDef    = symbolDefById(inst.symbolId);
        var aSide = anchorPin && anchorDef ? effectivePinSide(anchorDef, anchorPin, anchorInst) : (anchorPin && anchorPin.side);
        var pSide = pin       && pinDef    ? effectivePinSide(pinDef,    pin,       inst)       : (pin       && pin.side);
        lShapedSegments(
          anchorXY.x, anchorXY.y, xy.x, xy.y, aSide, pSide
        ).forEach(function (s) {
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
      // Resolve the click to a pin: first try Fabric's direct hit
      // (clicked exactly on the hotspot), then fall back to the
      // nearest-pin scan within NET_PIN_SNAP_RADIUS. The fallback is
      // what gives custom symbols their forgiving snap — the user
      // doesn't have to land exactly on the small terminator dot.
      var hitData = null;
      if (obj && obj.data && obj.data.kind === 'pin-hotspot') {
        hitData = {
          instanceId: obj.data.instanceId,
          pinNumber:  obj.data.pinNumber,
          x: obj.data.x,
          y: obj.data.y,
        };
      } else {
        var near = findNearestPin(p.x, p.y);
        if (near) {
          hitData = {
            instanceId: near.instance.id,
            pinNumber:  near.pin.number,
            x: near.x,
            y: near.y,
          };
        }
      }
      if (hitData) {
        if (!STATE.netDrawingFrom) {
          STATE.netDrawingFrom = hitData;
          // Light up the source pin immediately so the user knows
          // which pin they just picked, even before they move the
          // mouse to pick a second pin.
          var srcInst = findInstance(hitData.instanceId);
          var srcPin  = srcInst && findPin(srcInst, hitData.pinNumber);
          if (srcInst && srcPin) {
            updateNetPinHighlight({ instance: srcInst, pin: srcPin });
          }
        } else {
          var fromEp = {
            instanceId: STATE.netDrawingFrom.instanceId,
            pinNumber:  STATE.netDrawingFrom.pinNumber,
          };
          var net = commitNetSegment(fromEp, {
            instanceId: hitData.instanceId,
            pinNumber:  hitData.pinNumber,
          });
          STATE.netDrawingFrom = null;
          clearNetPinHighlight();
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
        clearNetPinHighlight();
        if (STATE.netPreviewObj) {
          STATE.canvas.remove(STATE.netPreviewObj);
          STATE.netPreviewObj = null;
          STATE.canvas.requestRenderAll();
        }
      }
      return;
    }

    // ----- Label-tool mode -----
    // Click a wire to name the net it belongs to. The name shows as a
    // small label at the midpoint of the net's longest segment (e.g.
    // "SDA", "SCL", "3V3"). Cancel = no change.
    if (STATE.activeTool === 'label') {
      if (obj && obj.data && obj.data.kind === 'net') {
        var lblNetId = obj.data.netId;
        var lblNet = STATE.graph.nets.find(function (n) { return n.id === lblNetId; });
        if (lblNet) {
          var current = lblNet.name || '';
          var input = window.prompt(
            'Net name (e.g. SDA, SCL, 3V3) — leave blank to clear:',
            current
          );
          if (input !== null) {
            var trimmed = input.trim();
            lblNet.name = trimmed || null;
            STATE.selectedNetId = lblNetId;
            renderGraph();
            scheduleAutosave();
          }
        }
        // Drop back to select tool so the user can continue editing.
        setActiveTool('select');
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
    // ActiveSelection (multi-select) — let Fabric drag the group.
    if (obj && (obj.type === 'activeselection' || obj.type === 'activeSelection')) {
      return;
    }
    // Pan tool: empty-canvas drag pans the viewport. Track screen
    // position + base pan offset so each mouse:move computes an
    // absolute delta (no drift).
    if (STATE.activeTool === 'pan') {
      STATE.isPanning = true;
      STATE.panStartScreenX = opt.e ? opt.e.clientX : 0;
      STATE.panStartScreenY = opt.e ? opt.e.clientY : 0;
      STATE.panStartPanX = STATE.panX || 0;
      STATE.panStartPanY = STATE.panY || 0;
      if (STATE.canvas) {
        STATE.canvas.defaultCursor = 'grabbing';
        STATE.canvas.setCursor('grabbing');
      }
      return;
    }
    clearSelection();
  }

  // Net-drawing snap radius — when the cursor is within this many
  // scene-space pixels of a pin, the pin gets highlighted and a click
  // snaps the net endpoint exactly to the pin's wire-attachment point.
  // Generous enough to make pin picking forgiving on small symbols.
  var NET_PIN_SNAP_RADIUS = 24;

  // Find the pin closest to (sx, sy) in scene coordinates, within
  // NET_PIN_SNAP_RADIUS. Returns { instance, pin, x, y } or null.
  function findNearestPin(sx, sy) {
    var best = null;
    var bestDist2 = NET_PIN_SNAP_RADIUS * NET_PIN_SNAP_RADIUS;
    STATE.graph.instances.forEach(function (inst) {
      var def = symbolDefById(inst.symbolId);
      if (!def) return;
      def.pins.forEach(function (pin) {
        var p = pinScenePos(inst, pin);
        var dx = p.x - sx;
        var dy = p.y - sy;
        var d2 = dx * dx + dy * dy;
        if (d2 <= bestDist2) {
          bestDist2 = d2;
          best = { instance: inst, pin: pin, x: p.x, y: p.y };
        }
      });
    });
    return best;
  }

  // Recolour the actual stick + terminator + label of pins to brand
  // red so the user can see which pins are involved in net drawing:
  //  - the SOURCE pin stays lit from the moment they click it until
  //    they click the destination (so they know which pin they're
  //    drawing from)
  //  - the cursor's NEAREST pin lights up under the mouse so they
  //    see where a click will snap to
  // Both can be lit simultaneously (different pins) or collapse to
  // one entry (when the cursor is over the source pin).
  //
  // ``STATE.netPinHighlights`` is the current set: an array of
  //   { instanceId, pinNumber, restore: [{child, prop, orig}] }
  // entries. updateNetPinHighlight(hit) diffs the desired set against
  // it and applies only the deltas.
  function _desiredHighlights(hit) {
    var list = [];
    var seen = {};
    function add(instanceId, pinNumber) {
      if (instanceId == null || pinNumber == null) return;
      var key = instanceId + ':' + pinNumber;
      if (seen[key]) return;
      seen[key] = true;
      list.push({ instanceId: instanceId, pinNumber: pinNumber });
    }
    // While drawing, keep the source pin lit until the user clicks the
    // destination (or cancels).
    if (STATE.netDrawingFrom) {
      add(STATE.netDrawingFrom.instanceId, STATE.netDrawingFrom.pinNumber);
    }
    if (hit) add(hit.instance.id, hit.pin.number);
    return list;
  }
  function _applyHighlightTo(instanceId, pinNumber) {
    // Find the Fabric instance group and recolour the matching pieces.
    // Returns the restore-list (empty if not found).
    var group = null;
    STATE.canvas.getObjects().forEach(function (o) {
      if (o && o.data && o.data.kind === 'instance' &&
          o.data.instanceId === instanceId) {
        group = o;
      }
    });
    if (!group || !group.getObjects) return [];
    var restore = [];
    group.getObjects().forEach(function (child) {
      if (!child.data || child.data.kind !== 'pin-piece') return;
      if (child.data.pinNumber !== pinNumber) return;
      if (child.data.role === 'label') {
        restore.push({ child: child, prop: 'fill', orig: child.fill });
        child.set('fill', BRAND_RED);
      } else if (child.data.role === 'terminator') {
        restore.push({ child: child, prop: 'stroke', orig: child.stroke });
        restore.push({ child: child, prop: 'fill',   orig: child.fill });
        child.set('stroke', BRAND_RED);
        child.set('fill',   BRAND_RED);
      } else {
        restore.push({ child: child, prop: 'stroke',      orig: child.stroke });
        restore.push({ child: child, prop: 'strokeWidth', orig: child.strokeWidth });
        child.set('stroke', BRAND_RED);
        child.set('strokeWidth', 2);
      }
    });
    group.dirty = true;
    return restore;
  }
  function _restoreEntry(entry) {
    entry.restore.forEach(function (r) { r.child.set(r.prop, r.orig); });
  }
  function updateNetPinHighlight(hit) {
    if (!STATE.canvas) return;
    var desired = _desiredHighlights(hit);
    var current = STATE.netPinHighlights || [];
    var desiredKey = function (d) { return d.instanceId + ':' + d.pinNumber; };
    var have = {};
    current.forEach(function (c) { have[desiredKey(c)] = c; });
    var want = {};
    desired.forEach(function (d) { want[desiredKey(d)] = d; });
    var changed = false;
    // Restore highlights that should no longer be active.
    current.forEach(function (c) {
      if (!want[desiredKey(c)]) {
        _restoreEntry(c);
        changed = true;
      }
    });
    // Apply highlights that aren't currently active.
    var next = [];
    desired.forEach(function (d) {
      var existing = have[desiredKey(d)];
      if (existing) {
        next.push(existing);
        return;
      }
      var restore = _applyHighlightTo(d.instanceId, d.pinNumber);
      if (restore.length > 0) {
        next.push({
          instanceId: d.instanceId,
          pinNumber:  d.pinNumber,
          restore:    restore,
        });
        changed = true;
      }
    });
    STATE.netPinHighlights = next;
    if (changed) STATE.canvas.requestRenderAll();
  }
  function clearNetPinHighlight() {
    if (!STATE.canvas) return;
    (STATE.netPinHighlights || []).forEach(_restoreEntry);
    STATE.netPinHighlights = [];
    STATE.canvas.requestRenderAll();
  }

  function onCanvasMouseMove(opt) {
    // Pan-in-progress overrides everything else so panning works
    // regardless of which tool is selected.
    if (STATE.isPanning && opt.e) {
      var dx = opt.e.clientX - STATE.panStartScreenX;
      var dy = opt.e.clientY - STATE.panStartScreenY;
      STATE.panX = STATE.panStartPanX + dx;
      STATE.panY = STATE.panStartPanY + dy;
      applyViewport();
      return;
    }
    // Net tool active — always look for the nearest pin so the user
    // sees which pin a click will snap to (both before and after the
    // first endpoint is placed).
    if (STATE.activeTool === 'net') {
      var pm = pointerScene(opt);
      var hit = findNearestPin(pm.x, pm.y);
      updateNetPinHighlight(hit);
      if (!STATE.netDrawingFrom) return;
      // Mid-drag: route the preview to either the snapped pin or the
      // raw cursor position. Use the geometry-derived effective pin
      // side for both endpoints so wires exit/enter in the actual
      // outward direction (not just pin.side, which is wrong for
      // "reverse" designs where the body is opposite to pin.side).
      var endX = hit ? hit.x : pm.x;
      var endY = hit ? hit.y : pm.y;
      var from = STATE.netDrawingFrom;
      var fromInst = findInstance(from.instanceId);
      var fromPin  = fromInst && findPin(fromInst, from.pinNumber);
      var fromDef  = fromInst && symbolDefById(fromInst.symbolId);
      var fromSide = (fromPin && fromDef) ? effectivePinSide(fromDef, fromPin, fromInst) : (fromPin && fromPin.side);
      var toDef    = hit && hit.instance && symbolDefById(hit.instance.symbolId);
      var toSide   = (hit && hit.pin && toDef) ? effectivePinSide(toDef, hit.pin, hit.instance) : (hit && hit.pin && hit.pin.side);
      var segs = lShapedSegments(from.x, from.y, endX, endY, fromSide, toSide);
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
      return;
    }
    // Any non-net tool: make sure no stale highlight lingers.
    if (STATE.netPinHighlight) clearNetPinHighlight();
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
    if (!obj) return;

    // Multi-symbol drop. When the user has 2+ symbols rubber-band /
    // shift-click selected, Fabric wraps them in an ActiveSelection
    // and dragging moves them as a unit. We iterate the children,
    // pull each one's now-canvas-space left/top, and update the
    // corresponding instance in STATE.
    if (obj.type === 'activeselection' || obj.type === 'activeSelection') {
      var asMatrix = obj.calcTransformMatrix
        ? obj.calcTransformMatrix()
        : null;
      (obj.getObjects ? obj.getObjects() : []).forEach(function (child) {
        if (!child.data || child.data.kind !== 'instance') return;
        var inst = findInstance(child.data.instanceId);
        if (!inst) return;
        // Child positions inside an AS are stored relative to the AS
        // origin. Transform (child.left, child.top) through the AS
        // matrix to get canvas-space coords.
        var pt;
        if (asMatrix && fabric.util && fabric.util.transformPoint) {
          pt = fabric.util.transformPoint(
            { x: child.left, y: child.top },
            asMatrix
          );
        } else {
          // Fallback: use the bounding-rect centre, accepting the small
          // refLabel-offset bias rather than failing.
          var br = child.getBoundingRect ? child.getBoundingRect() : null;
          pt = br
            ? { x: br.left + br.width / 2, y: br.top + br.height / 2 }
            : { x: inst.x, y: inst.y };
        }
        inst.x = snap(pt.x);
        inst.y = snap(pt.y);
        rerouteNetsFor(inst.id);
      });
      requestAnimationFrame(function () {
        if (STATE.canvas) STATE.canvas.discardActiveObject();
        renderGraph();
      });
      scheduleAutosave();
      return;
    }

    if (!obj.data || obj.data.kind !== 'instance') return;
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
    // Leaving Net mode → drop any pin-snap highlight ring.
    if (tool !== 'net') clearNetPinHighlight();
    // The Rotate tool is a one-shot — apply to the selection then revert.
    if (tool === 'rotate') {
      rotateSelected(90);
      STATE.activeTool = 'select';
      tool = 'select';
    }
    Array.prototype.forEach.call(
      document.querySelectorAll('.se-tool-btn, .se-floating-tool-btn'),
      function (btn) {
        btn.classList.toggle('is-active', btn.dataset.tool === tool);
      }
    );
    // Tool → cursor / selection mode:
    //   select → rubber-band lasso enabled (default), regular pointer
    //   pan    → drag-to-pan the viewport, grab cursor
    //   any other (net, symbol, …) → crosshair, lasso off
    if (STATE.canvas) {
      STATE.canvas.selection = (tool === 'select');
      if (tool === 'pan') {
        STATE.canvas.defaultCursor = 'grab';
        STATE.canvas.hoverCursor   = 'grab';
      } else if (tool === 'select') {
        STATE.canvas.defaultCursor = 'default';
        STATE.canvas.hoverCursor   = 'move';
      } else {
        STATE.canvas.defaultCursor = 'crosshair';
        STATE.canvas.hoverCursor   = 'crosshair';
      }
    }
    if (tool !== 'symbol') {
      STATE.pendingSymbolId = null;
      Array.prototype.forEach.call(
        document.querySelectorAll('.se-symbol-item'),
        function (li) { li.classList.remove('is-pending'); }
      );
    }
    renderGraph();
  }

  // Symbol-library search/group state. ``query`` is the lower-cased
  // search string from the input; matches against name + refDes prefix.
  // ``collapsedCategories`` is a Set of category names the user has
  // folded; persists in localStorage so the panel comes back the way
  // they left it.
  var SYMBOL_LIBRARY_UI = {
    query: '',
    collapsedCategories: null,
  };

  var SYMBOL_COLLAPSE_KEY = 'kr-schematic-symbol-categories-collapsed';

  function getCollapsedCategories() {
    if (SYMBOL_LIBRARY_UI.collapsedCategories) return SYMBOL_LIBRARY_UI.collapsedCategories;
    var set = new Set();
    try {
      var stored = JSON.parse(localStorage.getItem(SYMBOL_COLLAPSE_KEY) || '[]');
      if (Array.isArray(stored)) stored.forEach(function (k) { set.add(k); });
    } catch (_) {}
    SYMBOL_LIBRARY_UI.collapsedCategories = set;
    return set;
  }

  function persistCollapsedCategories() {
    try {
      var arr = Array.from(getCollapsedCategories());
      localStorage.setItem(SYMBOL_COLLAPSE_KEY, JSON.stringify(arr));
    } catch (_) {}
  }

  // Stable category display order — built-ins first, library
  // categories interleaved next, Custom last. Any unknown categories
  // get appended in alphabetical order at the end. Library categories
  // (Active, Sensor, Module, Connector) come from the admin-curated
  // global library; the order here lines up Passive/Active/Power as
  // a natural grouping with sensors/modules below.
  var SYMBOL_CATEGORY_ORDER = [
    'Microcontrollers',
    'Passive', 'Active', 'Discrete', 'Power',
    'Sensor', 'Module', 'Connector',
    'Custom',
  ];

  function symbolMatchesQuery(sym, q) {
    if (!q) return true;
    var hay = ((sym.name || '') + ' ' + (sym.refDesPrefix || '') + ' ' + (sym.category || '')).toLowerCase();
    return hay.indexOf(q) !== -1;
  }

  function groupSymbolsByCategory(symbols) {
    var groups = {};
    symbols.forEach(function (sym) {
      var cat = sym.category || (sym.isCustom ? 'Custom' : 'Other');
      if (!groups[cat]) groups[cat] = [];
      groups[cat].push(sym);
    });
    // Order: predefined order first, then any extras alphabetically.
    var keys = Object.keys(groups);
    keys.sort(function (a, b) {
      var ai = SYMBOL_CATEGORY_ORDER.indexOf(a);
      var bi = SYMBOL_CATEGORY_ORDER.indexOf(b);
      if (ai === -1 && bi === -1) return a.localeCompare(b);
      if (ai === -1) return 1;
      if (bi === -1) return -1;
      return ai - bi;
    });
    return keys.map(function (cat) { return { category: cat, symbols: groups[cat] }; });
  }

  function renderSymbolList() {
    if (!dom.symbolList) return;
    var q = (SYMBOL_LIBRARY_UI.query || '').trim().toLowerCase();
    var filtered = SYMBOL_LIBRARY.filter(function (sym) { return symbolMatchesQuery(sym, q); });
    if (filtered.length === 0) {
      dom.symbolList.innerHTML =
        '<li class="se-symbol-empty">No symbols match "' + escapeHtml(q) + '".</li>';
      return;
    }
    var collapsed = getCollapsedCategories();
    var groups = groupSymbolsByCategory(filtered);
    // When searching, expand every category — otherwise hits inside a
    // collapsed group would be invisible.
    var anyQuery = !!q;

    var html = '';
    groups.forEach(function (g) {
      var isCollapsed = !anyQuery && collapsed.has(g.category);
      html +=
        '<li class="se-symbol-group' + (isCollapsed ? ' is-collapsed' : '') +
            '" data-symbol-group="' + escapeHtml(g.category) + '" role="presentation">' +
          '<i class="fas fa-chevron-down se-symbol-group-chev"></i>' +
          '<span>' + escapeHtml(g.category) + '</span>' +
          '<span class="se-symbol-group-count">' + g.symbols.length + '</span>' +
        '</li>';
      g.symbols.forEach(function (sym) {
        var customBadge = sym.isCustom
          ? '<span class="se-symbol-custom-badge" title="Designed in the Symbol Designer">Custom</span>'
          : '';
        html +=
          '<li class="se-symbol-item' + (sym.isCustom ? ' is-custom' : '') +
              (isCollapsed ? ' is-hidden' : '') +
              '" data-symbol-id="' + escapeHtml(sym.id) + '"' +
              ' data-symbol-category="' + escapeHtml(g.category) + '"' +
              ' role="option" tabindex="0">' +
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
    });
    dom.symbolList.innerHTML = html;
  }

  function wireSymbolSearch() {
    if (!dom.symbolSearch) return;
    dom.symbolSearch.addEventListener('input', function () {
      SYMBOL_LIBRARY_UI.query = dom.symbolSearch.value || '';
      renderSymbolList();
      if (dom.symbolSearchClear) {
        dom.symbolSearchClear.hidden = !SYMBOL_LIBRARY_UI.query;
      }
    });
    if (dom.symbolSearchClear) {
      dom.symbolSearchClear.addEventListener('click', function () {
        dom.symbolSearch.value = '';
        SYMBOL_LIBRARY_UI.query = '';
        dom.symbolSearchClear.hidden = true;
        renderSymbolList();
        dom.symbolSearch.focus();
      });
    }
    // Esc clears the search when the input is focused.
    dom.symbolSearch.addEventListener('keydown', function (e) {
      if (e.key === 'Escape' && SYMBOL_LIBRARY_UI.query) {
        e.preventDefault();
        e.stopPropagation();
        dom.symbolSearch.value = '';
        SYMBOL_LIBRARY_UI.query = '';
        if (dom.symbolSearchClear) dom.symbolSearchClear.hidden = true;
        renderSymbolList();
      }
    });
    // Group-header clicks (delegated) collapse/expand a category.
    if (dom.symbolList) {
      dom.symbolList.addEventListener('click', function (e) {
        var hdr = e.target.closest('.se-symbol-group');
        if (!hdr) return;
        var cat = hdr.dataset.symbolGroup;
        if (!cat) return;
        var set = getCollapsedCategories();
        if (set.has(cat)) set.delete(cat);
        else set.add(cat);
        persistCollapsedCategories();
        renderSymbolList();
      });
    }
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

  // Admin-curated global library — every project's editor pulls these
  // in alongside the project-scoped symbols + the built-in stubs.
  // Failures are silent (offline / API down → editor still works with
  // the local + built-in symbols).
  //
  // De-dup rule: when a library symbol has the same name as a
  // hardcoded built-in (Resistor, Capacitor, etc.), the LIBRARY
  // version wins — the built-in is removed from SYMBOL_LIBRARY so
  // the user doesn't see two entries for "Resistor" in the rail.
  // This way once an admin runs /admin/symbols/ → "Seed built-ins"
  // the editor automatically switches over to the editable library
  // versions and the hardcoded ones disappear.
  async function loadLibrarySymbolsIntoLibrary() {
    try {
      var r = await apiFetch(API + '/api/library/symbols');
      if (!r.ok) return;
      var rows = await r.json();
      if (!Array.isArray(rows)) return;
      // First pass: build the set of names the library claims.
      var libraryNames = {};
      rows.forEach(function (row) {
        if (row && row.name) libraryNames[row.name] = true;
      });
      // Drop any built-in (non-custom, non-library) whose name is now
      // owned by the library. Walk the array backwards so splice
      // doesn't shift the iteration index.
      for (var i = SYMBOL_LIBRARY.length - 1; i >= 0; i--) {
        var existing = SYMBOL_LIBRARY[i];
        if (existing.isCustom || existing.isLibrary) continue;
        if (libraryNames[existing.name]) {
          SYMBOL_LIBRARY.splice(i, 1);
        }
      }
      // Second pass: enliven each library row + append.
      rows.forEach(function (row) {
        var def = symbolDefFromLibrary(row);
        if (def) SYMBOL_LIBRARY.push(def);
      });
    } catch (_) { /* silent */ }
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

  // Capture a tightly-cropped PNG of the schematic content, framed
  // to the bbox of all on-canvas objects with a small margin. Used
  // by the embedded postMessage bridge so the instruction builder
  // can overlay the schematic onto a filmstrip thumbnail / step
  // canvas without showing the empty area around the diagram.
  // ``multiplier`` controls supersampling (default 2 = retina-ish).
  function captureSchematicPng(multiplier) {
    if (!STATE.canvas) return null;
    var mult = multiplier || 2;
    var canvas = STATE.canvas;
    var objs = canvas.getObjects();
    if (!objs || objs.length === 0) {
      // Nothing on the canvas yet — return a transparent dot so the
      // caller has *something* to draw (caller may choose to skip).
      return canvas.toDataURL({
        format: 'png',
        multiplier: 1,
        enableRetinaScaling: false,
      });
    }
    // Compute the bbox of every object in absolute (scene) coords.
    var minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    objs.forEach(function (o) {
      try {
        var r = o.getBoundingRect(true, true);
        if (!r) return;
        if (r.left < minX) minX = r.left;
        if (r.top < minY) minY = r.top;
        if (r.left + r.width > maxX) maxX = r.left + r.width;
        if (r.top + r.height > maxY) maxY = r.top + r.height;
      } catch (_) {}
    });
    if (!isFinite(minX)) return null;
    var margin = 20;

    // Robust crop: ``toDataURL`` only captures pixels that EXIST on
    // the source canvas; objects whose scene coords land outside
    // (0,0)–(canvas.width, canvas.height) get clipped at the canvas
    // edge regardless of what we pass to left/top/width/height.
    // Steps that authored content past the canvas extent then read
    // back cropped on the next snapshot.
    //
    // Fix: temporarily expand the canvas to fit the full content
    // bbox + margin AND shift the viewportTransform so (minX-margin,
    // minY-margin) maps to (0,0). Render to that expanded canvas,
    // capture, then restore the original dimensions + transform.
    var savedW   = canvas.getWidth();
    var savedH   = canvas.getHeight();
    var savedVPT = canvas.viewportTransform;

    var pad = margin;
    var contentW = Math.max(1, Math.ceil((maxX - minX) + pad * 2));
    var contentH = Math.max(1, Math.ceil((maxY - minY) + pad * 2));

    try {
      canvas.setDimensions({ width: contentW, height: contentH });
      // Translate scene-space so (minX - pad, minY - pad) lands at
      // canvas origin. Pure translate; no scale, since multiplier
      // handles supersampling at the toDataURL level.
      canvas.viewportTransform = [
        1, 0, 0, 1,
        -(minX - pad),
        -(minY - pad),
      ];
      canvas.calcOffset();
      return canvas.toDataURL({
        format: 'png',
        multiplier: mult,
        enableRetinaScaling: false,
      });
    } finally {
      canvas.setDimensions({ width: savedW, height: savedH });
      canvas.viewportTransform = savedVPT;
      canvas.calcOffset();
      canvas.requestRenderAll();
    }
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
    c.on('mouse:up', function () {
      if (STATE.isPanning) {
        STATE.isPanning = false;
        if (STATE.canvas) {
          // Restore the pan tool's idle cursor (grab); other tools
          // never enter the pan path so their cursors are unaffected.
          STATE.canvas.defaultCursor = 'grab';
          STATE.canvas.setCursor('grab');
        }
      }
    });
    c.on('object:moving',   onCanvasObjectMoving);
    c.on('object:modified', onCanvasObjectModified);


    syncCanvasDisplaySize();
    window.addEventListener('resize', syncCanvasDisplaySize);

    // Esc aborts a net-in-progress.
    document.addEventListener('keydown', function (e) {
      if (e.key === 'Escape') {
        if (STATE.netDrawingFrom) {
          STATE.netDrawingFrom = null;
          clearNetPinHighlight();
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

  // Toggle the zoom-reset button between zoom-to-fit and 100 %. Lives
  // here (not in the click closure) so the same toggle drives both
  // the local button AND the parent-bridge zoom-reset message from
  // the instruction builder when the schematic is embedded.
  var _zoomResetMode = 'fit';
  function toggleZoomReset() {
    if (_zoomResetMode === 'fit') {
      zoomToFit();
      _zoomResetMode = 'reset';
    } else {
      resetZoom();
      _zoomResetMode = 'fit';
    }
  }

  // Compute the bbox of every placed instance (including each pin's
  // outermost reach) and every net segment, then choose userZoom +
  // pan so that bbox fits comfortably inside the visible canvas.
  // Used on initial schematic load so the user sees their work
  // framed instead of staring at empty scene space.
  function zoomToFit() {
    if (!STATE.canvas) return;
    var minX = Infinity, maxX = -Infinity;
    var minY = Infinity, maxY = -Infinity;
    function bump(x, y) {
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
    // Instances: include each pin's wire-attach point so the frame
    // covers the full pin extents, not just the body centre.
    STATE.graph.instances.forEach(function (inst) {
      var def = symbolDefById(inst.symbolId);
      if (!def) {
        bump(inst.x, inst.y);
        return;
      }
      // Body bbox corners in scene coords.
      var halfW = def.bodyWidth / 2;
      var halfH = def.bodyHeight / 2;
      [[-halfW, -halfH], [halfW, -halfH], [halfW, halfH], [-halfW, halfH]]
        .forEach(function (c) {
          // localToScene wants body-local (top-left origin) coords.
          var lx = c[0] + halfW;
          var ly = c[1] + halfH;
          var p = localToScene(inst, def, lx, ly);
          bump(p.x, p.y);
        });
      // Pin wire-attach extents.
      def.pins.forEach(function (pin) {
        var p = pinScenePos(inst, pin);
        bump(p.x, p.y);
      });
    });
    // Net segments.
    STATE.graph.nets.forEach(function (net) {
      net.segments.forEach(function (s) {
        bump(s.x1, s.y1);
        bump(s.x2, s.y2);
      });
    });
    // Nothing on the canvas? Fall back to a 1× reset so we don't
    // divide by zero. Adding the user's first symbol later will be
    // visible at the default scale.
    if (!isFinite(minX) || !isFinite(maxX) ||
        !isFinite(minY) || !isFinite(maxY) ||
        maxX <= minX || maxY <= minY) {
      resetZoom();
      return;
    }
    var contentW = Math.max(1, maxX - minX);
    var contentH = Math.max(1, maxY - minY);
    var cw = STATE.canvas.getWidth();
    var ch = STATE.canvas.getHeight();
    // Leave a 10 % margin on each side so the bbox isn't kissing the
    // viewport edge.
    var margin = 0.9;
    var sFit = fitScale();
    // The viewport's effective scale is sFit * userZoom, and the
    // scene is rendered at scale × content size. Solve for the
    // userZoom that makes scale × contentDim = margin × viewportDim.
    var zX = (cw * margin) / (sFit * contentW);
    var zY = (ch * margin) / (sFit * contentH);
    var z  = Math.min(zX, zY);
    z = Math.max(0.1, Math.min(8, z));
    STATE.userZoom = z;
    // Centre the content. The viewport transform places SCENE_W/2,
    // SCENE_H/2 at (cw/2 - panX, ch/2 - panY) at scale `s`. We want
    // the content centre (midX, midY) at the viewport centre, so:
    //   midX × s + tx = cw/2
    //   tx = (cw - SCENE_W × s) / 2 + panX
    //   => panX = midX × s - SCENE_W/2 × s = (midX - SCENE_W/2) × s
    // but with the opposite sign so the content moves INTO view.
    var s = sFit * z;
    var midX = (minX + maxX) / 2;
    var midY = (minY + maxY) / 2;
    STATE.panX = -(midX - SCENE_W / 2) * s;
    STATE.panY = -(midY - SCENE_H / 2) * s;
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
    dom.symbolSearch   = document.getElementById('se-symbol-search');
    dom.symbolSearchClear = document.getElementById('se-symbol-search-clear');
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
    // Tool buttons — both the legacy left-rail grid (.se-tool-btn,
    // still in place in case any deployment is mid-rollout) AND the
    // new floating toolbar above the canvas (.se-floating-tool-btn).
    // Both share the same data-tool contract.
    Array.prototype.forEach.call(
      document.querySelectorAll('.se-tool-btn, .se-floating-tool-btn'),
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
    // Search box + category-header collapse handlers.
    wireSymbolSearch();

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
    if (dom.zoomReset) {
      dom.zoomReset.addEventListener('click', function () {
        toggleZoomReset();
        updateZoomPctLabel();
      });
    }
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
        case 'zoom-reset': toggleZoomReset(); postZoom(); break;
        case 'export-csv': exportCSV(); break;
        case 'export-png': exportPNG(); break;
        case 'png-snapshot': {
          // Parent requested a content-framed PNG of the schematic
          // for use as a step thumbnail / canvas overlay. Echo back
          // the requestId so the builder can correlate the reply
          // with the originating step id.
          var url = null;
          try { url = captureSchematicPng(msg.multiplier || 2); } catch (_) {}
          postToParent({
            kr_se_event: 'png-snapshot',
            requestId: msg.requestId || null,
            dataUrl: url,
          });
          break;
        }
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
        // Embedded mode supports a read-only snapshot path: load the
        // schematic, render it to the canvas, broadcast ``ready`` so
        // the parent can request a png-snapshot, then stop. This is
        // what the builder-renderer (used by /projects/view.html) and
        // the project editor's preview surface rely on so non-owner
        // visitors still see the diagram on the public page.
        if (isEmbedded()) {
          try {
            var roSchematic = await fetchSchematic();
            if (roSchematic) {
              STATE.schematic = roSchematic;
              loadGraphFromSchematic(roSchematic);
              await loadCustomSymbolsIntoLibrary();
              await loadLibrarySymbolsIntoLibrary();
              initCanvas();
              renderGraph();
              wireParentBridge();
              showOnly(dom.main);
              requestAnimationFrame(function () {
                zoomToFit();
                postToParent({ kr_se_event: 'ready' });
              });
            } else {
              postToParent({ kr_se_event: 'ready' });
            }
          } catch (_) {
            postToParent({ kr_se_event: 'ready' });
          }
          return;
        }
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
      // the tools-pane symbol list alongside the built-in stubs. Then
      // pull the admin-curated global library on top so promoted
      // symbols are always available too.
      await loadCustomSymbolsIntoLibrary();
      await loadLibrarySymbolsIntoLibrary();

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
          // Open the Symbol Designer on whatever the user already has
          // for this project — do NOT pass ``new=true``; that flag
          // makes the designer create a blank symbol on every load,
          // which silently spammed the library each time the user
          // bounced between the two editors. Users create a new
          // symbol explicitly via the designer's "+ New" button.
          var qs = '?project_id=' + encodeURIComponent(STATE.projectId);
          if (isEmbedded()) qs += '&embedded=1';
          window.location.href = '/projects/symbol/edit.html' + qs;
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
        // Default-frame the schematic to whatever the user has on it
        // so they don't open to empty scene space. Run AFTER the
        // workspace height settles + the graph renders so the canvas
        // dimensions are final.
        zoomToFit();
        // Tell the embedding instruction-builder we're ready to
        // serve png-snapshot requests. The parent listens for this
        // before firing its first capture so it doesn't race the
        // canvas init.
        postToParent({ kr_se_event: 'ready' });
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
