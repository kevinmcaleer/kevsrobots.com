/**
 * Symbol Designer — per-project schematic-symbol editor.
 *
 * Companion to the schematic editor (E2). The editor ships with a small
 * hard-coded ``SYMBOL_LIBRARY`` (Pico, R, C, LED, GND, V+, generic
 * 14-pin IC) and this page grows that library project-by-project: any
 * symbol designed here gets POSTed to
 * ``/api/projects/{id}/symbols`` and is then merged into the editor's
 * library list at runtime (see ``schematic-editor.js`` for the merge
 * call).
 *
 * Layout (see edit.html):
 *
 *   ┌─────────┬─────────────────────────────┬──────────────┐
 *   │ Library │   Toolbox + Canvas          │ Properties   │
 *   │ list    │   (16 px dot-grid)          │ panel        │
 *   │ (240)   │   origin cross at (0,0)     │ (260)        │
 *   └─────────┴─────────────────────────────┴──────────────┘
 *
 * Renderer: Fabric.js v6.4.0 (same engine as the schematic editor and
 * the instruction builder; CDN UMD, exposes ``window.fabric``).
 *
 * Coordinate system: a fixed SCENE_W × SCENE_H logical canvas with the
 * origin (0, 0) at the centre. Pin x/y and body shape coords are stored
 * relative to that origin; rendering offsets them by SCENE_W/2,
 * SCENE_H/2 so the origin cross sits in the middle of the canvas.
 *
 * State persistence: the active symbol's ``{ bodyShapes, pins }`` JSON
 * document is PUT to ``/api/projects/{id}/symbols/{sid}`` via a
 * debounced (500 ms) autosave. Symbol-level fields (name, refDes,
 * description, bom_item_id) go through the same PUT.
 *
 * Tools (toolbox; left-to-right):
 *   - Select (active by default)
 *   - Pin       — implemented (click on a side rail places a pin)
 *   - Net       — stubbed (toast); body-only design for v1
 *   - Name      — stubbed (toast); symbol-level name lives in the
 *                 properties panel for v1
 *   - Label     — stubbed (toast)
 *   - Line      — implemented (click two endpoints)
 *   - Rect      — implemented (click and drag)
 *   - Circle    — stubbed (toast)
 *   - Text      — stubbed (toast)
 *   - Rotate +/-90°, Flip H/V — implemented (apply to whole symbol)
 *
 * Stubbed tools document the v1 trim — re-enabling them is a follow-up.
 *
 * Public surface: an IIFE that auto-invokes on DOMContentLoaded. No
 * external entry point — the page wires everything up via ``init()``.
 */
(function () {
  'use strict';

  // ====================================================================
  // 1. CONSTANTS
  // ====================================================================

  var API = 'https://projects.kevsrobots.com';

  // Logical scene size — small enough to feel like a "symbol editor"
  // canvas (not a schematic page) but generous enough for big ICs.
  var SCENE_W = 640;
  var SCENE_H = 480;

  // 16 px pitch dot grid, same as the schematic editor so the two
  // workspaces share a visual + snap idiom.
  var GRID = 16;
  var AUTOSAVE_MS = 500;

  // Brand colours
  var INK = '#222222';
  var BRAND_RED = '#c8312a';
  var BG = '#ffffff';
  var GRID_DOT = '#cfd3d8';
  var PIN_RAIL = 'rgba(200, 49, 42, 0.06)';   // very faint red strip

  // Side-rail width — clicks within this distance of the canvas edge
  // are interpreted as "place a pin on that side".
  var RAIL_W = 48;

  // Default pin defaults for newly placed pins.
  var DEFAULT_PIN_TYPE = 'I/O';

  // The set of tools that are implemented in v1. Anything else
  // produces a "coming soon" toast.
  var IMPLEMENTED_TOOLS = {
    select: true,
    pin: true,
    line: true,
    rect: true,
    // Side-effect toolbox buttons (apply to selection, then revert).
    'rotate-ccw': true,
    'rotate-cw': true,
    'flip-h': true,
    'flip-v': true,
  };

  // ====================================================================
  // 2. STATE
  // ====================================================================

  var STATE = {
    projectId: null,
    project: null,
    me: null,
    isOwner: false,
    symbols: [],             // [ProjectSymbolResponse]
    bomItems: [],            // [BOMItemResponse] — for the linked-BOM dropdown
    activeSymbolId: null,
    activeSymbol: null,      // { bodyShapes: [], pins: [] } — parsed JSON
    activeTool: 'select',
    selectedShapeId: null,
    selectedPinId: null,
    librarySearch: '',
    saveStatus: 'saved',
    autosaveTimer: null,
    canvas: null,            // fabric.Canvas
    // Transient: drag-to-draw state for the rect tool.
    rectDrawing: null,       // { x0, y0, previewObj }
    // Transient: click-A → click-B state for the line tool.
    linePending: null,       // { x, y, previewObj }
    // Initial query string flags.
    initNew: false,
    initBomItemId: null,
    initSymbolId: null,
  };

  function emptySymbolDoc() {
    return { bodyShapes: [], pins: [] };
  }

  var dom = {};

  // ====================================================================
  // 3. UTILITIES
  // ====================================================================

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

  function nextId(prefix) {
    return prefix + '-' + Math.random().toString(36).slice(2, 10);
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
      } catch (_) { /* fall through */ }
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

  function showToast(msg) {
    if (!dom.toast) return;
    dom.toast.textContent = msg;
    dom.toast.classList.add('is-visible');
    if (showToast._t) clearTimeout(showToast._t);
    showToast._t = setTimeout(function () {
      dom.toast.classList.remove('is-visible');
    }, 1800);
  }

  // ====================================================================
  // 4. COORDINATE TRANSFORMS
  // ====================================================================
  //
  // Symbol-space coords are centred on (0, 0) (the origin cross). Fabric
  // canvas coords are 0..SCENE_W / 0..SCENE_H. ``s2c`` / ``c2s``
  // translate between the two.

  function s2c(x, y) {
    return { x: x + SCENE_W / 2, y: y + SCENE_H / 2 };
  }
  function c2s(x, y) {
    return { x: x - SCENE_W / 2, y: y - SCENE_H / 2 };
  }

  // Snap a Fabric-canvas point to symbol-space grid (round-trip via
  // c2s → snap → s2c so the result stays exactly aligned to the dots).
  function snapCanvasXY(cx, cy) {
    var s = c2s(cx, cy);
    var ss = { x: snap(s.x), y: snap(s.y) };
    return s2c(ss.x, ss.y);
  }

  // Classify a snapped *symbol-space* point as belonging to one of the
  // four side rails (top/right/bottom/left) or to the body interior.
  // Used by the Pin tool — rail clicks place pins on that side.
  function classifyPinSide(sx, sy) {
    var halfW = SCENE_W / 2;
    var halfH = SCENE_H / 2;
    var dLeft   = sx + halfW;             // distance from left edge
    var dRight  = halfW - sx;             // from right edge
    var dTop    = sy + halfH;
    var dBottom = halfH - sy;
    var min = Math.min(dLeft, dRight, dTop, dBottom);
    if (min > RAIL_W) return null;        // interior — body shape, not a pin
    if (min === dLeft)   return 'left';
    if (min === dRight)  return 'right';
    if (min === dTop)    return 'top';
    return 'bottom';
  }

  // Snap a free pin x/y onto the corresponding side's rail axis. Pins
  // on left/right live at the canvas edge (x = ±halfW), pins on
  // top/bottom at y = ±halfH; the perpendicular coord stays snapped.
  function snapPinToSide(sx, sy, side) {
    var halfW = SCENE_W / 2;
    var halfH = SCENE_H / 2;
    sx = snap(sx); sy = snap(sy);
    switch (side) {
      case 'left':   return { x: -halfW + GRID, y: sy };
      case 'right':  return { x:  halfW - GRID, y: sy };
      case 'top':    return { x: sx, y: -halfH + GRID };
      case 'bottom': return { x: sx, y:  halfH - GRID };
    }
    return { x: sx, y: sy };
  }

  // ====================================================================
  // 5. GRID PATTERN + SIDE RAILS
  // ====================================================================

  function buildGridPattern() {
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
  }

  // Faint coloured strips on the four canvas edges — visual cue that
  // those are pin rails. Drawn as plain rects so they participate in
  // the same clear()/render() loop as everything else.
  function buildSideRails() {
    var rails = [];
    var mkRect = function (x, y, w, h) {
      return new fabric.Rect({
        left: x, top: y, width: w, height: h,
        fill: PIN_RAIL,
        stroke: null,
        selectable: false,
        evented: false,
      });
    };
    rails.push(mkRect(0, 0, RAIL_W, SCENE_H));               // left
    rails.push(mkRect(SCENE_W - RAIL_W, 0, RAIL_W, SCENE_H)); // right
    rails.push(mkRect(0, 0, SCENE_W, RAIL_W));               // top
    rails.push(mkRect(0, SCENE_H - RAIL_W, SCENE_W, RAIL_W)); // bottom
    return rails;
  }

  function buildOriginCross() {
    var c = s2c(0, 0);
    return [
      new fabric.Line([c.x - 8, c.y, c.x + 8, c.y], {
        stroke: BRAND_RED, strokeWidth: 1, selectable: false, evented: false,
      }),
      new fabric.Line([c.x, c.y - 8, c.x, c.y + 8], {
        stroke: BRAND_RED, strokeWidth: 1, selectable: false, evented: false,
      }),
    ];
  }

  // ====================================================================
  // 6. SHAPE / PIN MUTATION
  // ====================================================================

  function findShape(id) {
    if (!STATE.activeSymbol) return null;
    for (var i = 0; i < STATE.activeSymbol.bodyShapes.length; i++) {
      if (STATE.activeSymbol.bodyShapes[i].id === id) return STATE.activeSymbol.bodyShapes[i];
    }
    return null;
  }

  function findPin(id) {
    if (!STATE.activeSymbol) return null;
    for (var i = 0; i < STATE.activeSymbol.pins.length; i++) {
      if (STATE.activeSymbol.pins[i].id === id) return STATE.activeSymbol.pins[i];
    }
    return null;
  }

  function autoPinNumber() {
    if (!STATE.activeSymbol) return '1';
    var max = 0;
    STATE.activeSymbol.pins.forEach(function (p) {
      var n = parseInt(p.number, 10);
      if (!isNaN(n) && n > max) max = n;
    });
    return String(max + 1);
  }

  function addPin(sx, sy, side) {
    if (!STATE.activeSymbol) return;
    var snapped = snapPinToSide(sx, sy, side);
    var pin = {
      id: nextId('p'),
      number: autoPinNumber(),
      name: '',
      type: DEFAULT_PIN_TYPE,
      side: side,
      x: snapped.x,
      y: snapped.y,
    };
    STATE.activeSymbol.pins.push(pin);
    STATE.selectedPinId = pin.id;
    STATE.selectedShapeId = null;
    renderCanvas();
    renderPropertiesPanel();
    scheduleAutosave();
  }

  function addRectShape(x0, y0, x1, y1) {
    if (!STATE.activeSymbol) return;
    var sx = Math.min(x0, x1);
    var sy = Math.min(y0, y1);
    var w = Math.abs(x1 - x0);
    var h = Math.abs(y1 - y0);
    if (w < GRID || h < GRID) return;       // ignore zero-width drags
    var shape = {
      id: nextId('s'),
      kind: 'rect',
      x: sx,
      y: sy,
      w: w,
      h: h,
      stroke: INK,
      fill: null,
    };
    STATE.activeSymbol.bodyShapes.push(shape);
    STATE.selectedShapeId = shape.id;
    STATE.selectedPinId = null;
    renderCanvas();
    renderPropertiesPanel();
    scheduleAutosave();
  }

  function addLineShape(x1, y1, x2, y2) {
    if (!STATE.activeSymbol) return;
    if (x1 === x2 && y1 === y2) return;
    var shape = {
      id: nextId('s'),
      kind: 'line',
      x1: x1, y1: y1,
      x2: x2, y2: y2,
      stroke: INK,
    };
    STATE.activeSymbol.bodyShapes.push(shape);
    STATE.selectedShapeId = shape.id;
    STATE.selectedPinId = null;
    renderCanvas();
    renderPropertiesPanel();
    scheduleAutosave();
  }

  function deleteSelected() {
    if (!STATE.activeSymbol) return;
    var mutated = false;
    if (STATE.selectedPinId) {
      STATE.activeSymbol.pins = STATE.activeSymbol.pins.filter(function (p) {
        return p.id !== STATE.selectedPinId;
      });
      STATE.selectedPinId = null;
      mutated = true;
    }
    if (STATE.selectedShapeId) {
      STATE.activeSymbol.bodyShapes = STATE.activeSymbol.bodyShapes.filter(function (s) {
        return s.id !== STATE.selectedShapeId;
      });
      STATE.selectedShapeId = null;
      mutated = true;
    }
    if (mutated) {
      renderCanvas();
      renderPropertiesPanel();
      scheduleAutosave();
    }
  }

  // ====================================================================
  // 7. WHOLE-SYMBOL TRANSFORMS
  // ====================================================================
  //
  // Rotate / flip act on the whole symbol when nothing is selected,
  // and on the selected pin/shape otherwise. For v1 we only implement
  // the whole-symbol path (the selection paths are tagged TODO; the
  // common workflow is "design a symbol, rotate / flip it on the
  // schematic page" — the schematic editor already supports per-
  // instance rotate / flip on its instances).

  function rotateAll(deg) {
    if (!STATE.activeSymbol) return;
    var rad = deg * Math.PI / 180;
    var cos = Math.cos(rad);
    var sin = Math.sin(rad);
    var rot = function (x, y) {
      return { x: snap(x * cos - y * sin), y: snap(x * sin + y * cos) };
    };
    STATE.activeSymbol.pins.forEach(function (p) {
      var r = rot(p.x, p.y);
      p.x = r.x; p.y = r.y;
      // Side flips through the rotation cycle (left → top → right →
      // bottom → left for +90°; the opposite for -90°).
      p.side = rotateSide(p.side, deg);
    });
    STATE.activeSymbol.bodyShapes.forEach(function (s) {
      if (s.kind === 'rect') {
        var corners = [
          rot(s.x, s.y),
          rot(s.x + s.w, s.y + s.h),
        ];
        var nx = Math.min(corners[0].x, corners[1].x);
        var ny = Math.min(corners[0].y, corners[1].y);
        s.x = nx;
        s.y = ny;
        s.w = Math.abs(corners[1].x - corners[0].x);
        s.h = Math.abs(corners[1].y - corners[0].y);
      } else if (s.kind === 'line') {
        var a = rot(s.x1, s.y1);
        var b = rot(s.x2, s.y2);
        s.x1 = a.x; s.y1 = a.y;
        s.x2 = b.x; s.y2 = b.y;
      }
    });
    renderCanvas();
    scheduleAutosave();
  }

  function rotateSide(side, deg) {
    var order = ['left', 'top', 'right', 'bottom'];
    var idx = order.indexOf(side);
    if (idx < 0) return side;
    var steps = (deg / 90) % 4;
    return order[((idx + steps) % 4 + 4) % 4];
  }

  function flipAll(axis) {
    if (!STATE.activeSymbol) return;
    STATE.activeSymbol.pins.forEach(function (p) {
      if (axis === 'h') { p.x = -p.x; if (p.side === 'left') p.side = 'right'; else if (p.side === 'right') p.side = 'left'; }
      else              { p.y = -p.y; if (p.side === 'top') p.side = 'bottom'; else if (p.side === 'bottom') p.side = 'top'; }
    });
    STATE.activeSymbol.bodyShapes.forEach(function (s) {
      if (s.kind === 'rect') {
        if (axis === 'h') s.x = -s.x - s.w;
        else              s.y = -s.y - s.h;
      } else if (s.kind === 'line') {
        if (axis === 'h') { s.x1 = -s.x1; s.x2 = -s.x2; }
        else              { s.y1 = -s.y1; s.y2 = -s.y2; }
      }
    });
    renderCanvas();
    scheduleAutosave();
  }

  // ====================================================================
  // 8. CANVAS RENDER
  // ====================================================================

  function renderCanvas() {
    if (!STATE.canvas) return;
    STATE.canvas.discardActiveObject();
    STATE.canvas.clear();
    applyGridBackground();

    // Side rails first (visual hint for the Pin tool).
    buildSideRails().forEach(function (r) { STATE.canvas.add(r); });

    if (!STATE.activeSymbol) {
      // No symbol — just origin + rails so the empty state reads "this
      // is a canvas but you haven't picked anything yet".
      buildOriginCross().forEach(function (l) { STATE.canvas.add(l); });
      if (dom.emptyState) dom.emptyState.classList.add('is-visible');
      STATE.canvas.requestRenderAll();
      return;
    }
    if (dom.emptyState) dom.emptyState.classList.remove('is-visible');

    // Body shapes
    STATE.activeSymbol.bodyShapes.forEach(function (s) {
      var sel = (s.id === STATE.selectedShapeId);
      if (s.kind === 'rect') {
        var p = s2c(s.x, s.y);
        var rect = new fabric.Rect({
          left: p.x,
          top: p.y,
          width: s.w,
          height: s.h,
          fill: s.fill || 'rgba(255,255,255,0.01)',
          stroke: sel ? BRAND_RED : (s.stroke || INK),
          strokeWidth: sel ? 2.5 : 1.5,
          selectable: false,
          evented: true,
          hoverCursor: 'pointer',
          objectCaching: false,
        });
        rect.data = { kind: 'shape', id: s.id };
        STATE.canvas.add(rect);
      } else if (s.kind === 'line') {
        var a = s2c(s.x1, s.y1);
        var b = s2c(s.x2, s.y2);
        var line = new fabric.Line([a.x, a.y, b.x, b.y], {
          stroke: sel ? BRAND_RED : (s.stroke || INK),
          strokeWidth: sel ? 2.5 : 1.5,
          selectable: false,
          evented: true,
          hoverCursor: 'pointer',
          perPixelTargetFind: true,
          objectCaching: false,
        });
        line.data = { kind: 'shape', id: s.id };
        STATE.canvas.add(line);
      }
    });

    // Pins
    STATE.activeSymbol.pins.forEach(function (pin) {
      drawPin(pin);
    });

    // Origin cross last so it sits on top of body shapes.
    buildOriginCross().forEach(function (l) { STATE.canvas.add(l); });

    // Rect-draw preview (transient)
    if (STATE.rectDrawing && STATE.rectDrawing.previewObj) {
      STATE.canvas.add(STATE.rectDrawing.previewObj);
    }
    if (STATE.linePending && STATE.linePending.previewObj) {
      STATE.canvas.add(STATE.linePending.previewObj);
    }

    STATE.canvas.requestRenderAll();
  }

  function drawPin(pin) {
    var pos = s2c(pin.x, pin.y);
    var stickLen = 12;
    var ex = pos.x, ey = pos.y;
    var labelDx = 0, labelDy = 0;
    var labelAnchorX = 'left';
    var labelAnchorY = 'center';
    switch (pin.side) {
      case 'left':
        ex = pos.x + stickLen;
        labelDx = stickLen + 4;
        labelAnchorX = 'left';
        break;
      case 'right':
        ex = pos.x - stickLen;
        labelDx = -(stickLen + 4);
        labelAnchorX = 'right';
        break;
      case 'top':
        ey = pos.y + stickLen;
        labelDy = stickLen + 4;
        labelAnchorY = 'top';
        break;
      case 'bottom':
        ey = pos.y - stickLen;
        labelDy = -(stickLen + 4);
        labelAnchorY = 'bottom';
        break;
    }
    var sel = (pin.id === STATE.selectedPinId);
    // Stick
    var stick = new fabric.Line([pos.x, pos.y, ex, ey], {
      stroke: sel ? BRAND_RED : INK,
      strokeWidth: sel ? 2 : 1.5,
      selectable: false,
      evented: false,
    });
    STATE.canvas.add(stick);
    // Endpoint dot — the click target.
    var dot = new fabric.Circle({
      left: pos.x,
      top: pos.y,
      radius: 5,
      fill: sel ? BRAND_RED : '#fff',
      stroke: sel ? BRAND_RED : INK,
      strokeWidth: 1.2,
      originX: 'center',
      originY: 'center',
      selectable: false,
      evented: true,
      hoverCursor: 'pointer',
    });
    dot.data = { kind: 'pin', id: pin.id };
    STATE.canvas.add(dot);
    // Label
    var label = (pin.number || '') + (pin.name ? ' ' + pin.name : '');
    if (label.trim()) {
      var txt = new fabric.Text(label, {
        left: ex + labelDx,
        top: ey + labelDy,
        fontSize: 11,
        fontFamily: 'system-ui, -apple-system, "Segoe UI", sans-serif',
        fill: INK,
        originX: labelAnchorX,
        originY: labelAnchorY,
        selectable: false,
        evented: false,
      });
      STATE.canvas.add(txt);
    }
  }

  // ====================================================================
  // 9. CANVAS EVENTS
  // ====================================================================

  function pointerScene(opt) {
    return STATE.canvas.getPointer(opt.e);
  }

  function onCanvasMouseDown(opt) {
    var obj = opt.target;
    var p = pointerScene(opt);
    var snapped = snapCanvasXY(p.x, p.y);
    var sxy = c2s(snapped.x, snapped.y);

    if (!STATE.activeSymbol) return;

    switch (STATE.activeTool) {
      case 'pin': {
        var side = classifyPinSide(sxy.x, sxy.y);
        if (!side) {
          showToast('Click closer to an edge to place a pin.');
          return;
        }
        addPin(sxy.x, sxy.y, side);
        return;
      }
      case 'rect': {
        STATE.rectDrawing = {
          x0: sxy.x,
          y0: sxy.y,
          previewObj: null,
        };
        return;
      }
      case 'line': {
        if (!STATE.linePending) {
          STATE.linePending = { x: sxy.x, y: sxy.y, previewObj: null };
        } else {
          addLineShape(STATE.linePending.x, STATE.linePending.y, sxy.x, sxy.y);
          STATE.linePending = null;
          renderCanvas();
        }
        return;
      }
      case 'select':
      default: {
        if (obj && obj.data) {
          if (obj.data.kind === 'pin') {
            STATE.selectedPinId = obj.data.id;
            STATE.selectedShapeId = null;
          } else if (obj.data.kind === 'shape') {
            STATE.selectedShapeId = obj.data.id;
            STATE.selectedPinId = null;
          }
          renderCanvas();
          renderPropertiesPanel();
          return;
        }
        // Empty-canvas click clears the selection.
        STATE.selectedPinId = null;
        STATE.selectedShapeId = null;
        renderCanvas();
        renderPropertiesPanel();
        return;
      }
    }
  }

  function onCanvasMouseMove(opt) {
    if (!STATE.activeSymbol) return;
    if (STATE.activeTool === 'rect' && STATE.rectDrawing) {
      var p = pointerScene(opt);
      var snapped = snapCanvasXY(p.x, p.y);
      var sxy = c2s(snapped.x, snapped.y);
      var s = STATE.rectDrawing;
      var x = Math.min(s.x0, sxy.x);
      var y = Math.min(s.y0, sxy.y);
      var w = Math.abs(sxy.x - s.x0);
      var h = Math.abs(sxy.y - s.y0);
      var tl = s2c(x, y);
      if (s.previewObj) {
        STATE.canvas.remove(s.previewObj);
        s.previewObj = null;
      }
      s.previewObj = new fabric.Rect({
        left: tl.x,
        top: tl.y,
        width: w,
        height: h,
        fill: 'rgba(200, 49, 42, 0.05)',
        stroke: BRAND_RED,
        strokeWidth: 1.2,
        strokeDashArray: [4, 4],
        selectable: false,
        evented: false,
      });
      STATE.canvas.add(s.previewObj);
      STATE.canvas.requestRenderAll();
    } else if (STATE.activeTool === 'line' && STATE.linePending) {
      var p2 = pointerScene(opt);
      var snapped2 = snapCanvasXY(p2.x, p2.y);
      var lp = STATE.linePending;
      var a = s2c(lp.x, lp.y);
      if (lp.previewObj) {
        STATE.canvas.remove(lp.previewObj);
        lp.previewObj = null;
      }
      lp.previewObj = new fabric.Line([a.x, a.y, snapped2.x, snapped2.y], {
        stroke: BRAND_RED,
        strokeWidth: 1.2,
        strokeDashArray: [4, 4],
        selectable: false,
        evented: false,
      });
      STATE.canvas.add(lp.previewObj);
      STATE.canvas.requestRenderAll();
    }
  }

  function onCanvasMouseUp(opt) {
    if (STATE.activeTool === 'rect' && STATE.rectDrawing) {
      var p = pointerScene(opt);
      var snapped = snapCanvasXY(p.x, p.y);
      var sxy = c2s(snapped.x, snapped.y);
      var s = STATE.rectDrawing;
      STATE.rectDrawing = null;
      addRectShape(s.x0, s.y0, sxy.x, sxy.y);
    }
  }

  // ====================================================================
  // 10. PROPERTIES PANEL
  // ====================================================================

  function renderPropertiesPanel() {
    var hasSymbol = !!STATE.activeSymbol;
    var pin = STATE.selectedPinId ? findPin(STATE.selectedPinId) : null;
    var shape = STATE.selectedShapeId ? findShape(STATE.selectedShapeId) : null;

    // Symbol-level section
    if (dom.symbolSection) {
      dom.symbolSection.classList.toggle('d-none', !hasSymbol);
    }
    if (hasSymbol) {
      var sym = getActiveSymbolRow();
      if (dom.propName && document.activeElement !== dom.propName) {
        dom.propName.value = sym ? (sym.name || '') : '';
      }
      if (dom.propRefDes && document.activeElement !== dom.propRefDes) {
        dom.propRefDes.value = sym ? (sym.ref_des_prefix || 'U') : 'U';
      }
      if (dom.propDesc && document.activeElement !== dom.propDesc) {
        dom.propDesc.value = sym ? (sym.description || '') : '';
      }
      if (dom.propPinCount) {
        dom.propPinCount.textContent = String((STATE.activeSymbol.pins || []).length);
      }
      if (dom.propBom && document.activeElement !== dom.propBom) {
        dom.propBom.value = sym && sym.bom_item_id != null ? String(sym.bom_item_id) : '';
      }
    }

    // Pin section
    if (dom.pinSection) {
      dom.pinSection.classList.toggle('d-none', !pin);
    }
    if (pin) {
      if (dom.pinNumber && document.activeElement !== dom.pinNumber) {
        dom.pinNumber.value = pin.number || '';
      }
      if (dom.pinName && document.activeElement !== dom.pinName) {
        dom.pinName.value = pin.name || '';
      }
      Array.prototype.forEach.call(
        dom.pinTypeRadios,
        function (r) { r.checked = (r.value === (pin.type || DEFAULT_PIN_TYPE)); }
      );
      Array.prototype.forEach.call(
        dom.pinSideRadios,
        function (r) { r.checked = (r.value === pin.side); }
      );
    }

    // Shape section
    if (dom.shapeSection) {
      dom.shapeSection.classList.toggle('d-none', !shape);
    }
    if (shape) {
      // Show only the field group that matches this shape's kind.
      Array.prototype.forEach.call(
        dom.shapeSection.querySelectorAll('[data-shape-kind]'),
        function (el) {
          el.classList.toggle('d-none', el.dataset.shapeKind !== shape.kind);
        }
      );
      if (shape.kind === 'rect') {
        if (dom.shapeRectX) dom.shapeRectX.value = shape.x;
        if (dom.shapeRectY) dom.shapeRectY.value = shape.y;
        if (dom.shapeRectW) dom.shapeRectW.value = shape.w;
        if (dom.shapeRectH) dom.shapeRectH.value = shape.h;
      } else if (shape.kind === 'line') {
        if (dom.shapeLineX1) dom.shapeLineX1.value = shape.x1;
        if (dom.shapeLineY1) dom.shapeLineY1.value = shape.y1;
        if (dom.shapeLineX2) dom.shapeLineX2.value = shape.x2;
        if (dom.shapeLineY2) dom.shapeLineY2.value = shape.y2;
      }
    }
  }

  function getActiveSymbolRow() {
    if (STATE.activeSymbolId == null) return null;
    for (var i = 0; i < STATE.symbols.length; i++) {
      if (STATE.symbols[i].id === STATE.activeSymbolId) return STATE.symbols[i];
    }
    return null;
  }

  function setActiveSymbolField(field, value) {
    var row = getActiveSymbolRow();
    if (!row) return;
    row[field] = value;
    scheduleAutosave();
  }

  // ====================================================================
  // 11. LIBRARY LIST
  // ====================================================================

  function renderLibraryList() {
    if (!dom.libraryList) return;
    var q = (STATE.librarySearch || '').toLowerCase().trim();
    var visible = STATE.symbols.filter(function (s) {
      if (!q) return true;
      return String(s.name || '').toLowerCase().indexOf(q) >= 0;
    });
    if (visible.length === 0) {
      dom.libraryList.innerHTML =
        '<li class="sd-library-empty">' +
        (STATE.symbols.length === 0
          ? 'No symbols yet. Click <strong>+ New</strong> to design one.'
          : 'No symbols match this search.') +
        '</li>';
    } else {
      var html = visible.map(function (s) {
        var isActive = (s.id === STATE.activeSymbolId);
        var pinCount = countSymbolPins(s);
        return '' +
          '<li class="sd-library-item' + (isActive ? ' is-active' : '') + '"' +
          ' data-symbol-id="' + s.id + '" role="option" tabindex="0">' +
            '<span class="sd-library-item-icon">' +
              '<i class="fas fa-microchip"></i>' +
            '</span>' +
            '<span class="sd-library-item-name" title="' + escapeHtml(s.name) + '">' +
              escapeHtml(s.name) +
            '</span>' +
            '<span class="sd-library-item-sub">' +
              escapeHtml(s.ref_des_prefix || 'U') + ' · ' + pinCount + 'p' +
            '</span>' +
          '</li>';
      }).join('');
      dom.libraryList.innerHTML = html;
    }
    // Footer button enable state.
    var hasActive = !!getActiveSymbolRow();
    if (dom.libraryDuplicate) dom.libraryDuplicate.disabled = !hasActive;
    if (dom.libraryDelete) dom.libraryDelete.disabled = !hasActive;
  }

  function countSymbolPins(s) {
    if (!s || !s.symbol_data) return 0;
    try {
      var doc = JSON.parse(s.symbol_data);
      if (doc && Array.isArray(doc.pins)) return doc.pins.length;
    } catch (_) {}
    return 0;
  }

  // ====================================================================
  // 12. BOM-LINK DROPDOWN
  // ====================================================================

  function renderBomDropdown() {
    if (!dom.propBom) return;
    var current = '';
    var row = getActiveSymbolRow();
    if (row && row.bom_item_id != null) current = String(row.bom_item_id);
    var opts = '<option value="">(none)</option>';
    STATE.bomItems.forEach(function (b) {
      opts += '<option value="' + b.id + '"' +
        (String(b.id) === current ? ' selected' : '') + '>' +
        escapeHtml(b.name) + ' (×' + (b.quantity || 1) + ')' +
        '</option>';
    });
    dom.propBom.innerHTML = opts;
  }

  // ====================================================================
  // 13. AUTOSAVE
  // ====================================================================

  function scheduleAutosave() {
    if (STATE.activeSymbolId == null) return;
    if (STATE.autosaveTimer) clearTimeout(STATE.autosaveTimer);
    setSaveStatus('saving');
    STATE.autosaveTimer = setTimeout(function () {
      STATE.autosaveTimer = null;
      doAutosave();
    }, AUTOSAVE_MS);
  }

  async function doAutosave() {
    var row = getActiveSymbolRow();
    if (!row) return;
    var payload = {
      name: row.name,
      ref_des_prefix: row.ref_des_prefix || 'U',
      description: row.description || null,
      bom_item_id: row.bom_item_id == null ? null : row.bom_item_id,
      symbol_data: JSON.stringify(STATE.activeSymbol || emptySymbolDoc()),
    };
    try {
      var resp = await apiFetchWithTermsRetry(
        API + '/api/projects/' + STATE.projectId + '/symbols/' + row.id,
        {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload),
        }
      );
      if (!resp.ok) throw new Error('PUT symbol HTTP ' + resp.status);
      var updated = await resp.json();
      // Sync the in-memory row with the server's response (updated_at etc.).
      for (var k in updated) {
        if (Object.prototype.hasOwnProperty.call(updated, k)) {
          row[k] = updated[k];
        }
      }
      setSaveStatus('saved');
    } catch (e) {
      setSaveStatus('error', 'Save failed');
    }
  }

  // ====================================================================
  // 14. ACTIVATE A SYMBOL
  // ====================================================================

  function activateSymbol(symbolId) {
    var row = null;
    for (var i = 0; i < STATE.symbols.length; i++) {
      if (STATE.symbols[i].id === symbolId) { row = STATE.symbols[i]; break; }
    }
    if (!row) return;
    STATE.activeSymbolId = symbolId;
    STATE.activeSymbol = parseSymbolData(row.symbol_data);
    STATE.selectedPinId = null;
    STATE.selectedShapeId = null;
    setActiveTool('select');
    renderLibraryList();
    renderCanvas();
    renderPropertiesPanel();
  }

  function parseSymbolData(s) {
    if (!s) return emptySymbolDoc();
    try {
      var parsed = JSON.parse(s);
      if (parsed && typeof parsed === 'object') {
        return {
          bodyShapes: Array.isArray(parsed.bodyShapes) ? parsed.bodyShapes : [],
          pins: Array.isArray(parsed.pins) ? parsed.pins : [],
        };
      }
    } catch (_) { /* fall through to empty */ }
    return emptySymbolDoc();
  }

  // ====================================================================
  // 15. CRUD (frontend → backend)
  // ====================================================================

  async function createSymbol(opts) {
    opts = opts || {};
    var body = {
      name: opts.name || 'New symbol',
      ref_des_prefix: opts.ref_des_prefix || 'U',
      description: opts.description || null,
      bom_item_id: opts.bom_item_id == null ? null : opts.bom_item_id,
      symbol_data: JSON.stringify(opts.symbol_data || emptySymbolDoc()),
    };
    setSaveStatus('saving');
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + STATE.projectId + '/symbols',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      }
    );
    if (!resp.ok) {
      setSaveStatus('error', 'Create failed');
      throw new Error('POST symbol HTTP ' + resp.status);
    }
    var created = await resp.json();
    STATE.symbols.push(created);
    activateSymbol(created.id);
    renderLibraryList();
    renderBomDropdown();
    setSaveStatus('saved');
    return created;
  }

  async function duplicateActive() {
    var row = getActiveSymbolRow();
    if (!row) return;
    setSaveStatus('saving');
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + STATE.projectId + '/symbols/' + row.id + '/duplicate',
      { method: 'POST' }
    );
    if (!resp.ok) {
      setSaveStatus('error', 'Duplicate failed');
      return;
    }
    var dup = await resp.json();
    STATE.symbols.push(dup);
    activateSymbol(dup.id);
    renderLibraryList();
    setSaveStatus('saved');
  }

  async function deleteActive() {
    var row = getActiveSymbolRow();
    if (!row) return;
    if (!window.confirm('Delete "' + (row.name || 'this symbol') + '"? This cannot be undone.')) {
      return;
    }
    setSaveStatus('saving');
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + STATE.projectId + '/symbols/' + row.id,
      { method: 'DELETE' }
    );
    if (!resp.ok) {
      setSaveStatus('error', 'Delete failed');
      return;
    }
    STATE.symbols = STATE.symbols.filter(function (s) { return s.id !== row.id; });
    STATE.activeSymbolId = null;
    STATE.activeSymbol = null;
    renderLibraryList();
    renderCanvas();
    renderPropertiesPanel();
    setSaveStatus('saved');
  }

  // ====================================================================
  // 16. TOOL SWITCHING
  // ====================================================================

  function setActiveTool(tool) {
    // Side-effect buttons — apply then revert to select.
    if (tool === 'rotate-ccw') { rotateAll(-90); tool = 'select'; }
    else if (tool === 'rotate-cw') { rotateAll(90); tool = 'select'; }
    else if (tool === 'flip-h') { flipAll('h'); tool = 'select'; }
    else if (tool === 'flip-v') { flipAll('v'); tool = 'select'; }

    if (!IMPLEMENTED_TOOLS[tool]) {
      // Stubbed tools — show a toast and stay on the current tool. The
      // README handoff lists which ones are v1-trimmed; the toast points
      // the user at the implemented alternatives.
      showToast('"' + tool + '" tool is coming soon. Try Pin / Rect / Line for now.');
      // Don't change the active tool — leave the previous one selected.
      paintActiveToolButton();
      return;
    }
    STATE.activeTool = tool;
    // Clear transient drawing state when leaving rect/line.
    if (tool !== 'rect' && STATE.rectDrawing) {
      if (STATE.rectDrawing.previewObj) {
        STATE.canvas.remove(STATE.rectDrawing.previewObj);
      }
      STATE.rectDrawing = null;
    }
    if (tool !== 'line' && STATE.linePending) {
      if (STATE.linePending.previewObj) {
        STATE.canvas.remove(STATE.linePending.previewObj);
      }
      STATE.linePending = null;
    }
    paintActiveToolButton();
    renderCanvas();
  }

  function paintActiveToolButton() {
    Array.prototype.forEach.call(
      document.querySelectorAll('.sd-tool-btn'),
      function (btn) {
        btn.classList.toggle('is-active', btn.dataset.tool === STATE.activeTool);
      }
    );
  }

  // ====================================================================
  // 17. CANVAS INIT
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
    });
    var c = STATE.canvas;
    c.on('mouse:down', onCanvasMouseDown);
    c.on('mouse:move', onCanvasMouseMove);
    c.on('mouse:up',   onCanvasMouseUp);

    syncCanvasDisplaySize();
    window.addEventListener('resize', syncCanvasDisplaySize);

    document.addEventListener('keydown', function (e) {
      if (e.key === 'Escape') {
        if (STATE.rectDrawing) {
          if (STATE.rectDrawing.previewObj) STATE.canvas.remove(STATE.rectDrawing.previewObj);
          STATE.rectDrawing = null;
          renderCanvas();
        } else if (STATE.linePending) {
          if (STATE.linePending.previewObj) STATE.canvas.remove(STATE.linePending.previewObj);
          STATE.linePending = null;
          renderCanvas();
        } else {
          STATE.selectedPinId = null;
          STATE.selectedShapeId = null;
          renderCanvas();
          renderPropertiesPanel();
        }
      } else if (e.key === 'Delete' || e.key === 'Backspace') {
        if (document.activeElement &&
            /input|textarea|select/i.test(document.activeElement.tagName)) {
          return;
        }
        if (STATE.selectedPinId || STATE.selectedShapeId) {
          e.preventDefault();
          deleteSelected();
        }
      }
    });
  }

  function syncCanvasDisplaySize() {
    if (!STATE.canvas || !dom.canvasWrap) return;
    var rect = dom.canvasWrap.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    var scale = Math.min(rect.width / SCENE_W, rect.height / SCENE_H, 1.6);
    if (scale < 0.1) scale = 0.1;
    // Set CSS dimensions only; internal canvas pixels stay at SCENE_W
    // × SCENE_H so scene coords map 1:1 to canvas pixels. Browser
    // scales the rendered bitmap to the CSS box. Do NOT also call
    // setZoom() — combining cssOnly + setZoom double-scales: objects
    // render at (scene × zoom) internal pixels, then the browser
    // scales those down by (CSS/internal), so the visible scene ends
    // up at scale² of intended, tucked into the upper-left and
    // invisible at typical scales (the exact bug the schematic editor
    // hit).
    STATE.canvas.setDimensions(
      { width: SCENE_W * scale, height: SCENE_H * scale },
      { cssOnly: true }
    );
    if (STATE.canvas.getZoom() !== 1) STATE.canvas.setZoom(1);
  }

  // ====================================================================
  // 18. EVENT WIRING
  // ====================================================================

  function cacheDom() {
    dom.workspace      = document.getElementById('sd-workspace');
    dom.titlebar       = document.getElementById('sd-titlebar');
    dom.backLink       = document.getElementById('sd-back-link');
    dom.openSchematic  = document.getElementById('sd-open-schematic');
    dom.projectTitle   = document.getElementById('sd-project-title');
    dom.saveStatus     = document.getElementById('sd-save-status');
    dom.loading        = document.getElementById('sd-loading');
    dom.notOwner       = document.getElementById('sd-not-owner');
    dom.error          = document.getElementById('sd-error');
    dom.errorDetail    = document.getElementById('sd-error-detail');
    dom.main           = document.getElementById('sd-main');

    dom.libraryList    = document.getElementById('sd-library-list');
    dom.libraryNew     = document.getElementById('sd-library-new');
    dom.libraryDuplicate = document.getElementById('sd-library-duplicate');
    dom.libraryDelete  = document.getElementById('sd-library-delete');
    dom.librarySearch  = document.getElementById('sd-library-search');

    dom.canvasCol      = document.getElementById('sd-canvas-col');
    dom.canvasWrap     = document.getElementById('sd-canvas-wrap');
    dom.canvasEl       = document.getElementById('sd-canvas');
    dom.emptyState     = document.getElementById('sd-canvas-empty');
    dom.toast          = document.getElementById('sd-toast');

    dom.symbolSection  = document.getElementById('sd-props-symbol-section');
    dom.propName       = document.getElementById('sd-prop-name');
    dom.propRefDes     = document.getElementById('sd-prop-refdes');
    dom.propDesc       = document.getElementById('sd-prop-desc');
    dom.propPinCount   = document.getElementById('sd-prop-pin-count');
    dom.propBom        = document.getElementById('sd-prop-bom');

    dom.pinSection     = document.getElementById('sd-props-pin-section');
    dom.pinNumber      = document.getElementById('sd-pin-number');
    dom.pinName        = document.getElementById('sd-pin-name');
    dom.pinTypeRadios  = document.querySelectorAll('input[name="sd-pin-type"]');
    dom.pinSideRadios  = document.querySelectorAll('input[name="sd-pin-side"]');

    dom.shapeSection   = document.getElementById('sd-props-shape-section');
    dom.shapeRectX     = document.getElementById('sd-shape-rect-x');
    dom.shapeRectY     = document.getElementById('sd-shape-rect-y');
    dom.shapeRectW     = document.getElementById('sd-shape-rect-w');
    dom.shapeRectH     = document.getElementById('sd-shape-rect-h');
    dom.shapeLineX1    = document.getElementById('sd-shape-line-x1');
    dom.shapeLineY1    = document.getElementById('sd-shape-line-y1');
    dom.shapeLineX2    = document.getElementById('sd-shape-line-x2');
    dom.shapeLineY2    = document.getElementById('sd-shape-line-y2');
    dom.shapeDelete    = document.getElementById('sd-shape-delete');
  }

  function wireUi() {
    // Toolbox
    Array.prototype.forEach.call(
      document.querySelectorAll('.sd-tool-btn'),
      function (btn) {
        btn.addEventListener('click', function () {
          setActiveTool(btn.dataset.tool);
        });
      }
    );

    // Library list
    if (dom.libraryList) {
      dom.libraryList.addEventListener('click', function (e) {
        var li = e.target.closest('.sd-library-item');
        if (!li) return;
        var id = parseInt(li.dataset.symbolId, 10);
        if (!isNaN(id)) activateSymbol(id);
      });
    }
    if (dom.libraryNew) {
      dom.libraryNew.addEventListener('click', function () {
        createSymbol({
          name: 'New symbol',
          ref_des_prefix: 'U',
          bom_item_id: STATE.initBomItemId,
        }).catch(function () { /* setSaveStatus already showed error */ });
      });
    }
    if (dom.libraryDuplicate) {
      dom.libraryDuplicate.addEventListener('click', duplicateActive);
    }
    if (dom.libraryDelete) {
      dom.libraryDelete.addEventListener('click', deleteActive);
    }
    if (dom.librarySearch) {
      dom.librarySearch.addEventListener('input', function () {
        STATE.librarySearch = dom.librarySearch.value;
        renderLibraryList();
      });
    }

    // Symbol-level fields — autosave on blur, debounced commit on input.
    if (dom.propName) {
      dom.propName.addEventListener('input', function () {
        setActiveSymbolField('name', dom.propName.value || 'Untitled');
        renderLibraryList();
      });
    }
    if (dom.propRefDes) {
      dom.propRefDes.addEventListener('input', function () {
        setActiveSymbolField('ref_des_prefix', (dom.propRefDes.value || 'U').slice(0, 8));
      });
    }
    if (dom.propDesc) {
      dom.propDesc.addEventListener('input', function () {
        setActiveSymbolField('description', dom.propDesc.value || null);
      });
    }
    if (dom.propBom) {
      dom.propBom.addEventListener('change', function () {
        var v = dom.propBom.value;
        setActiveSymbolField('bom_item_id', v ? parseInt(v, 10) : null);
      });
    }

    // Pin fields
    if (dom.pinNumber) {
      dom.pinNumber.addEventListener('input', function () {
        var pin = STATE.selectedPinId && findPin(STATE.selectedPinId);
        if (!pin) return;
        pin.number = dom.pinNumber.value || '';
        renderCanvas();
        scheduleAutosave();
      });
    }
    if (dom.pinName) {
      dom.pinName.addEventListener('input', function () {
        var pin = STATE.selectedPinId && findPin(STATE.selectedPinId);
        if (!pin) return;
        pin.name = dom.pinName.value || '';
        renderCanvas();
        scheduleAutosave();
      });
    }
    Array.prototype.forEach.call(dom.pinTypeRadios, function (r) {
      r.addEventListener('change', function () {
        var pin = STATE.selectedPinId && findPin(STATE.selectedPinId);
        if (!pin || !r.checked) return;
        pin.type = r.value;
        scheduleAutosave();
      });
    });
    Array.prototype.forEach.call(dom.pinSideRadios, function (r) {
      r.addEventListener('change', function () {
        var pin = STATE.selectedPinId && findPin(STATE.selectedPinId);
        if (!pin || !r.checked) return;
        pin.side = r.value;
        // Re-snap the pin to its new side's rail axis.
        var snapped = snapPinToSide(pin.x, pin.y, r.value);
        pin.x = snapped.x; pin.y = snapped.y;
        renderCanvas();
        scheduleAutosave();
      });
    });

    // Shape fields — only edit by typing in the inputs.
    function wireShapeNumeric(el, prop) {
      if (!el) return;
      el.addEventListener('change', function () {
        var shape = STATE.selectedShapeId && findShape(STATE.selectedShapeId);
        if (!shape) return;
        var n = snap(parseInt(el.value, 10) || 0);
        shape[prop] = n;
        el.value = n;
        renderCanvas();
        scheduleAutosave();
      });
    }
    wireShapeNumeric(dom.shapeRectX, 'x');
    wireShapeNumeric(dom.shapeRectY, 'y');
    wireShapeNumeric(dom.shapeRectW, 'w');
    wireShapeNumeric(dom.shapeRectH, 'h');
    wireShapeNumeric(dom.shapeLineX1, 'x1');
    wireShapeNumeric(dom.shapeLineY1, 'y1');
    wireShapeNumeric(dom.shapeLineX2, 'x2');
    wireShapeNumeric(dom.shapeLineY2, 'y2');

    if (dom.shapeDelete) {
      dom.shapeDelete.addEventListener('click', deleteSelected);
    }
  }

  // ====================================================================
  // 19. BOOTSTRAP
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
  async function fetchSymbols() {
    var r = await apiFetch(API + '/api/projects/' + STATE.projectId + '/symbols');
    if (!r.ok) throw new Error('Symbols HTTP ' + r.status);
    return r.json();
  }
  async function fetchBom() {
    try {
      var r = await apiFetch(API + '/api/projects/' + STATE.projectId + '/bom');
      if (!r.ok) return [];
      var rows = await r.json();
      return Array.isArray(rows) ? rows : [];
    } catch (_) { return []; }
  }

  async function init() {
    cacheDom();
    var params = new URLSearchParams(window.location.search);
    // Accept both ``project_id`` (the spec's name) and ``id`` (the
    // shorter form the schematic editor uses) so the C1 / E2 deep
    // links can hit this page either way.
    STATE.projectId = params.get('project_id') || params.get('id');
    STATE.initNew = params.get('new') === 'true';
    STATE.initBomItemId = params.get('bom_item_id') ? parseInt(params.get('bom_item_id'), 10) : null;
    STATE.initSymbolId = params.get('symbol_id') ? parseInt(params.get('symbol_id'), 10) : null;

    if (!STATE.projectId) {
      if (dom.errorDetail) dom.errorDetail.textContent = 'Missing ?project_id=… in the URL.';
      showOnly(dom.error);
      return;
    }
    if (dom.backLink) {
      dom.backLink.href = '/projects/instructions/edit.html?id=' +
        encodeURIComponent(STATE.projectId);
    }
    if (dom.openSchematic) {
      dom.openSchematic.href = '/projects/schematic/edit.html?id=' +
        encodeURIComponent(STATE.projectId);
    }

    try {
      var me = await fetchMe();
      STATE.me = me;
      var project = await fetchProject();
      STATE.project = project;
      STATE.isOwner = !!(me && project && project.author_username === me.username);

      if (dom.projectTitle) {
        dom.projectTitle.textContent =
          'Symbol designer · ' + (project.title || 'Untitled');
      }

      if (!STATE.isOwner) {
        showOnly(dom.notOwner);
        return;
      }

      var results = await Promise.all([fetchSymbols(), fetchBom()]);
      STATE.symbols = Array.isArray(results[0]) ? results[0] : [];
      STATE.bomItems = Array.isArray(results[1]) ? results[1] : [];

      wireUi();
      initCanvas();
      renderLibraryList();
      renderBomDropdown();
      setSaveStatus('saved');

      // Initial selection:
      //   - ``?new=true`` (optionally with ``?bom_item_id=…``) → create
      //     a fresh symbol pre-linked to that BOM row.
      //   - ``?symbol_id=…`` → open that symbol.
      //   - Otherwise → open the first symbol in the list, if any.
      if (STATE.initNew) {
        await createSymbol({
          name: 'New symbol',
          ref_des_prefix: 'U',
          bom_item_id: STATE.initBomItemId,
        });
      } else if (STATE.initSymbolId) {
        activateSymbol(STATE.initSymbolId);
      } else if (STATE.symbols.length > 0) {
        activateSymbol(STATE.symbols[0].id);
      } else {
        renderCanvas();
        renderPropertiesPanel();
      }

      showOnly(dom.main);
      setTimeout(syncCanvasDisplaySize, 50);
    } catch (e) {
      if (e && e.status === 404 && dom.errorDetail) {
        dom.errorDetail.textContent = 'Project not found.';
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
