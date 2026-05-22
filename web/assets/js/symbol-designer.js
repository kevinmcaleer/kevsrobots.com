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
    pan: true,
    pin: true,
    line: true,
    rect: true,
    text: true,
    name: true,
    label: true,
    // Side-effect toolbox buttons (apply to selection, then revert).
    'rotate-ccw': true,
    'rotate-cw': true,
    'flip-h': true,
    'flip-v': true,
  };

  // Monospace stack with sensible PCB-style fallbacks. No external
  // webfont — Courier New is universally available; ui-monospace +
  // SF Mono pick up the platform's native programmer font on Mac.
  var TEXT_FONT_STACK = '"Courier New", "Source Code Pro", ui-monospace, "SF Mono", Menlo, Consolas, monospace';

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

  // Default-direction heuristic: a pin in the right half points right,
  // left half points left, etc. Clicks on the central body get a
  // sensible "outward" direction based on which quadrant is closest
  // to the edge.
  function defaultRotationFor(sx, sy) {
    var halfW = SCENE_W / 2;
    var halfH = SCENE_H / 2;
    var dLeft   = sx + halfW;
    var dRight  = halfW - sx;
    var dTop    = sy + halfH;
    var dBottom = halfH - sy;
    var min = Math.min(dLeft, dRight, dTop, dBottom);
    if (min === dRight)  return 0;     // points right
    if (min === dBottom) return 90;    // points down
    if (min === dLeft)   return 180;   // points left
    return 270;                        // points up
  }

  function addPin(sx, sy, rotation) {
    if (!STATE.activeSymbol) return;
    var grid = { x: snap(sx), y: snap(sy) };
    var rot = normaliseRotation(rotation);
    var pin = {
      id: nextId('p'),
      number: autoPinNumber(),
      name: '',
      type: DEFAULT_PIN_TYPE,
      rotation: rot,
      // Legacy `side` written alongside the new `rotation` so the
      // schematic editor's symbolDefFromCustom (which still reads
      // `side`) keeps working without a coordinated update.
      side: rotationToSide(rot),
      x: grid.x,
      y: grid.y,
    };
    STATE.activeSymbol.pins.push(pin);
    STATE.selectedPinId = pin.id;
    STATE.selectedShapeId = null;
    renderCanvas();
    renderPropertiesPanel();
    scheduleAutosave();
  }

  function normaliseRotation(r) {
    var n = parseInt(r, 10);
    if (isNaN(n)) return 0;
    n = ((n % 360) + 360) % 360;
    // Snap to one of the four cardinal directions.
    if (n < 45 || n >= 315) return 0;
    if (n < 135) return 90;
    if (n < 225) return 180;
    return 270;
  }

  function rotationToSide(rot) {
    switch (normaliseRotation(rot)) {
      case 0:   return 'right';
      case 90:  return 'bottom';
      case 180: return 'left';
      case 270: return 'top';
    }
    return 'right';
  }

  function sideToRotation(side) {
    switch (side) {
      case 'right':  return 0;
      case 'bottom': return 90;
      case 'left':   return 180;
      case 'top':    return 270;
    }
    return 0;
  }

  function pinRotation(pin) {
    // Tolerate legacy pins that only have `side`.
    if (pin && typeof pin.rotation === 'number') return normaliseRotation(pin.rotation);
    if (pin && pin.side) return sideToRotation(pin.side);
    return 0;
  }

  function setPinRotation(pin, rot) {
    pin.rotation = normaliseRotation(rot);
    pin.side = rotationToSide(pin.rotation);
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

  function addTextShape(sx, sy, text, kind) {
    if (!STATE.activeSymbol) return;
    var role = kind || 'text';
    var shape = {
      id: nextId('s'),
      kind: 'text',
      role: role,                    // 'text' | 'name' | 'label'
      x: snap(sx),
      y: snap(sy),
      text: text || 'Text',
      fontSize: role === 'name' ? 14 : 12,
      bold: role === 'name',
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

  // Duplicate the currently selected pin(s) and/or shape(s). Clones
  // are offset by one grid cell (down + right) so they aren't stacked
  // on top of the originals and become the new selection so the user
  // can press Cmd+D again to keep cloning. Handles both single-select
  // (STATE.selectedPinId / selectedShapeId) and Fabric's multi-select
  // ActiveSelection.
  function duplicateSelected() {
    if (!STATE.activeSymbol) return;
    var offset = GRID;

    // Build a list of {kind, id} pairs covering whatever is selected.
    var selection = [];
    var active = STATE.canvas && STATE.canvas.getActiveObjects
      ? (STATE.canvas.getActiveObjects() || [])
      : [];
    active.forEach(function (o) {
      if (!o || !o.data) return;
      if (o.data.kind === 'pin')   selection.push({ kind: 'pin',   id: o.data.id });
      else if (o.data.kind === 'shape') selection.push({ kind: 'shape', id: o.data.id });
    });
    // Fall back to STATE single-selection if Fabric had nothing.
    if (selection.length === 0) {
      if (STATE.selectedPinId)   selection.push({ kind: 'pin',   id: STATE.selectedPinId });
      if (STATE.selectedShapeId) selection.push({ kind: 'shape', id: STATE.selectedShapeId });
    }
    if (selection.length === 0) return;

    // Deduplicate (an item could theoretically appear twice if Fabric
    // and STATE disagree about what's selected).
    var seen = {};
    selection = selection.filter(function (s) {
      var k = s.kind + ':' + s.id;
      if (seen[k]) return false;
      seen[k] = true;
      return true;
    });

    var newIds = [];
    selection.forEach(function (sel) {
      if (sel.kind === 'pin') {
        var src = findPin(sel.id);
        if (!src) return;
        var clone = {
          id: nextId('p'),
          number: autoPinNumber(),
          name: src.name || '',
          type: src.type,
          rotation: src.rotation,
          side: src.side,
          x: src.x + offset,
          y: src.y + offset,
        };
        STATE.activeSymbol.pins.push(clone);
        newIds.push({ kind: 'pin', id: clone.id });
      } else if (sel.kind === 'shape') {
        var s = findShape(sel.id);
        if (!s) return;
        var c = JSON.parse(JSON.stringify(s));
        c.id = nextId('s');
        if (c.kind === 'rect' || c.kind === 'text') {
          c.x = (c.x || 0) + offset;
          c.y = (c.y || 0) + offset;
        } else if (c.kind === 'line') {
          c.x1 += offset; c.y1 += offset;
          c.x2 += offset; c.y2 += offset;
        }
        STATE.activeSymbol.bodyShapes.push(c);
        newIds.push({ kind: 'shape', id: c.id });
      }
    });
    if (newIds.length === 0) return;

    // Make the new clones the active selection so chained Cmd+D keeps
    // duplicating the fresh items (and the user sees what landed).
    if (newIds.length === 1) {
      if (newIds[0].kind === 'pin')   { STATE.selectedPinId = newIds[0].id;   STATE.selectedShapeId = null; }
      else                            { STATE.selectedShapeId = newIds[0].id; STATE.selectedPinId = null; }
    } else {
      STATE.selectedPinId = null;
      STATE.selectedShapeId = null;
    }
    if (STATE.canvas && STATE.canvas.discardActiveObject) {
      STATE.canvas.discardActiveObject();
    }
    renderCanvas();
    restoreActiveSelection(newIds);
    renderPropertiesPanel();
    scheduleAutosave();
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

  // Rotate or flip a single shape (rect or line) around its centre.
  // Used by the Transform section when a shape is selected.
  // Walk every Fabric object on the canvas and recolour it based on
  // whether Fabric considers it selected. Called from selection:*
  // event handlers AND at the tail of renderCanvas() so that any
  // surviving selection keeps its red highlight after a full rebuild.
  //
  // Colour rules mirror what renderCanvas() draws fresh:
  //   - Pin dot (circle): stroke + fill toggle (white → red)
  //   - Pin line/label   (pin-part): stroke / fill follow parent pin
  //   - Shape rect / line: stroke toggle
  //   - Shape text       (IText): fill toggle
  function applyFabricSelectionStyle() {
    if (!STATE.canvas) return;
    var active = STATE.canvas.getActiveObjects
      ? (STATE.canvas.getActiveObjects() || [])
      : [];
    var selectedPinIds = {};
    var selectedShapeIds = {};
    active.forEach(function (o) {
      if (!o || !o.data) return;
      if (o.data.kind === 'pin') selectedPinIds[o.data.id] = true;
      else if (o.data.kind === 'shape') selectedShapeIds[o.data.id] = true;
    });
    // Also include STATE's single-selection so single-click-without-
    // -Fabric-active (e.g. a programmatic pin-select while Fabric's
    // active object got discarded by a render) still paints red.
    if (STATE.selectedPinId)   selectedPinIds[STATE.selectedPinId] = true;
    if (STATE.selectedShapeId) selectedShapeIds[STATE.selectedShapeId] = true;
    STATE.canvas.getObjects().forEach(function (o) {
      if (!o || !o.data) return;
      var k = o.data.kind;
      var isSel = false;
      if (k === 'pin' || k === 'pin-part') {
        // Pin-parts share the dot's id and inherit its selection.
        isSel = !!selectedPinIds[o.data.id];
      } else if (k === 'shape') {
        isSel = !!selectedShapeIds[o.data.id];
      } else {
        return;
      }
      colourFabricObject(o, isSel);
    });
    STATE.canvas.requestRenderAll();
  }

  function colourFabricObject(o, isSel) {
    var redStroke = BRAND_RED;
    var blackStroke = INK;
    // Pin dot — the only filled circle on the canvas tagged 'pin'.
    if (o.data.kind === 'pin' && o.type === 'circle') {
      o.set({
        fill: isSel ? BRAND_RED : '#fff',
        stroke: isSel ? redStroke : blackStroke,
        strokeWidth: isSel ? 2 : 1.4,
      });
      return;
    }
    if (o.data.kind === 'pin-part') {
      if (o.type === 'line') {
        o.set({
          stroke: isSel ? redStroke : blackStroke,
          strokeWidth: isSel ? 2 : 1.5,
        });
      } else {
        // Pin label (text) — fill recolours.
        o.set({ fill: isSel ? redStroke : blackStroke });
      }
      return;
    }
    if (o.data.kind === 'shape') {
      // Branch on Fabric type since shapes cover rect / line / text.
      if (o.type === 'line' || o.type === 'rect') {
        o.set({
          stroke: isSel ? redStroke : blackStroke,
          strokeWidth: isSel ? 2.5 : 1.5,
        });
      } else if (o.type === 'i-text' || o.type === 'text' || o.type === 'IText') {
        o.set({ fill: isSel ? redStroke : blackStroke });
      }
    }
  }

  // After renderCanvas() rebuilds every Fabric object, the old
  // ActiveSelection's child references are stale. This helper takes a
  // list of {kind, id} entries (snapshotted before the render) and
  // assembles a fresh ActiveSelection over the matching new objects.
  // Single-id input falls back to setActiveObject so we don't wrap a
  // lone item in an AS unnecessarily.
  function restoreActiveSelection(idList) {
    if (!STATE.canvas || !idList || idList.length === 0) return;
    // Pins are rendered as multiple Fabric objects (line + circle
    // terminator + label) all tagged with the SAME pin id, but only
    // the circle has kind 'pin' (the line + label are 'pin-part').
    // Use the 'pin' object as the selectable handle for a pin, and
    // the 'shape' object for shapes.
    var byKey = {};
    STATE.canvas.getObjects().forEach(function (o) {
      if (!o.data) return;
      // 'pin' is the draggable dot; 'shape' is the rect / line / text.
      if (o.data.kind !== 'pin' && o.data.kind !== 'shape') return;
      byKey[o.data.kind + ':' + o.data.id] = o;
    });
    var fresh = [];
    idList.forEach(function (entry) {
      var hit = byKey[entry.kind + ':' + entry.id];
      if (hit) fresh.push(hit);
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

  // Mutate `shape` in-place to apply rotate-cw / rotate-ccw / flip-h /
  // flip-v about the shape's own centre or midpoint. No side effects —
  // callers handle renderCanvas + autosave (so batch callers like
  // the multi-select transform path don't re-render per item).
  function rotateOrFlipShapeInPlace(shape, kind) {
    if (!shape) return;
    if (shape.kind === 'rect') {
      if (kind === 'rotate-cw' || kind === 'rotate-ccw') {
        var cx = shape.x + shape.w / 2;
        var cy = shape.y + shape.h / 2;
        var nw = shape.h;
        var nh = shape.w;
        shape.w = nw;
        shape.h = nh;
        shape.x = snap(cx - nw / 2);
        shape.y = snap(cy - nh / 2);
      }
      // Flip on a rectangle is a visual no-op (rects are symmetric).
    } else if (shape.kind === 'line') {
      var midX = (shape.x1 + shape.x2) / 2;
      var midY = (shape.y1 + shape.y2) / 2;
      if (kind === 'rotate-cw' || kind === 'rotate-ccw') {
        var sign = kind === 'rotate-cw' ? 1 : -1;
        var dx1 = shape.x1 - midX, dy1 = shape.y1 - midY;
        var dx2 = shape.x2 - midX, dy2 = shape.y2 - midY;
        shape.x1 = snap(midX + (-sign * dy1));
        shape.y1 = snap(midY + (sign * dx1));
        shape.x2 = snap(midX + (-sign * dy2));
        shape.y2 = snap(midY + (sign * dx2));
      } else if (kind === 'flip-h') {
        shape.x1 = snap(2 * midX - shape.x1);
        shape.x2 = snap(2 * midX - shape.x2);
      } else if (kind === 'flip-v') {
        shape.y1 = snap(2 * midY - shape.y1);
        shape.y2 = snap(2 * midY - shape.y2);
      }
    } else if (shape.kind === 'text') {
      // Text shapes have no geometry to rotate yet (they're rendered
      // axis-aligned); flips are a visual no-op. Leaving them as a
      // no-op is fine until we support rotated/mirrored text.
    }
  }

  // Single-item entry point (used when one shape is selected). Wraps
  // rotateOrFlipShapeInPlace + the per-action side effects.
  function rotateOrFlipShape(shape, kind) {
    if (!shape) return;
    rotateOrFlipShapeInPlace(shape, kind);
    renderCanvas();
    renderPropertiesPanel();
    scheduleAutosave();
  }

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

    // (Side rails removed — pins can be placed anywhere now, so the
    // rail strips are misleading. The grid alone is enough of a visual
    // anchor for placement.)

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
          // Resizable: corner + side handles drag the rect's extents.
          // Rotation is locked because the Transform pane handles 90°
          // increments; freehand rotation isn't useful for a grid-snap
          // body shape.
          selectable: true,
          hasControls: true,
          hasBorders: true,
          lockRotation: true,
          lockScalingFlip: true,
          cornerColor: BRAND_RED,
          cornerStrokeColor: BRAND_RED,
          cornerStyle: 'rect',
          cornerSize: 8,
          transparentCorners: false,
          borderColor: BRAND_RED,
          // No rotation handle when there's no rotation to do.
          hoverCursor: 'move',
          objectCaching: false,
        });
        rect.setControlsVisibility({ mtr: false });
        rect.data = { kind: 'shape', id: s.id };
        STATE.canvas.add(rect);
      } else if (s.kind === 'text') {
        // Inline-editable text shape. Monospace by default — Courier
        // New base with platform programmer-font + Source Code Pro
        // fallbacks for a PCB-silkscreen feel. Double-click enters
        // edit mode.
        var tp = s2c(s.x, s.y);
        var ftxt = new fabric.IText(s.text || '', {
          left: tp.x,
          top: tp.y,
          fontSize: s.fontSize || 12,
          fontFamily: TEXT_FONT_STACK,
          fontWeight: s.bold ? 'bold' : 'normal',
          fill: sel ? BRAND_RED : (s.stroke || INK),
          // Editable + selectable + draggable. Lock scale + rotation
          // — font size changes happen via the properties panel.
          editable: true,
          selectable: true,
          hasControls: false,
          hasBorders: true,
          lockRotation: true,
          lockScalingX: true,
          lockScalingY: true,
          lockScalingFlip: true,
          borderColor: BRAND_RED,
          objectCaching: false,
        });
        ftxt.data = { kind: 'shape', id: s.id };
        // Sync the text back to STATE on edit exit so renderCanvas
        // rebuilds with the new string.
        ftxt.on('editing:exited', function () {
          var st = findShape(s.id);
          if (st) {
            st.text = ftxt.text;
            scheduleAutosave();
            // Sync the side-pane input if this text shape is the
            // currently-selected one — keeps the two edit paths in
            // step.
            if (STATE.selectedShapeId === s.id && dom.shapeTextValue) {
              dom.shapeTextValue.value = st.text;
            }
          }
        });
        // Selecting via double-click → edit mode also counts as a
        // shape selection so the properties pane updates.
        ftxt.on('selected', function () {
          STATE.selectedShapeId = s.id;
          STATE.selectedPinId = null;
          renderPropertiesPanel();
        });
        STATE.canvas.add(ftxt);
      } else if (s.kind === 'line') {
        var a = s2c(s.x1, s.y1);
        var b = s2c(s.x2, s.y2);
        var line = new fabric.Line([a.x, a.y, b.x, b.y], {
          stroke: sel ? BRAND_RED : (s.stroke || INK),
          strokeWidth: sel ? 2.5 : 1.5,
          selectable: true,
          // Lines get end-grip controls — fabric's mt/ml/mr/mb keep
          // the endpoints draggable so users can lengthen / re-aim
          // the line after placing it.
          hasControls: true,
          hasBorders: true,
          lockRotation: true,
          lockScalingFlip: true,
          cornerColor: BRAND_RED,
          cornerStrokeColor: BRAND_RED,
          cornerStyle: 'rect',
          cornerSize: 8,
          transparentCorners: false,
          borderColor: BRAND_RED,
          hoverCursor: 'move',
          perPixelTargetFind: true,
          objectCaching: false,
        });
        line.setControlsVisibility({ mtr: false });
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

    // Re-apply Fabric's selection-derived red highlight in case an
    // ActiveSelection survived this render (the transform flow drops
    // and re-creates it inside restoreActiveSelection, which runs
    // after renderCanvas — but selection-via-click paths leave the
    // AS in place and we still want fresh objects to reflect it).
    applyFabricSelectionStyle();

    STATE.canvas.requestRenderAll();
  }

  function drawPin(pin) {
    // Fusion-360-ish pin: line from the pin centre extending 2 GRID
    // units in the pin's direction, a small circle terminator at the
    // outer end of the line (where wires connect), and the pin name +
    // number positioned past the line's OUTER endpoint (relative to
    // the symbol's origin) — matches the schematic editor's label
    // placement so symbols read the same in both views.
    var pos = s2c(pin.x, pin.y);
    var sel = (pin.id === STATE.selectedPinId);
    var lineLen = 2 * GRID;
    var rot = pinRotation(pin);
    var dx = 0, dy = 0;
    switch (rot) {
      case 0:    dx = 1;  break;   // points right
      case 90:   dy = 1;  break;   // points down
      case 180:  dx = -1; break;   // points left
      case 270:  dy = -1; break;   // points up
    }
    var endX = pos.x + dx * lineLen;
    var endY = pos.y + dy * lineLen;
    // Label position derived from which endpoint of the pin's line
    // (anchor or terminator) is further from the SCENE origin. The
    // label sits past that endpoint, extending outward — same rule as
    // the schematic editor's buildInstanceFabric, so a symbol's pin
    // names render identically in the designer and the schematic.
    var horizontal = (rot === 0 || rot === 180);
    var labelPad = 6;
    var labelAnchorX = 'center';
    var labelAnchorY = 'center';
    var labelSceneX = pin.x, labelSceneY = pin.y;
    if (horizontal) {
      var termSceneX = pin.x + dx * lineLen;
      var outerX = (Math.abs(pin.x) >= Math.abs(termSceneX)) ? pin.x : termSceneX;
      var dirX = (outerX >= 0) ? 1 : -1;
      labelSceneX = outerX + dirX * labelPad;
      labelSceneY = pin.y;
      labelAnchorX = (dirX > 0) ? 'left' : 'right';
    } else {
      var termSceneY = pin.y + dy * lineLen;
      var outerY = (Math.abs(pin.y) >= Math.abs(termSceneY)) ? pin.y : termSceneY;
      var dirY = (outerY >= 0) ? 1 : -1;
      labelSceneX = pin.x;
      labelSceneY = outerY + dirY * labelPad;
      labelAnchorY = (dirY > 0) ? 'top' : 'bottom';
    }
    var labelCanvas = s2c(labelSceneX, labelSceneY);

    // Line — pin's wire-attachment direction. Tagged with the pin id
    // so the drag handler can find + move it as the user drags the
    // dot terminator.
    var stick = new fabric.Line([pos.x, pos.y, endX, endY], {
      stroke: sel ? BRAND_RED : INK,
      strokeWidth: sel ? 2 : 1.5,
      selectable: false,
      evented: false,
    });
    stick.data = { kind: 'pin-part', id: pin.id };
    STATE.canvas.add(stick);

    // Circle terminator at the OUTER end of the line — the actual
    // wire-attachment point AND the drag handle. The line + label
    // (added below) track its motion via the canvas object:moving
    // listener so the whole pin moves as one.
    var dot = new fabric.Circle({
      left: endX,
      top: endY,
      radius: 4,
      fill: sel ? BRAND_RED : '#fff',
      stroke: sel ? BRAND_RED : INK,
      strokeWidth: 1.4,
      originX: 'center',
      originY: 'center',
      // Draggable handle: no scale controls, no rotate handle, locked
      // scaling so the user can only translate.
      selectable: true,
      hasControls: false,
      hasBorders: false,
      lockRotation: true,
      lockScalingX: true,
      lockScalingY: true,
      lockScalingFlip: true,
      hoverCursor: 'move',
    });
    dot.data = { kind: 'pin', id: pin.id };
    // Stash a snapshot of the drag-handle's starting position so the
    // object:moving handler can compute per-tick deltas to push into
    // the linked line + label.
    dot._lastLeft = dot.left;
    dot._lastTop = dot.top;
    STATE.canvas.add(dot);

    // Label past the circle: "P$1 GP0" style. Pin number first (so
    // it lines up with the schematic editor's connection labels),
    // followed by an optional user-set name.
    var label = (pin.number ? 'P$' + pin.number : '') +
                (pin.name ? ' ' + pin.name : '');
    if (label.trim()) {
      var txt = new fabric.Text(label, {
        left: labelCanvas.x,
        top: labelCanvas.y,
        fontSize: 11,
        fontFamily: 'system-ui, -apple-system, "Segoe UI", sans-serif',
        fill: INK,
        originX: labelAnchorX,
        originY: labelAnchorY,
        selectable: false,
        evented: false,
      });
      txt.data = { kind: 'pin-part', id: pin.id };
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
        // Pins go anywhere now — no side-rail gate. Pick a sensible
        // default direction based on which body quadrant the click
        // landed in so most placements come out facing outward.
        addPin(sxy.x, sxy.y, defaultRotationFor(sxy.x, sxy.y));
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
      case 'text':
      case 'name':
      case 'label': {
        // Seed text per tool variant so each places something
        // meaningful — user edits inline by double-clicking the
        // text afterwards.
        var seed = STATE.activeTool === 'name'  ? '<name>'
                 : STATE.activeTool === 'label' ? 'Label'
                 :                                'Text';
        addTextShape(sxy.x, sxy.y, seed, STATE.activeTool);
        // Revert to select so the user can immediately position /
        // edit the new text rather than dropping another one on
        // the next click.
        setActiveTool('select');
        return;
      }
      case 'pan': {
        // Pan tool — empty-canvas drag pans the viewport. Track the
        // starting screen pos + base pan so each mouse:move computes
        // an absolute delta (no drift).
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
          // Don't renderCanvas here — that would wipe the very object
          // the user just mouse-down'd on and Fabric would lose its
          // drag-initiation target (the same bug the schematic editor
          // hit). Instead, repaint colours directly so the new
          // selection turns red and the previous one drops back to
          // black — we can't rely on Fabric's selection:updated event
          // to fire reliably for every click-to-click swap.
          applyFabricSelectionStyle();
          renderPropertiesPanel();
          return;
        }
        // ActiveSelection (multi-select) has no .data — the user is
        // about to drag the whole group. Don't touch the canvas; let
        // Fabric handle the drag.
        if (obj &&
            (obj.type === 'activeselection' || obj.type === 'activeSelection')) {
          return;
        }
        // Empty-canvas click — Fabric handles the rubber-band lasso
        // itself (canvas.selection is true in select mode). Clear the
        // previous selection so the properties pane updates AND the
        // formerly-selected item drops back to black.
        var hadSel = !!(STATE.selectedPinId || STATE.selectedShapeId);
        STATE.selectedPinId = null;
        STATE.selectedShapeId = null;
        if (hadSel) {
          applyFabricSelectionStyle();
          renderPropertiesPanel();
        }
        return;
      }
    }
  }

  function onCanvasMouseMove(opt) {
    // Pan-in-progress overrides every tool's preview rendering so
    // panning works regardless of which tool is selected (handy if a
    // user starts to pan while a draw tool is active and decides to
    // navigate before placing).
    if (STATE.isPanning && opt.e) {
      var dx = opt.e.clientX - STATE.panStartScreenX;
      var dy = opt.e.clientY - STATE.panStartScreenY;
      STATE.panX = STATE.panStartPanX + dx;
      STATE.panY = STATE.panStartPanY + dy;
      applyViewport();
      return;
    }
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
    if (STATE.isPanning) {
      STATE.isPanning = false;
      if (STATE.canvas) {
        // Pan tool idle cursor; setActiveTool puts the right cursor
        // on for every other tool.
        STATE.canvas.defaultCursor = 'grab';
        STATE.canvas.setCursor('grab');
      }
      return;
    }
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

    // Transform-section target hint + button enabled state.
    if (dom.transformTarget) {
      if (pin)        dom.transformTarget.textContent = '— selected pin';
      else if (shape) dom.transformTarget.textContent = '— selected shape';
      else if (hasSymbol) dom.transformTarget.textContent = '— whole symbol';
      else            dom.transformTarget.textContent = '— select an item';
    }
    var canTransform = !!(pin || shape || hasSymbol);
    [dom.transformRotateCw, dom.transformRotateCcw, dom.transformFlipH, dom.transformFlipV].forEach(function (b) {
      if (b) {
        b.disabled = !canTransform;
        b.classList.toggle('is-disabled', !canTransform);
      }
    });

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
    if (!pin) {
      // Selection cleared / not a pin — reset the tracker so the
      // next pin click is treated as a fresh selection (force-update).
      STATE._lastRenderedPinId = null;
    }
    if (pin) {
      // Track the last-rendered pin id. When the user clicks a
      // DIFFERENT pin we force-overwrite the input values even if
      // focus is currently in one of them — otherwise the previous
      // pin's number/name stays in the field and the user thinks
      // the selection didn't update. The activeElement check still
      // protects in-progress typing across autosave-triggered
      // re-renders (which keep the same selectedPinId).
      var pinChanged = STATE._lastRenderedPinId !== pin.id;
      STATE._lastRenderedPinId = pin.id;
      // Drop focus from a stale field on a pin swap so subsequent
      // keystrokes don't land in the wrong input.
      if (pinChanged && document.activeElement &&
          (document.activeElement === dom.pinNumber ||
           document.activeElement === dom.pinName)) {
        document.activeElement.blur();
      }
      if (dom.pinNumber && (pinChanged || document.activeElement !== dom.pinNumber)) {
        dom.pinNumber.value = pin.number || '';
      }
      if (dom.pinName && (pinChanged || document.activeElement !== dom.pinName)) {
        dom.pinName.value = pin.name || '';
      }
      Array.prototype.forEach.call(
        dom.pinTypeRadios,
        function (r) { r.checked = (r.value === (pin.type || DEFAULT_PIN_TYPE)); }
      );
      var currentRot = String(pinRotation(pin));
      Array.prototype.forEach.call(
        dom.pinRotationRadios,
        function (r) { r.checked = (r.value === currentRot); }
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
      } else if (shape.kind === 'text') {
        if (dom.shapeTextValue && document.activeElement !== dom.shapeTextValue) {
          dom.shapeTextValue.value = shape.text || '';
        }
        if (dom.shapeTextX) dom.shapeTextX.value = shape.x;
        if (dom.shapeTextY) dom.shapeTextY.value = shape.y;
        if (dom.shapeTextSize) {
          dom.shapeTextSize.value = shape.fontSize || 12;
        }
        if (dom.shapeTextBold) {
          dom.shapeTextBold.checked = (shape.role === 'name') || !!shape.bold;
        }
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
    // Admin-only Promote button — visible only when the current
    // user is in the ADMIN_USERNAMES allow-list (the server enforces
    // too; this is just a UI affordance to keep the chrome clean
    // for everyone else).
    if (dom.libraryPromote) {
      var canPromote = !!(STATE.me && STATE.me.is_admin && hasActive && !STATE.libraryMode);
      dom.libraryPromote.classList.toggle('d-none', !(STATE.me && STATE.me.is_admin));
      dom.libraryPromote.disabled = !canPromote;
    }
  }

  async function onPromoteClick() {
    var row = getActiveSymbolRow();
    if (!row) return;
    // Quick prompt for the category — could be a fancier modal,
    // but a confirm + prompt round-trip is enough for the curator
    // who is on this page intentionally to promote one symbol.
    var cats = ['Passive', 'Active', 'Power', 'Sensor', 'Module', 'Connector', 'Custom'];
    var picked = window.prompt(
      'Pick a category for "' + (row.name || 'this symbol') + '":\n' +
      cats.join(', '),
      'Custom'
    );
    if (picked == null) return; // cancelled
    picked = String(picked).trim();
    if (cats.indexOf(picked) === -1) {
      alert('Unknown category. Pick one of: ' + cats.join(', '));
      return;
    }
    // Make sure the latest design lands on the server before we
    // promote — otherwise the library copy would lag the on-screen
    // version by whatever's still in the autosave debounce window.
    try { await flushAutosave(); } catch (_) {}
    setSaveStatus('saving', 'Promoting…');
    try {
      var promoted = await promoteSymbolToLibrary(row, { category: picked });
      setSaveStatus('saved');
      alert('Promoted "' + (promoted.name || row.name) + '" to the ' +
            promoted.category + ' category.\n' +
            'Open /admin/symbols/ to manage the global library.');
    } catch (e) {
      setSaveStatus('error', 'Promote failed');
      alert('Could not promote: ' + (e && e.message || 'unknown error'));
    }
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

  // Cancel any debounced timer and fire the PUT immediately, returning
  // the promise so callers (e.g. the back-link click handler) can
  // await save completion before navigating away. Without this the
  // schematic editor would re-fetch /symbols before the latest edit
  // landed, showing an older version of the symbol.
  function flushAutosave() {
    if (STATE.autosaveTimer) {
      clearTimeout(STATE.autosaveTimer);
      STATE.autosaveTimer = null;
    }
    if (STATE.activeSymbolId == null) return Promise.resolve();
    return doAutosave();
  }
  // Expose for the back-link click handler below.
  STATE._flushAutosave = flushAutosave;

  // Pagehide / beforeunload safety net: if the user closes the tab,
  // refreshes, or hits the browser back button while an autosave is
  // still pending, fire a synchronous-ish PUT via `keepalive: true`
  // so the request survives page tear-down. Modern browsers buffer
  // up to ~64 KB of keepalive bodies; a symbol document is well
  // under that ceiling.
  window.addEventListener('pagehide', function () {
    if (!STATE.autosaveTimer && STATE.activeSymbolId == null) return;
    if (STATE.autosaveTimer) {
      clearTimeout(STATE.autosaveTimer);
      STATE.autosaveTimer = null;
    }
    var row = getActiveSymbolRow();
    if (!row) return;
    var payload = {
      name: row.name,
      ref_des_prefix: row.ref_des_prefix || 'U',
      description: row.description || null,
      symbol_data: JSON.stringify(STATE.activeSymbol || emptySymbolDoc()),
    };
    if (STATE.libraryMode) {
      payload.category = row.category || 'Custom';
    } else {
      payload.bom_item_id = (row.bom_item_id == null) ? null : row.bom_item_id;
    }
    try {
      fetch(symbolPutUrl(row.id), {
        method: 'PUT',
        credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
        keepalive: true,
      });
    } catch (_) { /* best-effort, no UI for failure here */ }
  });

  async function doAutosave() {
    var row = getActiveSymbolRow();
    if (!row) return;
    var payload = {
      name: row.name,
      ref_des_prefix: row.ref_des_prefix || 'U',
      description: row.description || null,
      symbol_data: JSON.stringify(STATE.activeSymbol || emptySymbolDoc()),
    };
    // Mode-specific fields. Pydantic ignores unknowns so it's safe to
    // send both shapes to either endpoint, but tidier to send only
    // what the target route accepts.
    if (STATE.libraryMode) {
      payload.category = row.category || 'Custom';
    } else {
      payload.bom_item_id = (row.bom_item_id == null) ? null : row.bom_item_id;
    }
    try {
      var resp = await apiFetchWithTermsRetry(
        symbolPutUrl(row.id),
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
        var pins = Array.isArray(parsed.pins) ? parsed.pins : [];
        // Backfill rotation on legacy pins that only carry the old
        // `side` field. New pins write both; old pins get the same
        // treatment on next save.
        pins.forEach(function (p) {
          if (typeof p.rotation !== 'number') {
            p.rotation = sideToRotation(p.side || 'right');
          }
          if (!p.side) p.side = rotationToSide(p.rotation);
        });
        return {
          bodyShapes: Array.isArray(parsed.bodyShapes) ? parsed.bodyShapes : [],
          pins: pins,
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
    // Tool → selection / cursor mode:
    //   select → rubber-band lasso ON (default), regular pointer
    //   pan    → drag-to-pan the viewport, grab cursor
    //   any other (pin, rect, line, …) → crosshair, lasso OFF
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

    // Selection styling — every time Fabric's selection changes
    // (single click, lasso, shift-pick, deselect), re-colour the
    // affected canvas objects so the user gets consistent visual
    // feedback: selected = brand red, unselected = INK black. Without
    // this, multi-selected items render via STATE.selectedPinId /
    // selectedShapeId which only track single-selection, so the
    // colours fall out of sync with what Fabric thinks is picked.
    c.on('selection:created', applyFabricSelectionStyle);
    c.on('selection:updated', applyFabricSelectionStyle);
    c.on('selection:cleared', applyFabricSelectionStyle);

    // Pin drag — the dot is the only selectable part. As it moves we
    // ferry the same delta into the line + label so the whole pin
    // visually tracks the cursor. On modified (mouseup) we snap and
    // commit to STATE, then renderCanvas() rebuilds everything fresh.
    //
    // Same logic applies to ActiveSelection drags (multi-pin pick):
    // the AS wraps the *dots* but the line/label pin-parts aren't
    // selectable so they sit OUTSIDE the AS. Without ferrying their
    // positions every frame, the user sees only the circles travel
    // and the lines + labels stay frozen at the original spot until
    // mouseup, which reads as a glitch.
    function ferryPinParts(idSet, dx, dy) {
      c.getObjects().forEach(function (o) {
        if (!o.data || o.data.kind !== 'pin-part') return;
        if (!idSet[o.data.id]) return;
        if (o.type === 'line') {
          o.set({ x1: o.x1 + dx, y1: o.y1 + dy, x2: o.x2 + dx, y2: o.y2 + dy });
        } else {
          o.set({ left: o.left + dx, top: o.top + dy });
        }
        o.setCoords();
      });
    }
    c.on('object:moving', function (e) {
      var obj = e.target;
      if (!obj) return;

      // Multi-pin drag — ActiveSelection wraps the selected dots
      // and/or shapes; the line + label objects sit OUTSIDE the AS
      // because they're not selectable. Move them by the AS's frame
      // delta so the whole pin tracks the cursor live.
      if (obj.type === 'activeselection' || obj.type === 'activeSelection') {
        var dxs = obj.left - (obj._lastLeft != null ? obj._lastLeft : obj.left);
        var dys = obj.top  - (obj._lastTop  != null ? obj._lastTop  : obj.top);
        obj._lastLeft = obj.left;
        obj._lastTop  = obj.top;
        if (dxs === 0 && dys === 0) return;
        var kids = obj.getObjects ? obj.getObjects() : [];
        var idSet = {};
        kids.forEach(function (child) {
          if (child && child.data && child.data.kind === 'pin') {
            idSet[child.data.id] = true;
          }
        });
        if (Object.keys(idSet).length > 0) ferryPinParts(idSet, dxs, dys);
        return;
      }

      // Single pin drag — the dragged target is the pin dot itself.
      if (!obj.data || obj.data.kind !== 'pin') return;
      var dx = obj.left - (obj._lastLeft != null ? obj._lastLeft : obj.left);
      var dy = obj.top  - (obj._lastTop  != null ? obj._lastTop  : obj.top);
      obj._lastLeft = obj.left;
      obj._lastTop  = obj.top;
      var single = {};
      single[obj.data.id] = true;
      ferryPinParts(single, dx, dy);
    });

    c.on('object:modified', function (e) {
      var obj = e.target;
      if (!obj) return;
      var deferRender = function () {
        requestAnimationFrame(function () { renderCanvas(); });
      };

      // Multi-symbol drop — ActiveSelection wrapping several pins
      // and/or shapes. Iterate children, transform each by the AS
      // matrix to recover canvas-space coords, then dispatch to the
      // single-item code per child.
      if (obj.type === 'activeselection' || obj.type === 'activeSelection') {
        var asMatrix = obj.calcTransformMatrix
          ? obj.calcTransformMatrix()
          : null;
        var kids = obj.getObjects ? obj.getObjects() : [];
        kids.forEach(function (child) {
          if (!child.data) return;
          var pt = (asMatrix && fabric.util && fabric.util.transformPoint)
            ? fabric.util.transformPoint(
                { x: child.left, y: child.top }, asMatrix)
            : { x: child.left, y: child.top };
          if (child.data.kind === 'pin') {
            var pin = findPin(child.data.id);
            if (!pin) return;
            var snapped = snapCanvasXY(pt.x, pt.y);
            var rot = pinRotation(pin);
            var dx = 0, dy = 0;
            switch (rot) {
              case 0:   dx = 1;  break;
              case 90:  dy = 1;  break;
              case 180: dx = -1; break;
              case 270: dy = -1; break;
            }
            var sceneEnd = c2s(snapped.x, snapped.y);
            pin.x = sceneEnd.x - dx * 2 * GRID;
            pin.y = sceneEnd.y - dy * 2 * GRID;
          } else if (child.data.kind === 'shape') {
            var shape = findShape(child.data.id);
            if (!shape) return;
            // Multi-select drag only translates (no per-child resize),
            // so just shift each shape's stored coords by the delta
            // from the child's prior canvas position.
            if (shape.kind === 'rect') {
              var snapped2 = snapCanvasXY(pt.x, pt.y);
              var scene = c2s(snapped2.x, snapped2.y);
              shape.x = scene.x;
              shape.y = scene.y;
            } else if (shape.kind === 'text') {
              var snT = snapCanvasXY(pt.x, pt.y);
              var scT = c2s(snT.x, snT.y);
              shape.x = scT.x;
              shape.y = scT.y;
            } else if (shape.kind === 'line') {
              // The line's local (x1,y1)→(x2,y2) length stays put;
              // we shift it so (x1,y1) ends up at the new top-left.
              var sceneTL = c2s(pt.x, pt.y);
              var origMinX = Math.min(shape.x1, shape.x2);
              var origMinY = Math.min(shape.y1, shape.y2);
              var dxL = snap(sceneTL.x) - origMinX;
              var dyL = snap(sceneTL.y) - origMinY;
              shape.x1 = shape.x1 + dxL;
              shape.y1 = shape.y1 + dyL;
              shape.x2 = shape.x2 + dxL;
              shape.y2 = shape.y2 + dyL;
            }
          }
        });
        requestAnimationFrame(function () {
          if (STATE.canvas) STATE.canvas.discardActiveObject();
          renderCanvas();
        });
        scheduleAutosave();
        return;
      }

      if (!obj.data) return;
      if (obj.data.kind === 'pin') {
        var pin = findPin(obj.data.id);
        if (!pin) return;
        var snapped = snapCanvasXY(obj.left, obj.top);
        var rot = pinRotation(pin);
        var dx = 0, dy = 0;
        switch (rot) {
          case 0:   dx = 1;  break;
          case 90:  dy = 1;  break;
          case 180: dx = -1; break;
          case 270: dy = -1; break;
        }
        var sceneEnd = c2s(snapped.x, snapped.y);
        pin.x = sceneEnd.x - dx * 2 * GRID;
        pin.y = sceneEnd.y - dy * 2 * GRID;
        deferRender();
        scheduleAutosave();
      } else if (obj.data.kind === 'shape') {
        var shape = findShape(obj.data.id);
        if (!shape) return;
        if (shape.kind === 'rect') {
          // Bake any scale into width/height so the next render
          // round-trips cleanly. obj.scaleX/Y are 1 when the user
          // only translated; > 1 when they grabbed a corner / side
          // handle and resized.
          var newW = Math.max(GRID, Math.round((obj.width * obj.scaleX) / GRID) * GRID);
          var newH = Math.max(GRID, Math.round((obj.height * obj.scaleY) / GRID) * GRID);
          var snappedTL = snapCanvasXY(obj.left, obj.top);
          var sceneTL = c2s(snappedTL.x, snappedTL.y);
          shape.x = sceneTL.x;
          shape.y = sceneTL.y;
          shape.w = newW;
          shape.h = newH;
        } else if (shape.kind === 'text') {
          // Text shapes translate only — no resize. Sync position +
          // the text content (in case the user edited it inline
          // via fabric.IText editing).
          var snTxt = snapCanvasXY(obj.left, obj.top);
          var scTxt = c2s(snTxt.x, snTxt.y);
          shape.x = scTxt.x;
          shape.y = scTxt.y;
          if (typeof obj.text === 'string') shape.text = obj.text;
        } else if (shape.kind === 'line') {
          // Lines may have been moved OR resized via end-grips.
          // Fabric stores line endpoints in the object's local space
          // (x1/y1/x2/y2) and applies left/top + scale on top. The
          // simplest correct recovery is to read the two endpoints
          // off the object in canvas space via fabric.util and
          // re-snap them to the grid.
          var m = obj.calcTransformMatrix
            ? obj.calcTransformMatrix()
            : null;
          if (m && fabric.util && fabric.util.transformPoint) {
            var p1c = fabric.util.transformPoint(
              { x: -obj.width / 2, y: -obj.height / 2 }, m);
            var p2c = fabric.util.transformPoint(
              { x:  obj.width / 2, y:  obj.height / 2 }, m);
            // The line's stored x1/y1 → x2/y2 isn't guaranteed to map
            // to "top-left → bottom-right" — work out which canvas
            // point each endpoint actually corresponds to by checking
            // sign of (x1 - x2).
            var aIsTL = (obj.x1 < obj.x2) === (p1c.x < p2c.x);
            var aIsTLY = (obj.y1 < obj.y2) === (p1c.y < p2c.y);
            var aIsTLBoth = aIsTL && aIsTLY;
            var a = aIsTLBoth ? p1c : p2c;
            var b = aIsTLBoth ? p2c : p1c;
            var a_s = c2s(a.x, a.y);
            var b_s = c2s(b.x, b.y);
            shape.x1 = snap(a_s.x);
            shape.y1 = snap(a_s.y);
            shape.x2 = snap(b_s.x);
            shape.y2 = snap(b_s.y);
          }
        }
        deferRender();
        scheduleAutosave();
      }
    });

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

  // Viewport state — matches the pattern used in the schematic editor
  // + instruction builder. Internal canvas pixels = wrap CSS pixels;
  // a viewportTransform applies the auto-fit scale (so the whole scene
  // fits in the wrap) multiplied by userZoom and offset by pan.
  STATE.userZoom = STATE.userZoom || 1;
  STATE.panX = STATE.panX || 0;
  STATE.panY = STATE.panY || 0;

  function syncCanvasDisplaySize() {
    if (!STATE.canvas || !dom.canvasWrap) return;
    var rect = dom.canvasWrap.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    var w = Math.max(1, Math.floor(rect.width));
    var h = Math.max(1, Math.floor(rect.height));
    STATE.canvas.setDimensions({ width: w, height: h });
    applyViewport();
  }

  function sdFitScale() {
    if (!STATE.canvas) return 1;
    var w = STATE.canvas.getWidth();
    var h = STATE.canvas.getHeight();
    var s = Math.min(w / SCENE_W, h / SCENE_H);
    return s > 0 ? s : 1;
  }

  function sdEffectiveScale() {
    return sdFitScale() * (STATE.userZoom || 1);
  }

  function applyViewport() {
    if (!STATE.canvas) return;
    var w = STATE.canvas.getWidth();
    var h = STATE.canvas.getHeight();
    var s = sdEffectiveScale();
    var tx = (w - SCENE_W * s) / 2 + STATE.panX;
    var ty = (h - SCENE_H * s) / 2 + STATE.panY;
    STATE.canvas.setViewportTransform([s, 0, 0, s, tx, ty]);
    updateZoomPctLabel();
  }

  function zoomBy(factor, focal) {
    var prev = STATE.userZoom || 1;
    var next = Math.max(0.25, Math.min(6, prev * factor));
    if (next === prev) return;
    if (focal && STATE.canvas) {
      var vt = STATE.canvas.viewportTransform;
      var sceneX = (focal.x - vt[4]) / vt[0];
      var sceneY = (focal.y - vt[5]) / vt[3];
      STATE.userZoom = next;
      var s = sdEffectiveScale();
      var w = STATE.canvas.getWidth();
      var h = STATE.canvas.getHeight();
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

  // Frame the active symbol's pins + body shapes inside the visible
  // canvas at ~90 % margin. Used by the zoom-reset button's toggle
  // (alongside resetZoom) so the user can flip between 100 % and
  // fitted views.
  function zoomToFit() {
    if (!STATE.canvas || !STATE.activeSymbol) { resetZoom(); return; }
    var pins = STATE.activeSymbol.pins || [];
    var shapes = STATE.activeSymbol.bodyShapes || [];
    var minX = Infinity, maxX = -Infinity;
    var minY = Infinity, maxY = -Infinity;
    function bump(x, y) {
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
    pins.forEach(function (p) {
      // Pin's anchor + stick + terminator extents (lineLen = 2*GRID).
      var lineLen = 2 * GRID;
      var rot = pinRotation(p);
      var dx = 0, dy = 0;
      switch (rot) {
        case 0:   dx = 1;  break;
        case 90:  dy = 1;  break;
        case 180: dx = -1; break;
        case 270: dy = -1; break;
      }
      bump(p.x, p.y);
      bump(p.x + dx * lineLen, p.y + dy * lineLen);
    });
    shapes.forEach(function (s) {
      if (s.kind === 'rect') {
        bump(s.x, s.y);
        bump(s.x + s.w, s.y + s.h);
      } else if (s.kind === 'line') {
        bump(s.x1, s.y1);
        bump(s.x2, s.y2);
      } else if (s.kind === 'text') {
        // Approximate text bbox — char width ≈ 0.6 × fontSize.
        var fs = s.fontSize || 12;
        var w = (s.text || '').length * fs * 0.6;
        var h = fs * 1.4;
        bump(s.x, s.y);
        bump(s.x + w, s.y + h);
      }
    });
    if (!isFinite(minX) || maxX <= minX || maxY <= minY) {
      resetZoom();
      return;
    }
    var contentW = Math.max(1, maxX - minX);
    var contentH = Math.max(1, maxY - minY);
    var cw = STATE.canvas.getWidth();
    var ch = STATE.canvas.getHeight();
    var margin = 0.9;
    var sFit = sdFitScale();
    var zX = (cw * margin) / (sFit * contentW);
    var zY = (ch * margin) / (sFit * contentH);
    var z  = Math.max(0.25, Math.min(6, Math.min(zX, zY)));
    STATE.userZoom = z;
    var s = sdEffectiveScale();
    var midX = (minX + maxX) / 2;
    var midY = (minY + maxY) / 2;
    // Scene coords are centred on (0, 0) — convert content midpoint
    // (which is already in scene coords) to a pan that places it at
    // the viewport centre.
    STATE.panX = -midX * s;
    STATE.panY = -midY * s;
    applyViewport();
  }

  function updateZoomPctLabel() {
    if (!dom.zoomPct) return;
    dom.zoomPct.textContent = Math.round((STATE.userZoom || 1) * 100) + '%';
  }

  // ====================================================================
  // 18. EVENT WIRING
  // ====================================================================

  function cacheDom() {
    dom.workspace      = document.getElementById('sd-workspace');
    dom.titlebar       = document.getElementById('sd-titlebar');
    dom.backLink       = document.getElementById('sd-back-link');
    // sd-open-schematic was removed — the back link now goes
    // directly to the schematic editor, no separate primary button
    // needed.
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
    dom.libraryPromote = document.getElementById('sd-library-promote');
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
    dom.pinRotationRadios = document.querySelectorAll('input[name="sd-pin-rotation"]');
    // Transform section (top of right pane) — operates on selected
    // pin or shape.
    dom.transformRotateCw  = document.getElementById('sd-transform-rotate-cw');
    dom.transformRotateCcw = document.getElementById('sd-transform-rotate-ccw');
    dom.transformFlipH     = document.getElementById('sd-transform-flip-h');
    dom.transformFlipV     = document.getElementById('sd-transform-flip-v');
    dom.transformTarget    = document.getElementById('sd-transform-target');
    // Zoom group
    dom.zoomIn         = document.getElementById('sd-zoom-in');
    dom.zoomOut        = document.getElementById('sd-zoom-out');
    dom.zoomReset      = document.getElementById('sd-zoom-reset');
    dom.zoomPct        = document.getElementById('sd-zoom-pct');

    dom.shapeSection   = document.getElementById('sd-props-shape-section');
    dom.shapeRectX     = document.getElementById('sd-shape-rect-x');
    dom.shapeRectY     = document.getElementById('sd-shape-rect-y');
    dom.shapeRectW     = document.getElementById('sd-shape-rect-w');
    dom.shapeRectH     = document.getElementById('sd-shape-rect-h');
    dom.shapeLineX1    = document.getElementById('sd-shape-line-x1');
    dom.shapeLineY1    = document.getElementById('sd-shape-line-y1');
    dom.shapeLineX2    = document.getElementById('sd-shape-line-x2');
    dom.shapeLineY2    = document.getElementById('sd-shape-line-y2');
    dom.shapeTextValue = document.getElementById('sd-shape-text-value');
    dom.shapeTextX     = document.getElementById('sd-shape-text-x');
    dom.shapeTextY     = document.getElementById('sd-shape-text-y');
    dom.shapeTextSize  = document.getElementById('sd-shape-text-size');
    dom.shapeTextBold  = document.getElementById('sd-shape-text-bold');
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
    if (dom.libraryPromote) {
      dom.libraryPromote.addEventListener('click', onPromoteClick);
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
    Array.prototype.forEach.call(dom.pinRotationRadios, function (r) {
      r.addEventListener('change', function () {
        var pin = STATE.selectedPinId && findPin(STATE.selectedPinId);
        if (!pin || !r.checked) return;
        setPinRotation(pin, parseInt(r.value, 10));
        renderCanvas();
        scheduleAutosave();
      });
    });
    function rotateSelectedPinBy(delta) {
      var pin = STATE.selectedPinId && findPin(STATE.selectedPinId);
      if (!pin) return;
      setPinRotation(pin, pinRotation(pin) + delta);
      renderCanvas();
      renderPropertiesPanel();
      scheduleAutosave();
    }
    // Apply rotate/flip to a single pin in-place. A pin's origin IS
    // its anchor (x, y) — rotating the pin just bumps its direction
    // field; the anchor doesn't move. flip-h/flip-v rotate 180° so a
    // pin facing right after a flip-h faces left, matching what the
    // user expects "mirror this pin in place" to mean.
    function applyTransformToPin(pin, kind) {
      if (!pin) return;
      if (kind === 'rotate-cw')        setPinRotation(pin, pinRotation(pin) + 90);
      else if (kind === 'rotate-ccw')  setPinRotation(pin, pinRotation(pin) - 90);
      else if (kind === 'flip-h' || kind === 'flip-v') {
        setPinRotation(pin, pinRotation(pin) + 180);
      }
    }
    // Transform section (top of right pane). Acts on whatever is
    // currently selected: a single pin/shape, a multi-select
    // (ActiveSelection) of pins and shapes, or — when nothing is
    // selected — falls back to the whole-symbol transforms.
    //
    // For multi-select we transform each child IN-PLACE about its own
    // origin rather than rotating the whole group as a unit. That's
    // what the user expects when they shift-pick four pins + a body
    // line and hit "rotate 90°": each pin's direction flips, the
    // line spins around its own midpoint, etc. — no positions move.
    function transformAction(kind) {
      // Check Fabric's active object first — that's what tells us
      // whether the user picked one or many items.
      var active = STATE.canvas && STATE.canvas.getActiveObject
        ? STATE.canvas.getActiveObject()
        : null;
      var isMulti = active &&
        (active.type === 'activeselection' || active.type === 'activeSelection');

      if (isMulti) {
        // Iterate the ActiveSelection's children, pull each one's
        // STATE pin/shape via the `data.id` we tagged at render time,
        // and apply the same per-item transform that the single-item
        // path uses.
        var kids = active.getObjects ? active.getObjects() : [];
        var touchedAny = false;
        // Snapshot the selected IDs BEFORE we render — renderCanvas
        // rebuilds every Fabric object from scratch, so the existing
        // AS holds stale references. We re-create an AS over the new
        // objects matching these ids so the user's selection stays
        // intact for the next rotate / flip click.
        var preservedIds = kids
          .filter(function (c) { return c && c.data && (c.data.kind === 'pin' || c.data.kind === 'shape'); })
          .map(function (c) { return { kind: c.data.kind, id: c.data.id }; });
        kids.forEach(function (child) {
          if (!child || !child.data) return;
          if (child.data.kind === 'pin') {
            var p = findPin(child.data.id);
            if (p) { applyTransformToPin(p, kind); touchedAny = true; }
          } else if (child.data.kind === 'shape') {
            var s = findShape(child.data.id);
            if (s) { rotateOrFlipShapeInPlace(s, kind); touchedAny = true; }
          }
        });
        if (touchedAny) {
          // Discard the AS so Fabric drops its now-stale references
          // before we render fresh objects. The selection comes back
          // immediately via restoreActiveSelection below.
          if (STATE.canvas && STATE.canvas.discardActiveObject) {
            STATE.canvas.discardActiveObject();
          }
          renderCanvas();
          restoreActiveSelection(preservedIds);
          renderPropertiesPanel();
          scheduleAutosave();
        }
        return;
      }

      var pin = STATE.selectedPinId && findPin(STATE.selectedPinId);
      var shape = STATE.selectedShapeId && findShape(STATE.selectedShapeId);
      if (pin) {
        if (kind === 'rotate-cw')      rotateSelectedPinBy(90);
        else if (kind === 'rotate-ccw') rotateSelectedPinBy(-90);
        else if (kind === 'flip-h' || kind === 'flip-v') rotateSelectedPinBy(180);
      } else if (shape) {
        // For shapes, rotate-cw / rotate-ccw means "rotate the
        // shape's geometry around its centre by 90°" — and flip-h/v
        // mirrors. Implemented for rects via swap-and-recentre;
        // lines via swap endpoints. Not the deepest transform but
        // covers the common cases.
        rotateOrFlipShape(shape, kind);
      } else if (STATE.activeSymbol) {
        // Whole-symbol fallback (kept from the old toolbox).
        if (kind === 'rotate-cw')       rotateAll(90);
        else if (kind === 'rotate-ccw') rotateAll(-90);
        else if (kind === 'flip-h')     flipAll('h');
        else if (kind === 'flip-v')     flipAll('v');
      }
    }
    if (dom.transformRotateCw)  dom.transformRotateCw.addEventListener('click', function () { transformAction('rotate-cw'); });
    if (dom.transformRotateCcw) dom.transformRotateCcw.addEventListener('click', function () { transformAction('rotate-ccw'); });
    if (dom.transformFlipH)     dom.transformFlipH.addEventListener('click', function () { transformAction('flip-h'); });
    if (dom.transformFlipV)     dom.transformFlipV.addEventListener('click', function () { transformAction('flip-v'); });

    // Zoom buttons + Ctrl+wheel + Cmd/Ctrl + / - / 0.
    if (dom.zoomIn)    dom.zoomIn.addEventListener('click', function () { zoomBy(1.25, null); });
    if (dom.zoomOut)   dom.zoomOut.addEventListener('click', function () { zoomBy(1 / 1.25, null); });
    // Zoom-reset toggles between zoom-to-fit and 100 %.
    if (dom.zoomReset) {
      var _sdZoomResetMode = 'fit';
      dom.zoomReset.addEventListener('click', function () {
        if (_sdZoomResetMode === 'fit') {
          zoomToFit();
          _sdZoomResetMode = 'reset';
        } else {
          resetZoom();
          _sdZoomResetMode = 'fit';
        }
      });
    }
    if (dom.canvasWrap) {
      dom.canvasWrap.addEventListener('wheel', function (e) {
        if (!(e.ctrlKey || e.metaKey)) return;
        if (!STATE.canvas) return;
        e.preventDefault();
        var rect = STATE.canvas.lowerCanvasEl.getBoundingClientRect();
        var focal = { x: e.clientX - rect.left, y: e.clientY - rect.top };
        var factor = e.deltaY < 0 ? 1.1 : 1 / 1.1;
        zoomBy(factor, focal);
      }, { passive: false });
    }
    document.addEventListener('keydown', function (e) {
      if (!(e.metaKey || e.ctrlKey)) return;
      if (document.activeElement &&
          /input|textarea|select/i.test(document.activeElement.tagName)) return;
      if (e.key === '+' || e.key === '=') { e.preventDefault(); zoomBy(1.25, null); }
      else if (e.key === '-' || e.key === '_') { e.preventDefault(); zoomBy(1 / 1.25, null); }
      else if (e.key === '0') { e.preventDefault(); resetZoom(); }
    });
    // R / Shift+R keyboard shortcut: rotate selected pin while not in
    // a text input. Mirrors KiCad / Fusion 360 muscle memory.
    document.addEventListener('keydown', function (e) {
      if (document.activeElement &&
          /input|textarea|select/i.test(document.activeElement.tagName)) return;
      if ((e.key === 'r' || e.key === 'R') && !e.metaKey && !e.ctrlKey) {
        if (!STATE.selectedPinId) return;
        e.preventDefault();
        rotateSelectedPinBy(e.shiftKey ? -90 : 90);
      }
    });

    // Cmd/Ctrl+D — duplicate the currently selected pin(s) / shape(s).
    // Clones are offset by one grid cell so the new item is visible
    // and not stacked exactly on top of the original. The new items
    // become the selection so the user can chain duplicates.
    document.addEventListener('keydown', function (e) {
      if (!(e.metaKey || e.ctrlKey)) return;
      if (document.activeElement &&
          /input|textarea|select/i.test(document.activeElement.tagName)) return;
      if (e.key !== 'd' && e.key !== 'D') return;
      e.preventDefault();
      duplicateSelected();
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
    wireShapeNumeric(dom.shapeTextX, 'x');
    wireShapeNumeric(dom.shapeTextY, 'y');

    // Text value — live update while typing (no snap).
    if (dom.shapeTextValue) {
      dom.shapeTextValue.addEventListener('input', function () {
        var shape = STATE.selectedShapeId && findShape(STATE.selectedShapeId);
        if (!shape || shape.kind !== 'text') return;
        shape.text = dom.shapeTextValue.value;
        renderCanvas();
        scheduleAutosave();
      });
    }
    if (dom.shapeTextSize) {
      dom.shapeTextSize.addEventListener('change', function () {
        var shape = STATE.selectedShapeId && findShape(STATE.selectedShapeId);
        if (!shape || shape.kind !== 'text') return;
        var n = Math.max(8, Math.min(48,
          parseInt(dom.shapeTextSize.value, 10) || 12));
        shape.fontSize = n;
        dom.shapeTextSize.value = n;
        renderCanvas();
        scheduleAutosave();
      });
    }
    if (dom.shapeTextBold) {
      dom.shapeTextBold.addEventListener('change', function () {
        var shape = STATE.selectedShapeId && findShape(STATE.selectedShapeId);
        if (!shape || shape.kind !== 'text') return;
        shape.bold = dom.shapeTextBold.checked;
        renderCanvas();
        scheduleAutosave();
      });
    }

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

  // Returns the URL the designer should PUT to when autosaving the
  // currently-active symbol. In project mode it's the per-project
  // /symbols/<id> route; in library mode (admin-only ?library_id=N
  // entry point) it's the global /api/library/symbols/<id> route.
  // Centralised here so flushAutosave + the pagehide handler stay
  // in sync.
  function symbolPutUrl(rowId) {
    if (STATE.libraryMode) {
      return API + '/api/library/symbols/' + rowId;
    }
    return API + '/api/projects/' + STATE.projectId + '/symbols/' + rowId;
  }

  // Promote the current project symbol to the global admin library.
  // Returns the new LibrarySymbol row on success, throws on failure.
  // Only callable when the current user is in the admin list (the
  // server enforces this; the UI just hides the trigger button for
  // non-admins).
  async function promoteSymbolToLibrary(row, overrides) {
    if (!row || row.id == null) throw new Error('No symbol to promote');
    var body = Object.assign({
      project_symbol_id: row.id,
      name: row.name,
      ref_des_prefix: row.ref_des_prefix || 'U',
      description: row.description || null,
      category: 'Custom',
    }, overrides || {});
    var resp = await apiFetchWithTermsRetry(
      API + '/api/library/symbols/promote-from-project',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      }
    );
    if (!resp.ok) throw new Error('Promote HTTP ' + resp.status);
    return resp.json();
  }

  async function fetchLibrarySymbol(libraryId) {
    var r = await apiFetch(API + '/api/library/symbols/' + libraryId);
    if (!r.ok) {
      var err = new Error('Library symbol HTTP ' + r.status);
      err.status = r.status;
      throw err;
    }
    return r.json();
  }

  async function fetchLibraryCategories() {
    try {
      var r = await apiFetch(API + '/api/library/symbols/categories');
      if (!r.ok) return [];
      return r.json();
    } catch (_) { return []; }
  }
  async function fetchBom() {
    try {
      var r = await apiFetch(API + '/api/projects/' + STATE.projectId + '/bom');
      if (!r.ok) return [];
      var rows = await r.json();
      return Array.isArray(rows) ? rows : [];
    } catch (_) { return []; }
  }

  // Embedded-mode postMessage bridge — mirrors the schematic editor's
  // protocol so the instruction builder's title-bar zoom controls
  // drive whichever editor is currently loaded in the iframe.
  function isEmbedded() {
    return document.body.classList.contains('is-embedded');
  }
  function postToParent(payload) {
    if (!isEmbedded() || !window.parent || window.parent === window) return;
    try { window.parent.postMessage(payload, '*'); } catch (_) {}
  }
  function postZoomToParent() {
    postToParent({ kr_se_event: 'zoom', percent: Math.round((STATE.userZoom || 1) * 100) });
  }
  function wireParentBridge() {
    if (!isEmbedded()) return;
    window.addEventListener('message', function (e) {
      var msg = e && e.data;
      if (!msg || msg.kr_se_action == null) return;
      switch (msg.kr_se_action) {
        case 'zoom-in':    zoomBy(1.25, null); postZoomToParent(); break;
        case 'zoom-out':   zoomBy(1 / 1.25, null); postZoomToParent(); break;
        case 'zoom-reset': resetZoom();   postZoomToParent(); break;
      }
    });
    postZoomToParent();
  }

  async function init() {
    cacheDom();
    var params = new URLSearchParams(window.location.search);
    // ``?library_id=N`` opens the designer in admin library-edit
    // mode: load a single LibrarySymbol, edit metadata + design,
    // PUT goes to the global library route. Bypasses the
    // project-id requirement entirely.
    var libraryIdParam = params.get('library_id');
    if (libraryIdParam) {
      STATE.libraryMode = true;
      STATE.libraryId = parseInt(libraryIdParam, 10);
      await initLibraryMode();
      return;
    }
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
      // One-step back: schematic editor for this project (the page
      // the user usually arrives from). If they entered via the
      // project editor instead, they can use the browser back button
      // or the schematic editor's own "Back to project editor" link.
      dom.backLink.href = '/projects/schematic/edit.html?id=' +
        encodeURIComponent(STATE.projectId);
      // Intercept clicks so any pending autosave fires AND completes
      // before we navigate. Without this the user can edit a symbol,
      // hit Back within the 500 ms autosave debounce window, and
      // arrive in the schematic editor showing the pre-edit version
      // of the symbol.
      dom.backLink.addEventListener('click', async function (e) {
        // Plain left-click only — let middle/cmd/ctrl-click open in a
        // new tab without intercept (the new tab will re-fetch on
        // load, so the user gets the latest version anyway).
        if (e.button !== 0 || e.metaKey || e.ctrlKey || e.shiftKey || e.altKey) return;
        var href = dom.backLink.getAttribute('href');
        if (!href) return;
        e.preventDefault();
        try { await flushAutosave(); } catch (_) {}
        window.location.href = href;
      });
    }
    // The redundant sd-open-schematic button was removed from the
    // title bar — the back link above is now the single exit path.

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
      // Embedded? Wire the parent-frame postMessage bridge so the
      // builder's title-bar zoom controls drive this canvas.
      wireParentBridge();
    } catch (e) {
      if (e && e.status === 404 && dom.errorDetail) {
        dom.errorDetail.textContent = 'Project not found.';
      }
      showOnly(dom.error);
    }
  }

  // ---------------------------------------------------------------
  // Library-edit mode (admin only). The Symbol Designer normally
  // works per-project; this path loads a single LibrarySymbol so
  // admins can refine the design without going via a project.
  // ---------------------------------------------------------------
  async function initLibraryMode() {
    if (!Number.isFinite(STATE.libraryId)) {
      if (dom.errorDetail) dom.errorDetail.textContent = 'Bad library_id in URL.';
      showOnly(dom.error);
      return;
    }
    // Back link goes to the admin symbols page, not a schematic.
    if (dom.backLink) {
      dom.backLink.href = '/admin/symbols/';
      dom.backLink.innerHTML =
        '<i class="fas fa-arrow-left me-1"></i> Back to library';
      dom.backLink.addEventListener('click', async function (e) {
        if (e.button !== 0 || e.metaKey || e.ctrlKey || e.shiftKey || e.altKey) return;
        var href = dom.backLink.getAttribute('href');
        if (!href) return;
        e.preventDefault();
        try { await flushAutosave(); } catch (_) {}
        window.location.href = href;
      });
    }
    try {
      // Admin gate. The server enforces too; this is just a friendly
      // early-return for non-admins.
      var me = await fetchMe();
      STATE.me = me;
      if (!me || me.is_admin !== true) {
        showOnly(dom.notOwner);
        return;
      }
      var sym = await fetchLibrarySymbol(STATE.libraryId);
      if (dom.projectTitle) {
        dom.projectTitle.textContent =
          'Library symbol · ' + (sym.name || ('#' + sym.id));
      }
      // Adapt the single library row into the same in-memory shape
      // the rest of the designer expects (it walks STATE.symbols and
      // uses ``id`` / ``name`` / ``symbol_data``). The category field
      // is preserved on the row so doAutosave can include it.
      STATE.symbols = [sym];
      STATE.bomItems = []; // library symbols don't link to a BOM
      wireUi();
      initCanvas();
      // Hide the BOM dropdown + the library list rail — neither
      // applies in library-edit mode.
      hideLibraryModeUiBits();
      setSaveStatus('saved');
      activateSymbol(sym.id);
      showOnly(dom.main);
      setTimeout(syncCanvasDisplaySize, 50);
    } catch (e) {
      if (e && e.status === 404 && dom.errorDetail) {
        dom.errorDetail.textContent = 'Library symbol not found.';
      }
      showOnly(dom.error);
    }
  }

  // Trim the chrome that doesn't apply to library editing. Library
  // symbols don't link to a BOM and aren't grouped into a per-
  // project list, so the BOM dropdown + the symbols list rail get
  // hidden. We keep the canvas + properties pane + toolbar intact.
  function hideLibraryModeUiBits() {
    var hideIds = [
      'sd-bom-section',         // BOM dropdown row
      'sd-library-list',        // left-rail symbol list
      'sd-library-section',     // its containing card / section
      'sd-new-symbol-btn',      // "+ New" — single-symbol edit only
      'sd-delete-symbol-btn',   // deletion happens via /admin/symbols
    ];
    hideIds.forEach(function (id) {
      var el = document.getElementById(id);
      if (el) el.style.display = 'none';
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
