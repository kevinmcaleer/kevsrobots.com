/**
 * Instruction Builder — Phase 1 (issue #178).
 *
 * Standalone full-screen page (web/projects/instructions/edit.html). Loads a
 * project's instruction + steps from the Projects API, then lets the owner
 * draw on a per-step Fabric.js canvas (image background + arrow / text /
 * rectangle / circle overlays). Autosaves canvas_json back to the
 * existing PUT /steps/{step_id} endpoint.
 *
 * Pinned to Fabric.js v6.4.0 (UMD build on jsdelivr exposes window.fabric).
 * If a future v6 release stops shipping a global, switch the CDN URL to
 * fabric@5.3.0 — its surface is close enough that only the
 * fabric.Image.fromURL call needs to migrate from a Promise back to a
 * callback.
 *
 * Out of scope for Phase 1 (defer to later phases): tablet / mobile
 * polish, callout presets, STL viewer, export pipeline, public viewer,
 * drag-to-reorder steps in this sidebar, multi-image / image series.
 */
(function () {
  'use strict';

  // -------- Constants & config ---------------------------------------

  var API = 'https://projects.kevsrobots.com';
  var CANVAS_W = 1200;
  var CANVAS_H = 900;
  var AUTOSAVE_MS = 800;
  var UNDO_DEBOUNCE_MS = 200;
  var UNDO_STACK_LIMIT = 50;

  // Phase 3a — STL viewer (lazy-loaded on first "Add STL" click).
  //
  // Pinned to Three.js v0.147. Two CDN paths are needed and they must
  // line up with each other:
  //
  //   * THREE_CDN — UMD-style build that still exposes a global `THREE`.
  //     The spec'd version (0.160) DOES still ship `build/three.min.js`
  //     with the global, but the matching `examples/js/loaders/STLLoader.js`
  //     was removed somewhere around 0.148 (verified: 404 on jsdelivr for
  //     0.148–0.160; 200 on 0.147 and 0.146). Pinning the whole pair to
  //     0.147 keeps the spec's "global THREE + global THREE.STLLoader"
  //     pattern intact without the loader having to be ESM.
  //   * STL_LOADER_CDN — classic-script STLLoader from the same release
  //     so it attaches to the right `THREE` global.
  //
  // ~600KB combined; only fetched once and never on pageload.
  var THREE_CDN = 'https://cdn.jsdelivr.net/npm/three@0.147.0/build/three.min.js';
  var STL_LOADER_CDN = 'https://cdn.jsdelivr.net/npm/three@0.147.0/examples/js/loaders/STLLoader.js';
  var STL_MAX_BYTES = 50 * 1024 * 1024; // 50 MB
  var STL_VIEW_SIZE = 400;              // square render area, in CSS pixels
  var STL_SCENE_BG = 0xE1F5EE;          // light blue, LEGO-style

  // -------- Module state ---------------------------------------------

  var state = {
    projectId: null,
    project: null,
    me: null,
    isOwner: false,
    instructionId: null,
    steps: [],          // ordered list from the API
    activeStepId: null, // null until first step is loaded
    activeTool: 'select',
    canvas: null,
    drawing: null,     // in-progress drag-shape: { type, obj, startX, startY }
    suppressEvents: false, // true while loadFromJSON is replaying
    // Per-step undo / redo stacks. Cleared when the active step changes
    // — there's no point in undoing into someone else's snapshot.
    undoStack: [],
    redoStack: [],
    lastSnapshot: null, // most recent canvas JSON string we accepted
  };

  // -------- DOM handles ----------------------------------------------

  var dom = {};
  function bindDom() {
    dom.workspace = document.getElementById('ib-workspace');
    dom.loading = document.getElementById('ib-loading');
    dom.notOwner = document.getElementById('ib-not-owner');
    dom.error = document.getElementById('ib-error');
    dom.errorDetail = document.getElementById('ib-error-detail');
    dom.main = document.getElementById('ib-main');
    dom.backLink = document.getElementById('ib-back-link');
    dom.openEditorNewTab = document.getElementById('ib-open-editor-newtab');
    dom.projectTitle = document.getElementById('ib-project-title');
    dom.saveStatus = document.getElementById('ib-save-status');
    dom.uploadImageBtn = document.getElementById('ib-upload-image-btn');
    dom.pickProjectImageBtn = document.getElementById('ib-pick-project-image-btn');
    dom.removeImageBtn = document.getElementById('ib-remove-image-btn');
    dom.imageInput = document.getElementById('ib-image-input');
    dom.addStlBtn = document.getElementById('ib-add-stl-btn');
    dom.color = document.getElementById('ib-color');
    dom.stroke = document.getElementById('ib-stroke');
    dom.strokeValue = document.getElementById('ib-stroke-value');
    dom.fontSize = document.getElementById('ib-fontsize');
    dom.rotation = document.getElementById('ib-rotation');
    dom.resetTransforms = document.getElementById('ib-reset-transforms');
    dom.bringToFront = document.getElementById('ib-bring-to-front');
    dom.bringForward = document.getElementById('ib-bring-forward');
    dom.sendBackward = document.getElementById('ib-send-backward');
    dom.sendToBack = document.getElementById('ib-send-to-back');
    dom.deleteSelected = document.getElementById('ib-delete-selected');
    dom.undoBtn = document.getElementById('ib-undo');
    dom.redoBtn = document.getElementById('ib-redo');
    dom.canvasEl = document.getElementById('ib-canvas');
    dom.canvasFrame = document.getElementById('ib-canvas-frame');
    dom.canvasEmpty = document.getElementById('ib-canvas-empty');
    dom.stepBadge = document.getElementById('ib-step-badge');
    dom.activeToolHint = document.getElementById('ib-active-tool-hint');
    dom.stepTitle = document.getElementById('ib-step-title');
    dom.stepDescription = document.getElementById('ib-step-description');
    dom.stepsList = document.getElementById('ib-steps-list');
    dom.addStepBtn = document.getElementById('ib-add-step');
    dom.toolButtons = document.querySelectorAll('.ib-tool-btn[data-tool]');
    // Export (Phase 2b / 2c)
    dom.exportProgress = document.getElementById('ib-export-progress');
    dom.exportMenuItems = document.querySelectorAll('.ib-export-menu [data-export]');
  }

  // -------- Helpers --------------------------------------------------

  function escapeHtml(s) {
    if (s === null || s === undefined) return '';
    var d = document.createElement('div');
    d.textContent = String(s);
    return d.innerHTML;
  }

  function apiFetch(url, opts) {
    opts = opts || {};
    if (window.ProjectAuth && typeof window.ProjectAuth.apiFetch === 'function') {
      return window.ProjectAuth.apiFetch(url, opts);
    }
    opts.credentials = opts.credentials || 'include';
    return fetch(url, opts);
  }

  // Wrap any write to retry once after the user accepts the T&Cs.
  async function apiFetchWithTermsRetry(url, opts) {
    var resp = await apiFetch(url, opts);
    if (resp.status === 403 && window.TermsGate && typeof window.TermsGate.handleResponse === 'function') {
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

  function setSaveStatus(kind, msg) {
    if (!dom.saveStatus) return;
    dom.saveStatus.classList.remove('is-saving', 'is-saved', 'is-error');
    if (kind === 'saving') {
      dom.saveStatus.classList.add('is-saving');
      dom.saveStatus.innerHTML = '<i class="fas fa-circle-notch fa-spin me-1"></i>' + escapeHtml(msg || 'Saving…');
    } else if (kind === 'saved') {
      dom.saveStatus.classList.add('is-saved');
      dom.saveStatus.innerHTML = '<i class="fas fa-check me-1"></i>' + escapeHtml(msg || 'Saved');
      setTimeout(function () {
        if (dom.saveStatus.classList.contains('is-saved')) {
          dom.saveStatus.textContent = '';
          dom.saveStatus.classList.remove('is-saved');
        }
      }, 1500);
    } else if (kind === 'error') {
      dom.saveStatus.classList.add('is-error');
      dom.saveStatus.innerHTML = '<i class="fas fa-exclamation-triangle me-1"></i>' +
        escapeHtml(msg || 'Save failed') +
        ' — <a href="#" id="ib-save-retry">Retry</a>';
      var retry = document.getElementById('ib-save-retry');
      if (retry) retry.addEventListener('click', function (e) {
        e.preventDefault();
        flushCanvasSave(true);
      });
    } else {
      dom.saveStatus.textContent = '';
    }
  }

  // Generic debouncer keyed by string id — keeps separate timers for
  // canvas autosave, step title, step description, etc.
  var _debounces = {};
  function debounce(key, ms, fn) {
    if (_debounces[key]) clearTimeout(_debounces[key]);
    _debounces[key] = setTimeout(function () {
      delete _debounces[key];
      fn();
    }, ms);
  }

  // -------- Backend calls --------------------------------------------

  async function fetchMe() {
    var resp = await apiFetch(API + '/api/auth/me');
    if (!resp.ok) return null;
    return resp.json();
  }

  async function fetchProject() {
    var resp = await apiFetch(API + '/api/projects/' + state.projectId);
    if (!resp.ok) {
      var err = new Error('Project HTTP ' + resp.status);
      err.status = resp.status;
      throw err;
    }
    return resp.json();
  }

  async function fetchInstruction() {
    var resp = await apiFetch(API + '/api/projects/' + state.projectId + '/instruction');
    if (resp.status === 404) return null;
    if (!resp.ok) throw new Error('Instruction HTTP ' + resp.status);
    return resp.json();
  }

  async function ensureInstruction() {
    if (state.instructionId) return state.instructionId;
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ title: null, description: null }),
      }
    );
    if (!resp.ok) throw new Error('Create instruction HTTP ' + resp.status);
    var body = await resp.json();
    state.instructionId = body.id;
    return body.id;
  }

  async function postStep() {
    await ensureInstruction();
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction/steps',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ title: '', description: '' }),
      }
    );
    if (!resp.ok) throw new Error('Add step HTTP ' + resp.status);
    return resp.json();
  }

  async function putStep(stepId, fields) {
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction/steps/' + stepId,
      {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(fields),
      }
    );
    if (!resp.ok) throw new Error('Update step HTTP ' + resp.status);
    return resp.json();
  }

  async function uploadImage(file) {
    var fd = new FormData();
    fd.append('file', file);
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/images',
      { method: 'POST', body: fd }
    );
    if (!resp.ok) throw new Error('Upload image HTTP ' + resp.status);
    return resp.json();
  }

  // -------- Canvas setup ---------------------------------------------

  function initCanvas() {
    // v6 UMD exposes window.fabric. Bail loudly if it isn't there — the
    // page is useless without it and the user deserves to see the
    // problem rather than a silent blank canvas.
    if (typeof fabric === 'undefined' || !fabric.Canvas) {
      throw new Error('Fabric.js failed to load');
    }
    state.canvas = new fabric.Canvas(dom.canvasEl, {
      width: CANVAS_W,
      height: CANVAS_H,
      backgroundColor: '#ffffff',
      preserveObjectStacking: true,
      selection: true,
    });

    var c = state.canvas;

    // Tool dispatch — Fabric handles select/drag for free in 'select'
    // mode, so we only intercept while another tool is active.
    c.on('mouse:down', onCanvasMouseDown);
    c.on('mouse:move', onCanvasMouseMove);
    c.on('mouse:up', onCanvasMouseUp);

    c.on('selection:created', onSelectionChanged);
    c.on('selection:updated', onSelectionChanged);
    c.on('selection:cleared', onSelectionChanged);

    // Live-update the rotation input as the user drags the corner handle.
    c.on('object:rotating', onObjectRotating);
    // After a translate / resize the active object's transform may have
    // changed without `selection:updated` firing — re-sync controls.
    c.on('object:modified', onSelectionChanged);

    // Autosave + undo snapshot triggers. The suppressEvents flag is
    // toggled while loadFromJSON is replaying so loading a step doesn't
    // immediately push a fresh snapshot back onto the undo stack.
    var changeEvents = ['object:added', 'object:modified', 'object:removed'];
    changeEvents.forEach(function (evt) {
      c.on(evt, function () {
        if (state.suppressEvents) return;
        scheduleAutosave();
        scheduleSnapshot();
        updateEmptyState();
      });
    });

    // Resize handling — Fabric needs setDimensions in CSS-pixel space
    // so the hit area matches the visible scaled canvas.
    syncCanvasDisplaySize();
    window.addEventListener('resize', syncCanvasDisplaySize);
  }

  // Fabric's internal coordinate space is fixed (CANVAS_W × CANVAS_H),
  // but the visible canvas resizes with the panel. Tell Fabric what the
  // current CSS size is so pointer events map correctly.
  function syncCanvasDisplaySize() {
    if (!state.canvas || !dom.canvasFrame) return;
    var rect = dom.canvasFrame.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    state.canvas.setDimensions(
      { width: rect.width, height: rect.height },
      { cssOnly: true }
    );
  }

  function updateEmptyState() {
    if (!state.canvas || !dom.canvasEmpty) return;
    var objects = state.canvas.getObjects();
    if (objects.length === 0) {
      dom.canvasEmpty.classList.remove('d-none');
    } else {
      dom.canvasEmpty.classList.add('d-none');
    }
  }

  // -------- Tool selection -------------------------------------------

  function setActiveTool(tool) {
    state.activeTool = tool;
    Array.prototype.forEach.call(dom.toolButtons, function (btn) {
      btn.classList.toggle('is-active', btn.dataset.tool === tool);
    });
    if (dom.activeToolHint) {
      var label = tool.charAt(0).toUpperCase() + tool.slice(1);
      dom.activeToolHint.textContent = 'Tool: ' + label;
    }
    if (state.canvas) {
      // Lock Fabric's own object selection while a drawing tool is
      // active so the click-drag below doesn't fight with Fabric's
      // group-select rectangle.
      state.canvas.selection = (tool === 'select');
      state.canvas.discardActiveObject();
      state.canvas.getObjects().forEach(function (o) {
        o.selectable = (tool === 'select') && !o.lockMovementX;
      });
      state.canvas.requestRenderAll();
    }
  }

  function onSelectionChanged() {
    if (!state.canvas) return;
    var active = state.canvas.getActiveObject();
    var hasSel = !!active;
    // Locked background images stay anchored at the bottom of the stack
    // and shouldn't be re-arranged, rotated, or reset by the user. Tag
    // the selection so Transform/Arrange controls disable on them.
    var isBg = !!(active && active.ibRole === 'background');
    var canTransform = hasSel && !isBg;
    if (dom.deleteSelected) dom.deleteSelected.disabled = !hasSel;
    if (dom.rotation) {
      dom.rotation.disabled = !canTransform;
      if (canTransform) {
        // Normalise to 0–359; Fabric's `angle` can be any number.
        var a = ((active.angle || 0) % 360 + 360) % 360;
        dom.rotation.value = String(Math.round(a));
      } else {
        dom.rotation.value = '0';
      }
    }
    if (dom.resetTransforms) dom.resetTransforms.disabled = !canTransform;
    if (dom.bringToFront) dom.bringToFront.disabled = !canTransform;
    if (dom.bringForward) dom.bringForward.disabled = !canTransform;
    if (dom.sendBackward) dom.sendBackward.disabled = !canTransform;
    if (dom.sendToBack) dom.sendToBack.disabled = !canTransform;
  }

  // Keep the rotation input in sync while the user drags the rotation
  // handle on the canvas — Fabric fires `object:rotating` continuously
  // and `object:modified` when the drag ends. We listen to both so the
  // number ticks live, but only the `modified` event triggers autosave +
  // an undo snapshot (already wired in initCanvas).
  function onObjectRotating(opt) {
    if (!dom.rotation || dom.rotation.disabled) return;
    var t = opt && opt.target;
    if (!t) return;
    var a = ((t.angle || 0) % 360 + 360) % 360;
    dom.rotation.value = String(Math.round(a));
  }

  // -------- Pointer handlers (arrow / rect / circle / text) ----------

  function getPointer(opt) {
    // Fabric returns canvas-coordinate-space pointer regardless of CSS
    // scaling, which is what we want here.
    return state.canvas.getPointer(opt.e);
  }

  function onCanvasMouseDown(opt) {
    var tool = state.activeTool;
    if (tool === 'select') return;
    var p = getPointer(opt);
    var color = (dom.color && dom.color.value) || '#dc3545';
    var stroke = parseInt((dom.stroke && dom.stroke.value) || '3', 10);
    var fontSize = parseInt((dom.fontSize && dom.fontSize.value) || '20', 10);

    if (tool === 'text') {
      var text = new fabric.IText('Text', {
        left: p.x,
        top: p.y,
        fill: color,
        fontSize: fontSize,
        fontFamily: 'sans-serif',
        editable: true,
      });
      state.canvas.add(text);
      state.canvas.setActiveObject(text);
      // Fabric v6: enterEditing puts the IText in inline-edit mode.
      try { text.enterEditing(); text.selectAll(); } catch (_) { /* harmless */ }
      // Snap back to select once placed so the next click doesn't keep
      // adding new text boxes.
      setActiveTool('select');
      return;
    }

    if (tool === 'arrow') {
      var line = new fabric.Line([p.x, p.y, p.x, p.y], {
        stroke: color,
        strokeWidth: stroke,
        selectable: false,
        evented: false,
      });
      state.canvas.add(line);
      state.drawing = { type: 'arrow', obj: line, startX: p.x, startY: p.y, color: color, stroke: stroke };
      return;
    }

    if (tool === 'rectangle') {
      var rect = new fabric.Rect({
        left: p.x, top: p.y, width: 1, height: 1,
        fill: 'transparent', stroke: color, strokeWidth: stroke,
        selectable: false, evented: false,
      });
      state.canvas.add(rect);
      state.drawing = { type: 'rectangle', obj: rect, startX: p.x, startY: p.y };
      return;
    }

    if (tool === 'circle') {
      var el = new fabric.Ellipse({
        left: p.x, top: p.y, rx: 1, ry: 1,
        fill: 'transparent', stroke: color, strokeWidth: stroke,
        selectable: false, evented: false,
      });
      state.canvas.add(el);
      state.drawing = { type: 'circle', obj: el, startX: p.x, startY: p.y };
      return;
    }
  }

  function onCanvasMouseMove(opt) {
    if (!state.drawing) return;
    var p = getPointer(opt);
    var d = state.drawing;
    if (d.type === 'arrow') {
      d.obj.set({ x2: p.x, y2: p.y });
    } else if (d.type === 'rectangle') {
      var x = Math.min(d.startX, p.x);
      var y = Math.min(d.startY, p.y);
      d.obj.set({
        left: x, top: y,
        width: Math.abs(p.x - d.startX),
        height: Math.abs(p.y - d.startY),
      });
    } else if (d.type === 'circle') {
      var cx = (d.startX + p.x) / 2;
      var cy = (d.startY + p.y) / 2;
      var rx = Math.abs(p.x - d.startX) / 2;
      var ry = Math.abs(p.y - d.startY) / 2;
      d.obj.set({
        left: cx - rx, top: cy - ry,
        rx: Math.max(1, rx), ry: Math.max(1, ry),
      });
    }
    d.obj.setCoords();
    state.canvas.requestRenderAll();
  }

  function onCanvasMouseUp() {
    if (!state.drawing) return;
    var d = state.drawing;
    var canvas = state.canvas;

    if (d.type === 'arrow') {
      // Replace the temporary line with a Group(line + triangle head).
      // Doing the swap on mouse-up keeps the in-progress drag cheap
      // (just a single line) and means the arrow head only ever needs
      // to be positioned once.
      var x1 = d.obj.x1, y1 = d.obj.y1, x2 = d.obj.x2, y2 = d.obj.y2;
      canvas.remove(d.obj);
      if (Math.abs(x2 - x1) < 2 && Math.abs(y2 - y1) < 2) {
        // Treat a near-zero drag as a cancel — don't litter the canvas
        // with invisible 0-length arrows.
        state.drawing = null;
        setActiveTool('select');
        return;
      }
      var arrow = makeArrow(x1, y1, x2, y2, d.color, d.stroke);
      canvas.add(arrow);
      canvas.setActiveObject(arrow);
    } else {
      // Make the new shape selectable now that drawing is finished, and
      // re-add it so Fabric's selection layer picks it up cleanly.
      d.obj.set({ selectable: true, evented: true });
      canvas.setActiveObject(d.obj);
    }

    state.drawing = null;
    setActiveTool('select');
    canvas.requestRenderAll();
  }

  // Build an arrow as a Group(line + triangle) so the user can drag /
  // rotate / resize the whole thing as a single object.
  function makeArrow(x1, y1, x2, y2, color, stroke) {
    var dx = x2 - x1;
    var dy = y2 - y1;
    var angle = Math.atan2(dy, dx) * 180 / Math.PI;
    var headSize = Math.max(10, stroke * 4);

    var line = new fabric.Line([x1, y1, x2, y2], {
      stroke: color,
      strokeWidth: stroke,
      originX: 'center',
      originY: 'center',
    });
    var head = new fabric.Triangle({
      left: x2,
      top: y2,
      originX: 'center',
      originY: 'center',
      width: headSize,
      height: headSize,
      fill: color,
      angle: angle + 90, // Triangle's default points up; rotate to match line
    });

    return new fabric.Group([line, head], {
      selectable: true,
      evented: true,
    });
  }

  // -------- Image upload ---------------------------------------------

  function onUploadClick() {
    if (dom.imageInput) dom.imageInput.click();
  }

  async function onImagePicked(evt) {
    var file = evt.target.files && evt.target.files[0];
    if (!file) return;
    setSaveStatus('saving', 'Uploading image…');
    try {
      var resp = await uploadImage(file);
      var url = API + '/api/projects/' + state.projectId + '/images/' + resp.id + '/view';
      await addBackgroundImage(url);
      setSaveStatus('saved', 'Image added');
      flushCanvasSave();
    } catch (e) {
      setSaveStatus('error', 'Image upload failed');
    } finally {
      // Reset the input so picking the same file twice still fires change.
      evt.target.value = '';
    }
  }

  async function addBackgroundImage(url) {
    // Drop any previous background image first — Phase 1 supports one
    // background per step.
    removeBackgroundImage();

    // v6: fabric.Image.fromURL returns a Promise. (v5 used a callback.)
    var img;
    try {
      img = await fabric.Image.fromURL(url, { crossOrigin: 'anonymous' });
    } catch (e) {
      throw new Error('Failed to load image into canvas');
    }
    if (!img) throw new Error('Image returned empty');

    // Fit-into-canvas: scale so the longer side fills the canvas, then
    // centre it. Don't crop — leave a bit of empty space if the aspect
    // ratio mismatches.
    var scale = Math.min(CANVAS_W / img.width, CANVAS_H / img.height);
    img.set({
      left: (CANVAS_W - img.width * scale) / 2,
      top: (CANVAS_H - img.height * scale) / 2,
      scaleX: scale,
      scaleY: scale,
      selectable: false,
      evented: false,
      // Tag as the background object so we can find / remove it later.
      // Fabric persists custom keys through toJSON if we list them in
      // toJSON's propertiesToInclude — we do that in serializeCanvas.
      ibRole: 'background',
    });
    state.canvas.add(img);
    state.canvas.sendObjectToBack(img);
    if (dom.removeImageBtn) dom.removeImageBtn.disabled = false;
    state.canvas.requestRenderAll();
  }

  function findBackgroundImage() {
    if (!state.canvas) return null;
    var objs = state.canvas.getObjects();
    for (var i = 0; i < objs.length; i++) {
      if (objs[i].ibRole === 'background') return objs[i];
    }
    return null;
  }

  function removeBackgroundImage() {
    var bg = findBackgroundImage();
    if (bg) {
      state.canvas.remove(bg);
      state.canvas.requestRenderAll();
    }
    if (dom.removeImageBtn) dom.removeImageBtn.disabled = !findBackgroundImage();
  }

  function onRemoveImageClick() {
    if (!findBackgroundImage()) return;
    removeBackgroundImage();
    scheduleAutosave();
    scheduleSnapshot();
    updateEmptyState();
  }

  // -------- Step serialization / loading -----------------------------

  function serializeCanvas() {
    // toJSON with our custom 'ibRole' key so background-image flagging
    // survives a round-trip.
    return state.canvas.toJSON(['ibRole']);
  }

  function loadCanvasFromStep(step) {
    state.suppressEvents = true;
    var raw = (step && step.canvas_json) || '';
    var parsed = null;
    if (raw) {
      try { parsed = JSON.parse(raw); } catch (_) {
        console.warn('Step canvas_json is not valid JSON — starting blank');
      }
    }

    return new Promise(function (resolve) {
      function done() {
        state.suppressEvents = false;
        // Reset selectable flags so the active tool re-applies cleanly.
        setActiveTool(state.activeTool);
        // Background button reflects current state
        if (dom.removeImageBtn) dom.removeImageBtn.disabled = !findBackgroundImage();
        // Re-snapshot baseline — undo from here should land on the
        // freshly-loaded state, not on whatever the previous step had.
        state.undoStack = [];
        state.redoStack = [];
        state.lastSnapshot = JSON.stringify(serializeCanvas());
        updateUndoRedoButtons();
        updateEmptyState();
        resolve();
      }

      if (!parsed) {
        state.canvas.clear();
        state.canvas.backgroundColor = '#ffffff';
        state.canvas.requestRenderAll();
        done();
        return;
      }

      try {
        state.canvas.loadFromJSON(parsed, function () {
          state.canvas.requestRenderAll();
          done();
        });
      } catch (e) {
        console.warn('loadFromJSON failed, falling back to blank canvas', e);
        state.canvas.clear();
        state.canvas.requestRenderAll();
        done();
      }
    });
  }

  // -------- Autosave (canvas) ----------------------------------------

  function scheduleAutosave() {
    if (!state.activeStepId) return;
    debounce('canvas-autosave', AUTOSAVE_MS, function () { flushCanvasSave(); });
  }

  async function flushCanvasSave(isRetry) {
    if (!state.activeStepId) return;
    var json = JSON.stringify(serializeCanvas());
    setSaveStatus('saving', 'Saving…');
    try {
      await putStep(state.activeStepId, { canvas_json: json });
      setSaveStatus('saved');
    } catch (e) {
      setSaveStatus('error', isRetry ? 'Save failed again' : 'Save failed');
    }
  }

  // -------- Step title / description autosave ------------------------

  function wireStepFields() {
    if (dom.stepTitle) {
      dom.stepTitle.addEventListener('blur', function () {
        if (!state.activeStepId) return;
        debounce('step-title', 100, function () {
          var stepId = state.activeStepId;
          var value = dom.stepTitle.value || null;
          setSaveStatus('saving', 'Saving…');
          putStep(stepId, { title: value })
            .then(function (updated) {
              // Reflect in the steps-list label without re-fetching.
              var s = state.steps.find(function (x) { return x.id === stepId; });
              if (s) s.title = updated.title;
              renderStepsList();
              setSaveStatus('saved');
            })
            .catch(function () { setSaveStatus('error', 'Title save failed'); });
        });
      });
    }
    if (dom.stepDescription) {
      dom.stepDescription.addEventListener('blur', function () {
        if (!state.activeStepId) return;
        debounce('step-desc', 100, function () {
          var stepId = state.activeStepId;
          var value = dom.stepDescription.value || null;
          setSaveStatus('saving', 'Saving…');
          putStep(stepId, { description: value })
            .then(function (updated) {
              var s = state.steps.find(function (x) { return x.id === stepId; });
              if (s) s.description = updated.description;
              setSaveStatus('saved');
            })
            .catch(function () { setSaveStatus('error', 'Description save failed'); });
        });
      });
    }
  }

  // -------- Undo / redo ----------------------------------------------

  function scheduleSnapshot() {
    debounce('undo-snapshot', UNDO_DEBOUNCE_MS, function () { takeSnapshot(); });
  }

  function takeSnapshot() {
    var snap = JSON.stringify(serializeCanvas());
    if (snap === state.lastSnapshot) return;
    if (state.lastSnapshot !== null) {
      state.undoStack.push(state.lastSnapshot);
      if (state.undoStack.length > UNDO_STACK_LIMIT) state.undoStack.shift();
    }
    state.lastSnapshot = snap;
    // Any new edit invalidates the redo branch — standard undo state machine.
    state.redoStack = [];
    updateUndoRedoButtons();
  }

  function applySnapshot(jsonStr) {
    var parsed;
    try { parsed = JSON.parse(jsonStr); } catch (_) { return; }
    state.suppressEvents = true;
    state.canvas.loadFromJSON(parsed, function () {
      state.canvas.requestRenderAll();
      state.suppressEvents = false;
      state.lastSnapshot = jsonStr;
      setActiveTool(state.activeTool);
      if (dom.removeImageBtn) dom.removeImageBtn.disabled = !findBackgroundImage();
      updateEmptyState();
      updateUndoRedoButtons();
      scheduleAutosave();
    });
  }

  function onUndoClick() {
    if (state.undoStack.length === 0) return;
    var prev = state.undoStack.pop();
    if (state.lastSnapshot !== null) state.redoStack.push(state.lastSnapshot);
    applySnapshot(prev);
  }

  function onRedoClick() {
    if (state.redoStack.length === 0) return;
    var next = state.redoStack.pop();
    if (state.lastSnapshot !== null) state.undoStack.push(state.lastSnapshot);
    applySnapshot(next);
  }

  function updateUndoRedoButtons() {
    if (dom.undoBtn) dom.undoBtn.disabled = state.undoStack.length === 0;
    if (dom.redoBtn) dom.redoBtn.disabled = state.redoStack.length === 0;
  }

  // -------- Transform controls (rotation, reset) ---------------------

  function onRotationInputChange() {
    if (!state.canvas || !dom.rotation) return;
    var active = state.canvas.getActiveObject();
    if (!active || active.ibRole === 'background') return;
    var raw = parseInt(dom.rotation.value, 10);
    if (isNaN(raw)) return;
    // Normalise to 0–359 so wrap-around (e.g. -10 → 350) feels natural.
    var deg = ((raw % 360) + 360) % 360;
    active.set('angle', deg);
    active.setCoords();
    state.canvas.requestRenderAll();
    // Manually fire `object:modified` so autosave + undo snapshot pick
    // up the change — `set('angle', …)` doesn't emit that on its own.
    state.canvas.fire('object:modified', { target: active });
  }

  function onResetTransformsClick() {
    if (!state.canvas) return;
    var active = state.canvas.getActiveObject();
    if (!active || active.ibRole === 'background') return;
    // Reset scale / rotation / skew to identity. We deliberately leave
    // `left` / `top` alone — we don't remember the object's first-placed
    // position, and snapping it to the canvas origin would feel jarring.
    // The user can drag it back if they want.
    active.set({
      scaleX: 1,
      scaleY: 1,
      angle: 0,
      skewX: 0,
      skewY: 0,
      flipX: false,
      flipY: false,
    });
    active.setCoords();
    state.canvas.requestRenderAll();
    if (dom.rotation) dom.rotation.value = '0';
    // Fire `object:modified` so autosave + undo snapshot capture it.
    state.canvas.fire('object:modified', { target: active });
  }

  // -------- Arrange / layer ordering ---------------------------------

  // After any layer reorder, re-pin the locked background to the very
  // bottom of the stack so the user can't accidentally bury it.
  function reanchorBackground() {
    var bg = findBackgroundImage();
    if (bg) state.canvas.sendObjectToBack(bg);
  }

  function arrangeActive(op) {
    if (!state.canvas) return;
    var active = state.canvas.getActiveObject();
    if (!active || active.ibRole === 'background') return;
    // Fabric v6 renamed v5's `bringToFront(obj)` style to methods on
    // the canvas that take the object: `bringObjectToFront`,
    // `bringObjectForward`, `sendObjectBackwards`, `sendObjectToBack`.
    if (op === 'front') state.canvas.bringObjectToFront(active);
    else if (op === 'forward') state.canvas.bringObjectForward(active);
    else if (op === 'backward') state.canvas.sendObjectBackwards(active);
    else if (op === 'back') state.canvas.sendObjectToBack(active);
    reanchorBackground();
    state.canvas.requestRenderAll();
    // Fire `object:modified` so autosave + undo snapshot trigger.
    state.canvas.fire('object:modified', { target: active });
  }

  // -------- Delete selected ------------------------------------------

  function onDeleteSelected() {
    if (!state.canvas) return;
    var active = state.canvas.getActiveObjects();
    if (!active || active.length === 0) return;
    active.forEach(function (o) {
      // Don't let the user delete the background via the toolbar —
      // they should use "Remove image" so the button state stays sane.
      if (o.ibRole === 'background') return;
      state.canvas.remove(o);
    });
    state.canvas.discardActiveObject();
    state.canvas.requestRenderAll();
    onSelectionChanged();
  }

  // -------- Step list / switching ------------------------------------

  function renderStepsList() {
    if (!dom.stepsList) return;
    if (state.steps.length === 0) {
      dom.stepsList.innerHTML =
        '<li class="text-muted small fst-italic" style="cursor:default;background:transparent">' +
        'No steps yet — click <strong>Add step</strong> below.</li>';
      return;
    }
    var html = state.steps.map(function (s) {
      var isActive = (s.id === state.activeStepId);
      var label = s.title && s.title.trim() ? s.title : 'Untitled step';
      return (
        '<li data-step-id="' + s.id + '" class="' + (isActive ? 'is-active' : '') + '">' +
          '<span class="ib-step-num">' + s.step_number + '</span>' +
          '<span class="ib-step-label">' + escapeHtml(label) + '</span>' +
        '</li>'
      );
    }).join('');
    dom.stepsList.innerHTML = html;
    Array.prototype.forEach.call(
      dom.stepsList.querySelectorAll('li[data-step-id]'),
      function (li) {
        li.addEventListener('click', function () {
          var id = parseInt(li.dataset.stepId, 10);
          if (id && id !== state.activeStepId) switchToStep(id);
        });
      }
    );

    // Update step badge ("Step N of M")
    if (dom.stepBadge) {
      var idx = state.steps.findIndex(function (s) { return s.id === state.activeStepId; });
      if (idx >= 0) {
        dom.stepBadge.textContent = 'Step ' + (idx + 1) + ' of ' + state.steps.length;
      } else if (state.steps.length === 0) {
        dom.stepBadge.textContent = 'No steps';
      }
    }
  }

  async function switchToStep(stepId) {
    // Flush any pending canvas save for the OUTGOING step before we
    // swap the canvas out — otherwise the debounced save would PUT the
    // new step's content to the old step's id.
    if (_debounces['canvas-autosave']) {
      clearTimeout(_debounces['canvas-autosave']);
      delete _debounces['canvas-autosave'];
      try { await flushCanvasSave(); } catch (_) { /* user can retry from indicator */ }
    }

    state.activeStepId = stepId;
    var step = state.steps.find(function (s) { return s.id === stepId; });
    if (!step) return;

    // Reset field values for the new step
    if (dom.stepTitle) dom.stepTitle.value = step.title || '';
    if (dom.stepDescription) dom.stepDescription.value = step.description || '';

    await loadCanvasFromStep(step);
    renderStepsList();
  }

  async function onAddStep() {
    dom.addStepBtn.disabled = true;
    setSaveStatus('saving', 'Adding step…');
    try {
      var step = await postStep();
      // Refetch the canonical list so step_number reordering (if any)
      // stays consistent with the backend.
      var fresh = await fetchInstruction();
      if (fresh && fresh.steps) {
        state.steps = fresh.steps;
      } else {
        state.steps.push(step);
      }
      await switchToStep(step.id);
      setSaveStatus('saved');
    } catch (e) {
      setSaveStatus('error', 'Add step failed');
    } finally {
      dom.addStepBtn.disabled = false;
    }
  }

  // -------- Export (Phase 2b PDF / Phase 2c ZIP) ---------------------
  //
  // Both paths share a single "render every step to a PNG dataURL"
  // helper: the PDF path POSTs the array to the API and downloads the
  // server response; the ZIP path bundles them client-side with JSZip
  // (no backend involvement).
  //
  // The render helper mirrors the off-screen StaticCanvas pattern used
  // by ``instruction-viewer.js`` — duplicated rather than shared since
  // the two pages don't currently bundle code together. If a future
  // phase extracts a shared canvas module both files can dedupe.

  function setExportProgress(msg, kind) {
    if (!dom.exportProgress) return;
    dom.exportProgress.classList.remove('d-none', 'is-error');
    if (kind === 'error') dom.exportProgress.classList.add('is-error');
    if (msg) {
      dom.exportProgress.textContent = msg;
    } else {
      dom.exportProgress.classList.add('d-none');
      dom.exportProgress.textContent = '';
    }
  }

  function hideExportProgress(delay) {
    if (!dom.exportProgress) return;
    setTimeout(function () { setExportProgress(null); }, delay || 0);
  }

  // Parse a canvas_json blob defensively. Same shape as the viewer's
  // helper — accept legitimate scenes, reject empty/garbage so the
  // off-screen render skips them rather than producing a blank PNG.
  function parseCanvasJsonForExport(raw) {
    if (!raw) return null;
    var trimmed = String(raw).trim();
    if (!trimmed) return null;
    if (trimmed === '{}' || trimmed === '[]' || trimmed === 'null') return null;
    try {
      var parsed = JSON.parse(trimmed);
      if (!parsed) return null;
      var objs = Array.isArray(parsed.objects) ? parsed.objects : [];
      if (objs.length === 0 && !parsed.backgroundImage && !parsed.background) {
        return null;
      }
      return parsed;
    } catch (_) {
      return null;
    }
  }

  // Render one step into a PNG dataURL using the supplied off-screen
  // canvas. ``multiplier: 2`` produces a higher-res image than the
  // builder's on-screen rendering — better for print + downstream tools.
  // Returns a placeholder blank-PNG dataURL when the step has no canvas
  // content so the export still has one image per step.
  function renderStepToDataUrl(off, step) {
    return new Promise(function (resolve) {
      var parsed = parseCanvasJsonForExport(step.canvas_json);
      function blankDataUrl() {
        // Render an empty white canvas at multiplier 2.
        try {
          off.clear();
          off.backgroundColor = '#ffffff';
          off.renderAll();
          resolve(off.toDataURL({ format: 'png', multiplier: 2 }));
        } catch (_) {
          // 1x1 transparent PNG as the absolute fallback.
          resolve('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII=');
        }
      }
      if (!parsed) { blankDataUrl(); return; }
      try {
        off.loadFromJSON(parsed, function () {
          try {
            off.renderAll();
            var url = off.toDataURL({ format: 'png', multiplier: 2 });
            resolve(url);
          } catch (_) {
            resolve(null);
          }
          try { off.clear(); off.backgroundColor = '#ffffff'; } catch (_) {}
        });
      } catch (_) {
        blankDataUrl();
      }
    });
  }

  // Flush any pending canvas autosave so the next API read sees the
  // latest state. Mirrors the inline flush in ``switchToStep`` but
  // exposed as a helper so the export flow can call it without
  // duplicating the timer handling.
  async function flushPendingCanvasSave() {
    if (_debounces['canvas-autosave']) {
      clearTimeout(_debounces['canvas-autosave']);
      delete _debounces['canvas-autosave'];
      try { await flushCanvasSave(); } catch (_) { /* surfaced on indicator */ }
    }
  }

  // Pull the canonical step list from the API after the autosave flush.
  // The returned list is what the export pipeline renders — so any
  // unsaved title / description / canvas state has to make it through
  // autosave first.
  async function loadStepsForExport() {
    var instruction = await fetchInstruction();
    if (!instruction || !Array.isArray(instruction.steps) || instruction.steps.length === 0) {
      throw new Error('No steps to export');
    }
    var sorted = instruction.steps.slice().sort(function (a, b) {
      return (a.step_number || 0) - (b.step_number || 0);
    });
    return sorted;
  }

  // Render every step to a PNG dataURL. Reuses a single off-screen
  // StaticCanvas across all steps to keep the cost down.
  async function renderAllStepsToDataUrls(steps) {
    if (typeof fabric === 'undefined' || !fabric.StaticCanvas) {
      throw new Error('Fabric.js is not loaded');
    }
    var offEl = document.createElement('canvas');
    offEl.width = CANVAS_W;
    offEl.height = CANVAS_H;
    var off = new fabric.StaticCanvas(offEl, {
      width: CANVAS_W,
      height: CANVAS_H,
      backgroundColor: '#ffffff',
      enableRetinaScaling: false,
    });
    var out = [];
    try {
      for (var i = 0; i < steps.length; i++) {
        setExportProgress('Rendering step ' + (i + 1) + ' of ' + steps.length + '…');
        var url = await renderStepToDataUrl(off, steps[i]);
        out.push(url);
      }
    } finally {
      try { off.dispose(); } catch (_) {}
    }
    return out;
  }

  function triggerDownload(blob, filename) {
    var url = URL.createObjectURL(blob);
    var a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    // Defer revoke so Safari has a moment to start the download.
    setTimeout(function () { URL.revokeObjectURL(url); }, 1000);
  }

  async function exportPdf(layout) {
    setExportProgress('Preparing export…');
    try {
      await flushPendingCanvasSave();
      var steps = await loadStepsForExport();
      var dataUrls = await renderAllStepsToDataUrls(steps);

      setExportProgress('Building PDF…');
      var payload = {
        steps: steps.map(function (s, i) {
          return {
            step_number: s.step_number || (i + 1),
            title: s.title || null,
            description: s.description || null,
            image_data_url: dataUrls[i],
          };
        }),
        steps_per_page: layout,
        include_title_page: true,
        project_title: (state.project && state.project.title) || null,
      };
      var resp = await apiFetchWithTermsRetry(
        API + '/api/projects/' + state.projectId + '/instruction/export/pdf',
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload),
        }
      );
      if (!resp.ok) {
        throw new Error('Server returned HTTP ' + resp.status);
      }
      var blob = await resp.blob();
      setExportProgress('Downloading…');
      triggerDownload(blob, 'instructions-' + state.projectId + '.pdf');
      hideExportProgress(1500);
    } catch (e) {
      setExportProgress('Export failed — please try again', 'error');
      hideExportProgress(4000);
    }
  }

  // Phase 3b/3c — Animated GIF / MP4 video export. Same render-then-POST
  // pipeline as the PDF path; the server runs FFmpeg over the supplied
  // PNG sequence + branded closing slide. ``format`` is either 'gif' or
  // 'mp4' and chooses the endpoint suffix + filename extension +
  // download MIME type.
  async function exportVideo(format) {
    var label = (format === 'mp4') ? 'MP4 video' : 'animated GIF';
    setExportProgress('Preparing export…');
    try {
      await flushPendingCanvasSave();
      var steps = await loadStepsForExport();
      var dataUrls = await renderAllStepsToDataUrls(steps);

      // Video encodes take noticeably longer than PDFs (FFmpeg + a
      // two-pass GIF palettegen or an H.264 single pass) — keep the
      // user informed so they don't think the page is hung. The
      // backend caps each FFmpeg invocation at 120s; the frontend
      // doesn't set its own fetch timeout — we let the server be the
      // source of truth on "too long".
      setExportProgress('Encoding ' + label + '… this can take a minute');
      var payload = {
        steps: steps.map(function (s, i) {
          return {
            step_number: s.step_number || (i + 1),
            title: s.title || null,
            description: s.description || null,
            image_data_url: dataUrls[i],
          };
        }),
        steps_per_page: 1,
        include_title_page: false,
        project_title: (state.project && state.project.title) || null,
      };
      var resp = await apiFetchWithTermsRetry(
        API + '/api/projects/' + state.projectId + '/instruction/export/' + format,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload),
        }
      );
      if (!resp.ok) {
        throw new Error('Server returned HTTP ' + resp.status);
      }
      var blob = await resp.blob();
      setExportProgress('Downloading…');
      triggerDownload(blob, 'instructions-' + state.projectId + '.' + format);
      hideExportProgress(1500);
    } catch (e) {
      setExportProgress('Export failed — please try again', 'error');
      hideExportProgress(4000);
    }
  }

  async function exportZip() {
    if (typeof window.JSZip === 'undefined') {
      setExportProgress('ZIP library failed to load — try the PDF option instead', 'error');
      hideExportProgress(4000);
      return;
    }
    setExportProgress('Preparing export…');
    try {
      await flushPendingCanvasSave();
      var steps = await loadStepsForExport();
      var dataUrls = await renderAllStepsToDataUrls(steps);

      setExportProgress('Building ZIP…');
      var zip = new window.JSZip();
      var prefix = 'data:image/png;base64,';
      var metadata = { steps: [] };
      for (var i = 0; i < steps.length; i++) {
        var url = dataUrls[i];
        var idx = String(i + 1).padStart(3, '0');
        var name = 'step-' + idx + '.png';
        if (url && url.indexOf(prefix) === 0) {
          zip.file(name, url.substring(prefix.length), { base64: true });
        }
        metadata.steps.push({
          step_number: steps[i].step_number || (i + 1),
          title: steps[i].title || null,
          description: steps[i].description || null,
          filename: name,
        });
      }
      metadata.project_title = (state.project && state.project.title) || null;
      metadata.project_id = state.projectId;
      metadata.exported_at = new Date().toISOString();
      zip.file('metadata.json', JSON.stringify(metadata, null, 2));

      var blob = await zip.generateAsync({ type: 'blob' });
      setExportProgress('Downloading…');
      triggerDownload(blob, 'instructions-' + state.projectId + '.zip');
      hideExportProgress(1500);
    } catch (e) {
      setExportProgress('Export failed — please try again', 'error');
      hideExportProgress(4000);
    }
  }

  function wireExportMenu() {
    if (!dom.exportMenuItems) return;
    Array.prototype.forEach.call(dom.exportMenuItems, function (item) {
      item.addEventListener('click', function (e) {
        e.preventDefault();
        var kind = item.dataset.export;
        if (kind === 'pdf') {
          var layout = parseInt(item.dataset.layout, 10) || 1;
          exportPdf(layout);
        } else if (kind === 'gif' || kind === 'mp4') {
          exportVideo(kind);
        } else if (kind === 'zip') {
          exportZip();
        }
      });
    });
  }

  // -------- STL viewer (Phase 3a) -----------------------------------
  //
  // STL flow vs. "Upload image" flow — important distinction:
  //
  //   * "Upload image" uploads to the server, gets a stable URL back,
  //     and places the image as a LOCKED BACKGROUND (selectable: false,
  //     evented: false, sent to back). It IS the canvas background.
  //   * "Add STL" parses the STL entirely in the browser (no server),
  //     renders it with Three.js, captures the WebGL view as a PNG, and
  //     places that PNG on the Fabric canvas as a REGULAR MOVABLE OBJECT
  //     (selectable: true, evented: true). It sits ON TOP of whatever
  //     background is already there — typically a photo of the
  //     assembly — and the user can drag / resize / rotate it.
  //
  // The STL file itself isn't persisted. If the owner wants a different
  // angle later they re-upload — the modal makes this explicit.

  // Module-level state for the STL modal. Created on first click,
  // reused thereafter (same pattern as terms-gate's _ensureModal).
  var stl = {
    modalNode: null,
    bsModal: null,
    libsPromise: null,    // cached Promise for the Three.js + STLLoader lazy-load
    fileInput: null,
    renderArea: null,
    statusText: null,
    errorBox: null,
    useBtn: null,
    cancelBtn: null,
    viewButtons: null,
    renderer: null,
    scene: null,
    camera: null,
    mesh: null,
    light: null,
    dirLight: null,
    cameraDistance: 2.6,  // initial isometric distance (matches (1.5,1.5,1.5))
    sceneReady: false,
    hasModel: false,
    // Spherical coordinates for drag-rotate (radians)
    spherical: { theta: Math.PI / 4, phi: Math.PI / 3 },
    drag: null,           // { lastX, lastY } while pointer is down
  };

  // Lazy-load Three.js + STLLoader. Returns a Promise that resolves
  // when both globals are present. Re-uses an existing in-flight load
  // if called again. Rejects with a single error message on any failure
  // so the modal can show a friendly "couldn't load the 3D viewer" line.
  function loadStlLibs() {
    if (stl.libsPromise) return stl.libsPromise;
    stl.libsPromise = new Promise(function (resolve, reject) {
      function loadScript(src, marker) {
        return new Promise(function (res, rej) {
          var existing = document.querySelector('script[data-ib-stl="' + marker + '"]');
          if (existing) {
            // Already in flight or done — hook into its lifecycle.
            if (existing.dataset.loaded === '1') { res(); return; }
            existing.addEventListener('load', function () { res(); });
            existing.addEventListener('error', function () { rej(new Error('Load failed: ' + src)); });
            return;
          }
          var s = document.createElement('script');
          s.src = src;
          s.async = true;
          s.setAttribute('data-ib-stl', marker);
          s.onload = function () { s.dataset.loaded = '1'; res(); };
          s.onerror = function () { rej(new Error('Load failed: ' + src)); };
          document.head.appendChild(s);
        });
      }
      loadScript(THREE_CDN, 'three')
        .then(function () {
          if (typeof window.THREE === 'undefined') {
            throw new Error('THREE global missing');
          }
          return loadScript(STL_LOADER_CDN, 'stlloader');
        })
        .then(function () {
          if (typeof window.THREE === 'undefined' || !window.THREE.STLLoader) {
            throw new Error('STLLoader missing');
          }
          resolve();
        })
        .catch(function (err) {
          // Reset so the next attempt re-tries the network — a CDN blip
          // shouldn't permanently disable the feature.
          stl.libsPromise = null;
          reject(err);
        });
    });
    return stl.libsPromise;
  }

  // Build the STL modal once and cache it. Modal markup mirrors the
  // Bootstrap structure used elsewhere on the site (terms-gate / project
  // editor) so the existing Bootstrap CSS handles centring + backdrop.
  function ensureStlModal() {
    if (stl.modalNode) return stl.modalNode;
    var modalId = 'ib-stl-modal';
    var html =
      '<div class="modal fade ib-stl-modal" id="' + modalId + '" tabindex="-1" ' +
      '  aria-labelledby="' + modalId + '-label" aria-hidden="true" ' +
      '  data-bs-backdrop="static" data-bs-keyboard="true">' +
      '  <div class="modal-dialog modal-dialog-centered">' +
      '    <div class="modal-content">' +
      '      <div class="modal-header">' +
      '        <h5 class="modal-title" id="' + modalId + '-label">' +
      '          <i class="fas fa-cube me-2 text-primary"></i>Add 3D model (STL)' +
      '        </h5>' +
      '        <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>' +
      '      </div>' +
      '      <div class="modal-body">' +
      '        <div class="mb-3">' +
      '          <input type="file" class="form-control form-control-sm" accept=".stl" id="ib-stl-file">' +
      '        </div>' +
      '        <div class="ib-stl-render-area" id="ib-stl-render-area">' +
      '          <div class="ib-stl-status text-muted small" id="ib-stl-status">' +
      '            Choose an STL file to preview it here.' +
      '          </div>' +
      '          <div class="ib-stl-error alert alert-danger d-none" id="ib-stl-error" role="alert"></div>' +
      '        </div>' +
      '        <div class="ib-stl-views mt-3">' +
      '          <button type="button" class="ib-stl-view-btn" data-stl-view="top">Top</button>' +
      '          <button type="button" class="ib-stl-view-btn" data-stl-view="front">Front</button>' +
      '          <button type="button" class="ib-stl-view-btn" data-stl-view="side">Side</button>' +
      '          <button type="button" class="ib-stl-view-btn is-active" data-stl-view="iso">Isometric</button>' +
      '        </div>' +
      '        <p class="ib-stl-help text-muted small mt-2 mb-0">' +
      '          Drag to rotate &middot; Scroll to zoom &middot; Large files may take a moment to load.' +
      '        </p>' +
      '        <p class="ib-stl-note text-muted small mt-1 mb-0">' +
      '          <i class="fas fa-info-circle me-1"></i>' +
      '          Only the rendered view is saved to the step — re-upload to change the angle later.' +
      '        </p>' +
      '      </div>' +
      '      <div class="modal-footer">' +
      '        <button type="button" class="btn btn-secondary" data-bs-dismiss="modal" id="ib-stl-cancel">Cancel</button>' +
      '        <button type="button" class="btn btn-primary" id="ib-stl-use" disabled>Use this view</button>' +
      '      </div>' +
      '    </div>' +
      '  </div>' +
      '</div>';
    var wrap = document.createElement('div');
    wrap.innerHTML = html;
    var node = wrap.firstElementChild;
    document.body.appendChild(node);

    stl.modalNode = node;
    stl.fileInput = node.querySelector('#ib-stl-file');
    stl.renderArea = node.querySelector('#ib-stl-render-area');
    stl.statusText = node.querySelector('#ib-stl-status');
    stl.errorBox = node.querySelector('#ib-stl-error');
    stl.useBtn = node.querySelector('#ib-stl-use');
    stl.cancelBtn = node.querySelector('#ib-stl-cancel');
    stl.viewButtons = node.querySelectorAll('.ib-stl-view-btn');

    stl.fileInput.addEventListener('change', onStlFilePicked);
    stl.useBtn.addEventListener('click', onStlUseView);
    Array.prototype.forEach.call(stl.viewButtons, function (btn) {
      btn.addEventListener('click', function () {
        onStlViewButton(btn.dataset.stlView);
      });
    });

    // Bootstrap modal instance — we let Bootstrap manage backdrop /
    // focus etc., we just call .show() / .hide().
    if (window.bootstrap && window.bootstrap.Modal) {
      stl.bsModal = new window.bootstrap.Modal(node);
    } else {
      // Bootstrap not present — synthesise minimal show/hide via the
      // 'show' class so the page still works. Unlikely on this site
      // (Bootstrap is loaded globally) but defensive against any
      // future config change.
      stl.bsModal = {
        show: function () { node.classList.add('show'); node.style.display = 'block'; },
        hide: function () { node.classList.remove('show'); node.style.display = 'none'; },
      };
    }

    // Reset state when the modal is dismissed — clear the file input
    // and any model so the next open starts fresh.
    node.addEventListener('hidden.bs.modal', function () {
      try { stl.fileInput.value = ''; } catch (_) {}
      stlClearError();
      stl.hasModel = false;
      stl.useBtn.disabled = true;
      stl.statusText.textContent = 'Choose an STL file to preview it here.';
      stl.statusText.classList.remove('d-none');
      if (stl.scene && stl.mesh) {
        try { stl.scene.remove(stl.mesh); } catch (_) {}
        if (stl.mesh.geometry) { try { stl.mesh.geometry.dispose(); } catch (_) {} }
        if (stl.mesh.material) { try { stl.mesh.material.dispose(); } catch (_) {} }
        stl.mesh = null;
      }
      if (stl.renderer) {
        try { stl.renderer.clear(); } catch (_) {}
      }
    });

    return node;
  }

  function stlShowError(msg) {
    if (!stl.errorBox) return;
    stl.errorBox.textContent = msg;
    stl.errorBox.classList.remove('d-none');
  }
  function stlClearError() {
    if (stl.errorBox) {
      stl.errorBox.textContent = '';
      stl.errorBox.classList.add('d-none');
    }
  }

  // Build the Three.js scene once per modal lifetime — we reuse it
  // across multiple file picks. Returns true on success, false if the
  // libs aren't loaded yet (caller should retry after loadStlLibs).
  function initStlScene() {
    if (stl.sceneReady) return true;
    var THREE = window.THREE;
    if (!THREE) return false;

    // Renderer — preserveDrawingBuffer is REQUIRED so toDataURL() can
    // read the framebuffer back. Without it, Chrome happily renders to
    // screen but toDataURL returns a blank PNG.
    stl.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    stl.renderer.setPixelRatio(window.devicePixelRatio || 1);
    stl.renderer.setSize(STL_VIEW_SIZE, STL_VIEW_SIZE, false);
    stl.renderer.setClearColor(STL_SCENE_BG, 1);

    // Attach the renderer's canvas inside the render area, replacing
    // the status text wrapper but keeping the error overlay alongside.
    stl.renderer.domElement.classList.add('ib-stl-canvas');
    stl.renderArea.appendChild(stl.renderer.domElement);

    stl.scene = new THREE.Scene();
    stl.scene.background = new THREE.Color(STL_SCENE_BG);

    stl.camera = new THREE.PerspectiveCamera(35, 1, 0.1, 100);
    setStlView('iso'); // also positions camera

    stl.light = new THREE.HemisphereLight(0xffffff, 0x444466, 1.0);
    stl.scene.add(stl.light);

    stl.dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
    stl.dirLight.position.copy(stl.camera.position);
    stl.scene.add(stl.dirLight);

    // Pointer / wheel input on the renderer canvas
    var el = stl.renderer.domElement;
    el.addEventListener('mousedown', onStlPointerDown);
    el.addEventListener('wheel', onStlWheel, { passive: false });
    el.addEventListener('touchstart', onStlTouchStart, { passive: false });
    el.addEventListener('touchmove', onStlTouchMove, { passive: false });
    el.addEventListener('touchend', onStlTouchEnd);

    stl.sceneReady = true;
    return true;
  }

  // Position camera + look-at for one of the four preset views.
  // Updates `cameraDistance` / `spherical` so subsequent drag-rotate /
  // zoom interactions start from the right baseline.
  function setStlView(name) {
    if (!stl.camera) return;
    var THREE = window.THREE;
    var pos;
    if (name === 'top') {
      // Tiny non-zero z so up vector resolves (camera-looks-straight-down
      // is degenerate for the default up axis).
      pos = new THREE.Vector3(0, 2, 0.001);
    } else if (name === 'front') {
      pos = new THREE.Vector3(0, 0, 2);
    } else if (name === 'side') {
      pos = new THREE.Vector3(2, 0, 0);
    } else {
      pos = new THREE.Vector3(1.5, 1.5, 1.5);
    }
    stl.camera.position.copy(pos);
    stl.camera.lookAt(0, 0, 0);
    stl.cameraDistance = pos.length();
    // Update spherical coords from new position so drag continues smoothly.
    stl.spherical.theta = Math.atan2(pos.x, pos.z);
    stl.spherical.phi = Math.acos(Math.max(-1, Math.min(1, pos.y / stl.cameraDistance)));

    if (stl.dirLight) stl.dirLight.position.copy(stl.camera.position);

    // Toggle button highlights
    if (stl.viewButtons) {
      Array.prototype.forEach.call(stl.viewButtons, function (b) {
        b.classList.toggle('is-active', b.dataset.stlView === name);
      });
    }
    renderStlScene();
  }

  function renderStlScene() {
    if (stl.renderer && stl.scene && stl.camera) {
      stl.renderer.render(stl.scene, stl.camera);
    }
  }

  // Apply current spherical coords -> camera position. Used by
  // drag-rotate and scroll-zoom. Both call renderStlScene afterwards.
  function applyStlSpherical() {
    if (!stl.camera) return;
    var r = stl.cameraDistance;
    var phi = Math.max(0.05, Math.min(Math.PI - 0.05, stl.spherical.phi));
    stl.spherical.phi = phi;
    stl.camera.position.x = r * Math.sin(phi) * Math.sin(stl.spherical.theta);
    stl.camera.position.y = r * Math.cos(phi);
    stl.camera.position.z = r * Math.sin(phi) * Math.cos(stl.spherical.theta);
    stl.camera.lookAt(0, 0, 0);
    if (stl.dirLight) stl.dirLight.position.copy(stl.camera.position);
  }

  function onStlPointerDown(e) {
    if (!stl.hasModel) return;
    e.preventDefault();
    stl.drag = { lastX: e.clientX, lastY: e.clientY };
    window.addEventListener('mousemove', onStlPointerMove);
    window.addEventListener('mouseup', onStlPointerUp);
  }
  function onStlPointerMove(e) {
    if (!stl.drag) return;
    var dx = e.clientX - stl.drag.lastX;
    var dy = e.clientY - stl.drag.lastY;
    stl.drag.lastX = e.clientX;
    stl.drag.lastY = e.clientY;
    // Convert pixel deltas to radians. Tuning: 0.005 rad/px feels
    // about right at the modal's 400px size.
    stl.spherical.theta -= dx * 0.005;
    stl.spherical.phi -= dy * 0.005;
    applyStlSpherical();
    renderStlScene();
  }
  function onStlPointerUp() {
    stl.drag = null;
    window.removeEventListener('mousemove', onStlPointerMove);
    window.removeEventListener('mouseup', onStlPointerUp);
  }
  function onStlWheel(e) {
    if (!stl.hasModel) return;
    e.preventDefault();
    var delta = Math.sign(e.deltaY);
    stl.cameraDistance = Math.max(0.5, Math.min(5.0, stl.cameraDistance + delta * 0.15));
    applyStlSpherical();
    renderStlScene();
  }
  function onStlTouchStart(e) {
    if (!stl.hasModel || e.touches.length !== 1) return;
    e.preventDefault();
    stl.drag = { lastX: e.touches[0].clientX, lastY: e.touches[0].clientY };
  }
  function onStlTouchMove(e) {
    if (!stl.drag || e.touches.length !== 1) return;
    e.preventDefault();
    var t = e.touches[0];
    var dx = t.clientX - stl.drag.lastX;
    var dy = t.clientY - stl.drag.lastY;
    stl.drag.lastX = t.clientX;
    stl.drag.lastY = t.clientY;
    stl.spherical.theta -= dx * 0.005;
    stl.spherical.phi -= dy * 0.005;
    applyStlSpherical();
    renderStlScene();
  }
  function onStlTouchEnd() {
    stl.drag = null;
  }

  function onAddStlClick() {
    var node = ensureStlModal();
    stlClearError();
    stl.bsModal.show();

    // Kick off the lib load now (even before a file is chosen) so the
    // network round-trip overlaps with the user picking a file.
    loadStlLibs()
      .then(function () {
        // Build the scene as soon as libs are ready — input file will
        // populate it. If a previous instance is already built, this
        // is a no-op.
        initStlScene();
      })
      .catch(function () {
        stlShowError("Couldn't load the 3D viewer. Please try again or check your connection.");
        // Disable the file input so the user can't pick a file we'd
        // fail to parse anyway. Cancel still works.
        if (stl.fileInput) stl.fileInput.disabled = true;
      });
  }

  function onStlFilePicked(evt) {
    var file = evt.target.files && evt.target.files[0];
    if (!file) return;
    if (file.size > STL_MAX_BYTES) {
      stlShowError('STL is too large. Please use a file under 50 MB.');
      evt.target.value = '';
      return;
    }
    stlClearError();
    stl.statusText.textContent = 'Loading…';
    stl.statusText.classList.remove('d-none');
    stl.useBtn.disabled = true;

    // Make sure libs are loaded + scene is up before we parse.
    loadStlLibs()
      .then(function () {
        if (!initStlScene()) {
          throw new Error('Scene init failed');
        }
        return file.arrayBuffer();
      })
      .then(function (buffer) {
        var THREE = window.THREE;
        var loader = new THREE.STLLoader();
        var geometry;
        try {
          geometry = loader.parse(buffer);
        } catch (_) {
          stlShowError("Couldn't parse this STL. Make sure it's a valid binary or ASCII STL.");
          stl.statusText.textContent = '';
          return;
        }
        if (!geometry) {
          stlShowError("Couldn't parse this STL. Make sure it's a valid binary or ASCII STL.");
          stl.statusText.textContent = '';
          return;
        }
        // Centre + normalise scale so the longest dimension is 1 unit.
        // STLs in the wild vary from millimetres to "blender unit"
        // arbitrary scale — normalising keeps the camera distance sane.
        geometry.computeBoundingBox();
        var bb = geometry.boundingBox;
        var cx = (bb.min.x + bb.max.x) / 2;
        var cy = (bb.min.y + bb.max.y) / 2;
        var cz = (bb.min.z + bb.max.z) / 2;
        geometry.translate(-cx, -cy, -cz);
        var sx = bb.max.x - bb.min.x;
        var sy = bb.max.y - bb.min.y;
        var sz = bb.max.z - bb.min.z;
        var longest = Math.max(sx, sy, sz) || 1;
        var scale = 1 / longest;
        geometry.scale(scale, scale, scale);
        if (geometry.computeVertexNormals) geometry.computeVertexNormals();

        // Replace any existing mesh
        if (stl.mesh) {
          stl.scene.remove(stl.mesh);
          if (stl.mesh.geometry) { try { stl.mesh.geometry.dispose(); } catch (_) {} }
          if (stl.mesh.material) { try { stl.mesh.material.dispose(); } catch (_) {} }
        }
        var material = new THREE.MeshStandardMaterial({
          color: 0xb3c8d0,
          metalness: 0.1,
          roughness: 0.5,
        });
        stl.mesh = new THREE.Mesh(geometry, material);
        stl.scene.add(stl.mesh);
        stl.hasModel = true;
        stl.useBtn.disabled = false;
        stl.statusText.textContent = '';
        stl.statusText.classList.add('d-none');
        renderStlScene();
      })
      .catch(function () {
        stlShowError("Couldn't parse this STL. Make sure it's a valid binary or ASCII STL.");
        stl.statusText.textContent = '';
      });
  }

  function onStlViewButton(name) {
    if (!stl.sceneReady) return;
    setStlView(name);
  }

  // Capture the current Three.js render as a PNG, close the modal, and
  // place the PNG on the Fabric canvas as a regular movable object.
  function onStlUseView() {
    if (!stl.hasModel || !stl.renderer) return;
    // Ensure the latest scene state is in the framebuffer before we
    // read it back. preserveDrawingBuffer keeps the pixels around but
    // a fresh render guarantees we capture the current camera pose.
    renderStlScene();
    var dataUrl;
    try {
      dataUrl = stl.renderer.domElement.toDataURL('image/png');
    } catch (_) {
      stlShowError("Couldn't capture the view. Please try a different angle.");
      return;
    }
    if (!dataUrl || dataUrl.length < 100) {
      stlShowError("Couldn't capture the view. Please try a different angle.");
      return;
    }
    // Close first so the user sees the result land on the canvas.
    try { stl.bsModal.hide(); } catch (_) {}

    placeStlPngOnCanvas(dataUrl).catch(function (e) {
      setSaveStatus('error', 'Failed to add STL view');
      console.warn('STL place failed', e);
    });
  }

  // Add a PNG dataURL to the Fabric canvas as a regular movable image.
  // Distinct from addBackgroundImage: this is NOT a locked background.
  async function placeStlPngOnCanvas(dataUrl) {
    if (!state.canvas) return;
    var img;
    try {
      // v6: fabric.Image.fromURL accepts dataURLs and returns a Promise.
      img = await fabric.Image.fromURL(dataUrl);
    } catch (e) {
      throw new Error('Failed to load STL view into canvas');
    }
    if (!img) throw new Error('STL image returned empty');

    // Place at canvas centre, scaled so the longest dim fits within
    // 50% of canvas width — leaves the user room to add overlays
    // around it.
    var targetMax = CANVAS_W * 0.5;
    var longest = Math.max(img.width, img.height) || 1;
    var scale = targetMax / longest;
    img.set({
      originX: 'center',
      originY: 'center',
      left: CANVAS_W / 2,
      top: CANVAS_H / 2,
      scaleX: scale,
      scaleY: scale,
      // Regular movable object — NOT a background. The user can drag,
      // resize, rotate, and (via Delete-selected) remove it.
      selectable: true,
      evented: true,
      ibRole: 'stl',
    });
    state.canvas.add(img);
    state.canvas.setActiveObject(img);
    state.canvas.requestRenderAll();

    // Trigger autosave so canvas_json captures the new object — the
    // object:added listener also schedules one, but calling explicitly
    // here belt-and-braces it against any future suppressEvents quirks.
    scheduleAutosave();
    scheduleSnapshot();
    updateEmptyState();
  }

  // -------- Project-images picker (issue #184) -----------------------
  //
  // Lets the owner add an image that's ALREADY been uploaded to this
  // project (via the project editor's image gallery) without having to
  // re-upload it. Distinct from the "Upload image" flow:
  //
  //   * "Upload image" → uploads a new image and places it as a LOCKED
  //     BACKGROUND (selectable: false, evented: false, sent to back).
  //     One per step; replaces any existing background.
  //   * "Add from project images" → places the picked image as a
  //     REGULAR MOVABLE OBJECT (selectable: true, evented: true), at
  //     canvas centre, scaled to ~50% width. Multiple per step are fine
  //     and they layer above any background. This is for additive
  //     composition — the user can keep stacking project images, STL
  //     views, arrows, etc.
  //
  // Modal markup is built lazily on first click and reused thereafter
  // (same pattern as the STL modal above).

  var pickerModal = {
    node: null,
    bsModal: null,
    grid: null,
    empty: null,
    loading: null,
  };

  function ensurePickerModal() {
    if (pickerModal.node) return pickerModal.node;
    var modalId = 'ib-image-picker-modal';
    var html =
      '<div class="modal fade ib-picker-modal" id="' + modalId + '" tabindex="-1" ' +
      '  aria-labelledby="' + modalId + '-label" aria-hidden="true" ' +
      '  data-bs-backdrop="static" data-bs-keyboard="true">' +
      '  <div class="modal-dialog modal-dialog-centered modal-lg">' +
      '    <div class="modal-content">' +
      '      <div class="modal-header">' +
      '        <h5 class="modal-title" id="' + modalId + '-label">' +
      '          <i class="fas fa-images me-2 text-primary"></i>Add from project images' +
      '        </h5>' +
      '        <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>' +
      '      </div>' +
      '      <div class="modal-body">' +
      '        <div class="ib-picker-loading text-muted small" id="ib-picker-loading">' +
      '          <i class="fas fa-circle-notch fa-spin me-1"></i> Loading images…' +
      '        </div>' +
      '        <div class="ib-picker-empty text-muted small d-none" id="ib-picker-empty">' +
      "          No images uploaded yet. Use <strong>Upload image</strong> to add one, or upload images from the project editor's image gallery." +
      '        </div>' +
      '        <div class="ib-picker-grid d-none" id="ib-picker-grid"></div>' +
      '      </div>' +
      '    </div>' +
      '  </div>' +
      '</div>';
    var wrap = document.createElement('div');
    wrap.innerHTML = html;
    var node = wrap.firstElementChild;
    document.body.appendChild(node);

    pickerModal.node = node;
    pickerModal.grid = node.querySelector('#ib-picker-grid');
    pickerModal.empty = node.querySelector('#ib-picker-empty');
    pickerModal.loading = node.querySelector('#ib-picker-loading');

    if (window.bootstrap && window.bootstrap.Modal) {
      pickerModal.bsModal = new window.bootstrap.Modal(node);
    } else {
      pickerModal.bsModal = {
        show: function () { node.classList.add('show'); node.style.display = 'block'; },
        hide: function () { node.classList.remove('show'); node.style.display = 'none'; },
      };
    }
    return node;
  }

  function setPickerView(view) {
    if (!pickerModal.node) return;
    pickerModal.loading.classList.toggle('d-none', view !== 'loading');
    pickerModal.empty.classList.toggle('d-none', view !== 'empty');
    pickerModal.grid.classList.toggle('d-none', view !== 'grid');
  }

  function renderPickerGrid(images) {
    if (!pickerModal.grid) return;
    var html = images.map(function (img) {
      var src = API + '/api/projects/' + state.projectId + '/images/' + img.id + '/view';
      var caption = img.caption && img.caption.trim() ? img.caption : (img.filename || 'image');
      return (
        '<button type="button" class="ib-picker-tile" data-image-id="' + img.id + '" ' +
        '        data-image-src="' + escapeHtml(src) + '" ' +
        '        title="' + escapeHtml(caption) + '">' +
        '  <img src="' + escapeHtml(src) + '" alt="" loading="lazy">' +
        '  <span class="ib-picker-tile-label">' + escapeHtml(caption) + '</span>' +
        '</button>'
      );
    }).join('');
    pickerModal.grid.innerHTML = html;
    Array.prototype.forEach.call(
      pickerModal.grid.querySelectorAll('.ib-picker-tile'),
      function (tile) {
        tile.addEventListener('click', function () {
          var src = tile.dataset.imageSrc;
          if (!src) return;
          try { pickerModal.bsModal.hide(); } catch (_) {}
          placeProjectImageOnCanvas(src).catch(function () {
            setSaveStatus('error', 'Failed to add image');
          });
        });
      }
    );
  }

  // Default "no images" copy, separated so the error branch can swap to
  // it and the next open resets back to the friendly version.
  var PICKER_EMPTY_HTML =
    "No images uploaded yet. Use <strong>Upload image</strong> to add one, " +
    "or upload images from the project editor's image gallery.";

  async function onPickProjectImageClick() {
    ensurePickerModal();
    // Reset empty-state copy so a previous transient failure doesn't
    // persist as the user's first impression on re-open.
    if (pickerModal.empty) pickerModal.empty.innerHTML = PICKER_EMPTY_HTML;
    setPickerView('loading');
    try { pickerModal.bsModal.show(); } catch (_) {}
    try {
      var resp = await apiFetch(API + '/api/projects/' + state.projectId + '/images');
      if (!resp.ok) throw new Error('HTTP ' + resp.status);
      var images = await resp.json();
      if (!Array.isArray(images) || images.length === 0) {
        setPickerView('empty');
        return;
      }
      renderPickerGrid(images);
      setPickerView('grid');
    } catch (_) {
      // Reuse the empty state for the failure path — the message tells
      // the user how to recover (re-upload from the editor) and avoids a
      // separate error UI just for this transient case.
      if (pickerModal.empty) {
        pickerModal.empty.innerHTML = "Couldn't load your project images. Please try again in a moment.";
      }
      setPickerView('empty');
    }
  }

  // Add an existing project image to the Fabric canvas as a regular
  // movable object (NOT a locked background — picker is for additive
  // layering, distinct from the "Upload image" background flow).
  async function placeProjectImageOnCanvas(url) {
    if (!state.canvas) return;
    var img;
    try {
      img = await fabric.Image.fromURL(url, { crossOrigin: 'anonymous' });
    } catch (e) {
      throw new Error('Failed to load image into canvas');
    }
    if (!img) throw new Error('Project image returned empty');

    // Centre, scaled so the longest dim fits ~50% of canvas width —
    // same placement convention as the STL flow so multiple
    // additive layers feel consistent.
    var targetMax = CANVAS_W * 0.5;
    var longest = Math.max(img.width, img.height) || 1;
    var scale = targetMax / longest;
    img.set({
      originX: 'center',
      originY: 'center',
      left: CANVAS_W / 2,
      top: CANVAS_H / 2,
      scaleX: scale,
      scaleY: scale,
      selectable: true,
      evented: true,
      // Tag so we can tell this apart from background / STL objects if
      // a future feature needs to (e.g. relinking to the source image).
      ibRole: 'project-image',
    });
    state.canvas.add(img);
    state.canvas.setActiveObject(img);
    state.canvas.requestRenderAll();
    scheduleAutosave();
    scheduleSnapshot();
    updateEmptyState();
  }

  // -------- Init flow ------------------------------------------------

  async function init() {
    bindDom();
    var params = new URLSearchParams(window.location.search);
    state.projectId = params.get('id');

    if (!state.projectId) {
      showOnly(dom.error);
      if (dom.errorDetail) dom.errorDetail.textContent =
        'No project id was passed. Open the builder from a project editor.';
      return;
    }

    // Update back / new-tab links with the project id
    if (dom.backLink) dom.backLink.href = '/projects/editor.html?id=' + encodeURIComponent(state.projectId);
    if (dom.openEditorNewTab) dom.openEditorNewTab.href = '/projects/editor.html?id=' + encodeURIComponent(state.projectId);

    // Auth probe
    try {
      state.me = await fetchMe();
    } catch (_) { state.me = null; }
    if (!state.me || !state.me.username) {
      // Not logged in — bounce to login with a return-to so we can come
      // straight back after sign-in.
      var ret = encodeURIComponent(window.location.pathname + window.location.search);
      window.location.href = '/login?return_to=' + ret;
      return;
    }

    // Load project + check ownership
    try {
      state.project = await fetchProject();
    } catch (e) {
      showOnly(dom.error);
      if (dom.errorDetail) {
        if (e.status === 404) dom.errorDetail.textContent = 'Project not found.';
        else dom.errorDetail.textContent = 'Couldn\'t load this project. Please try again.';
      }
      return;
    }

    if (dom.projectTitle) {
      dom.projectTitle.textContent = state.project.title || 'Project';
    }

    state.isOwner = (state.project.author_username === state.me.username);
    if (!state.isOwner) {
      showOnly(dom.notOwner);
      return;
    }

    // Load existing instruction + steps (or create on demand)
    var instruction;
    try {
      instruction = await fetchInstruction();
    } catch (e) {
      showOnly(dom.error);
      if (dom.errorDetail) dom.errorDetail.textContent = 'Couldn\'t load instructions for this project.';
      return;
    }

    if (instruction) {
      state.instructionId = instruction.id;
      state.steps = instruction.steps || [];
    } else {
      state.steps = [];
    }

    // If there are no steps yet, create the first one for the user so the
    // page has something to render. This mirrors what we'd want a fresh
    // visitor to see — a usable canvas, not an empty void.
    if (state.steps.length === 0) {
      try {
        var first = await postStep();
        state.steps = [first];
      } catch (e) {
        showOnly(dom.error);
        if (dom.errorDetail) dom.errorDetail.textContent = 'Couldn\'t create the first step.';
        return;
      }
    }

    // Page is good to render
    showOnly(dom.main);

    initCanvas();
    wireToolbar();
    wireStepFields();
    wireExportMenu();
    if (dom.addStepBtn) dom.addStepBtn.addEventListener('click', onAddStep);

    // Render the steps list and load the first step's canvas
    state.activeStepId = state.steps[0].id;
    if (dom.stepTitle) dom.stepTitle.value = state.steps[0].title || '';
    if (dom.stepDescription) dom.stepDescription.value = state.steps[0].description || '';
    renderStepsList();
    await loadCanvasFromStep(state.steps[0]);

    // Final layout pass once the panels have settled.
    setTimeout(syncCanvasDisplaySize, 50);
  }

  function wireToolbar() {
    // Tool buttons
    Array.prototype.forEach.call(dom.toolButtons, function (btn) {
      btn.addEventListener('click', function () {
        setActiveTool(btn.dataset.tool);
      });
    });
    // Style controls — live updates only matter for the next-drawn
    // object; existing selection is also re-styled in real time so the
    // user can pick a shape and recolour it.
    if (dom.stroke) {
      dom.stroke.addEventListener('input', function () {
        if (dom.strokeValue) dom.strokeValue.textContent = dom.stroke.value;
        applyStyleToSelection();
      });
    }
    if (dom.color) {
      dom.color.addEventListener('input', applyStyleToSelection);
    }
    if (dom.fontSize) {
      dom.fontSize.addEventListener('input', applyStyleToSelection);
    }
    if (dom.uploadImageBtn) dom.uploadImageBtn.addEventListener('click', onUploadClick);
    if (dom.pickProjectImageBtn) dom.pickProjectImageBtn.addEventListener('click', onPickProjectImageClick);
    if (dom.imageInput) dom.imageInput.addEventListener('change', onImagePicked);
    if (dom.removeImageBtn) dom.removeImageBtn.addEventListener('click', onRemoveImageClick);
    if (dom.addStlBtn) dom.addStlBtn.addEventListener('click', onAddStlClick);
    if (dom.deleteSelected) dom.deleteSelected.addEventListener('click', onDeleteSelected);
    if (dom.undoBtn) dom.undoBtn.addEventListener('click', onUndoClick);
    if (dom.redoBtn) dom.redoBtn.addEventListener('click', onRedoClick);

    // Transform controls — only fire when there's an active non-background
    // selection (controls are disabled otherwise, but bind regardless so
    // re-enable + change works without re-wiring).
    if (dom.rotation) {
      dom.rotation.addEventListener('change', onRotationInputChange);
    }
    if (dom.resetTransforms) {
      dom.resetTransforms.addEventListener('click', onResetTransformsClick);
    }

    // Arrange controls — bring-to-front / forward / backward / to-back.
    if (dom.bringToFront) dom.bringToFront.addEventListener('click', function () { arrangeActive('front'); });
    if (dom.bringForward) dom.bringForward.addEventListener('click', function () { arrangeActive('forward'); });
    if (dom.sendBackward) dom.sendBackward.addEventListener('click', function () { arrangeActive('backward'); });
    if (dom.sendToBack) dom.sendToBack.addEventListener('click', function () { arrangeActive('back'); });

    // Keyboard shortcuts: Cmd/Ctrl+Z = undo, Cmd/Ctrl+Shift+Z = redo,
    // Delete/Backspace = delete selected (but NOT while typing in a
    // text overlay or sidebar input).
    document.addEventListener('keydown', function (e) {
      var tag = (e.target && e.target.tagName) || '';
      var typing = (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT');
      var editingText = state.canvas && state.canvas.getActiveObject() &&
        state.canvas.getActiveObject().isType && state.canvas.getActiveObject().isType('i-text') &&
        state.canvas.getActiveObject().isEditing;
      if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'z') {
        if (typing || editingText) return;
        e.preventDefault();
        if (e.shiftKey) onRedoClick(); else onUndoClick();
      } else if ((e.key === 'Delete' || e.key === 'Backspace') && !typing && !editingText) {
        if (state.canvas && state.canvas.getActiveObject()) {
          e.preventDefault();
          onDeleteSelected();
        }
      }
    });
  }

  function applyStyleToSelection() {
    if (!state.canvas) return;
    var active = state.canvas.getActiveObject();
    if (!active) return;
    var color = dom.color ? dom.color.value : null;
    var stroke = dom.stroke ? parseInt(dom.stroke.value, 10) : null;
    var fontSize = dom.fontSize ? parseInt(dom.fontSize.value, 10) : null;
    // Text objects use `fill` for colour and `fontSize` for size;
    // shapes use `stroke` + `strokeWidth`. Apply whichever is relevant.
    if (active.isType && active.isType('i-text')) {
      if (color) active.set({ fill: color });
      if (fontSize) active.set({ fontSize: fontSize });
    } else {
      if (color) active.set({ stroke: color });
      if (stroke) active.set({ strokeWidth: stroke });
    }
    state.canvas.requestRenderAll();
    // Treat this as an edit so it autosaves + ends up in the undo stack.
    scheduleAutosave();
    scheduleSnapshot();
  }

  // Kick off once Fabric + DOM are both ready.
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
