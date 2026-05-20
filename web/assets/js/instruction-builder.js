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
    dom.removeImageBtn = document.getElementById('ib-remove-image-btn');
    dom.imageInput = document.getElementById('ib-image-input');
    dom.color = document.getElementById('ib-color');
    dom.stroke = document.getElementById('ib-stroke');
    dom.strokeValue = document.getElementById('ib-stroke-value');
    dom.fontSize = document.getElementById('ib-fontsize');
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
    var hasSel = !!state.canvas.getActiveObject();
    if (dom.deleteSelected) dom.deleteSelected.disabled = !hasSel;
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
        } else if (kind === 'zip') {
          exportZip();
        }
      });
    });
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
    if (dom.imageInput) dom.imageInput.addEventListener('change', onImagePicked);
    if (dom.removeImageBtn) dom.removeImageBtn.addEventListener('click', onRemoveImageClick);
    if (dom.deleteSelected) dom.deleteSelected.addEventListener('click', onDeleteSelected);
    if (dom.undoBtn) dom.undoBtn.addEventListener('click', onUndoClick);
    if (dom.redoBtn) dom.redoBtn.addEventListener('click', onRedoClick);

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
