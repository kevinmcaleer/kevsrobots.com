/**
 * Instruction Builder — A5 Activity-Rail layout (design_handoff_instructions_builder/).
 *
 * Foundation pass: zone structure + photos pane + tools pane + layers
 * pane + BOM/Assets/Circuit stubs + filmstrip + camera capture + tweaks
 * settings popover. The floating Inspector HUD and the image-context
 * toolbar above selected images are scoped to a follow-up PR.
 *
 * Engine carry-overs from the previous (three-column) implementation:
 *   - Fabric.js v6.4.0 canvas (CDN UMD, exposes window.fabric)
 *   - Per-step canvas_json persisted via PUT /steps/{id}
 *   - Step CRUD + autosave (800ms debounce) + undo/redo (50 snapshots,
 *     per-step memory)
 *   - T&Cs gate retry via window.TermsGate.handleResponse
 *   - Owner check / login redirect
 *   - STL viewer modal (Three.js v0.147 lazy-loaded)
 *   - Existing image-picker modal ("Add from project images")
 *   - Export pipeline (PDF 1/2/4 per page, GIF, MP4, ZIP)
 *
 * The functions named in those bullets keep their semantics intact; only
 * the surrounding chrome (DOM bindings, layout, new pane wiring) is new.
 */
(function () {
  'use strict';

  // ====================================================================
  // 1. CONSTANTS & MODULE STATE
  // ====================================================================

  var API = 'https://projects.kevsrobots.com';
  var CANVAS_W = 1200;
  var CANVAS_H = 900;
  var AUTOSAVE_MS = 800;
  var UNDO_DEBOUNCE_MS = 200;
  var UNDO_STACK_LIMIT = 50;

  // STL viewer (Phase 3a) — Three.js v0.147 keeps the global + classic-
  // script STLLoader contract intact; later versions broke that.
  var THREE_CDN = 'https://cdn.jsdelivr.net/npm/three@0.147.0/build/three.min.js';
  var STL_LOADER_CDN = 'https://cdn.jsdelivr.net/npm/three@0.147.0/examples/js/loaders/STLLoader.js';
  var STL_MAX_BYTES = 50 * 1024 * 1024;
  var STL_VIEW_SIZE = 400;
  var STL_SCENE_BG = 0xE1F5EE;

  // UI state — persisted to localStorage under this key. Bumped to v1 so
  // future schema changes can migrate cleanly.
  var UI_STATE_KEY = 'kr-instructions-ui-state-v1';
  var UI_STATE_DEFAULT = {
    activeRailPane: 'photos',
    secondaryPaneWidth: 240,
    hudVisible: true,
    hudPos: { x: null, y: null },
    filmstripCollapsed: false,
    showGrid: false,
    density: 'roomy',
  };

  // Filenames that look like camera captures get the camera badge in the
  // Photos pane. Anything starting with `cam_` (the name our capture
  // upload assigns) counts.
  var CAM_FILENAME_PREFIX = 'cam_';

  var state = {
    projectId: null,
    project: null,
    me: null,
    isOwner: false,
    instructionId: null,
    steps: [],
    activeStepId: null,
    activeTool: 'select',
    canvas: null,
    drawing: null,
    suppressEvents: false,
    undoStack: [],
    redoStack: [],
    lastSnapshot: null,
    // A5 additions
    projectImages: [],        // Cached project images list (Photos pane)
    selectedPhotoIds: {},     // id -> true, for shift-click multi-select
    activeRailPane: 'photos', // mirrors uiState.activeRailPane
    uiState: null,            // hydrated from localStorage in init()
    backgroundPhotoIds: {},   // photo id -> true, for the ★ badge
    railShortcutsBound: false,
  };

  // Camera capture modal — built lazily on first open.
  var cam = {
    modalNode: null,
    bsModal: null,
    video: null,
    statusText: null,
    errorBox: null,
    captureBtn: null,
    stream: null,
  };

  // ====================================================================
  // 2. DOM HANDLES
  // ====================================================================

  var dom = {};
  function bindDom() {
    // Title bar
    dom.workspace = document.getElementById('ib-workspace');
    dom.titlebar = document.getElementById('ib-titlebar');
    dom.backLink = document.getElementById('ib-back-link');
    dom.openEditorNewTab = document.getElementById('ib-open-editor-newtab');
    dom.projectTitle = document.getElementById('ib-project-title');
    dom.saveStatus = document.getElementById('ib-save-status');
    dom.exportProgress = document.getElementById('ib-export-progress');
    dom.exportMenuItems = document.querySelectorAll('.ib-export-menu [data-export]');
    dom.hudToggle = document.getElementById('ib-hud-toggle');

    // Top-level state regions
    dom.loading = document.getElementById('ib-loading');
    dom.notOwner = document.getElementById('ib-not-owner');
    dom.error = document.getElementById('ib-error');
    dom.errorDetail = document.getElementById('ib-error-detail');
    dom.main = document.getElementById('ib-main');

    // Rail
    dom.rail = document.getElementById('ib-rail');
    dom.railButtons = document.querySelectorAll('.ib-rail-btn[data-rail-pane]');
    dom.railSettings = document.getElementById('ib-rail-settings');

    // Secondary pane shell
    dom.secondary = document.getElementById('ib-secondary');
    dom.secondaryResize = document.getElementById('ib-secondary-resize');
    dom.panes = document.querySelectorAll('.ib-pane[data-pane]');

    // Photos pane
    dom.photosUpload = document.getElementById('ib-photos-upload');
    dom.photosCamera = document.getElementById('ib-photos-camera');
    dom.photosInput = document.getElementById('ib-photos-input');
    dom.photosGrid = document.getElementById('ib-photos-grid');

    // Tools pane — preserved from previous implementation
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
    dom.toolButtons = document.querySelectorAll('.ib-tool-btn[data-tool]');
    dom.stepTitle = document.getElementById('ib-step-title');
    dom.stepDescription = document.getElementById('ib-step-description');

    // BOM pane stub
    dom.bomOpenEditor = document.getElementById('ib-bom-open-editor');

    // Layers pane
    dom.layersList = document.getElementById('ib-layers-list');

    // Canvas
    dom.canvasArea = document.getElementById('ib-canvas-area');
    dom.canvasEl = document.getElementById('ib-canvas');
    dom.canvasFrame = document.getElementById('ib-canvas-frame');
    dom.canvasEmpty = document.getElementById('ib-canvas-empty');
    dom.inspectorHud = document.getElementById('ib-inspector-hud');

    // Filmstrip
    dom.filmstrip = document.getElementById('ib-filmstrip');
    dom.filmstripTrack = document.getElementById('ib-filmstrip-track');
    dom.filmstripCount = document.getElementById('ib-filmstrip-count');
    dom.filmstripToggle = document.getElementById('ib-filmstrip-toggle');
    dom.filmstripAdd = document.getElementById('ib-filmstrip-add');

    // Settings popover
    dom.settingsPop = document.getElementById('ib-settings-pop');
    dom.settingsGrid = document.getElementById('ib-settings-grid');
    dom.settingsDensity = document.querySelectorAll('input[name="ib-density"]');
  }

  // ====================================================================
  // 3. GENERIC HELPERS
  // ====================================================================

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

  // ====================================================================
  // 4. UI STATE (localStorage)
  // ====================================================================

  function loadUiState() {
    try {
      var raw = window.localStorage.getItem(UI_STATE_KEY);
      if (!raw) return Object.assign({}, UI_STATE_DEFAULT);
      var parsed = JSON.parse(raw);
      if (!parsed || typeof parsed !== 'object') return Object.assign({}, UI_STATE_DEFAULT);
      return Object.assign({}, UI_STATE_DEFAULT, parsed);
    } catch (_) {
      return Object.assign({}, UI_STATE_DEFAULT);
    }
  }

  function saveUiState() {
    if (!state.uiState) return;
    debounce('ui-state-save', 250, function () {
      try {
        window.localStorage.setItem(UI_STATE_KEY, JSON.stringify(state.uiState));
      } catch (_) { /* localStorage may be unavailable / full; non-fatal */ }
    });
  }

  function applyUiStateToDom() {
    var ui = state.uiState;
    if (!ui) return;
    // Density
    if (dom.workspace) dom.workspace.setAttribute('data-density', ui.density || 'roomy');
    Array.prototype.forEach.call(dom.settingsDensity || [], function (radio) {
      radio.checked = (radio.value === ui.density);
    });
    // Grid
    if (dom.settingsGrid) dom.settingsGrid.checked = !!ui.showGrid;
    if (dom.canvasFrame) dom.canvasFrame.classList.toggle('is-grid', !!ui.showGrid);
    // Secondary pane width
    if (dom.secondary && typeof ui.secondaryPaneWidth === 'number') {
      dom.secondary.style.width = Math.max(200, Math.min(400, ui.secondaryPaneWidth)) + 'px';
    }
    // HUD toggle visual — the panel itself is empty in this PR.
    if (dom.hudToggle) {
      dom.hudToggle.setAttribute('aria-pressed', ui.hudVisible ? 'true' : 'false');
      dom.hudToggle.classList.toggle('is-active', !!ui.hudVisible);
    }
    document.body.classList.toggle('hud-visible', !!ui.hudVisible);
    // Filmstrip collapsed
    if (dom.filmstrip) dom.filmstrip.classList.toggle('is-collapsed', !!ui.filmstripCollapsed);
    updateFilmstripToggleLabel();
    // Active rail pane
    setActiveRailPane(ui.activeRailPane || 'photos', { persist: false });
  }

  // ====================================================================
  // 5. BACKEND CALLS (preserved verbatim)
  // ====================================================================

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

  async function deleteStep(stepId) {
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction/steps/' + stepId,
      { method: 'DELETE' }
    );
    if (!resp.ok && resp.status !== 204) throw new Error('Delete step HTTP ' + resp.status);
    return true;
  }

  async function uploadImage(file, filename) {
    var fd = new FormData();
    if (filename) {
      // Re-wrap the blob so the server sees the override filename.
      fd.append('file', file, filename);
    } else {
      fd.append('file', file);
    }
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/images',
      { method: 'POST', body: fd }
    );
    if (!resp.ok) throw new Error('Upload image HTTP ' + resp.status);
    return resp.json();
  }

  async function listProjectImages() {
    var resp = await apiFetch(API + '/api/projects/' + state.projectId + '/images');
    if (!resp.ok) throw new Error('List images HTTP ' + resp.status);
    var images = await resp.json();
    return Array.isArray(images) ? images : [];
  }

  function projectImageViewUrl(imageId) {
    return API + '/api/projects/' + state.projectId + '/images/' + imageId + '/view';
  }

  // ====================================================================
  // 6. CANVAS INIT + TOOL DISPATCH (preserved)
  // ====================================================================

  function initCanvas() {
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

    c.on('mouse:down', onCanvasMouseDown);
    c.on('mouse:move', onCanvasMouseMove);
    c.on('mouse:up', onCanvasMouseUp);

    c.on('selection:created', onSelectionChanged);
    c.on('selection:updated', onSelectionChanged);
    c.on('selection:cleared', onSelectionChanged);

    c.on('object:rotating', onObjectRotating);
    c.on('object:modified', onSelectionChanged);

    // Autosave + undo snapshot triggers + layers list re-render.
    var changeEvents = ['object:added', 'object:modified', 'object:removed'];
    changeEvents.forEach(function (evt) {
      c.on(evt, function () {
        if (state.suppressEvents) return;
        scheduleAutosave();
        scheduleSnapshot();
        updateEmptyState();
        renderLayersList();
      });
    });
    // Layers list also reacts to selection changes (so the active row
    // highlights correctly).
    c.on('selection:created', renderLayersList);
    c.on('selection:updated', renderLayersList);
    c.on('selection:cleared', renderLayersList);

    syncCanvasDisplaySize();
    window.addEventListener('resize', syncCanvasDisplaySize);

    // Drag-and-drop from the Photos pane onto the canvas.
    wireCanvasDropZone();
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

  function setActiveTool(tool) {
    state.activeTool = tool;
    Array.prototype.forEach.call(dom.toolButtons, function (btn) {
      btn.classList.toggle('is-active', btn.dataset.tool === tool);
    });
    if (state.canvas) {
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
    var isBg = !!(active && active.ibRole === 'background');
    var canTransform = hasSel && !isBg;
    if (dom.deleteSelected) dom.deleteSelected.disabled = !hasSel;
    if (dom.rotation) {
      dom.rotation.disabled = !canTransform;
      if (canTransform) {
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

  function onObjectRotating(opt) {
    if (!dom.rotation || dom.rotation.disabled) return;
    var t = opt && opt.target;
    if (!t) return;
    var a = ((t.angle || 0) % 360 + 360) % 360;
    dom.rotation.value = String(Math.round(a));
  }

  // ====================================================================
  // 7. POINTER HANDLERS — arrow / rect / circle / text (preserved)
  // ====================================================================

  function getPointer(opt) {
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
      try { text.enterEditing(); text.selectAll(); } catch (_) { /* harmless */ }
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
      var x1 = d.obj.x1, y1 = d.obj.y1, x2 = d.obj.x2, y2 = d.obj.y2;
      canvas.remove(d.obj);
      if (Math.abs(x2 - x1) < 2 && Math.abs(y2 - y1) < 2) {
        state.drawing = null;
        setActiveTool('select');
        return;
      }
      var arrow = makeArrow(x1, y1, x2, y2, d.color, d.stroke);
      canvas.add(arrow);
      canvas.setActiveObject(arrow);
    } else {
      d.obj.set({ selectable: true, evented: true });
      canvas.setActiveObject(d.obj);
    }

    state.drawing = null;
    setActiveTool('select');
    canvas.requestRenderAll();
  }

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
      angle: angle + 90,
    });

    return new fabric.Group([line, head], {
      selectable: true,
      evented: true,
    });
  }

  // ====================================================================
  // 8. IMAGE UPLOAD + BACKGROUND IMAGE (preserved)
  // ====================================================================

  function onUploadClick() {
    if (dom.imageInput) dom.imageInput.click();
  }

  async function onImagePicked(evt) {
    var file = evt.target.files && evt.target.files[0];
    if (!file) return;
    setSaveStatus('saving', 'Uploading image…');
    try {
      var resp = await uploadImage(file);
      var url = projectImageViewUrl(resp.id);
      await addBackgroundImage(url);
      setSaveStatus('saved', 'Image added');
      flushCanvasSave();
      // Refresh the Photos pane so the new image shows up there too.
      refreshProjectImages();
    } catch (e) {
      setSaveStatus('error', 'Image upload failed');
    } finally {
      evt.target.value = '';
    }
  }

  async function addBackgroundImage(url) {
    removeBackgroundImage();

    var img;
    try {
      img = await fabric.Image.fromURL(url, { crossOrigin: 'anonymous' });
    } catch (e) {
      throw new Error('Failed to load image into canvas');
    }
    if (!img) throw new Error('Image returned empty');

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
      ibSourceUrl: url,
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

  // ====================================================================
  // 9. STEP SERIALIZATION / LOADING (preserved)
  // ====================================================================

  function serializeCanvas() {
    // toJSON with our custom 'ibRole' key so background-image flagging
    // survives a round-trip, plus ibSourceUrl so we can detect which
    // photo a step uses as a background (drives the ★ badge in Photos).
    return state.canvas.toJSON(['ibRole', 'ibSourceUrl']);
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
        setActiveTool(state.activeTool);
        if (dom.removeImageBtn) dom.removeImageBtn.disabled = !findBackgroundImage();
        state.undoStack = [];
        state.redoStack = [];
        state.lastSnapshot = JSON.stringify(serializeCanvas());
        updateUndoRedoButtons();
        updateEmptyState();
        renderLayersList();
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

  // ====================================================================
  // 10. AUTOSAVE — canvas + step title/description (preserved)
  // ====================================================================

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
      // The set of "photos used as backgrounds across all steps" may
      // have shifted; recompute so the ★ badge updates.
      recomputeBackgroundPhotoIds();
      renderPhotosGrid();
    } catch (e) {
      setSaveStatus('error', isRetry ? 'Save failed again' : 'Save failed');
    }
  }

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
              var s = state.steps.find(function (x) { return x.id === stepId; });
              if (s) s.title = updated.title;
              renderFilmstrip();
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

  // ====================================================================
  // 11. UNDO / REDO (preserved)
  // ====================================================================

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
      renderLayersList();
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

  // ====================================================================
  // 12. TRANSFORM + ARRANGE (preserved)
  // ====================================================================

  function onRotationInputChange() {
    if (!state.canvas || !dom.rotation) return;
    var active = state.canvas.getActiveObject();
    if (!active || active.ibRole === 'background') return;
    var raw = parseInt(dom.rotation.value, 10);
    if (isNaN(raw)) return;
    var deg = ((raw % 360) + 360) % 360;
    active.set('angle', deg);
    active.setCoords();
    state.canvas.requestRenderAll();
    state.canvas.fire('object:modified', { target: active });
  }

  function onResetTransformsClick() {
    if (!state.canvas) return;
    var active = state.canvas.getActiveObject();
    if (!active || active.ibRole === 'background') return;
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
    state.canvas.fire('object:modified', { target: active });
  }

  function reanchorBackground() {
    var bg = findBackgroundImage();
    if (bg) state.canvas.sendObjectToBack(bg);
  }

  function arrangeActive(op) {
    if (!state.canvas) return;
    var active = state.canvas.getActiveObject();
    if (!active || active.ibRole === 'background') return;
    if (op === 'front') state.canvas.bringObjectToFront(active);
    else if (op === 'forward') state.canvas.bringObjectForward(active);
    else if (op === 'backward') state.canvas.sendObjectBackwards(active);
    else if (op === 'back') state.canvas.sendObjectToBack(active);
    reanchorBackground();
    state.canvas.requestRenderAll();
    state.canvas.fire('object:modified', { target: active });
  }

  function onDeleteSelected() {
    if (!state.canvas) return;
    var active = state.canvas.getActiveObjects();
    if (!active || active.length === 0) return;
    active.forEach(function (o) {
      if (o.ibRole === 'background') return;
      state.canvas.remove(o);
    });
    state.canvas.discardActiveObject();
    state.canvas.requestRenderAll();
    onSelectionChanged();
  }

  // ====================================================================
  // 13. ACTIVITY RAIL — pane switching + Cmd/Ctrl+1..6 shortcuts
  // ====================================================================

  function setActiveRailPane(name, opts) {
    opts = opts || {};
    // Clicking the currently-active pane collapses the secondary pane.
    if (state.activeRailPane === name && opts.toggleable) {
      var collapsed = dom.secondary.classList.toggle('is-collapsed');
      // Persist as "no active pane while collapsed" by leaving
      // activeRailPane unchanged so re-clicking the same button reopens
      // to the same content.
      if (collapsed) {
        // Mirror collapsed state on the rail buttons (none visually active)
        Array.prototype.forEach.call(dom.railButtons, function (btn) {
          btn.classList.remove('is-active');
        });
      } else {
        Array.prototype.forEach.call(dom.railButtons, function (btn) {
          btn.classList.toggle('is-active', btn.dataset.railPane === name);
        });
      }
      // Recompute Fabric's CSS size now that the column reflowed.
      setTimeout(syncCanvasDisplaySize, 50);
      return;
    }

    state.activeRailPane = name;
    if (dom.secondary) dom.secondary.classList.remove('is-collapsed');
    Array.prototype.forEach.call(dom.railButtons, function (btn) {
      btn.classList.toggle('is-active', btn.dataset.railPane === name);
    });
    Array.prototype.forEach.call(dom.panes, function (pane) {
      pane.classList.toggle('is-active', pane.dataset.pane === name);
    });

    // Some panes need a refresh on switch — defer one frame so the pane
    // is visible before we measure / draw into it.
    if (name === 'photos') refreshProjectImages();
    if (name === 'layers') renderLayersList();

    if (opts.persist !== false && state.uiState) {
      state.uiState.activeRailPane = name;
      saveUiState();
    }
    setTimeout(syncCanvasDisplaySize, 50);
  }

  function wireRail() {
    Array.prototype.forEach.call(dom.railButtons, function (btn) {
      btn.addEventListener('click', function () {
        setActiveRailPane(btn.dataset.railPane, { toggleable: true });
      });
    });

    // Settings gear opens the popover (Tweaks).
    if (dom.railSettings) {
      dom.railSettings.addEventListener('click', toggleSettingsPop);
    }

    // Keyboard shortcuts (Cmd/Ctrl+1..6) — bound once, ignored when
    // focus is in an input / textarea / contenteditable.
    if (state.railShortcutsBound) return;
    state.railShortcutsBound = true;
    document.addEventListener('keydown', function (e) {
      if (!(e.metaKey || e.ctrlKey)) return;
      var tag = (e.target && e.target.tagName) || '';
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      if (e.target && e.target.isContentEditable) return;
      var map = ['photos', 'tools', 'bom', 'assets', 'circuit', 'layers'];
      var idx = parseInt(e.key, 10);
      if (idx >= 1 && idx <= 6) {
        e.preventDefault();
        setActiveRailPane(map[idx - 1]);
      }
    });
  }

  // ====================================================================
  // 14. SECONDARY PANE RESIZE
  // ====================================================================

  function wireSecondaryResize() {
    if (!dom.secondaryResize || !dom.secondary) return;
    var dragging = false;
    var startX = 0;
    var startWidth = 240;

    dom.secondaryResize.addEventListener('mousedown', function (e) {
      dragging = true;
      startX = e.clientX;
      startWidth = dom.secondary.getBoundingClientRect().width;
      dom.secondaryResize.classList.add('is-dragging');
      // Disable text selection during drag.
      document.body.style.userSelect = 'none';
      e.preventDefault();
    });

    document.addEventListener('mousemove', function (e) {
      if (!dragging) return;
      var w = startWidth + (e.clientX - startX);
      w = Math.max(200, Math.min(400, w));
      dom.secondary.style.width = w + 'px';
      syncCanvasDisplaySize();
    });

    document.addEventListener('mouseup', function () {
      if (!dragging) return;
      dragging = false;
      dom.secondaryResize.classList.remove('is-dragging');
      document.body.style.userSelect = '';
      // Persist
      if (state.uiState) {
        var w = dom.secondary.getBoundingClientRect().width;
        state.uiState.secondaryPaneWidth = Math.round(w);
        saveUiState();
      }
    });
  }

  // ====================================================================
  // 15. PHOTOS PANE — list / upload / camera / drag-to-canvas
  // ====================================================================

  async function refreshProjectImages() {
    if (!dom.photosGrid) return;
    try {
      state.projectImages = await listProjectImages();
      recomputeBackgroundPhotoIds();
      renderPhotosGrid();
    } catch (_) {
      dom.photosGrid.innerHTML =
        '<div class="ib-photos-empty">Couldn\'t load your photos. Refresh to try again.</div>';
    }
  }

  // Scan the existing steps' canvas_json for objects tagged as
  // backgrounds and remember which photo each one came from so the
  // Photos grid can show the ★ badge.
  function recomputeBackgroundPhotoIds() {
    var found = {};
    var byId = {};
    (state.projectImages || []).forEach(function (img) {
      var url = projectImageViewUrl(img.id);
      byId[url] = img.id;
    });

    (state.steps || []).forEach(function (step) {
      var raw = step && step.canvas_json;
      if (!raw) return;
      try {
        var parsed = JSON.parse(raw);
        var objs = (parsed && parsed.objects) || [];
        objs.forEach(function (o) {
          if (o && o.ibRole === 'background' && o.ibSourceUrl && byId[o.ibSourceUrl]) {
            found[byId[o.ibSourceUrl]] = true;
          }
        });
      } catch (_) { /* skip malformed */ }
    });
    state.backgroundPhotoIds = found;
  }

  function renderPhotosGrid() {
    if (!dom.photosGrid) return;
    var imgs = state.projectImages || [];

    if (imgs.length === 0) {
      dom.photosGrid.innerHTML =
        '<div class="ib-photos-empty">' +
        'No photos yet. Use <strong>Upload</strong> or <strong>Camera</strong>, ' +
        'or add images from the project editor.' +
        '</div>' +
        '<button type="button" class="ib-photo-add" id="ib-photo-add-empty" title="Upload">' +
        '<i class="fas fa-plus"></i></button>';
      var add = document.getElementById('ib-photo-add-empty');
      if (add) add.addEventListener('click', function () {
        if (dom.photosInput) dom.photosInput.click();
      });
      return;
    }

    var html = imgs.map(function (img) {
      var url = projectImageViewUrl(img.id);
      var caption = img.caption && img.caption.trim() ? img.caption :
        (img.filename || 'image');
      var fname = String(img.filename || img.caption || '');
      var isCam = fname.toLowerCase().indexOf(CAM_FILENAME_PREFIX) === 0;
      var isBg = !!state.backgroundPhotoIds[img.id];
      var isSelected = !!state.selectedPhotoIds[img.id];

      var badges = '';
      if (isBg) {
        badges += '<span class="ib-photo-tile-badge is-star" title="Used as step background">' +
          '<i class="fas fa-star"></i></span>';
      }
      if (isCam) {
        badges += '<span class="ib-photo-tile-badge" title="Camera capture">' +
          '<i class="fas fa-camera"></i></span>';
      }

      return (
        '<div class="ib-photo-tile' + (isSelected ? ' is-selected' : '') + '"' +
        '     draggable="true"' +
        '     data-photo-id="' + img.id + '"' +
        '     data-photo-src="' + escapeHtml(url) + '"' +
        '     title="' + escapeHtml(caption) + '">' +
        '  <img src="' + escapeHtml(url) + '" alt="" loading="lazy">' +
        (badges ? '<div class="ib-photo-tile-badges">' + badges + '</div>' : '') +
        '  <div class="ib-photo-tile-caption">' + escapeHtml(caption) + '</div>' +
        '</div>'
      );
    }).join('');

    // Trailing + tile to trigger upload.
    html +=
      '<button type="button" class="ib-photo-add" id="ib-photo-add-trailing" title="Upload">' +
      '<i class="fas fa-plus"></i></button>';

    dom.photosGrid.innerHTML = html;

    var trailing = document.getElementById('ib-photo-add-trailing');
    if (trailing) trailing.addEventListener('click', function () {
      if (dom.photosInput) dom.photosInput.click();
    });

    // Wire each tile: click to (de)select, shift-click to toggle in
    // multi-select, drag to canvas.
    Array.prototype.forEach.call(
      dom.photosGrid.querySelectorAll('.ib-photo-tile'),
      function (tile) {
        tile.addEventListener('click', function (e) {
          var id = parseInt(tile.dataset.photoId, 10);
          if (e.shiftKey) {
            state.selectedPhotoIds[id] = !state.selectedPhotoIds[id];
            if (!state.selectedPhotoIds[id]) delete state.selectedPhotoIds[id];
            tile.classList.toggle('is-selected');
          } else {
            // Single click clears multi-select.
            state.selectedPhotoIds = {};
            Array.prototype.forEach.call(
              dom.photosGrid.querySelectorAll('.ib-photo-tile.is-selected'),
              function (t) { t.classList.remove('is-selected'); }
            );
          }
        });

        tile.addEventListener('dragstart', function (e) {
          var id = parseInt(tile.dataset.photoId, 10);
          var src = tile.dataset.photoSrc;
          var ids = [];
          // If this tile is part of a multi-select, drag all selected.
          if (state.selectedPhotoIds[id]) {
            Object.keys(state.selectedPhotoIds).forEach(function (k) {
              ids.push(parseInt(k, 10));
            });
          } else {
            ids = [id];
          }
          // Resolve to {id, src} pairs.
          var items = ids.map(function (pid) {
            var match = state.projectImages.find(function (im) { return im.id === pid; });
            if (!match) return null;
            return { id: pid, src: projectImageViewUrl(pid) };
          }).filter(Boolean);

          var payload = (items.length === 1)
            ? { kind: 'photo', id: id, src: src }
            : { kind: 'photos', items: items };
          try {
            e.dataTransfer.effectAllowed = 'copy';
            e.dataTransfer.setData('application/json', JSON.stringify(payload));
            // Fallback for browsers that strip custom types (Firefox is fine
            // with application/json, but text/plain is universal).
            e.dataTransfer.setData('text/plain', src || '');
          } catch (_) { /* setData can throw in some sandboxes */ }
        });
      }
    );
  }

  function wirePhotosPane() {
    if (dom.photosUpload) dom.photosUpload.addEventListener('click', function () {
      if (dom.photosInput) dom.photosInput.click();
    });
    if (dom.photosInput) dom.photosInput.addEventListener('change', onPhotosInputChange);
    if (dom.photosCamera) dom.photosCamera.addEventListener('click', onCameraButtonClick);
  }

  async function onPhotosInputChange(evt) {
    var files = evt.target.files;
    if (!files || files.length === 0) return;
    setSaveStatus('saving', 'Uploading photos…');
    var ok = 0, fail = 0;
    for (var i = 0; i < files.length; i++) {
      try {
        await uploadImage(files[i]);
        ok++;
      } catch (_) {
        fail++;
      }
    }
    if (ok > 0) setSaveStatus('saved', ok + ' photo' + (ok === 1 ? '' : 's') + ' added');
    if (fail > 0 && ok === 0) setSaveStatus('error', 'Photo upload failed');
    evt.target.value = '';
    await refreshProjectImages();
  }

  // ====================================================================
  // 16. CAMERA CAPTURE MODAL — getUserMedia preview + capture
  // ====================================================================

  function ensureCameraModal() {
    if (cam.modalNode) return cam.modalNode;
    cam.modalNode = document.getElementById('ib-camera-modal');
    if (!cam.modalNode) return null;
    cam.video = document.getElementById('ib-camera-video');
    cam.statusText = document.getElementById('ib-camera-status');
    cam.errorBox = document.getElementById('ib-camera-error');
    cam.captureBtn = document.getElementById('ib-camera-capture');

    if (cam.captureBtn) cam.captureBtn.addEventListener('click', onCameraCapture);

    if (window.bootstrap && window.bootstrap.Modal) {
      cam.bsModal = new window.bootstrap.Modal(cam.modalNode);
    } else {
      cam.bsModal = {
        show: function () { cam.modalNode.classList.add('show'); cam.modalNode.style.display = 'block'; },
        hide: function () { cam.modalNode.classList.remove('show'); cam.modalNode.style.display = 'none'; },
      };
    }

    // Tear down the stream when the modal closes.
    cam.modalNode.addEventListener('hidden.bs.modal', stopCameraStream);
    return cam.modalNode;
  }

  function stopCameraStream() {
    if (cam.stream) {
      try {
        cam.stream.getTracks().forEach(function (t) { t.stop(); });
      } catch (_) {}
      cam.stream = null;
    }
    if (cam.video) cam.video.srcObject = null;
    if (cam.captureBtn) cam.captureBtn.disabled = true;
  }

  function showCameraError(msg) {
    if (!cam.errorBox) return;
    cam.errorBox.innerHTML = escapeHtml(msg) +
      ' <button type="button" class="btn btn-sm btn-outline-primary ms-2" id="ib-camera-fallback">' +
      '<i class="fas fa-upload me-1"></i> Use Upload instead</button>';
    cam.errorBox.classList.remove('d-none');
    if (cam.statusText) cam.statusText.classList.add('d-none');
    var fallback = document.getElementById('ib-camera-fallback');
    if (fallback) fallback.addEventListener('click', function () {
      try { cam.bsModal.hide(); } catch (_) {}
      if (dom.photosInput) dom.photosInput.click();
    });
  }

  function clearCameraError() {
    if (!cam.errorBox) return;
    cam.errorBox.classList.add('d-none');
    cam.errorBox.textContent = '';
  }

  async function onCameraButtonClick() {
    var node = ensureCameraModal();
    if (!node) return;
    clearCameraError();
    if (cam.statusText) {
      cam.statusText.classList.remove('d-none');
      cam.statusText.textContent = 'Waiting for camera permission…';
    }
    if (cam.captureBtn) cam.captureBtn.disabled = true;
    try { cam.bsModal.show(); } catch (_) {}

    // Kick off getUserMedia.
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      showCameraError("Camera isn't available in this browser.");
      return;
    }
    try {
      cam.stream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 960 } },
        audio: false,
      });
      cam.video.srcObject = cam.stream;
      try { await cam.video.play(); } catch (_) {}
      if (cam.statusText) cam.statusText.classList.add('d-none');
      if (cam.captureBtn) cam.captureBtn.disabled = false;
    } catch (e) {
      showCameraError("Couldn't access the camera. Check the browser's permission, or use Upload instead.");
    }
  }

  function onCameraCapture() {
    if (!cam.stream || !cam.video) return;
    var w = cam.video.videoWidth || 1280;
    var h = cam.video.videoHeight || 960;
    var off = document.createElement('canvas');
    off.width = w;
    off.height = h;
    var ctx = off.getContext('2d');
    try {
      ctx.drawImage(cam.video, 0, 0, w, h);
    } catch (_) {
      showCameraError("Couldn't capture the frame. Try again.");
      return;
    }

    off.toBlob(async function (blob) {
      if (!blob) {
        showCameraError("Couldn't encode the captured frame.");
        return;
      }
      var iso = new Date().toISOString().replace(/[:.]/g, '-');
      var filename = CAM_FILENAME_PREFIX + iso + '.jpg';
      setSaveStatus('saving', 'Uploading capture…');
      try {
        await uploadImage(blob, filename);
        setSaveStatus('saved', 'Capture uploaded');
        try { cam.bsModal.hide(); } catch (_) {}
        await refreshProjectImages();
      } catch (_) {
        setSaveStatus('error', 'Capture upload failed');
      }
    }, 'image/jpeg', 0.92);
  }

  // ====================================================================
  // 17. CANVAS DROP ZONE — receive photo drags from the Photos pane
  // ====================================================================

  function wireCanvasDropZone() {
    if (!dom.canvasFrame) return;
    dom.canvasFrame.addEventListener('dragover', function (e) {
      e.preventDefault();
      e.dataTransfer.dropEffect = 'copy';
      dom.canvasFrame.classList.add('is-dropping');
    });
    dom.canvasFrame.addEventListener('dragleave', function () {
      dom.canvasFrame.classList.remove('is-dropping');
    });
    dom.canvasFrame.addEventListener('drop', async function (e) {
      e.preventDefault();
      dom.canvasFrame.classList.remove('is-dropping');
      var raw = '';
      try { raw = e.dataTransfer.getData('application/json') || ''; } catch (_) {}
      if (!raw) {
        try { raw = e.dataTransfer.getData('text/plain') || ''; } catch (_) {}
        if (raw) {
          // Bare URL — treat as a single photo drop.
          await dropPhotosOnCanvas([{ id: null, src: raw }], e);
        }
        return;
      }
      var payload;
      try { payload = JSON.parse(raw); } catch (_) { return; }
      if (!payload) return;
      if (payload.kind === 'photo') {
        await dropPhotosOnCanvas([{ id: payload.id, src: payload.src }], e);
      } else if (payload.kind === 'photos' && Array.isArray(payload.items)) {
        await dropPhotosOnCanvas(payload.items, e);
      }
    });
  }

  // Convert a canvas-frame mouse drop position into Fabric internal
  // coordinates (CANVAS_W × CANVAS_H logical, scaled to the rendered
  // size via CSS).
  function frameDropToCanvasXY(e) {
    var rect = dom.canvasFrame.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return { x: CANVAS_W / 2, y: CANVAS_H / 2 };
    var sx = CANVAS_W / rect.width;
    var sy = CANVAS_H / rect.height;
    return {
      x: (e.clientX - rect.left) * sx,
      y: (e.clientY - rect.top) * sy,
    };
  }

  async function dropPhotosOnCanvas(items, e) {
    if (!state.canvas || !items || items.length === 0) return;
    var base = frameDropToCanvasXY(e);
    for (var i = 0; i < items.length; i++) {
      try {
        var img = await fabric.Image.fromURL(items[i].src, { crossOrigin: 'anonymous' });
        if (!img) continue;
        // Cascade 20px per item so a multi-drop is visually distinct.
        var ox = base.x + i * 20;
        var oy = base.y + i * 20;
        // Scale so the longest dim is ~50% of canvas width — same
        // convention used by the existing "Add from project images"
        // flow so regular movable images feel consistent.
        var targetMax = CANVAS_W * 0.5;
        var longest = Math.max(img.width, img.height) || 1;
        var scale = targetMax / longest;
        img.set({
          originX: 'center',
          originY: 'center',
          left: ox,
          top: oy,
          scaleX: scale,
          scaleY: scale,
          selectable: true,
          evented: true,
          ibRole: 'photo',
          ibSourceUrl: items[i].src,
        });
        state.canvas.add(img);
        if (i === items.length - 1) state.canvas.setActiveObject(img);
      } catch (err) {
        console.warn('Drop failed for', items[i].src, err);
      }
    }
    state.canvas.requestRenderAll();
    scheduleAutosave();
    scheduleSnapshot();
    updateEmptyState();
    renderLayersList();
  }

  // ====================================================================
  // 18. LAYERS PANE
  // ====================================================================

  function objectKindLabel(o) {
    if (!o) return { icon: 'fas fa-question', label: 'Object' };
    if (o.ibRole === 'background') return { icon: 'fas fa-image', label: 'Background image' };
    if (o.ibRole === 'stl') return { icon: 'fas fa-cube', label: 'STL view' };
    if (o.ibRole === 'photo' || o.ibRole === 'project-image') return { icon: 'fas fa-image', label: 'Image' };
    if (o.type === 'image' || (o.isType && o.isType('image'))) return { icon: 'fas fa-image', label: 'Image' };
    if (o.type === 'i-text' || (o.isType && o.isType('i-text'))) {
      var txt = (o.text || '').trim().slice(0, 24);
      return { icon: 'fas fa-font', label: txt ? ('Text: ' + txt) : 'Text' };
    }
    if (o.type === 'rect' || (o.isType && o.isType('rect'))) return { icon: 'far fa-square', label: 'Rectangle' };
    if (o.type === 'ellipse' || (o.isType && o.isType('ellipse'))) return { icon: 'far fa-circle', label: 'Ellipse' };
    if (o.type === 'group' || (o.isType && o.isType('group'))) return { icon: 'fas fa-long-arrow-alt-right', label: 'Arrow' };
    return { icon: 'fas fa-shapes', label: o.type || 'Object' };
  }

  function renderLayersList() {
    if (!dom.layersList) return;
    if (!state.canvas) {
      dom.layersList.innerHTML = '';
      return;
    }
    var objs = state.canvas.getObjects();
    if (objs.length === 0) {
      dom.layersList.innerHTML =
        '<li class="ib-layers-list-empty">No objects on this step yet.</li>';
      return;
    }
    var active = state.canvas.getActiveObject();
    // Render top-to-bottom (the topmost canvas object first) so the
    // visual order matches what's on screen.
    var html = objs.slice().reverse().map(function (o, displayIdx) {
      var k = objectKindLabel(o);
      var isActive = (active === o);
      // We use the canvas object's index in the underlying array for
      // drop-target math, not the reversed display index.
      var realIdx = objs.indexOf(o);
      return (
        '<li class="' + (isActive ? 'is-active' : '') + '"' +
        '    draggable="true"' +
        '    data-layer-index="' + realIdx + '">' +
        '  <span class="ib-layer-icon"><i class="' + k.icon + '"></i></span>' +
        '  <span class="ib-layer-label">' + escapeHtml(k.label) + '</span>' +
        '</li>'
      );
    }).join('');
    dom.layersList.innerHTML = html;

    Array.prototype.forEach.call(dom.layersList.querySelectorAll('li[data-layer-index]'),
      function (li) {
        li.addEventListener('click', function () {
          var idx = parseInt(li.dataset.layerIndex, 10);
          var target = state.canvas.getObjects()[idx];
          if (!target) return;
          state.canvas.setActiveObject(target);
          state.canvas.requestRenderAll();
        });

        // Drag-to-reorder.
        li.addEventListener('dragstart', function (e) {
          e.dataTransfer.effectAllowed = 'move';
          try { e.dataTransfer.setData('text/x-ib-layer', li.dataset.layerIndex); } catch (_) {}
          li.classList.add('is-dragging');
        });
        li.addEventListener('dragend', function () { li.classList.remove('is-dragging'); });
        li.addEventListener('dragover', function (e) {
          e.preventDefault();
          li.classList.add('is-dragover');
        });
        li.addEventListener('dragleave', function () { li.classList.remove('is-dragover'); });
        li.addEventListener('drop', function (e) {
          e.preventDefault();
          li.classList.remove('is-dragover');
          var fromIdx;
          try { fromIdx = parseInt(e.dataTransfer.getData('text/x-ib-layer'), 10); } catch (_) {}
          var toIdx = parseInt(li.dataset.layerIndex, 10);
          if (isNaN(fromIdx) || isNaN(toIdx) || fromIdx === toIdx) return;
          reorderLayerInStack(fromIdx, toIdx);
        });
      });
  }

  // Move the object currently at `fromIdx` to `toIdx` in the Fabric
  // stack. Implemented via repeated bring/send-forward calls so the
  // background's locked position at the bottom stays intact (we re-anchor
  // it explicitly afterwards).
  function reorderLayerInStack(fromIdx, toIdx) {
    if (!state.canvas) return;
    var objs = state.canvas.getObjects();
    var obj = objs[fromIdx];
    if (!obj || obj.ibRole === 'background') return;
    state.canvas.moveObjectTo
      ? state.canvas.moveObjectTo(obj, toIdx)
      : moveObjectToFallback(obj, toIdx);
    reanchorBackground();
    state.canvas.requestRenderAll();
    state.canvas.fire('object:modified', { target: obj });
    // The list will re-render via the object:modified handler.
  }

  // Fabric v6 exposes canvas.moveObjectTo; if a future minor renames or
  // removes it we fall back to nudging the object one slot at a time.
  function moveObjectToFallback(obj, target) {
    if (!state.canvas) return;
    var cur = state.canvas.getObjects().indexOf(obj);
    if (cur < 0) return;
    while (cur < target) {
      state.canvas.bringObjectForward(obj);
      cur++;
    }
    while (cur > target) {
      state.canvas.sendObjectBackwards(obj);
      cur--;
    }
  }

  // ====================================================================
  // 19. FILMSTRIP — step thumbnails, reorder, add, collapse
  // ====================================================================

  function renderFilmstrip() {
    if (!dom.filmstripTrack) return;
    var html = state.steps.map(function (s) {
      var isActive = (s.id === state.activeStepId);
      var caption = (s.title && s.title.trim()) ? s.title : 'Untitled';
      return (
        '<div class="ib-step-tile' + (isActive ? ' is-active' : '') + '"' +
        '     draggable="true"' +
        '     data-step-id="' + s.id + '">' +
        '  <div class="ib-step-tile-num">' + s.step_number + '</div>' +
        '  <div class="ib-step-tile-preview">' +
        '    <span>step ' + s.step_number + '</span>' +
        '  </div>' +
        '  <div class="ib-step-tile-caption" title="' + escapeHtml(caption) + '">' +
                escapeHtml(caption) +
        '</div>' +
        '</div>'
      );
    }).join('');
    dom.filmstripTrack.innerHTML = html;
    if (dom.filmstripCount) {
      dom.filmstripCount.textContent = state.steps.length === 1
        ? '· 1 step'
        : '· ' + state.steps.length + ' steps';
    }

    Array.prototype.forEach.call(dom.filmstripTrack.querySelectorAll('.ib-step-tile'),
      function (tile) {
        tile.addEventListener('click', function () {
          var id = parseInt(tile.dataset.stepId, 10);
          if (id && id !== state.activeStepId) switchToStep(id);
        });
        // Drag-to-reorder — drop fires a PUT step_number.
        tile.addEventListener('dragstart', function (e) {
          e.dataTransfer.effectAllowed = 'move';
          try { e.dataTransfer.setData('text/x-ib-step', tile.dataset.stepId); } catch (_) {}
          tile.classList.add('is-dragging');
        });
        tile.addEventListener('dragend', function () { tile.classList.remove('is-dragging'); });
        tile.addEventListener('dragover', function (e) {
          e.preventDefault();
          tile.classList.add('is-dragover');
        });
        tile.addEventListener('dragleave', function () { tile.classList.remove('is-dragover'); });
        tile.addEventListener('drop', function (e) {
          e.preventDefault();
          tile.classList.remove('is-dragover');
          var fromId;
          try { fromId = parseInt(e.dataTransfer.getData('text/x-ib-step'), 10); } catch (_) {}
          var toId = parseInt(tile.dataset.stepId, 10);
          if (!fromId || !toId || fromId === toId) return;
          reorderStep(fromId, toId);
        });
      });
  }

  function updateFilmstripToggleLabel() {
    if (!dom.filmstripToggle) return;
    var collapsed = dom.filmstrip && dom.filmstrip.classList.contains('is-collapsed');
    if (collapsed) {
      dom.filmstripToggle.innerHTML = '<i class="fas fa-chevron-up"></i> show';
      dom.filmstripToggle.title = 'Show steps';
    } else {
      dom.filmstripToggle.innerHTML = '<i class="fas fa-chevron-down"></i> hide';
      dom.filmstripToggle.title = 'Hide steps';
    }
  }

  function wireFilmstrip() {
    if (dom.filmstripAdd) dom.filmstripAdd.addEventListener('click', onAddStep);
    if (dom.filmstripToggle) {
      dom.filmstripToggle.addEventListener('click', function () {
        if (!dom.filmstrip) return;
        dom.filmstrip.classList.toggle('is-collapsed');
        updateFilmstripToggleLabel();
        if (state.uiState) {
          state.uiState.filmstripCollapsed = dom.filmstrip.classList.contains('is-collapsed');
          saveUiState();
        }
        // Canvas reflows when the filmstrip collapses — resync Fabric.
        setTimeout(syncCanvasDisplaySize, 50);
      });
    }
  }

  async function reorderStep(fromId, toId) {
    var fromStep = state.steps.find(function (s) { return s.id === fromId; });
    var toStep = state.steps.find(function (s) { return s.id === toId; });
    if (!fromStep || !toStep) return;
    setSaveStatus('saving', 'Reordering steps…');
    try {
      await putStep(fromId, { step_number: toStep.step_number });
      var fresh = await fetchInstruction();
      if (fresh && fresh.steps) state.steps = fresh.steps;
      renderFilmstrip();
      setSaveStatus('saved');
    } catch (_) {
      setSaveStatus('error', 'Reorder failed');
    }
  }

  // ====================================================================
  // 20. STEP SWITCHING + ADD
  // ====================================================================

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

    if (dom.stepTitle) dom.stepTitle.value = step.title || '';
    if (dom.stepDescription) dom.stepDescription.value = step.description || '';

    await loadCanvasFromStep(step);
    renderFilmstrip();
    renderLayersList();
  }

  async function onAddStep() {
    if (dom.filmstripAdd) dom.filmstripAdd.disabled = true;
    setSaveStatus('saving', 'Adding step…');
    try {
      var step = await postStep();
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
      if (dom.filmstripAdd) dom.filmstripAdd.disabled = false;
    }
  }

  // ====================================================================
  // 21. EXPORT (PDF / GIF / MP4 / ZIP) — preserved verbatim
  // ====================================================================

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

  function renderStepToDataUrl(off, step) {
    return new Promise(function (resolve) {
      var parsed = parseCanvasJsonForExport(step.canvas_json);
      function blankDataUrl() {
        try {
          off.clear();
          off.backgroundColor = '#ffffff';
          off.renderAll();
          resolve(off.toDataURL({ format: 'png', multiplier: 2 }));
        } catch (_) {
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
  // latest state. Mirrors the inline flush in `switchToStep` but exposed
  // as a helper for the export flow.
  async function flushPendingCanvasSave() {
    if (_debounces['canvas-autosave']) {
      clearTimeout(_debounces['canvas-autosave']);
      delete _debounces['canvas-autosave'];
      try { await flushCanvasSave(); } catch (_) { /* surfaced on indicator */ }
    }
  }

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

  async function exportVideo(format) {
    var label = (format === 'mp4') ? 'MP4 video' : 'animated GIF';
    setExportProgress('Preparing export…');
    try {
      await flushPendingCanvasSave();
      var steps = await loadStepsForExport();
      var dataUrls = await renderAllStepsToDataUrls(steps);

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

  // ====================================================================
  // 22. STL VIEWER (Phase 3a — preserved verbatim)
  // ====================================================================

  var stl = {
    modalNode: null,
    bsModal: null,
    libsPromise: null,
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
    cameraDistance: 2.6,
    sceneReady: false,
    hasModel: false,
    spherical: { theta: Math.PI / 4, phi: Math.PI / 3 },
    drag: null,
  };

  function loadStlLibs() {
    if (stl.libsPromise) return stl.libsPromise;
    stl.libsPromise = new Promise(function (resolve, reject) {
      function loadScript(src, marker) {
        return new Promise(function (res, rej) {
          var existing = document.querySelector('script[data-ib-stl="' + marker + '"]');
          if (existing) {
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
          stl.libsPromise = null;
          reject(err);
        });
    });
    return stl.libsPromise;
  }

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

    if (window.bootstrap && window.bootstrap.Modal) {
      stl.bsModal = new window.bootstrap.Modal(node);
    } else {
      stl.bsModal = {
        show: function () { node.classList.add('show'); node.style.display = 'block'; },
        hide: function () { node.classList.remove('show'); node.style.display = 'none'; },
      };
    }

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

  function initStlScene() {
    if (stl.sceneReady) return true;
    var THREE = window.THREE;
    if (!THREE) return false;

    stl.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    stl.renderer.setPixelRatio(window.devicePixelRatio || 1);
    stl.renderer.setSize(STL_VIEW_SIZE, STL_VIEW_SIZE, false);
    stl.renderer.setClearColor(STL_SCENE_BG, 1);

    stl.renderer.domElement.classList.add('ib-stl-canvas');
    stl.renderArea.appendChild(stl.renderer.domElement);

    stl.scene = new THREE.Scene();
    stl.scene.background = new THREE.Color(STL_SCENE_BG);

    stl.camera = new THREE.PerspectiveCamera(35, 1, 0.1, 100);
    setStlView('iso');

    stl.light = new THREE.HemisphereLight(0xffffff, 0x444466, 1.0);
    stl.scene.add(stl.light);

    stl.dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
    stl.dirLight.position.copy(stl.camera.position);
    stl.scene.add(stl.dirLight);

    var el = stl.renderer.domElement;
    el.addEventListener('mousedown', onStlPointerDown);
    el.addEventListener('wheel', onStlWheel, { passive: false });
    el.addEventListener('touchstart', onStlTouchStart, { passive: false });
    el.addEventListener('touchmove', onStlTouchMove, { passive: false });
    el.addEventListener('touchend', onStlTouchEnd);

    stl.sceneReady = true;
    return true;
  }

  function setStlView(name) {
    if (!stl.camera) return;
    var THREE = window.THREE;
    var pos;
    if (name === 'top') {
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
    stl.spherical.theta = Math.atan2(pos.x, pos.z);
    stl.spherical.phi = Math.acos(Math.max(-1, Math.min(1, pos.y / stl.cameraDistance)));

    if (stl.dirLight) stl.dirLight.position.copy(stl.camera.position);

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

    loadStlLibs()
      .then(function () { initStlScene(); })
      .catch(function () {
        stlShowError("Couldn't load the 3D viewer. Please try again or check your connection.");
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

  function onStlUseView() {
    if (!stl.hasModel || !stl.renderer) return;
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
    try { stl.bsModal.hide(); } catch (_) {}

    placeStlPngOnCanvas(dataUrl).catch(function (e) {
      setSaveStatus('error', 'Failed to add STL view');
      console.warn('STL place failed', e);
    });
  }

  async function placeStlPngOnCanvas(dataUrl) {
    if (!state.canvas) return;
    var img;
    try {
      img = await fabric.Image.fromURL(dataUrl);
    } catch (e) {
      throw new Error('Failed to load STL view into canvas');
    }
    if (!img) throw new Error('STL image returned empty');

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
      ibRole: 'stl',
    });
    state.canvas.add(img);
    state.canvas.setActiveObject(img);
    state.canvas.requestRenderAll();

    scheduleAutosave();
    scheduleSnapshot();
    updateEmptyState();
  }

  // ====================================================================
  // 23. PROJECT-IMAGES PICKER MODAL (issue #184 — preserved)
  // ====================================================================

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
      var src = projectImageViewUrl(img.id);
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

  var PICKER_EMPTY_HTML =
    "No images uploaded yet. Use <strong>Upload image</strong> to add one, " +
    "or upload images from the project editor's image gallery.";

  async function onPickProjectImageClick() {
    ensurePickerModal();
    if (pickerModal.empty) pickerModal.empty.innerHTML = PICKER_EMPTY_HTML;
    setPickerView('loading');
    try { pickerModal.bsModal.show(); } catch (_) {}
    try {
      var images = await listProjectImages();
      if (images.length === 0) { setPickerView('empty'); return; }
      renderPickerGrid(images);
      setPickerView('grid');
    } catch (_) {
      if (pickerModal.empty) {
        pickerModal.empty.innerHTML = "Couldn't load your project images. Please try again in a moment.";
      }
      setPickerView('empty');
    }
  }

  async function placeProjectImageOnCanvas(url) {
    if (!state.canvas) return;
    var img;
    try {
      img = await fabric.Image.fromURL(url, { crossOrigin: 'anonymous' });
    } catch (e) {
      throw new Error('Failed to load image into canvas');
    }
    if (!img) throw new Error('Project image returned empty');

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
      ibRole: 'project-image',
      ibSourceUrl: url,
    });
    state.canvas.add(img);
    state.canvas.setActiveObject(img);
    state.canvas.requestRenderAll();
    scheduleAutosave();
    scheduleSnapshot();
    updateEmptyState();
    renderLayersList();
  }

  // ====================================================================
  // 24. SETTINGS POPOVER (Tweaks)
  // ====================================================================

  function toggleSettingsPop() {
    if (!dom.settingsPop) return;
    var open = !dom.settingsPop.hasAttribute('hidden');
    if (open) {
      dom.settingsPop.setAttribute('hidden', '');
    } else {
      dom.settingsPop.removeAttribute('hidden');
      // Click-outside-to-close.
      setTimeout(function () {
        document.addEventListener('click', closeOnOutsideClick, { once: true });
      }, 0);
    }
  }

  function closeOnOutsideClick(e) {
    if (!dom.settingsPop) return;
    if (dom.settingsPop.contains(e.target)) {
      // Click was inside the popover — keep it open and re-arm.
      document.addEventListener('click', closeOnOutsideClick, { once: true });
      return;
    }
    if (dom.railSettings && dom.railSettings.contains(e.target)) {
      // The toggle handler will close/open; don't double-close.
      return;
    }
    dom.settingsPop.setAttribute('hidden', '');
  }

  function wireSettings() {
    if (dom.settingsGrid) {
      dom.settingsGrid.addEventListener('change', function () {
        if (!state.uiState) return;
        state.uiState.showGrid = !!dom.settingsGrid.checked;
        if (dom.canvasFrame) dom.canvasFrame.classList.toggle('is-grid', state.uiState.showGrid);
        saveUiState();
      });
    }
    Array.prototype.forEach.call(dom.settingsDensity || [], function (radio) {
      radio.addEventListener('change', function () {
        if (!radio.checked || !state.uiState) return;
        state.uiState.density = radio.value;
        if (dom.workspace) dom.workspace.setAttribute('data-density', radio.value);
        saveUiState();
        setTimeout(syncCanvasDisplaySize, 50);
      });
    });
  }

  function wireHudToggle() {
    if (!dom.hudToggle) return;
    dom.hudToggle.addEventListener('click', function () {
      if (!state.uiState) return;
      state.uiState.hudVisible = !state.uiState.hudVisible;
      dom.hudToggle.setAttribute('aria-pressed', state.uiState.hudVisible ? 'true' : 'false');
      dom.hudToggle.classList.toggle('is-active', state.uiState.hudVisible);
      document.body.classList.toggle('hud-visible', state.uiState.hudVisible);
      // PR 2: also show/hide the #ib-inspector-hud panel content.
      saveUiState();
    });
  }

  // ====================================================================
  // 25. TOOLBAR / KEYBOARD WIRING
  // ====================================================================

  function wireToolbar() {
    Array.prototype.forEach.call(dom.toolButtons, function (btn) {
      btn.addEventListener('click', function () {
        setActiveTool(btn.dataset.tool);
      });
    });
    if (dom.stroke) {
      dom.stroke.addEventListener('input', function () {
        if (dom.strokeValue) dom.strokeValue.textContent = dom.stroke.value;
        applyStyleToSelection();
      });
    }
    if (dom.color) dom.color.addEventListener('input', applyStyleToSelection);
    if (dom.fontSize) dom.fontSize.addEventListener('input', applyStyleToSelection);
    if (dom.uploadImageBtn) dom.uploadImageBtn.addEventListener('click', onUploadClick);
    if (dom.pickProjectImageBtn) dom.pickProjectImageBtn.addEventListener('click', onPickProjectImageClick);
    if (dom.imageInput) dom.imageInput.addEventListener('change', onImagePicked);
    if (dom.removeImageBtn) dom.removeImageBtn.addEventListener('click', onRemoveImageClick);
    if (dom.addStlBtn) dom.addStlBtn.addEventListener('click', onAddStlClick);
    if (dom.deleteSelected) dom.deleteSelected.addEventListener('click', onDeleteSelected);
    if (dom.undoBtn) dom.undoBtn.addEventListener('click', onUndoClick);
    if (dom.redoBtn) dom.redoBtn.addEventListener('click', onRedoClick);

    if (dom.rotation) dom.rotation.addEventListener('change', onRotationInputChange);
    if (dom.resetTransforms) dom.resetTransforms.addEventListener('click', onResetTransformsClick);

    if (dom.bringToFront) dom.bringToFront.addEventListener('click', function () { arrangeActive('front'); });
    if (dom.bringForward) dom.bringForward.addEventListener('click', function () { arrangeActive('forward'); });
    if (dom.sendBackward) dom.sendBackward.addEventListener('click', function () { arrangeActive('backward'); });
    if (dom.sendToBack) dom.sendToBack.addEventListener('click', function () { arrangeActive('back'); });

    // Keyboard shortcuts: Cmd/Ctrl+Z = undo, Cmd/Ctrl+Shift+Z = redo,
    // Delete/Backspace = delete selected.
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
    if (active.isType && active.isType('i-text')) {
      if (color) active.set({ fill: color });
      if (fontSize) active.set({ fontSize: fontSize });
    } else {
      if (color) active.set({ stroke: color });
      if (stroke) active.set({ strokeWidth: stroke });
    }
    state.canvas.requestRenderAll();
    scheduleAutosave();
    scheduleSnapshot();
  }

  // ====================================================================
  // 26. INIT FLOW
  // ====================================================================

  async function init() {
    bindDom();

    // Hydrate UI state from localStorage before we touch the DOM so the
    // first paint already reflects the user's saved preferences.
    state.uiState = loadUiState();
    state.activeRailPane = state.uiState.activeRailPane || 'photos';

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
    if (dom.bomOpenEditor) dom.bomOpenEditor.href = '/projects/editor.html?id=' + encodeURIComponent(state.projectId) + '#editor-bom-section';

    // Auth probe
    try {
      state.me = await fetchMe();
    } catch (_) { state.me = null; }
    if (!state.me || !state.me.username) {
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

    // Page is good to render.
    showOnly(dom.main);

    initCanvas();
    wireToolbar();
    wireStepFields();
    wireExportMenu();
    wireRail();
    wireSecondaryResize();
    wirePhotosPane();
    wireFilmstrip();
    wireSettings();
    wireHudToggle();

    // Apply persisted UI state (density / grid / pane / filmstrip / etc.).
    applyUiStateToDom();

    // Render the steps list and load the first step's canvas.
    state.activeStepId = state.steps[0].id;
    if (dom.stepTitle) dom.stepTitle.value = state.steps[0].title || '';
    if (dom.stepDescription) dom.stepDescription.value = state.steps[0].description || '';
    renderFilmstrip();
    await loadCanvasFromStep(state.steps[0]);

    // Prime the Photos pane content even if it isn't visible — the first
    // user click into it should feel instant.
    refreshProjectImages();

    // Final layout pass once the panels have settled.
    setTimeout(syncCanvasDisplaySize, 50);
  }

  // Kick off once Fabric + DOM are both ready.
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
