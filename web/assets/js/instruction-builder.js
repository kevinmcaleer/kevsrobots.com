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
    // C1: ids of BOM rows expanded into their asset drawer. Persisted so
    // re-opening the pane (or the page) restores the expansion. Multiple
    // rows can be expanded simultaneously (no accordion).
    bomExpandedRowIds: [],
  };

  // Filenames that look like camera captures get the camera badge in the
  // Photos pane. Anything starting with `cam_` (the name our capture
  // upload assigns) counts.
  var CAM_FILENAME_PREFIX = 'cam_';

  // Inspector HUD — brand swatches. The four swatches match the design
  // sketch (`design_handoff_instructions_builder/`). A "custom" tile next
  // to them opens the native colour picker.
  var HUD_SWATCHES = ['#c8312a', '#ffffff', '#222222', '#33aa88'];

  // @imgly/background-removal — lazy-loaded ESM bundle for the
  // image-context toolbar's "Cutout" action. jsdelivr serves browser-
  // compatible ESM via the `/+esm` path; the library is ~25MB including
  // its WASM model and only loads on first cutout click.
  var IMGLY_BG_REMOVAL_URL = 'https://cdn.jsdelivr.net/npm/@imgly/background-removal@1.7.0/+esm';

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
    // Canvas viewport zoom/pan. userZoom = 1 = "fit to view". Pan is
    // additive offset in CSS pixels applied on top of the auto-centred
    // viewport transform.
    userZoom: 1,
    panX: 0,
    panY: 0,
    // C1 — BOM pane
    bomItems: null,           // list of BOMItemResponse, null = not loaded yet
    bomItemsLoading: false,
    bomItemsError: null,
    bomItemsLoaded: false,    // true once a successful fetch has populated bomItems
    bomPartAssets: {},        // part_id -> Asset[] (derived from /api/parts/{slug})
    bomPartAssetsPending: {}, // part_id -> Promise (de-dupe in-flight fetches)
    // Symbol Designer integration: user-designed symbols (one row per
    // /api/projects/{id}/symbols entry). When a symbol's bom_item_id
    // matches a BOM row, the symbol surfaces as a `⌗` chip in that
    // row's asset drawer. Loaded once alongside the BOM list.
    projectSymbols: [],
    projectSymbolsLoaded: false,
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
    dom.zoomIn         = document.getElementById('ib-zoom-in');
    dom.zoomOut        = document.getElementById('ib-zoom-out');
    dom.zoomReset      = document.getElementById('ib-zoom-reset');
    dom.zoomPct        = document.getElementById('ib-zoom-pct');
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

    // BOM pane (C1)
    dom.bomOpenEditor = document.getElementById('ib-bom-open-editor');
    dom.bomList = document.getElementById('ib-bom-list');
    dom.bomBody = document.getElementById('ib-bom-body');
    dom.bomRefresh = document.getElementById('ib-bom-refresh');

    // Layers pane
    dom.layersList = document.getElementById('ib-layers-list');

    // Canvas
    dom.canvasArea = document.getElementById('ib-canvas-area');
    dom.canvasEl = document.getElementById('ib-canvas');
    dom.canvasFrame = document.getElementById('ib-canvas-frame');
    dom.canvasEmpty = document.getElementById('ib-canvas-empty');
    // B3: alternate canvas-area views, one per non-photo step type.
    dom.canvasText = document.getElementById('ib-canvas-text');
    dom.canvasTextEmpty = document.getElementById('ib-canvas-text-empty');
    dom.canvasTextBody = document.getElementById('ib-canvas-text-body');
    dom.canvasVideo = document.getElementById('ib-canvas-video');
    dom.canvasVideoInner = document.getElementById('ib-canvas-video-inner');
    dom.canvasSchematic = document.getElementById('ib-canvas-schematic');

    // Schematic pane (step-type picker)
    dom.stepTypeRadios = document.querySelectorAll('input[name="ib-step-type"]');
    dom.stepTypeConfigs = document.querySelectorAll('.ib-step-type-config');
    dom.stepBodyInput = document.getElementById('ib-step-body-input');
    dom.stepVideoInput = document.getElementById('ib-step-video-input');
    dom.stepVideoPreview = document.getElementById('ib-step-video-preview');
    // E2: full-schematic-editor "open" CTAs (one in the schematic-step
    // pane card, one in the canvas-area placeholder).
    dom.openSchematicEditorPane = document.getElementById('ib-open-schematic-editor-pane');
    dom.openSchematicEditorCanvas = document.getElementById('ib-open-schematic-editor-canvas');
    dom.inspectorHud = document.getElementById('ib-inspector-hud');
    dom.hudHeader = document.getElementById('ib-hud-header');
    dom.hudBody = document.getElementById('ib-hud-body');
    dom.hudMinimize = document.getElementById('ib-hud-minimize');
    dom.hudClose = document.getElementById('ib-hud-close');
    dom.imageToolbar = document.getElementById('ib-image-toolbar');

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
    // HUD: title-bar button reflects visibility + panel is shown/hidden.
    // The panel content is rendered selection-aware by renderHudBody.
    applyHudVisibility(!!ui.hudVisible);
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

  async function postStep(initialFields) {
    await ensureInstruction();
    var body = Object.assign({ title: '', description: '' }, initialFields || {});
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction/steps',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
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

    // PR 2 — Inspector HUD + image-context toolbar follow the active
    // object as it moves / scales / rotates. The refresh helpers are
    // focus-aware (HUD inputs in focus aren't overwritten) and
    // visibility-aware (no-ops when nothing's selected).
    var liveEvents = ['object:moving', 'object:scaling', 'object:rotating',
                      'object:skewing', 'object:modified'];
    liveEvents.forEach(function (evt) {
      c.on(evt, function () {
        refreshHudInputsFromCanvas();
        var img = activeImageOrNull();
        if (img) positionImageToolbar(img);
      });
    });

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

  // Canvas viewport: internal canvas pixel dims = CSS dims = frame
  // dims. A viewportTransform applies the auto-fit scale (so the whole
  // scene lands inside the frame at default zoom) plus user zoom and
  // pan offsets. Scene coords stay fixed (CANVAS_W × CANVAS_H); only
  // the rendering scale changes.
  function syncCanvasDisplaySize() {
    if (!state.canvas || !dom.canvasFrame) return;
    var rect = dom.canvasFrame.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    var w = Math.max(1, Math.floor(rect.width));
    var h = Math.max(1, Math.floor(rect.height));
    state.canvas.setDimensions({ width: w, height: h });
    applyCanvasViewport();
  }

  function fitScale() {
    if (!state.canvas) return 1;
    var w = state.canvas.getWidth();
    var h = state.canvas.getHeight();
    var s = Math.min(w / CANVAS_W, h / CANVAS_H);
    return s > 0 ? s : 1;
  }

  function effectiveScale() {
    return fitScale() * (state.userZoom || 1);
  }

  function applyCanvasViewport() {
    if (!state.canvas) return;
    var w = state.canvas.getWidth();
    var h = state.canvas.getHeight();
    var s = effectiveScale();
    var tx = (w - CANVAS_W * s) / 2 + state.panX;
    var ty = (h - CANVAS_H * s) / 2 + state.panY;
    state.canvas.setViewportTransform([s, 0, 0, s, tx, ty]);
    // After a viewport change, anything that's anchored to a scene
    // position via CSS — image-context toolbar, HUD when tracking a
    // moving object — needs to be re-laid out.
    var img = activeImageOrNull();
    if (img) positionImageToolbar(img);
    refreshHudInputsFromCanvas();
    updateZoomPctLabel();
  }

  function zoomCanvasBy(factor, focal) {
    var prev = state.userZoom || 1;
    var next = Math.max(0.2, Math.min(6, prev * factor));
    if (next === prev) return;
    if (focal && state.canvas) {
      // Keep the point under the cursor fixed across the zoom step.
      var vt = state.canvas.viewportTransform;
      var sceneX = (focal.x - vt[4]) / vt[0];
      var sceneY = (focal.y - vt[5]) / vt[3];
      state.userZoom = next;
      var s = effectiveScale();
      var w = state.canvas.getWidth();
      var h = state.canvas.getHeight();
      state.panX = focal.x - sceneX * s - (w - CANVAS_W * s) / 2;
      state.panY = focal.y - sceneY * s - (h - CANVAS_H * s) / 2;
    } else {
      state.userZoom = next;
    }
    applyCanvasViewport();
  }

  function resetCanvasZoom() {
    state.userZoom = 1;
    state.panX = 0;
    state.panY = 0;
    applyCanvasViewport();
  }

  function updateZoomPctLabel() {
    if (!dom.zoomPct) return;
    dom.zoomPct.textContent = Math.round((state.userZoom || 1) * 100) + '%';
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

    // PR 2 — keep the HUD body + image-context toolbar in sync with the
    // current selection. Both are no-ops when the HUD is hidden / there
    // isn't a relevant single-object selection.
    if (state.uiState && state.uiState.hudVisible) renderHudBody();
    refreshImageToolbar();
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
      await addBackgroundImage(url, file.name);
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

  async function addBackgroundImage(url, filename) {
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
      ibFilename: filename || '',
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
    // photo a step uses as a background (drives the ★ badge in Photos),
    // plus ibFilename so the Layers pane keeps showing the friendly
    // filename per image after a reload.
    return state.canvas.toJSON(['ibRole', 'ibSourceUrl', 'ibFilename']);
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
    // Keep the in-memory step copy in sync so the filmstrip's
    // background-thumb regen reads fresh data.
    var activeStep = state.steps.find(function (s) { return s.id === state.activeStepId; });
    if (activeStep) activeStep.canvas_json = json;
    setSaveStatus('saving', 'Saving…');
    try {
      await putStep(state.activeStepId, { canvas_json: json });
      setSaveStatus('saved');
      // The set of "photos used as backgrounds across all steps" may
      // have shifted; recompute so the ★ badge updates.
      recomputeBackgroundPhotoIds();
      renderPhotosGrid();
      // Refresh the active step's filmstrip thumb so it shows what's
      // actually on the canvas.
      refreshActiveStepThumb();
    } catch (e) {
      setSaveStatus('error', isRetry ? 'Save failed again' : 'Save failed');
    }
  }

  // Per-field "value at focus" snapshot — we only push a step-undo entry
  // if the field actually changed between focus and blur, otherwise
  // tabbing through fields would litter the undo stack with no-ops.
  var stepFieldFocusValues = { title: null, description: null, stepId: null };

  function wireStepFields() {
    if (dom.stepTitle) {
      dom.stepTitle.addEventListener('focus', function () {
        stepFieldFocusValues.stepId = state.activeStepId;
        stepFieldFocusValues.title = dom.stepTitle.value || '';
      });
      dom.stepTitle.addEventListener('blur', function () {
        if (!state.activeStepId) return;
        debounce('step-title', 100, function () {
          var stepId = state.activeStepId;
          var value = dom.stepTitle.value || null;
          var prior = stepFieldFocusValues.stepId === stepId
            ? stepFieldFocusValues.title
            : null;
          // No-op if unchanged
          if ((prior || '') === (value || '')) return;
          setSaveStatus('saving', 'Saving…');
          putStep(stepId, { title: value })
            .then(function (updated) {
              var s = state.steps.find(function (x) { return x.id === stepId; });
              if (s) s.title = updated.title;
              renderFilmstrip();
              setSaveStatus('saved');
              // Inverse: restore the prior title. Redo restores the new
              // value — captured in closure as `value` so the redo entry
              // mirrors the original op exactly.
              pushStepUndo({
                type: 'edit-title',
                label: 'renamed step',
                apply: async function () {
                  await putStep(stepId, { title: prior || null });
                  var s2 = state.steps.find(function (x) { return x.id === stepId; });
                  if (s2) s2.title = prior || null;
                  if (state.activeStepId === stepId && dom.stepTitle) {
                    dom.stepTitle.value = prior || '';
                  }
                  renderFilmstrip();
                },
                invert: async function () {
                  await putStep(stepId, { title: value });
                  var s2 = state.steps.find(function (x) { return x.id === stepId; });
                  if (s2) s2.title = value;
                  if (state.activeStepId === stepId && dom.stepTitle) {
                    dom.stepTitle.value = value || '';
                  }
                  renderFilmstrip();
                },
              });
            })
            .catch(function () { setSaveStatus('error', 'Title save failed'); });
        });
      });
    }
    if (dom.stepDescription) {
      dom.stepDescription.addEventListener('focus', function () {
        stepFieldFocusValues.stepId = state.activeStepId;
        stepFieldFocusValues.description = dom.stepDescription.value || '';
      });
      dom.stepDescription.addEventListener('blur', function () {
        if (!state.activeStepId) return;
        debounce('step-desc', 100, function () {
          var stepId = state.activeStepId;
          var value = dom.stepDescription.value || null;
          var prior = stepFieldFocusValues.stepId === stepId
            ? stepFieldFocusValues.description
            : null;
          if ((prior || '') === (value || '')) return;
          setSaveStatus('saving', 'Saving…');
          putStep(stepId, { description: value })
            .then(function (updated) {
              var s = state.steps.find(function (x) { return x.id === stepId; });
              if (s) s.description = updated.description;
              setSaveStatus('saved');
              pushStepUndo({
                type: 'edit-description',
                label: 'edited description',
                apply: async function () {
                  await putStep(stepId, { description: prior || null });
                  var s2 = state.steps.find(function (x) { return x.id === stepId; });
                  if (s2) s2.description = prior || null;
                  if (state.activeStepId === stepId && dom.stepDescription) {
                    dom.stepDescription.value = prior || '';
                  }
                },
                invert: async function () {
                  await putStep(stepId, { description: value });
                  var s2 = state.steps.find(function (x) { return x.id === stepId; });
                  if (s2) s2.description = value;
                  if (state.activeStepId === stepId && dom.stepDescription) {
                    dom.stepDescription.value = value || '';
                  }
                },
              });
            })
            .catch(function () { setSaveStatus('error', 'Description save failed'); });
        });
      });
    }
  }

  // ====================================================================
  // 10b. STEP TYPES (B3) — type picker + canvas-area swap + per-type
  //      body / video URL persistence. Lives next to wireStepFields
  //      because it shares the autosave + step-undo plumbing.
  // ====================================================================

  // The five valid step types. Anything not in this set is treated as
  // 'photo' (the legacy default + back-end fall-through).
  var STEP_TYPES = ['photo', 'schematic', 'text', 'video', 'blank'];

  // Track focus values for the body / video inputs so we only push a
  // step-undo entry when the field actually changed. Mirrors the
  // existing stepFieldFocusValues pattern for title / description.
  var stepTypeFocusValues = { body: null, videoUrl: null, stepId: null };

  function currentStepTypeOf(step) {
    if (!step) return 'photo';
    var t = step.step_type;
    return STEP_TYPES.indexOf(t) >= 0 ? t : 'photo';
  }

  function activeStepObject() {
    if (!state.activeStepId) return null;
    return state.steps.find(function (s) { return s.id === state.activeStepId; });
  }

  // Extract a YouTube video id from any of the common URL shapes; null
  // for non-YouTube URLs (which renders as a raw <video controls>).
  function extractYouTubeId(url) {
    if (!url) return null;
    var m = String(url).match(/(?:youtube\.com\/watch\?v=|youtu\.be\/|youtube\.com\/embed\/)([\w-]{11})/);
    return m ? m[1] : null;
  }

  // Update the canvas area's visibility per step type. The Fabric
  // canvas frame stays in the DOM in every case (so the rest of the
  // builder still has something to reference) — we just hide it for
  // text / video / schematic and show the corresponding alt panel.
  function applyCanvasAreaForType(stepType) {
    var fabricVisible = (stepType === 'photo' || stepType === 'blank');
    if (dom.canvasFrame) dom.canvasFrame.classList.toggle('d-none', !fabricVisible);
    if (dom.canvasText) dom.canvasText.classList.toggle('d-none', stepType !== 'text');
    if (dom.canvasVideo) dom.canvasVideo.classList.toggle('d-none', stepType !== 'video');
    if (dom.canvasSchematic) dom.canvasSchematic.classList.toggle('d-none', stepType !== 'schematic');
    // Body class drives CSS rules that swap title-bar affordances per
    // step type — e.g. hide the HUD toggle on schematic steps where
    // the HUD has nothing to inspect.
    document.body.dataset.activeStepType = stepType || 'photo';
    // Sync the title-bar zoom label to whichever canvas now owns the
    // viewport. Schematic-step zoom comes back over postMessage when
    // the iframe is ready; until then show "—". For Fabric-backed
    // step types reflect state.userZoom directly.
    if (stepType === 'schematic') {
      if (dom.zoomPct) dom.zoomPct.textContent = '—';
    } else {
      updateZoomPctLabel();
    }
    // Schematic steps mount the full editor as an iframe inside the
    // canvas area — lazy-load on first switch so projects that never
    // use schematics don't pay the request.
    if (stepType === 'schematic') mountSchematicIframe();
    // Fabric tracks the CSS size of the canvas; if we just toggled
    // visibility, give it a moment to settle and re-measure so pointer
    // events map to the right scene coords.
    if (fabricVisible) setTimeout(syncCanvasDisplaySize, 50);
  }

  // Lazy-mount the schematic editor iframe. Idempotent — sets the src
  // once and re-uses the iframe on subsequent step switches. We point
  // at the same standalone editor page; embedding it via iframe means
  // the user's experience inside the schematic step matches the full-
  // screen route without a Refactor of schematic-editor.js.
  function mountSchematicIframe() {
    var iframe = document.getElementById('ib-canvas-schematic-iframe');
    if (!iframe || !state.projectId) return;
    if (iframe.dataset.mounted === '1') return;
    // The standalone editor's own logic (ensureSchematic) creates the
    // schematic row idempotently, so we don't need to call
    // ensureProjectSchematicId() here.
    iframe.src = '/projects/schematic/edit.html?id=' +
      encodeURIComponent(state.projectId) + '&embedded=1';
    iframe.dataset.mounted = '1';
  }

  // Apply the text-step body to the read-only preview slot in the
  // canvas area. Plain-text rendering for B3 — markdown is acceptable
  // for a follow-up. We render via textContent on a <pre>-ish element
  // so we don't have to sanitise.
  function renderCanvasTextBody(step) {
    if (!dom.canvasTextBody || !dom.canvasTextEmpty) return;
    var body = (step && step.body) || '';
    if (!body.trim()) {
      dom.canvasTextEmpty.classList.remove('d-none');
      dom.canvasTextBody.classList.add('d-none');
      dom.canvasTextBody.textContent = '';
    } else {
      dom.canvasTextEmpty.classList.add('d-none');
      dom.canvasTextBody.classList.remove('d-none');
      // Plain-text body for B3 — preserve line breaks via white-space CSS.
      dom.canvasTextBody.textContent = body;
    }
  }

  // Build the video embed inside the canvas area for video steps.
  // YouTube → iframe, raw URLs → <video controls>, empty / unrecognised
  // → empty-state placeholder.
  function renderCanvasVideo(step) {
    if (!dom.canvasVideoInner) return;
    var url = (step && step.video_url) || '';
    if (!url) {
      dom.canvasVideoInner.innerHTML =
        '<div class="ib-canvas-video-empty">' +
        '  <i class="fas fa-video mb-2"></i>' +
        '  <p class="mb-0">Paste a YouTube or MP4 URL in the <strong>Schematic</strong> pane.</p>' +
        '</div>';
      return;
    }
    var ytId = extractYouTubeId(url);
    if (ytId) {
      dom.canvasVideoInner.innerHTML =
        '<iframe class="ib-canvas-video-iframe" ' +
        '        src="https://www.youtube.com/embed/' + encodeURIComponent(ytId) + '" ' +
        '        title="YouTube video" ' +
        '        frameborder="0" ' +
        '        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" ' +
        '        allowfullscreen></iframe>';
    } else {
      dom.canvasVideoInner.innerHTML =
        '<video class="ib-canvas-video-el" controls preload="metadata" src="' + escapeHtml(url) + '"></video>';
    }
  }

  // Build the small video preview inside the Schematic pane (below the
  // URL input). Same id-extraction as the canvas-area renderer but
  // styled smaller. Empty for an empty URL.
  function renderVideoPreview(url) {
    if (!dom.stepVideoPreview) return;
    url = (url || '').trim();
    if (!url) {
      dom.stepVideoPreview.innerHTML = '';
      return;
    }
    var ytId = extractYouTubeId(url);
    if (ytId) {
      dom.stepVideoPreview.innerHTML =
        '<iframe class="ib-step-video-iframe" ' +
        '        src="https://www.youtube.com/embed/' + encodeURIComponent(ytId) + '" ' +
        '        title="YouTube preview" ' +
        '        frameborder="0" allowfullscreen></iframe>';
    } else if (/\.(mp4|webm|ogg)(\?|$)/i.test(url)) {
      dom.stepVideoPreview.innerHTML =
        '<video class="ib-step-video-el" controls preload="metadata" src="' + escapeHtml(url) + '"></video>';
    } else {
      dom.stepVideoPreview.innerHTML =
        '<p class="small text-muted mb-0"><i class="fas fa-info-circle me-1"></i>' +
        'URL doesn\'t look like a YouTube link or a video file — saved verbatim.</p>';
    }
  }

  // Sync the type-picker pane's controls to the currently-active step.
  // Called whenever the active step changes (switchToStep, init, etc.).
  function syncStepTypePane() {
    var step = activeStepObject();
    var stepType = currentStepTypeOf(step);

    // Radios
    Array.prototype.forEach.call(dom.stepTypeRadios || [], function (input) {
      input.checked = (input.value === stepType);
    });

    // Per-type config sections
    Array.prototype.forEach.call(dom.stepTypeConfigs || [], function (section) {
      section.classList.toggle('d-none', section.dataset.config !== stepType);
    });

    // Text body input
    if (dom.stepBodyInput) {
      dom.stepBodyInput.value = (step && step.body) || '';
    }

    // Video URL input + preview
    if (dom.stepVideoInput) {
      dom.stepVideoInput.value = (step && step.video_url) || '';
    }
    renderVideoPreview(step && step.video_url);

    // Canvas area visibility
    applyCanvasAreaForType(stepType);
    if (stepType === 'text') renderCanvasTextBody(step);
    if (stepType === 'video') renderCanvasVideo(step);
  }

  // E2: cache the project's single schematic id once it's known. The
  // first POST to /schematic is idempotent so flipping a step to
  // ``step_type=schematic`` can call it without checking first.
  var schematicIdCache = null;

  async function ensureProjectSchematicId() {
    if (schematicIdCache) return schematicIdCache;
    try {
      var resp = await apiFetchWithTermsRetry(
        API + '/api/projects/' + state.projectId + '/schematic',
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({}),
        }
      );
      if (!resp.ok) return null;
      var body = await resp.json();
      schematicIdCache = body.id;
      return schematicIdCache;
    } catch (_) {
      return null;
    }
  }

  function schematicEditorUrl() {
    return '/projects/schematic/edit.html?id=' +
      encodeURIComponent(state.projectId || '');
  }

  async function onStepTypeChange(newType) {
    if (STEP_TYPES.indexOf(newType) < 0) return;
    var step = activeStepObject();
    if (!step) return;
    var stepId = step.id;
    var priorType = currentStepTypeOf(step);
    if (priorType === newType) return;

    setSaveStatus('saving', 'Saving…');
    try {
      // NB: we intentionally don't clear canvas_json when switching to
      // text / video / schematic — flipping back to photo or blank
      // restores any overlays the user had drawn. Same goes for body /
      // video_url across the other direction.
      var updated = await putStep(stepId, { step_type: newType });
      // Update local cache.
      step.step_type = updated.step_type;

      // E2: when the step becomes a schematic step, ensure the project
      // has a schematic record and link this step to it. The POST is
      // idempotent so re-calling it for each step is safe — the
      // schematicIdCache means we hit the network at most once per
      // session.
      if (newType === 'schematic') {
        var schematicId = await ensureProjectSchematicId();
        if (schematicId && step.schematic_id !== schematicId) {
          try {
            var stepResp = await putStep(stepId, { schematic_id: schematicId });
            step.schematic_id = stepResp.schematic_id;
          } catch (_) {
            // Linkage failure is non-fatal — the editor still opens via the
            // ?id=projectId URL; the step just won't carry the schematic_id
            // until the next type-switch.
          }
        }
      }

      setSaveStatus('saved');
      syncStepTypePane();
      renderFilmstrip();
      pushStepUndo({
        type: 'change-step-type',
        label: 'changed step type',
        apply: async function () {
          var reverted = await putStep(stepId, { step_type: priorType });
          var s = state.steps.find(function (x) { return x.id === stepId; });
          if (s) s.step_type = reverted.step_type;
          if (state.activeStepId === stepId) syncStepTypePane();
          renderFilmstrip();
        },
        invert: async function () {
          var reapplied = await putStep(stepId, { step_type: newType });
          var s = state.steps.find(function (x) { return x.id === stepId; });
          if (s) s.step_type = reapplied.step_type;
          if (state.activeStepId === stepId) syncStepTypePane();
          renderFilmstrip();
        },
      });
    } catch (_) {
      setSaveStatus('error', 'Change step type failed');
    }
  }

  function wireStepTypePane() {
    // Radios — change fires immediately (no debounce; type switch is a
    // single coarse-grained user action, not a stream of inputs).
    Array.prototype.forEach.call(dom.stepTypeRadios || [], function (input) {
      input.addEventListener('change', function () {
        if (!input.checked) return;
        onStepTypeChange(input.value);
      });
    });

    // Text body — autosave on blur (debounce 500ms so a quick blur into
    // a re-focus doesn't fire a save). Mirrors the existing
    // wireStepFields pattern for title / description.
    if (dom.stepBodyInput) {
      dom.stepBodyInput.addEventListener('focus', function () {
        stepTypeFocusValues.stepId = state.activeStepId;
        stepTypeFocusValues.body = dom.stepBodyInput.value || '';
      });
      dom.stepBodyInput.addEventListener('blur', function () {
        if (!state.activeStepId) return;
        debounce('step-body', 500, function () {
          var stepId = state.activeStepId;
          var value = dom.stepBodyInput.value || null;
          var prior = stepTypeFocusValues.stepId === stepId
            ? stepTypeFocusValues.body
            : null;
          if ((prior || '') === (value || '')) return;
          setSaveStatus('saving', 'Saving…');
          putStep(stepId, { body: value })
            .then(function (updated) {
              var s = state.steps.find(function (x) { return x.id === stepId; });
              if (s) s.body = updated.body;
              if (state.activeStepId === stepId) renderCanvasTextBody(s);
              setSaveStatus('saved');
              pushStepUndo({
                type: 'edit-body',
                label: 'edited body',
                apply: async function () {
                  await putStep(stepId, { body: prior || null });
                  var s2 = state.steps.find(function (x) { return x.id === stepId; });
                  if (s2) s2.body = prior || null;
                  if (state.activeStepId === stepId && dom.stepBodyInput) {
                    dom.stepBodyInput.value = prior || '';
                  }
                  if (state.activeStepId === stepId) renderCanvasTextBody(s2);
                },
                invert: async function () {
                  await putStep(stepId, { body: value });
                  var s2 = state.steps.find(function (x) { return x.id === stepId; });
                  if (s2) s2.body = value;
                  if (state.activeStepId === stepId && dom.stepBodyInput) {
                    dom.stepBodyInput.value = value || '';
                  }
                  if (state.activeStepId === stepId) renderCanvasTextBody(s2);
                },
              });
            })
            .catch(function () { setSaveStatus('error', 'Body save failed'); });
        });
      });
    }

    // Video URL — same blur-save pattern; also re-renders the preview
    // live on input so the user gets visual feedback before saving.
    if (dom.stepVideoInput) {
      dom.stepVideoInput.addEventListener('input', function () {
        renderVideoPreview(dom.stepVideoInput.value);
      });
      dom.stepVideoInput.addEventListener('focus', function () {
        stepTypeFocusValues.stepId = state.activeStepId;
        stepTypeFocusValues.videoUrl = dom.stepVideoInput.value || '';
      });
      dom.stepVideoInput.addEventListener('blur', function () {
        if (!state.activeStepId) return;
        debounce('step-video', 500, function () {
          var stepId = state.activeStepId;
          var value = dom.stepVideoInput.value || null;
          var prior = stepTypeFocusValues.stepId === stepId
            ? stepTypeFocusValues.videoUrl
            : null;
          if ((prior || '') === (value || '')) return;
          setSaveStatus('saving', 'Saving…');
          putStep(stepId, { video_url: value })
            .then(function (updated) {
              var s = state.steps.find(function (x) { return x.id === stepId; });
              if (s) s.video_url = updated.video_url;
              if (state.activeStepId === stepId) renderCanvasVideo(s);
              setSaveStatus('saved');
              pushStepUndo({
                type: 'edit-video-url',
                label: 'edited video URL',
                apply: async function () {
                  await putStep(stepId, { video_url: prior || null });
                  var s2 = state.steps.find(function (x) { return x.id === stepId; });
                  if (s2) s2.video_url = prior || null;
                  if (state.activeStepId === stepId && dom.stepVideoInput) {
                    dom.stepVideoInput.value = prior || '';
                  }
                  if (state.activeStepId === stepId) {
                    renderVideoPreview(prior);
                    renderCanvasVideo(s2);
                  }
                },
                invert: async function () {
                  await putStep(stepId, { video_url: value });
                  var s2 = state.steps.find(function (x) { return x.id === stepId; });
                  if (s2) s2.video_url = value;
                  if (state.activeStepId === stepId && dom.stepVideoInput) {
                    dom.stepVideoInput.value = value || '';
                  }
                  if (state.activeStepId === stepId) {
                    renderVideoPreview(value);
                    renderCanvasVideo(s2);
                  }
                },
              });
            })
            .catch(function () { setSaveStatus('error', 'Video URL save failed'); });
        });
      });
    }

    // E2: full schematic editor "open" CTAs (pane card + canvas-area
    // placeholder). Both navigate to the new full-screen editor; the
    // link is computed at click-time because state.projectId isn't
    // populated until init() runs.
    function openSchematicEditor(ev) {
      if (ev && typeof ev.preventDefault === 'function') ev.preventDefault();
      // Ensure the schematic exists (idempotent) so the link lands on
      // an editor that already has its row. The await is fire-and-
      // forget — even if the POST is in-flight when we navigate, the
      // editor itself will POST again on load.
      ensureProjectSchematicId();
      window.open(schematicEditorUrl(), '_blank', 'noopener');
    }
    if (dom.openSchematicEditorPane) {
      dom.openSchematicEditorPane.addEventListener('click', openSchematicEditor);
    }
    if (dom.openSchematicEditorCanvas) {
      dom.openSchematicEditorCanvas.addEventListener('click', openSchematicEditor);
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
    if (dom.undoBtn) {
      // The undo button reflects either stack — if either has content,
      // Cmd/Ctrl+Z (and the button) will do *something*. Canvas undo
      // wins when both are populated (see onUndoShortcut).
      dom.undoBtn.disabled = state.undoStack.length === 0 &&
        stepUndoStack.length === 0;
    }
    if (dom.redoBtn) {
      // Same symmetry for redo — canvas redo first, step redo as fall-
      // through. Either stack populated => button enabled.
      dom.redoBtn.disabled = state.redoStack.length === 0 &&
        stepRedoStack.length === 0;
    }
  }

  // ====================================================================
  // 11b. STEP-LEVEL UNDO / REDO
  // ====================================================================
  //
  // The canvas undo stack (`state.undoStack`, above) is per-step and gets
  // wiped when the user switches steps. That's correct for canvas-object
  // edits, but the user's "undo to all operations" intent reasonably
  // includes step CRUD too. So we keep a *separate* page-level pair of
  // stacks (undo + redo) for: step add, step delete, step reorder, step
  // title change, step description change, step body/video/type changes.
  //
  // Design choice: a separate pair of stacks (rather than a unified mixed
  // stack with a `scope: 'canvas' | 'step'` discriminator) keeps the diff
  // small — the existing canvas-undo code stays untouched, and the new
  // step ops never need to interleave with the canvas snapshots inside a
  // single step. Cmd/Ctrl+Z runs canvas undo first; only when the canvas
  // stack is empty does it fall back to step undo. Cmd/Ctrl+Shift+Z does
  // the symmetric thing for redo. This matches how most editors handle
  // it — local edits get undone before "scene-level" ones.
  //
  // Entries are stored as { type, label, apply, invert } where:
  //   - apply():  the undo direction (reverts the op).
  //   - invert(): the redo direction (re-applies the op).
  // Both are async; either can return an optional string to override the
  // default toast label ("Undid: <label>" or "Redid: <label>").
  //
  // Linear-undo semantics: any *new* step op (via pushStepUndo) clears
  // stepRedoStack — once the user diverges from the undone branch, the
  // redo history is gone.
  //
  // We do NOT persist either stack to localStorage — replaying server
  // mutations after a reload (or in another tab) is too risky.

  var stepUndoStack = [];
  var stepRedoStack = [];
  var STEP_UNDO_LIMIT = 50;

  function pushStepUndo(entry) {
    if (!entry || !entry.type || typeof entry.apply !== 'function') return;
    stepUndoStack.push(entry);
    if (stepUndoStack.length > STEP_UNDO_LIMIT) stepUndoStack.shift();
    // A fresh op always invalidates the redo branch.
    stepRedoStack = [];
    updateUndoRedoButtons();
  }

  async function popStepUndo() {
    if (stepUndoStack.length === 0) return false;
    var entry = stepUndoStack.pop();
    updateUndoRedoButtons();
    try {
      var ret = await entry.apply();
      var msg = (typeof ret === 'string' && ret)
        ? ret
        : ('Undid: ' + (entry.label || entry.type));
      showStepToast(msg);
      // Only entries with a working invert() participate in redo. Skip
      // legacy entries that pre-date this PR (none in tree, but defensive).
      if (typeof entry.invert === 'function') {
        stepRedoStack.push(entry);
        if (stepRedoStack.length > STEP_UNDO_LIMIT) stepRedoStack.shift();
      }
      updateUndoRedoButtons();
    } catch (e) {
      console.warn('Step undo failed', e);
      setSaveStatus('error', "Undo failed");
    }
    return true;
  }

  async function popStepRedo() {
    if (stepRedoStack.length === 0) return false;
    var entry = stepRedoStack.pop();
    updateUndoRedoButtons();
    try {
      var ret = await entry.invert();
      var msg = (typeof ret === 'string' && ret)
        ? ret
        : ('Redid: ' + (entry.label || entry.type));
      showStepToast(msg, 'redo');
      // Mirror: redoing puts the entry back onto the undo stack so a
      // subsequent Cmd/Ctrl+Z reverts it again.
      stepUndoStack.push(entry);
      if (stepUndoStack.length > STEP_UNDO_LIMIT) stepUndoStack.shift();
      updateUndoRedoButtons();
    } catch (e) {
      console.warn('Step redo failed', e);
      setSaveStatus('error', "Redo failed");
    }
    return true;
  }

  // ====================================================================
  // 11c. STEP-LEVEL TOAST
  // ====================================================================
  //
  // Brief bottom-centred snackbar that confirms a step-level undo or
  // redo. The change might be subtle (a description revert), so the user
  // needs to see what happened. Auto-dismisses after ~2s.

  var stepToastNode = null;
  var stepToastTimer = null;

  function ensureStepToast() {
    if (stepToastNode) return stepToastNode;
    stepToastNode = document.createElement('div');
    stepToastNode.className = 'ib-undo-toast';
    stepToastNode.setAttribute('role', 'status');
    stepToastNode.setAttribute('aria-live', 'polite');
    document.body.appendChild(stepToastNode);
    return stepToastNode;
  }

  function showStepToast(msg, kind) {
    var node = ensureStepToast();
    var icon = kind === 'redo' ? 'fa-redo' : 'fa-undo';
    node.innerHTML = '<i class="fas ' + icon + '"></i>' + escapeHtml(msg);
    node.classList.add('is-visible');
    if (stepToastTimer) clearTimeout(stepToastTimer);
    stepToastTimer = setTimeout(function () {
      node.classList.remove('is-visible');
      stepToastTimer = null;
    }, 2000);
  }

  // Shortcut entrypoint. Runs canvas undo first (more recent / more
  // local); falls back to step undo when the canvas stack is empty.
  function onUndoShortcut() {
    if (state.undoStack.length > 0) {
      onUndoClick();
      return;
    }
    if (stepUndoStack.length > 0) {
      popStepUndo();
    }
    // If both stacks are empty, the shortcut is a no-op — caller already
    // preventDefault'd the keystroke so the browser doesn't intercept.
  }

  // Symmetric redo entrypoint. Canvas redo wins when populated; step
  // redo handles the fall-through. Same priority logic as undo so the
  // two stacks stay aligned: a user who undoes a string of canvas ops
  // then a string of step ops can redo them in the same order.
  function onRedoShortcut() {
    if (state.redoStack.length > 0) {
      onRedoClick();
      return;
    }
    if (stepRedoStack.length > 0) {
      popStepRedo();
    }
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
    // C1: lazy first-render of the BOM pane. We refresh on demand
    // (refresh button), not on every pane switch, because the BOM rarely
    // changes within a single editor session.
    if (name === 'bom' && !state.bomItemsLoaded) loadBomItems();

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
      // Layer labels for pre-fix legacy images rely on a lookup by
      // ibSourceUrl into state.projectImages — re-render now that the
      // list is populated so the names appear without needing a step
      // switch.
      renderLayersList();
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
          // Resolve to {id, src, filename} triples so the dropped image
          // can surface a meaningful label in the Layers pane.
          var items = ids.map(function (pid) {
            var match = state.projectImages.find(function (im) { return im.id === pid; });
            if (!match) return null;
            return {
              id: pid,
              src: projectImageViewUrl(pid),
              filename: match.filename || match.caption || ''
            };
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
      } else if (payload.kind === 'asset' && payload.src) {
        // C1: BOM asset chip (or whole row) dropped onto the canvas.
        // For now, drop it as a regular movable image — same path as a
        // photo. Once the canvas object kind 'asset' (BOM-linked) lands
        // in the schema, this can branch on `assetId` and place an
        // asset-typed Fabric object instead of a generic image.
        await dropPhotosOnCanvas([{ id: null, src: payload.src }], e);
      } else if (payload.kind === 'symbol' && payload.symbolId) {
        // Symbol Designer integration: rasterise the symbol_data off-
        // screen and drop the resulting PNG dataURL onto the canvas.
        // The drop behaves exactly like any other asset image from
        // there — selectable, resizable, deletable.
        var symSrc = await rasteriseSymbolToDataUrl(payload.symbolId);
        if (symSrc) {
          await dropPhotosOnCanvas([{ id: null, src: symSrc }], e);
        }
      }
    });
  }

  // Off-screen Fabric canvas → PNG dataURL. Mirrors the visual idiom of
  // the schematic editor's symbol rendering (body rect + sticks +
  // labels) but doesn't import the editor module — duplicating a
  // ~50-line render is cheaper than coupling the two pages. Same idea
  // the slideshow viewer uses for its render-to-png snapshots.
  async function rasteriseSymbolToDataUrl(symbolId) {
    var row = (state.projectSymbols || []).find(function (s) {
      return s && s.id === symbolId;
    });
    if (!row || !row.symbol_data) return null;
    var doc;
    try { doc = JSON.parse(row.symbol_data); } catch (_) { return null; }
    if (!doc || typeof doc !== 'object') return null;
    var shapes = Array.isArray(doc.bodyShapes) ? doc.bodyShapes : [];
    var pins = Array.isArray(doc.pins) ? doc.pins : [];
    // Bounding box (centred coords) — with generous padding so labels
    // and pin sticks don't get clipped.
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
    var pad = 32;
    var minX = Math.min.apply(null, xs) - pad;
    var minY = Math.min.apply(null, ys) - pad;
    var maxX = Math.max.apply(null, xs) + pad;
    var maxY = Math.max.apply(null, ys) + pad;
    var W = Math.max(maxX - minX, 64);
    var H = Math.max(maxY - minY, 64);
    var off = document.createElement('canvas');
    off.width = W;
    off.height = H;
    var ctx = off.getContext('2d');
    ctx.clearRect(0, 0, W, H);
    // Body rect — bounding box of shapes (so the symbol reads as a
    // single "chip"). Light stroke, white fill.
    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1.5;
    ctx.fillStyle = '#ffffff';
    // Body backdrop — uses the *unpadded* bounding box so the white
    // fill sits behind shapes but not behind labels / pin sticks (those
    // extend into the padded margin around the body).
    var sMinX = Math.min.apply(null, xs);
    var sMinY = Math.min.apply(null, ys);
    var sMaxX = Math.max.apply(null, xs);
    var sMaxY = Math.max.apply(null, ys);
    ctx.fillRect(sMinX - minX, sMinY - minY,
                 sMaxX - sMinX, sMaxY - sMinY);
    ctx.strokeRect(sMinX - minX, sMinY - minY,
                   sMaxX - sMinX, sMaxY - sMinY);
    // Body shapes
    shapes.forEach(function (s) {
      if (s.kind === 'rect') {
        ctx.strokeRect(s.x - minX, s.y - minY, s.w, s.h);
      } else if (s.kind === 'line') {
        ctx.beginPath();
        ctx.moveTo(s.x1 - minX, s.y1 - minY);
        ctx.lineTo(s.x2 - minX, s.y2 - minY);
        ctx.stroke();
      }
    });
    // Pins: short stick + label.
    ctx.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
    ctx.fillStyle = '#222';
    pins.forEach(function (p) {
      var px = p.x - minX, py = p.y - minY;
      var stickLen = 12;
      var ex = px, ey = py;
      switch (p.side) {
        case 'left':   ex = px + stickLen; break;
        case 'right':  ex = px - stickLen; break;
        case 'top':    ey = py + stickLen; break;
        case 'bottom': ey = py - stickLen; break;
      }
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(ex, ey);
      ctx.stroke();
      // Dot
      ctx.beginPath();
      ctx.arc(px, py, 3.5, 0, Math.PI * 2);
      ctx.fillStyle = '#ffffff';
      ctx.fill();
      ctx.stroke();
      // Label
      var label = (p.number || '') + (p.name ? ' ' + p.name : '');
      if (label.trim()) {
        ctx.fillStyle = '#222';
        ctx.textBaseline = 'middle';
        switch (p.side) {
          case 'left':   ctx.textAlign = 'left';   ctx.fillText(label, ex + 3, ey); break;
          case 'right':  ctx.textAlign = 'right';  ctx.fillText(label, ex - 3, ey); break;
          case 'top':    ctx.textAlign = 'center'; ctx.fillText(label, ex, ey + 8); break;
          case 'bottom': ctx.textAlign = 'center'; ctx.fillText(label, ex, ey - 8); break;
        }
      }
    });
    return off.toDataURL('image/png');
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
          ibFilename: items[i].filename || '',
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
  // 17.5 BOM PANE (C1) — expandable rows + draggable asset chips
  //
  // The BOM secondary pane is a *secondary view* on the project's
  // Bill of Materials. Source of truth lives in the project editor —
  // here authors only browse what's already linked and drag rows /
  // assets onto the canvas to illustrate steps.
  //
  // Data model (today):
  //   GET /api/projects/{id}/bom returns BOMItemResponse[] with fields
  //   { id, name, quantity, part_id, part_slug, ... }. The schema does
  //   NOT yet ship a per-part "assets" list — that's blocked on the
  //   parts-catalog asset graphics feature + the Symbol designer. So we
  //   derive a fallback asset list for each part from its catalog
  //   ``image_url`` (one photo-kind asset, or zero if the part has no
  //   image set). When the API grows a real assets[] field, swap the
  //   derivation for the schema-supplied list — the rest of the pane
  //   (chips, drag, drawer) works unchanged.
  // ====================================================================

  function loadBomItems(opts) {
    opts = opts || {};
    if (!state.projectId) return;
    if (state.bomItemsLoading) return;
    state.bomItemsLoading = true;
    state.bomItemsError = null;
    if (!opts.silent) renderBomPane();
    apiFetch(API + '/api/projects/' + state.projectId + '/bom')
      .then(function (resp) {
        if (!resp.ok) throw new Error('BOM HTTP ' + resp.status);
        return resp.json();
      })
      .then(function (items) {
        state.bomItems = Array.isArray(items) ? items : [];
        state.bomItemsLoaded = true;
        state.bomItemsLoading = false;
        renderBomPane();
        // Kick off asset fetches for rows that have a part_slug — these
        // run in parallel and re-render each row as they land.
        state.bomItems.forEach(function (item) {
          if (item.part_id && item.part_slug) {
            ensurePartAssets(item.part_id, item.part_slug);
          }
        });
        // Symbol Designer integration: pull the project's symbols once
        // so any with bom_item_id can surface as ⌗ chips in their
        // matching BOM row's asset drawer.
        loadProjectSymbols();
      })
      .catch(function (err) {
        state.bomItemsLoading = false;
        state.bomItemsError = err && err.message ? err.message : 'Failed to load BOM';
        renderBomPane();
      });
  }

  // Fetch the project's user-designed symbols (one row per Symbol
  // Designer entry). On success any matching BOM row's drawer picks up
  // a ⌗ chip via assetsForBomItem(); on failure the chips just don't
  // appear (the BOM pane itself still works).
  function loadProjectSymbols() {
    if (!state.projectId) return;
    if (state.projectSymbolsLoaded) {
      renderBomPane();
      return;
    }
    apiFetch(API + '/api/projects/' + state.projectId + '/symbols')
      .then(function (resp) {
        if (!resp.ok) throw new Error('Symbols HTTP ' + resp.status);
        return resp.json();
      })
      .then(function (rows) {
        state.projectSymbols = Array.isArray(rows) ? rows : [];
        state.projectSymbolsLoaded = true;
        renderBomPane();
      })
      .catch(function () {
        // Silent — no chips, but the BOM pane is unaffected.
        state.projectSymbols = [];
        state.projectSymbolsLoaded = true;
      });
  }

  // Fetch the part's catalog detail once and cache its derived asset
  // list. Today this is just `[{ id, label, kind: 'photo', src }]` if
  // the part has an image_url, or `[]` otherwise. The BOM row's
  // asset-count chip + drawer re-render when this lands.
  function ensurePartAssets(partId, partSlug) {
    if (!partId || !partSlug) return Promise.resolve([]);
    if (state.bomPartAssets[partId] !== undefined) {
      return Promise.resolve(state.bomPartAssets[partId]);
    }
    if (state.bomPartAssetsPending[partId]) {
      return state.bomPartAssetsPending[partId];
    }
    var p = apiFetch(API + '/api/parts/' + encodeURIComponent(partSlug))
      .then(function (resp) {
        if (!resp.ok) throw new Error('Part HTTP ' + resp.status);
        return resp.json();
      })
      .then(function (part) {
        var assets = [];
        if (part && part.image_url) {
          assets.push({
            id: 'part-' + partId + '-image',
            label: 'Photo',
            kind: 'photo',
            src: part.image_url,
          });
        }
        state.bomPartAssets[partId] = assets;
        delete state.bomPartAssetsPending[partId];
        // Re-render only the affected row (or full pane if simpler).
        renderBomPane();
        return assets;
      })
      .catch(function () {
        // Treat fetch failures as "no assets" rather than blocking the
        // whole pane. The drawer shows the empty-state hint.
        state.bomPartAssets[partId] = [];
        delete state.bomPartAssetsPending[partId];
        renderBomPane();
        return [];
      });
    state.bomPartAssetsPending[partId] = p;
    return p;
  }

  function assetsForBomItem(item) {
    if (!item) return [];
    // Combine inline + part-derived assets with any Symbol Designer
    // chips for this BOM row. Symbol chips are prepended so they read
    // first in the drawer — they're the most-specific "this part is
    // wired in this way" affordance.
    var baseAssets;
    if (Array.isArray(item.assets)) {
      baseAssets = item.assets;
    } else if (item.part_id && state.bomPartAssets[item.part_id]) {
      baseAssets = state.bomPartAssets[item.part_id];
    } else {
      baseAssets = [];
    }
    var symbolChips = symbolChipsForBomItem(item);
    if (symbolChips.length === 0) return baseAssets;
    return symbolChips.concat(baseAssets);
  }

  // Find project symbols whose bom_item_id matches this BOM row's id,
  // and convert each into an Asset-shaped object the drawer renderer
  // already knows how to lay out. The ``kind: 'symbol'`` flag is the
  // distinguishing field — chip rendering and the drag payload both
  // branch on it.
  function symbolChipsForBomItem(item) {
    if (!item || !item.id || !Array.isArray(state.projectSymbols)) return [];
    var out = [];
    state.projectSymbols.forEach(function (sym) {
      if (sym && sym.bom_item_id === item.id) {
        out.push({
          id: 'symbol-' + sym.id,
          symbolId: sym.id,
          label: sym.name || ('Symbol #' + sym.id),
          kind: 'symbol',
          chip: '⌗',  // ⌗
        });
      }
    });
    return out;
  }

  function bomExpandedSet() {
    var ui = state.uiState || {};
    var arr = Array.isArray(ui.bomExpandedRowIds) ? ui.bomExpandedRowIds : [];
    var set = {};
    arr.forEach(function (id) { set[String(id)] = true; });
    return set;
  }

  function toggleBomRowExpanded(rowId) {
    if (!state.uiState) return;
    var ids = Array.isArray(state.uiState.bomExpandedRowIds)
      ? state.uiState.bomExpandedRowIds.slice() : [];
    var key = String(rowId);
    var idx = -1;
    for (var i = 0; i < ids.length; i++) {
      if (String(ids[i]) === key) { idx = i; break; }
    }
    if (idx >= 0) ids.splice(idx, 1);
    else ids.push(rowId);
    state.uiState.bomExpandedRowIds = ids;
    saveUiState();
    renderBomPane();
  }

  function renderBomPane() {
    if (!dom.bomList) return;

    if (state.bomItemsLoading && !state.bomItemsLoaded) {
      dom.bomList.innerHTML =
        '<div class="ib-bom-loading">' +
        '<i class="fas fa-circle-notch fa-spin me-1"></i> Loading BOM…' +
        '</div>';
      return;
    }

    if (state.bomItemsError) {
      dom.bomList.innerHTML =
        '<div class="ib-bom-error">' +
        '<p>Couldn\'t load the BOM.</p>' +
        '<button type="button" class="btn btn-sm btn-outline-primary" id="ib-bom-retry">' +
        '<i class="fas fa-rotate-right me-1"></i> Retry</button>' +
        '</div>';
      var retry = document.getElementById('ib-bom-retry');
      if (retry) retry.addEventListener('click', function () { loadBomItems(); });
      return;
    }

    var items = state.bomItems || [];
    if (items.length === 0) {
      dom.bomList.innerHTML =
        '<div class="ib-bom-empty">' +
        '<div class="ib-bom-empty-icon"><i class="fas fa-receipt"></i></div>' +
        '<p>This project has no Bill of Materials yet.</p>' +
        '<p class="small text-muted">Add parts in the project editor\'s BOM section.</p>' +
        '<a class="btn btn-sm btn-outline-primary" href="' +
        escapeHtml('/projects/editor.html?id=' + encodeURIComponent(state.projectId) +
                   '#editor-bom-section') +
        '"><i class="fas fa-external-link-alt me-1"></i> Open BOM in editor</a>' +
        '</div>';
      return;
    }

    var expanded = bomExpandedSet();
    var html = items.map(function (item) { return renderBomRowHtml(item, expanded); }).join('');
    dom.bomList.innerHTML = html;

    // Wire each row's DnD + click handlers.
    Array.prototype.forEach.call(
      dom.bomList.querySelectorAll('.ib-bom-row'),
      function (rowEl) { wireBomRow(rowEl); }
    );
  }

  function renderBomRowHtml(item, expanded) {
    var rowId = item.id;
    var qty = item.quantity != null ? item.quantity : 1;
    var name = item.name || 'Unnamed part';
    var assets = assetsForBomItem(item);
    var count = assets.length;
    var loading = !!(item.part_id && item.part_slug &&
                     state.bomPartAssets[item.part_id] === undefined);
    var isExpanded = !!expanded[String(rowId)];
    var chipText = loading
      ? '<i class="fas fa-circle-notch fa-spin"></i>'
      : escapeHtml(String(count)) + ' <i class="far fa-images"></i>';
    var chipTitle = loading
      ? 'Loading assets…'
      : (count + ' asset' + (count === 1 ? '' : 's') + ' attached');

    // Build the deep-link URL for "design symbol" before encoding into
    // the dropdown markup. ``new=true`` so the symbol designer creates
    // a fresh row pre-linked to this BOM item via ``bom_item_id``.
    var designSymbolUrl = '/projects/symbol/edit.html?project_id=' +
      encodeURIComponent(state.projectId) +
      '&new=true&bom_item_id=' + encodeURIComponent(rowId);

    return '' +
      '<div class="ib-bom-row' + (isExpanded ? ' is-expanded' : '') + '"' +
      ' data-bom-row-id="' + escapeHtml(String(rowId)) + '"' +
      ' draggable="true">' +
        '<div class="ib-bom-row-head" role="button" tabindex="0"' +
        ' aria-expanded="' + (isExpanded ? 'true' : 'false') + '">' +
          '<span class="ib-bom-qty" aria-label="Quantity ' +
          escapeHtml(String(qty)) + '">×&nbsp;' + escapeHtml(String(qty)) + '</span>' +
          '<span class="ib-bom-name" title="' + escapeHtml(name) + '">' +
          escapeHtml(name) + '</span>' +
          '<span class="ib-bom-asset-chip" title="' + escapeHtml(chipTitle) + '">' +
          chipText + '</span>' +
          // Bootstrap dropdown for the ⋮ menu — was an inert button in
          // the C1 merge; gaining "design symbol" links + a placeholder
          // for future actions.
          '<div class="dropdown ib-bom-row-menu-wrap">' +
            '<button type="button" class="ib-bom-row-menu" aria-label="More actions"' +
            ' tabindex="-1" data-bs-toggle="dropdown" data-bs-auto-close="true"' +
            ' aria-expanded="false" title="More actions">' +
              '<i class="fas fa-ellipsis-vertical"></i>' +
            '</button>' +
            '<ul class="dropdown-menu dropdown-menu-end ib-bom-row-menu-list">' +
              '<li>' +
                '<a class="dropdown-item ib-bom-menu-design-symbol" href="' +
                escapeHtml(designSymbolUrl) + '">' +
                  '<span class="ib-bom-menu-glyph">&#x270E;</span> design symbol&hellip;' +
                '</a>' +
              '</li>' +
              '<li>' +
                '<span class="dropdown-item disabled ib-bom-menu-disabled"' +
                ' aria-disabled="true" title="Use the project editor\'s BOM section">' +
                  '<i class="fas fa-trash me-2"></i>remove from BOM' +
                '</span>' +
              '</li>' +
            '</ul>' +
          '</div>' +
          '<button type="button" class="ib-bom-row-toggle" aria-label="' +
          (isExpanded ? 'Collapse' : 'Expand') + '" tabindex="-1">' +
            '<i class="fas fa-chevron-' + (isExpanded ? 'up' : 'down') + '"></i>' +
          '</button>' +
        '</div>' +
        (isExpanded
          ? '<div class="ib-bom-row-drawer">' + renderBomDrawerHtml(item, assets, loading) + '</div>'
          : '') +
      '</div>';
  }

  function renderBomDrawerHtml(item, assets, loading) {
    if (loading) {
      return '<div class="ib-bom-drawer-loading">' +
        '<i class="fas fa-circle-notch fa-spin me-1"></i> Loading assets…' +
        '</div>';
    }
    if (!assets || assets.length === 0) {
      return '<div class="ib-bom-drawer-empty">' +
        'No assets attached yet — add an image to this part in the ' +
        '<a href="' +
        (item.part_slug
          ? escapeHtml('/parts/view.html?slug=' + encodeURIComponent(item.part_slug))
          : '#') +
        '" target="_blank" rel="noopener">Parts Catalog</a>, or ' +
        '<a href="' +
        escapeHtml('/projects/symbol/edit.html?project_id=' +
          encodeURIComponent(state.projectId) + '&new=true&bom_item_id=' +
          encodeURIComponent(item.id)) +
        '">design a <code>&#x2317;</code> symbol</a> for it.' +
        '</div>';
    }
    return '<div class="ib-bom-assets">' + assets.map(function (a) {
      var safeLabel = escapeHtml(a.label || 'Asset');
      var safeKind = escapeHtml(a.kind || 'photo');
      // Symbol-kind chips show a `⌗` glyph instead of an image thumb
      // and carry a different drag payload (no src; the canvas drop
      // handler rasterises the symbol_data at drop time).
      if (a.kind === 'symbol') {
        var safeSymId = escapeHtml(String(a.symbolId || ''));
        return '<div class="ib-bom-asset ib-bom-asset-symbol" draggable="true"' +
          ' data-asset-kind="symbol"' +
          ' data-asset-label="' + safeLabel + '"' +
          ' data-symbol-id="' + safeSymId + '"' +
          ' title="' + safeLabel + ' — drag onto canvas to place a schematic symbol">' +
            '<div class="ib-bom-asset-thumb ib-bom-asset-symbol-thumb">' +
              '<span class="ib-bom-symbol-glyph">&#x2317;</span>' +
            '</div>' +
            '<div class="ib-bom-asset-label">' + safeLabel + '</div>' +
          '</div>';
      }
      var safeSrc = escapeHtml(a.src || '');
      return '<div class="ib-bom-asset" draggable="true"' +
        ' data-asset-src="' + safeSrc + '"' +
        ' data-asset-label="' + safeLabel + '"' +
        ' data-asset-kind="' + safeKind + '"' +
        ' title="' + safeLabel + ' — drag onto canvas">' +
          '<div class="ib-bom-asset-thumb">' +
            (a.src
              ? '<img src="' + safeSrc + '" alt="' + safeLabel + '" loading="lazy">'
              : '<i class="far fa-image"></i>') +
          '</div>' +
          '<div class="ib-bom-asset-label">' + safeLabel + '</div>' +
        '</div>';
    }).join('') + '</div>';
  }

  function wireBomRow(rowEl) {
    var rowId = rowEl.getAttribute('data-bom-row-id');
    var head = rowEl.querySelector('.ib-bom-row-head');
    var toggleBtn = rowEl.querySelector('.ib-bom-row-toggle');
    var menuBtn = rowEl.querySelector('.ib-bom-row-menu');

    // Click on head (anywhere except the buttons / dropdown) toggles drawer.
    if (head) {
      head.addEventListener('click', function (e) {
        var t = e.target;
        // Ignore clicks on the menu / toggle buttons / dropdown items —
        // they each have their own handlers.
        if (t && (t.closest('.ib-bom-row-menu-wrap') ||
                  t.closest('.ib-bom-row-toggle'))) return;
        toggleBomRowExpanded(rowId);
      });
      head.addEventListener('keydown', function (e) {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          toggleBomRowExpanded(rowId);
        }
      });
    }
    if (toggleBtn) {
      toggleBtn.addEventListener('click', function (e) {
        e.stopPropagation();
        toggleBomRowExpanded(rowId);
      });
    }
    if (menuBtn) {
      // Bootstrap handles open/close via data-bs-toggle. We just stop
      // propagation so the row doesn't toggle on the same click.
      menuBtn.addEventListener('click', function (e) { e.stopPropagation(); });
    }
    // Keep dropdown-item clicks from bubbling into the row (which would
    // toggle the drawer mid-navigation).
    Array.prototype.forEach.call(
      rowEl.querySelectorAll('.ib-bom-row-menu-list .dropdown-item'),
      function (a) {
        a.addEventListener('click', function (e) { e.stopPropagation(); });
      }
    );

    // Drag the row itself → default asset (first in list, photo today).
    rowEl.addEventListener('dragstart', function (e) {
      var item = (state.bomItems || []).find(function (b) {
        return String(b.id) === String(rowId);
      });
      var assets = assetsForBomItem(item);
      var firstAsset = assets.length > 0 ? assets[0] : null;
      if (!firstAsset) {
        // No default asset — abort the drag so the canvas drop is a no-op,
        // and show a hint. We still allow dragstart but with no payload,
        // so the drop handler silently ignores it.
        e.preventDefault();
        setSaveStatus('error', 'No assets yet for this part. Add an image in the Parts Catalog.');
        setTimeout(function () { setSaveStatus(null); }, 2500);
        return;
      }
      try {
        e.dataTransfer.effectAllowed = 'copy';
        var payload = {
          kind: 'asset',
          src: firstAsset.src,
          label: firstAsset.label,
          assetKind: firstAsset.kind,
          bomRowId: rowId,
        };
        e.dataTransfer.setData('application/json', JSON.stringify(payload));
        e.dataTransfer.setData('text/plain', firstAsset.src || '');
      } catch (_) { /* setData can throw in some sandboxes */ }
    });

    // Per-chip drag handlers (only present when row is expanded).
    Array.prototype.forEach.call(
      rowEl.querySelectorAll('.ib-bom-asset'),
      function (chip) {
        chip.addEventListener('dragstart', function (e) {
          // Stop the wrapping row's dragstart from also firing (it would
          // overwrite the chip's payload with the row default).
          e.stopPropagation();
          var label = chip.getAttribute('data-asset-label') || 'Asset';
          var kind = chip.getAttribute('data-asset-kind') || 'photo';
          // Symbol-kind chips: payload carries a symbolId rather than a
          // src; the canvas drop handler rasterises the symbol_data
          // off-screen into a PNG and places it like any other asset.
          // This is the "rasterised symbol on a photo step" path the
          // design handoff README references.
          if (kind === 'symbol') {
            var symbolId = parseInt(chip.getAttribute('data-symbol-id'), 10);
            if (!symbolId) { e.preventDefault(); return; }
            try {
              e.dataTransfer.effectAllowed = 'copy';
              e.dataTransfer.setData('application/json', JSON.stringify({
                kind: 'symbol',
                symbolId: symbolId,
                label: label,
                bomRowId: rowId,
              }));
              e.dataTransfer.setData('text/plain', label);
            } catch (_) {}
            return;
          }
          var src = chip.getAttribute('data-asset-src') || '';
          if (!src) { e.preventDefault(); return; }
          try {
            e.dataTransfer.effectAllowed = 'copy';
            e.dataTransfer.setData('application/json', JSON.stringify({
              kind: 'asset',
              src: src,
              label: label,
              assetKind: kind,
              bomRowId: rowId,
            }));
            e.dataTransfer.setData('text/plain', src);
          } catch (_) {}
        });
      }
    );
  }

  function wireBomPane() {
    if (dom.bomRefresh) {
      dom.bomRefresh.addEventListener('click', function () {
        // Hard refresh — clear the per-part asset cache too so the
        // drawers re-pick up any newly-added catalog images.
        state.bomPartAssets = {};
        state.bomPartAssetsPending = {};
        state.bomItemsLoaded = false;
        loadBomItems();
      });
    }
    // The "project editor's BOM section" link's href is set in init()
    // alongside the other project-id-bearing URLs. No click handler
    // needed — let the browser navigate naturally.
  }

  // ====================================================================
  // 18. LAYERS PANE
  // ====================================================================

  // Resolve a friendly filename for an image-type Fabric object.
  // Pre-fix canvases lack ibFilename AND sometimes ibRole/ibSourceUrl,
  // so this walks every signal we have:
  //   1. The canonical ibFilename property we set on placement.
  //   2. ibSourceUrl → look up in state.projectImages by image id
  //      embedded in the URL (gives the real upload filename).
  //   3. Fabric's underlying HTMLImageElement.src — what the browser
  //      actually loaded — falls back to the last URL path segment.
  function filenameForFabricImage(o) {
    if (!o) return '';
    var fname = (o.ibFilename || '').trim();
    if (fname) return fname;
    var url = o.ibSourceUrl || '';
    if (!url && typeof o.getSrc === 'function') {
      try { url = o.getSrc(); } catch (_) {}
    }
    if (!url && o._element && o._element.src) url = o._element.src;
    if (!url) return '';
    // Project-images URL → look up the friendly filename.
    var m = /\/api\/projects\/\d+\/images\/(\d+)\/view/.exec(url);
    if (m) {
      var id = parseInt(m[1], 10);
      if (id && state.projectImages) {
        var match = state.projectImages.find(function (im) { return im.id === id; });
        if (match) return match.filename || match.caption || '';
      }
    }
    // Last resort: the last URL path segment if it looks like a filename.
    try {
      var u = new URL(url, window.location.origin);
      var last = u.pathname.split('/').filter(Boolean).pop();
      if (last && last.indexOf('.') > 0) return decodeURIComponent(last);
    } catch (_) {}
    return '';
  }

  function objectKindLabel(o) {
    if (!o) return { icon: 'fas fa-question', label: 'Object' };
    // For image objects, prefer the source filename when we have one —
    // a stack of "Image, Image, Image" is useless on a step with several
    // photos / project-images dropped in. Falls back to a Photos-pane
    // lookup for legacy canvases written before ibFilename was a thing.
    var fname = filenameForFabricImage(o);
    if (o.ibRole === 'background') {
      return { icon: 'fas fa-image', label: fname ? ('Background: ' + fname) : 'Background image' };
    }
    if (o.ibRole === 'stl') return { icon: 'fas fa-cube', label: fname ? ('STL: ' + fname) : 'STL view' };
    if (o.ibRole === 'photo' || o.ibRole === 'project-image') {
      return { icon: 'fas fa-image', label: fname || 'Image' };
    }
    if (o.type === 'image' || (o.isType && o.isType('image'))) {
      return { icon: 'fas fa-image', label: fname || 'Image' };
    }
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

  // B3: per-type badge for the filmstrip thumb. Returns an HTML snippet
  // (the icon-only pill) or empty string for the blank type, which is
  // unbadged by design (it's the same as photo-without-bg visually).
  function stepTypeBadgeHtml(stepType) {
    var iconMap = {
      photo: 'fa-image',
      schematic: 'fa-diagram-project',
      text: 'fa-pen-to-square',
      video: 'fa-video',
    };
    var icon = iconMap[stepType];
    if (!icon) return '';  // blank or unknown → no badge
    return (
      '<span class="ib-step-tile-badge" title="' + escapeHtml(stepType) + '">' +
      '<i class="fas ' + icon + '"></i>' +
      '</span>'
    );
  }

  // Step-thumbnail cache: stepId → dataURL (or null = "rendered, empty").
  // Filled lazily by renderStepThumbsLater(). renderFilmstrip uses it to
  // paint a real mini canvas preview into each tile's .preview slot.
  if (!state.stepThumbs) state.stepThumbs = {};

  function renderFilmstrip() {
    if (!dom.filmstripTrack) return;
    // The delete control is disabled (still rendered) on the only-step
    // case — that way the user sees the affordance and gets a hint via
    // the title attribute / toast rather than the button silently
    // disappearing.
    var isOnlyStep = state.steps.length === 1;
    var html = state.steps.map(function (s) {
      var isActive = (s.id === state.activeStepId);
      var caption = (s.title && s.title.trim()) ? s.title : 'Untitled';
      var badge = stepTypeBadgeHtml(currentStepTypeOf(s));
      var deleteTitle = isOnlyStep
        ? "Can't delete the only step"
        : 'Delete step ' + s.step_number;
      var deleteBtn = (
        '<button type="button" class="ib-step-tile-delete"' +
        '        data-step-id="' + s.id + '"' +
        '        aria-label="' + escapeHtml(deleteTitle) + '"' +
        '        title="' + escapeHtml(deleteTitle) + '"' +
                (isOnlyStep ? ' disabled' : '') + '>' +
        '<i class="fas fa-xmark"></i>' +
        '</button>'
      );
      // Preview content: either a cached thumbnail dataURL drawn as a
      // background image, or a step-type icon + fallback label.
      var thumb = state.stepThumbs[s.id];
      var previewInner;
      var previewStyle = '';
      if (thumb) {
        previewStyle = ' style="background-image:url(\'' + thumb + '\');"';
        previewInner = '';
      } else {
        previewInner = '<span>step ' + s.step_number + '</span>';
      }
      return (
        '<div class="ib-step-tile' + (isActive ? ' is-active' : '') + '"' +
        '     draggable="true"' +
        '     data-step-id="' + s.id + '">' +
        '  <div class="ib-step-tile-num">' + s.step_number + '</div>' +
        badge +
        deleteBtn +
        '  <div class="ib-step-tile-preview' + (thumb ? ' has-thumb' : '') + '"' +
              previewStyle + '>' + previewInner + '</div>' +
        '  <div class="ib-step-tile-caption" title="' + escapeHtml(caption) +
              '" data-step-id="' + s.id + '">' +
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
        tile.addEventListener('click', function (ev) {
          // Don't fire step-switch when the click landed on the delete
          // button — that has its own handler below.
          if (ev.target && ev.target.closest && ev.target.closest('.ib-step-tile-delete')) {
            return;
          }
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

    // Wire the per-tile delete buttons. Bound separately so the click
    // doesn't bubble into the tile's switchToStep handler.
    Array.prototype.forEach.call(
      dom.filmstripTrack.querySelectorAll('.ib-step-tile-delete'),
      function (btn) {
        btn.addEventListener('click', function (ev) {
          ev.stopPropagation();
          ev.preventDefault();
          if (btn.disabled) {
            showStepToast("Can't delete the only step — add a new step first.");
            return;
          }
          var id = parseInt(btn.dataset.stepId, 10);
          if (id) onDeleteStepClick(id);
        });
        // Drag events on the tile shouldn't start when the user is
        // mousing down on the delete button.
        btn.addEventListener('mousedown', function (ev) { ev.stopPropagation(); });
        btn.addEventListener('dragstart', function (ev) { ev.preventDefault(); ev.stopPropagation(); });
      });

    // Inline rename: double-click the caption to edit. Saves on blur or
    // Enter; Escape reverts. Single-click on the tile body still
    // switches steps via the tile-level click handler above.
    Array.prototype.forEach.call(
      dom.filmstripTrack.querySelectorAll('.ib-step-tile-caption'),
      function (cap) {
        cap.addEventListener('dblclick', function (ev) {
          ev.stopPropagation();
          ev.preventDefault();
          startStepCaptionEdit(cap);
        });
      });
  }

  // Replace a caption span with an inline input, save on blur / Enter,
  // cancel on Escape. Mirrors the title autosave from the right-side
  // settings pane, just at the filmstrip surface.
  function startStepCaptionEdit(captionEl) {
    var stepId = parseInt(captionEl.dataset.stepId, 10);
    if (!stepId) return;
    var step = state.steps.find(function (s) { return s.id === stepId; });
    if (!step) return;
    var original = (step.title && step.title.trim()) ? step.title : '';
    var input = document.createElement('input');
    input.type = 'text';
    input.className = 'ib-step-tile-caption-input';
    input.value = original;
    input.placeholder = 'Untitled';
    input.maxLength = 200;
    captionEl.replaceWith(input);
    input.focus();
    input.select();

    var done = false;
    function finish(saveIt) {
      if (done) return;
      done = true;
      var next = saveIt ? input.value.trim() : original;
      // Restore the span before any await — keeps the strip stable.
      var span = document.createElement('div');
      span.className = 'ib-step-tile-caption';
      span.dataset.stepId = String(stepId);
      span.title = next || 'Untitled';
      span.textContent = next || 'Untitled';
      span.addEventListener('dblclick', function (ev) {
        ev.stopPropagation();
        ev.preventDefault();
        startStepCaptionEdit(span);
      });
      input.replaceWith(span);
      if (saveIt && next !== original) {
        renameStepFromFilmstrip(step, next);
      }
    }
    input.addEventListener('blur', function () { finish(true); });
    input.addEventListener('keydown', function (e) {
      if (e.key === 'Enter') { e.preventDefault(); finish(true); }
      else if (e.key === 'Escape') { e.preventDefault(); finish(false); }
    });
    // Clicking the input shouldn't switch steps.
    input.addEventListener('click', function (ev) { ev.stopPropagation(); });
    input.addEventListener('mousedown', function (ev) { ev.stopPropagation(); });
  }

  async function renameStepFromFilmstrip(step, newTitle) {
    var oldTitle = step.title || '';
    step.title = newTitle;
    setSaveStatus('saving', 'Saving title…');
    try {
      await putStep(step.id, { title: newTitle });
      setSaveStatus('saved');
      // If this is the active step, sync the right-side settings input
      // so it doesn't show the stale value.
      if (step.id === state.activeStepId && dom.stepTitle) {
        dom.stepTitle.value = newTitle;
      }
      pushStepUndo({
        type: 'rename-step',
        label: 'renamed step',
        apply: function () {
          step.title = oldTitle;
          if (step.id === state.activeStepId && dom.stepTitle) dom.stepTitle.value = oldTitle;
          renderFilmstrip();
          return putStep(step.id, { title: oldTitle });
        },
        invert: function () {
          step.title = newTitle;
          if (step.id === state.activeStepId && dom.stepTitle) dom.stepTitle.value = newTitle;
          renderFilmstrip();
          return putStep(step.id, { title: newTitle });
        },
      });
    } catch (_) {
      step.title = oldTitle;
      setSaveStatus('error', 'Title save failed');
    }
    renderFilmstrip();
  }

  // ---- Filmstrip step-thumbnail generator ------------------------------
  //
  // Renders each step's canvas_json to a tiny PNG via an off-screen
  // fabric.StaticCanvas and stashes the dataURL in state.stepThumbs.
  // Re-renders happen on save (active step) and on initial filmstrip
  // paint.

  var STEP_THUMB_W = 140;
  var STEP_THUMB_H = Math.round(STEP_THUMB_W * (CANVAS_H / CANVAS_W));
  var _stepThumbCanvas = null;
  var _stepThumbBusy = false;
  var _stepThumbQueued = false;

  function _ensureThumbCanvas() {
    if (_stepThumbCanvas) return _stepThumbCanvas;
    if (typeof fabric === 'undefined' || !fabric.StaticCanvas) return null;
    _stepThumbCanvas = new fabric.StaticCanvas(null, {
      width: CANVAS_W,
      height: CANVAS_H,
      enableRetinaScaling: false,
    });
    _stepThumbCanvas.backgroundColor = '#ffffff';
    return _stepThumbCanvas;
  }

  function _renderOneStepThumb(canvasJsonString) {
    return new Promise(function (resolve) {
      var c = _ensureThumbCanvas();
      if (!c) return resolve(null);
      if (!canvasJsonString) {
        c.clear();
        c.backgroundColor = '#ffffff';
        c.renderAll();
        try {
          resolve(c.toDataURL({
            format: 'png',
            multiplier: STEP_THUMB_W / CANVAS_W,
          }));
        } catch (_) { resolve(null); }
        return;
      }
      try {
        c.loadFromJSON(canvasJsonString, function () {
          c.backgroundColor = '#ffffff';
          c.renderAll();
          try {
            resolve(c.toDataURL({
              format: 'png',
              multiplier: STEP_THUMB_W / CANVAS_W,
            }));
          } catch (_) { resolve(null); }
        });
      } catch (_) { resolve(null); }
    });
  }

  async function rebuildAllStepThumbs() {
    if (_stepThumbBusy) { _stepThumbQueued = true; return; }
    _stepThumbBusy = true;
    try {
      for (var i = 0; i < state.steps.length; i++) {
        var s = state.steps[i];
        // For non-photo/blank steps we don't have a canvas to render —
        // fall back to null (tile shows the type icon + label).
        var t = s.step_type || 'photo';
        if (t !== 'photo' && t !== 'blank') {
          state.stepThumbs[s.id] = null;
          continue;
        }
        // Active step's most recent canvas_json may be ahead of what's
        // on the server — but autosave normally flushes before this
        // runs, so canvas_json is good enough. We refresh the active
        // step explicitly after each save via refreshActiveStepThumb.
        var url = await _renderOneStepThumb(s.canvas_json);
        state.stepThumbs[s.id] = url;
      }
      renderFilmstrip();
    } finally {
      _stepThumbBusy = false;
      if (_stepThumbQueued) { _stepThumbQueued = false; rebuildAllStepThumbs(); }
    }
  }

  async function refreshActiveStepThumb() {
    if (!state.canvas || !state.activeStepId) return;
    if (typeof fabric === 'undefined' || !fabric.StaticCanvas) return;
    try {
      var json = JSON.stringify(serializeCanvas());
      var url = await _renderOneStepThumb(json);
      state.stepThumbs[state.activeStepId] = url;
      // Update only the active step's preview slot in place — avoids a
      // full filmstrip re-render with all the event-handler rebinding.
      var tile = dom.filmstripTrack &&
        dom.filmstripTrack.querySelector('.ib-step-tile[data-step-id="' + state.activeStepId + '"]');
      if (tile) {
        var preview = tile.querySelector('.ib-step-tile-preview');
        if (preview) {
          if (url) {
            preview.style.backgroundImage = "url('" + url + "')";
            preview.classList.add('has-thumb');
            preview.innerHTML = '';
          } else {
            preview.style.backgroundImage = '';
            preview.classList.remove('has-thumb');
          }
        }
      }
    } catch (_) { /* swallow — non-fatal */ }
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
    // The +add tile is now a Bootstrap dropdown toggle — clicking it
    // shows the step-type menu. Each menu item is wired below to call
    // onAddStep(type). Bootstrap handles the open/close animation via
    // data-bs-toggle="dropdown".
    var addMenu = document.querySelector('.ib-filmstrip-add-menu');
    if (addMenu) {
      Array.prototype.forEach.call(
        addMenu.querySelectorAll('[data-add-step-type]'),
        function (item) {
          item.addEventListener('click', function (ev) {
            ev.preventDefault();
            onAddStep(item.dataset.addStepType);
          });
        }
      );
    }
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
    var previousStepNumber = fromStep.step_number;
    var newStepNumber = toStep.step_number;
    setSaveStatus('saving', 'Reordering steps…');
    try {
      await putStep(fromId, { step_number: newStepNumber });
      var fresh = await fetchInstruction();
      if (fresh && fresh.steps) state.steps = fresh.steps;
      renderFilmstrip();
      setSaveStatus('saved');
      // Inverse: PUT step_number back to the previous value. The other
      // steps' numbers get re-balanced by the server. Redo restores the
      // new target position.
      pushStepUndo({
        type: 'reorder-step',
        label: 'reordered step',
        apply: async function () {
          await putStep(fromId, { step_number: previousStepNumber });
          var fresh2 = await fetchInstruction();
          if (fresh2 && fresh2.steps) state.steps = fresh2.steps;
          renderFilmstrip();
        },
        invert: async function () {
          await putStep(fromId, { step_number: newStepNumber });
          var fresh2 = await fetchInstruction();
          if (fresh2 && fresh2.steps) state.steps = fresh2.steps;
          renderFilmstrip();
        },
      });
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
    // B3: refresh the step-type pane + canvas-area swap for this step.
    syncStepTypePane();
    renderFilmstrip();
    renderLayersList();
  }

  async function onAddStep(stepType) {
    if (dom.filmstripAdd) dom.filmstripAdd.disabled = true;
    setSaveStatus('saving', 'Adding step…');
    try {
      // Validate before sending — server's Literal type also rejects, but
      // catching here gives a cleaner failure mode.
      var allowed = { photo:1, schematic:1, text:1, video:1, blank:1 };
      var initial = (stepType && allowed[stepType]) ? { step_type: stepType } : {};
      var step = await postStep(initial);
      var fresh = await fetchInstruction();
      if (fresh && fresh.steps) {
        state.steps = fresh.steps;
      } else {
        state.steps.push(step);
      }
      var previousActiveId = state.activeStepId;
      await switchToStep(step.id);
      setSaveStatus('saved');
      // Record inverse: delete this newly-created step + restore the
      // previously-active step. Redo POSTs a fresh empty step (the user
      // hadn't filled anything in yet — any subsequent edits would have
      // pushed their own undo entries on top).
      var addedStepRef = step;
      pushStepUndo({
        type: 'add-step',
        label: 'added step',
        apply: async function () {
          try {
            await deleteStep(addedStepRef.id);
            var fresh2 = await fetchInstruction();
            if (fresh2 && fresh2.steps) state.steps = fresh2.steps;
            else state.steps = state.steps.filter(function (s) { return s.id !== addedStepRef.id; });
            if (previousActiveId && state.steps.some(function (s) { return s.id === previousActiveId; })) {
              await switchToStep(previousActiveId);
            } else if (state.steps.length > 0) {
              await switchToStep(state.steps[0].id);
            }
            renderFilmstrip();
          } catch (err) {
            console.warn('Undo add-step failed', err);
            throw err;
          }
        },
        invert: async function () {
          var newStep = await postStep();
          addedStepRef = newStep;
          var fresh2 = await fetchInstruction();
          if (fresh2 && fresh2.steps) state.steps = fresh2.steps;
          else state.steps.push(newStep);
          await switchToStep(newStep.id);
          renderFilmstrip();
        },
      });
    } catch (e) {
      setSaveStatus('error', 'Add step failed');
    } finally {
      if (dom.filmstripAdd) dom.filmstripAdd.disabled = false;
    }
  }

  // Delete a step from the filmstrip. Confirms, snapshots the full
  // pre-delete state for undo (recreate via POST + PUT-all-fields), and
  // switches to a sensible neighbour. The last remaining step can't be
  // deleted (the button is disabled) — the click handler in
  // renderFilmstrip catches the disabled case before this is reached.
  async function onDeleteStepClick(stepId) {
    var step = state.steps.find(function (s) { return s.id === stepId; });
    if (!step) return;
    if (state.steps.length <= 1) {
      // Belt-and-braces: the button is disabled in this state but a
      // programmatic call could still get here.
      showStepToast("Can't delete the only step — add a new step first.");
      return;
    }
    var confirmed = window.confirm(
      'Delete this step? Its overlays will be lost — use Cmd/Ctrl+Z to undo.'
    );
    if (!confirmed) return;

    // Capture the full pre-delete state so the undo can recreate it
    // verbatim. We grab from the in-memory copy (which is in sync with
    // the server thanks to the autosave plumbing). If the user is mid-
    // edit, flushCanvasSave will run when switchToStep fires on the
    // neighbour selection, but for the *current* step we capture the
    // last-saved canvas_json — anything later than that wouldn't have
    // landed on the server and so wouldn't survive a refresh anyway.
    var savedState = {
      title: step.title || null,
      description: step.description || null,
      step_type: step.step_type || 'photo',
      body: step.body || null,
      video_url: step.video_url || null,
      schematic_id: step.schematic_id || null,
      canvas_json: step.canvas_json || null,
      step_number: step.step_number,
    };
    var deletedStepNumber = step.step_number;
    // If we're deleting the active step we need to switch elsewhere
    // first. Prefer the previous step; fall back to the new first step
    // (which the renumbered list will surface).
    var wasActive = (state.activeStepId === stepId);

    setSaveStatus('saving', 'Deleting step…');
    try {
      await deleteStep(stepId);
      var fresh = await fetchInstruction();
      if (fresh && fresh.steps) {
        state.steps = fresh.steps;
      } else {
        state.steps = state.steps.filter(function (s) { return s.id !== stepId; });
      }
      if (wasActive && state.steps.length > 0) {
        // Pick the step that now occupies (deletedStepNumber - 1), or
        // step 1 if we just removed step 1.
        var targetIdx = Math.max(0, deletedStepNumber - 2);
        if (targetIdx >= state.steps.length) targetIdx = state.steps.length - 1;
        await switchToStep(state.steps[targetIdx].id);
      } else {
        renderFilmstrip();
      }
      setSaveStatus('saved');
      showStepToast('Deleted step ' + deletedStepNumber + ' · Cmd/Ctrl+Z to undo');

      // Undo: recreate the step (new id), restore all fields, slot back
      // into the original step_number, and re-activate it if it was
      // active. Redo: delete the freshly-recreated step again.
      var recreatedRef = { id: null };
      pushStepUndo({
        type: 'delete-step',
        label: 'deleted step ' + deletedStepNumber,
        apply: async function () {
          var created = await postStep();
          // PUT everything back. step_number is part of the same PUT so
          // the server re-balances neighbours into position.
          var updated = await putStep(created.id, {
            title: savedState.title,
            description: savedState.description,
            step_type: savedState.step_type,
            body: savedState.body,
            video_url: savedState.video_url,
            schematic_id: savedState.schematic_id,
            canvas_json: savedState.canvas_json,
            step_number: savedState.step_number,
          });
          recreatedRef.id = updated.id;
          var fresh2 = await fetchInstruction();
          if (fresh2 && fresh2.steps) state.steps = fresh2.steps;
          await switchToStep(updated.id);
          renderFilmstrip();
        },
        invert: async function () {
          if (!recreatedRef.id) return;
          await deleteStep(recreatedRef.id);
          var fresh2 = await fetchInstruction();
          if (fresh2 && fresh2.steps) state.steps = fresh2.steps;
          else state.steps = state.steps.filter(function (s) { return s.id !== recreatedRef.id; });
          if (state.activeStepId === recreatedRef.id && state.steps.length > 0) {
            var targetIdx = Math.max(0, deletedStepNumber - 2);
            if (targetIdx >= state.steps.length) targetIdx = state.steps.length - 1;
            await switchToStep(state.steps[targetIdx].id);
          } else {
            renderFilmstrip();
          }
        },
      });
    } catch (e) {
      setSaveStatus('error', 'Delete step failed');
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
        } else if (kind === 'schematic-csv' || kind === 'schematic-png') {
          // Forward into the embedded schematic editor via postMessage.
          // Only meaningful when the active step is a schematic step;
          // otherwise show a brief toast hint.
          if (currentActiveStepType() !== 'schematic') {
            setSaveStatus('error', 'Switch to a schematic step first');
            return;
          }
          var action = kind === 'schematic-csv' ? 'export-csv' : 'export-png';
          if (!postToSchematicIframe(action)) {
            setSaveStatus('error', 'Schematic editor not ready yet');
          }
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
      var fname = img.filename || img.caption || '';
      return (
        '<button type="button" class="ib-picker-tile" data-image-id="' + img.id + '" ' +
        '        data-image-src="' + escapeHtml(src) + '" ' +
        '        data-image-filename="' + escapeHtml(fname) + '" ' +
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
          var filename = tile.dataset.imageFilename || '';
          if (!src) return;
          try { pickerModal.bsModal.hide(); } catch (_) {}
          placeProjectImageOnCanvas(src, filename).catch(function () {
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

  async function placeProjectImageOnCanvas(url, filename) {
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
      ibFilename: filename || '',
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
      applyHudVisibility(state.uiState.hudVisible);
      saveUiState();
    });
  }

  // ====================================================================
  // 24b. INSPECTOR HUD — selection-aware floating panel
  // ====================================================================
  //
  // The HUD lives over the canvas (`#ib-inspector-hud`). It listens to
  // Fabric selection events and reflects / writes the active object's
  // position, size, rotation, z-index, opacity, colour and a handful of
  // actions. Position is persisted to localStorage under uiState.hudPos.
  //
  // Focus-aware input refresh: while the user is typing in an HUD input,
  // canvas events (e.g. an `object:moving` triggered by a drag) must not
  // overwrite the input's value. We check `document.activeElement` before
  // each write — if it matches the input we're about to update, we skip
  // it. The `data-hud-field` attribute on each input identifies which
  // field a given DOM node represents.

  // Set true while we're programmatically mutating the active Fabric
  // object from an HUD input — so the resulting `object:modified` event
  // doesn't loop back and stomp the input we're editing.
  var hudWriting = false;
  // Set true while we're programmatically mutating an HUD input from the
  // canvas — so the input event the form would otherwise fire doesn't
  // bounce back into Fabric.
  var hudRendering = false;

  function applyHudVisibility(visible) {
    if (dom.hudToggle) {
      dom.hudToggle.setAttribute('aria-pressed', visible ? 'true' : 'false');
      dom.hudToggle.classList.toggle('is-active', !!visible);
    }
    document.body.classList.toggle('hud-visible', !!visible);
    if (dom.inspectorHud) {
      if (visible) {
        dom.inspectorHud.removeAttribute('hidden');
        // Re-position to the last-saved coordinates (or default) and
        // re-render the body so the user sees content immediately.
        applyHudPosition();
        renderHudBody();
      } else {
        dom.inspectorHud.setAttribute('hidden', '');
      }
    }
  }

  // Clamp + apply the stored HUD position. If no position is stored
  // (first load or after a "close" + reopen), fall back to the default
  // top-right.
  function applyHudPosition() {
    if (!dom.inspectorHud || !dom.canvasArea) return;
    var stored = state.uiState && state.uiState.hudPos;
    if (!stored || stored.x === null || stored.x === undefined ||
        stored.y === null || stored.y === undefined) {
      // Default: 18px from top, 18px from right of the canvas area.
      dom.inspectorHud.style.top = '18px';
      dom.inspectorHud.style.right = '18px';
      dom.inspectorHud.style.left = '';
      return;
    }
    var area = dom.canvasArea.getBoundingClientRect();
    var hud = dom.inspectorHud.getBoundingClientRect();
    var w = hud.width || 240;
    var h = hud.height || 100;
    var x = Math.max(0, Math.min(area.width - w, stored.x));
    var y = Math.max(0, Math.min(area.height - h, stored.y));
    dom.inspectorHud.style.left = x + 'px';
    dom.inspectorHud.style.top = y + 'px';
    dom.inspectorHud.style.right = '';
  }

  // Classify a Fabric object into an HUD display name. Backgrounds and
  // multi-selections get filtered out before this is called.
  function hudObjectName(o) {
    if (!o) return 'Object';
    if (o.ibRole === 'background') return 'Background';
    var t = o.type || '';
    if (t === 'image' || (o.isType && o.isType('image'))) return 'Image';
    if (t === 'i-text' || t === 'text' || t === 'textbox') return 'Text';
    if (t === 'rect') return 'Rectangle';
    if (t === 'ellipse' || t === 'circle') return 'Circle';
    if (t === 'group') return 'Arrow';
    return t.charAt(0).toUpperCase() + t.slice(1);
  }

  // What Fabric "colour" property does this object expose? Images don't
  // expose a meaningful colour for the HUD — return null to hide the row.
  function hudColorProperty(o) {
    if (!o) return null;
    if (o.type === 'image' || (o.isType && o.isType('image'))) return null;
    if (o.isType && o.isType('i-text')) return 'fill';
    if (o.type === 'i-text' || o.type === 'text' || o.type === 'textbox') return 'fill';
    return 'stroke';
  }

  function hudReadColor(o) {
    var prop = hudColorProperty(o);
    if (!prop) return null;
    return o.get ? o.get(prop) : o[prop];
  }

  function hudApplyColor(o, color) {
    var prop = hudColorProperty(o);
    if (!prop) return;
    var changes = {};
    changes[prop] = color;
    if (o.set) o.set(changes); else o[prop] = color;
    // Arrow groups need the colour propagated to their children (line +
    // triangle head), otherwise the visible colour doesn't change.
    if (o.type === 'group' && typeof o.forEachObject === 'function') {
      o.forEachObject(function (child) {
        if (child.type === 'triangle') child.set('fill', color);
        else if (child.set) child.set('stroke', color);
      });
    }
  }

  // Read the displayed (post-scale) width/height of a Fabric object in
  // its local coordinate system. Used for the W/H inputs.
  function hudDimensions(o) {
    if (!o) return { w: 0, h: 0 };
    var w = (o.width || 0) * (o.scaleX || 1);
    var h = (o.height || 0) * (o.scaleY || 1);
    return { w: Math.round(w), h: Math.round(h) };
  }

  // Set the displayed (post-scale) width/height by re-computing scaleX
  // / scaleY against the object's intrinsic width/height. Falls back to
  // 1px if the intrinsic dim is zero.
  function hudSetWidth(o, newW) {
    if (!o || !o.width || newW <= 0) return;
    o.set('scaleX', newW / o.width);
  }
  function hudSetHeight(o, newH) {
    if (!o || !o.height || newH <= 0) return;
    o.set('scaleY', newH / o.height);
  }

  // Build the HUD body for the given active object. Returns true if the
  // HUD has selection content; false if it's showing the empty state.
  function renderHudBody() {
    if (!dom.hudBody) return false;
    var active = state.canvas ? state.canvas.getActiveObject() : null;
    var isMulti = active && active.type === 'activeSelection';
    // Treat background and multi-select as "no useful single object".
    if (!active || isMulti || active.ibRole === 'background') {
      dom.hudBody.innerHTML =
        '<div class="ib-hud-empty">Select an object to edit its properties.</div>';
      return false;
    }

    var name = hudObjectName(active);
    var dims = hudDimensions(active);
    var left = Math.round(active.left || 0);
    var top = Math.round(active.top || 0);
    var rot = Math.round(((active.angle || 0) % 360 + 360) % 360);
    if (rot > 180) rot -= 360;
    var zIdx = state.canvas.getObjects().indexOf(active);
    var op = Math.round((active.opacity == null ? 1 : active.opacity) * 100);
    var color = hudReadColor(active);
    var showColor = !!hudColorProperty(active);

    var swatchesHtml = '';
    if (showColor) {
      swatchesHtml = '<div class="ib-hud-colors" data-hud-field-group="color">';
      HUD_SWATCHES.forEach(function (c) {
        var isActive = color && c.toLowerCase() === String(color).toLowerCase();
        swatchesHtml +=
          '<button type="button" class="ib-hud-swatch' + (isActive ? ' is-active' : '') + '"' +
          ' style="background:' + c + '" data-hud-swatch="' + c + '"' +
          ' title="' + c + '"></button>';
      });
      swatchesHtml +=
        '<label class="ib-hud-swatch-custom" title="Custom colour">' +
        '<input type="color" data-hud-field="color"' +
        ' value="' + (color ? String(color) : '#c8312a') + '">' +
        '</label>';
      swatchesHtml += '</div>';
    }

    dom.hudBody.innerHTML =
      '<h6 class="ib-hud-name">' + escapeHtml(name) + '</h6>' +
      '<div class="ib-hud-grid">' +
      '  <div class="ib-hud-field">' +
      '    <span class="ib-hud-field-label">X</span>' +
      '    <input type="number" data-hud-field="x" value="' + left + '" step="1">' +
      '  </div>' +
      '  <div class="ib-hud-field">' +
      '    <span class="ib-hud-field-label">Y</span>' +
      '    <input type="number" data-hud-field="y" value="' + top + '" step="1">' +
      '  </div>' +
      '  <div class="ib-hud-field">' +
      '    <span class="ib-hud-field-label">W</span>' +
      '    <input type="number" data-hud-field="w" value="' + dims.w + '" step="1" min="1">' +
      '  </div>' +
      '  <div class="ib-hud-field">' +
      '    <span class="ib-hud-field-label">H</span>' +
      '    <input type="number" data-hud-field="h" value="' + dims.h + '" step="1" min="1">' +
      '  </div>' +
      '</div>' +
      '<div class="ib-hud-grid">' +
      '  <div class="ib-hud-field">' +
      '    <span class="ib-hud-field-label">Rot&deg;</span>' +
      '    <input type="number" data-hud-field="rot" value="' + rot +
      '" step="1" min="-180" max="180">' +
      '  </div>' +
      '  <div class="ib-hud-field">' +
      '    <span class="ib-hud-field-label">Z</span>' +
      '    <input type="number" data-hud-field="z" value="' + zIdx + '" step="1" min="0">' +
      '  </div>' +
      '</div>' +
      '<div class="ib-hud-slider-row">' +
      '  <label for="ib-hud-opacity">Opacity</label>' +
      '  <input type="range" id="ib-hud-opacity" data-hud-field="opacity"' +
      '         min="0" max="100" step="1" value="' + op + '">' +
      '  <span class="ib-hud-slider-value" data-hud-field-display="opacity">' + op + '%</span>' +
      '</div>' +
      swatchesHtml +
      '<div class="ib-hud-actions">' +
      '  <button type="button" class="ib-hud-action" data-hud-action="rotate-ccw"' +
      '          title="Rotate -90&deg;"><i class="fas fa-rotate-left"></i></button>' +
      '  <button type="button" class="ib-hud-action" data-hud-action="rotate-cw"' +
      '          title="Rotate +90&deg;"><i class="fas fa-rotate-right"></i></button>' +
      '  <button type="button" class="ib-hud-action" data-hud-action="flip-h"' +
      '          title="Flip horizontal"><i class="fas fa-arrows-left-right"></i></button>' +
      '  <button type="button" class="ib-hud-action" data-hud-action="flip-v"' +
      '          title="Flip vertical"><i class="fas fa-arrows-up-down"></i></button>' +
      '  <button type="button" class="ib-hud-action is-danger" data-hud-action="delete"' +
      '          title="Delete"><i class="fas fa-trash"></i></button>' +
      '</div>';

    wireHudBody(active);
    return true;
  }

  // Wire the (just-rendered) HUD body to its current active object.
  // Re-runs each time the body is re-rendered.
  function wireHudBody(active) {
    if (!dom.hudBody || !state.canvas) return;

    function fireModified() {
      try {
        state.canvas.fire('object:modified', { target: active });
      } catch (_) { /* harmless if Fabric is mid-frame */ }
    }

    // Numeric inputs (X/Y/W/H, Rot°, Z) — write on `input` so dragging
    // the spinner / typing reflects live, but the hudWriting flag stops
    // us echoing back to ourselves.
    var numericFields = dom.hudBody.querySelectorAll('input[data-hud-field]');
    Array.prototype.forEach.call(numericFields, function (inp) {
      var field = inp.dataset.hudField;
      var handler = function () {
        if (hudRendering) return;
        var raw = parseFloat(inp.value);
        if (isNaN(raw)) return;
        hudWriting = true;
        try {
          if (field === 'x') active.set('left', raw);
          else if (field === 'y') active.set('top', raw);
          else if (field === 'w') hudSetWidth(active, Math.max(1, raw));
          else if (field === 'h') hudSetHeight(active, Math.max(1, raw));
          else if (field === 'rot') {
            var deg = ((raw % 360) + 360) % 360;
            active.set('angle', deg);
          } else if (field === 'z') {
            var objs = state.canvas.getObjects();
            var target = Math.max(0, Math.min(objs.length - 1, Math.round(raw)));
            if (state.canvas.moveObjectTo) state.canvas.moveObjectTo(active, target);
            else moveObjectToFallback(active, target);
            reanchorBackground();
          } else if (field === 'opacity') {
            active.set('opacity', raw / 100);
            var disp = dom.hudBody.querySelector('[data-hud-field-display="opacity"]');
            if (disp) disp.textContent = Math.round(raw) + '%';
          } else if (field === 'color') {
            // Native colour picker — applies live as the user picks.
            hudApplyColor(active, inp.value);
            highlightHudSwatch(inp.value);
          }
          active.setCoords();
          state.canvas.requestRenderAll();
        } finally {
          hudWriting = false;
        }
      };

      // Opacity + color fire constantly; debounce the modified event so
      // we don't generate undo snapshots per pixel of slider movement.
      if (field === 'opacity' || field === 'color') {
        inp.addEventListener('input', handler);
        inp.addEventListener('change', function () { fireModified(); });
      } else {
        inp.addEventListener('input', handler);
        inp.addEventListener('change', fireModified);
      }
    });

    // Brand swatches — single-click apply.
    Array.prototype.forEach.call(
      dom.hudBody.querySelectorAll('[data-hud-swatch]'),
      function (btn) {
        btn.addEventListener('click', function () {
          var c = btn.dataset.hudSwatch;
          hudWriting = true;
          try {
            hudApplyColor(active, c);
            state.canvas.requestRenderAll();
            highlightHudSwatch(c);
            var picker = dom.hudBody.querySelector('input[data-hud-field="color"]');
            if (picker) picker.value = (c[0] === '#' && c.length === 7) ? c : picker.value;
          } finally {
            hudWriting = false;
          }
          fireModified();
        });
      }
    );

    // Action icons — rotate / flip / delete.
    Array.prototype.forEach.call(
      dom.hudBody.querySelectorAll('[data-hud-action]'),
      function (btn) {
        btn.addEventListener('click', function () {
          var act = btn.dataset.hudAction;
          if (act === 'delete') {
            state.canvas.remove(active);
            state.canvas.discardActiveObject();
            state.canvas.requestRenderAll();
            renderHudBody();
            return;
          }
          hudWriting = true;
          try {
            if (act === 'rotate-ccw') {
              active.set('angle', (((active.angle || 0) - 90) % 360 + 360) % 360);
            } else if (act === 'rotate-cw') {
              active.set('angle', (((active.angle || 0) + 90) % 360 + 360) % 360);
            } else if (act === 'flip-h') {
              active.set('flipX', !active.flipX);
            } else if (act === 'flip-v') {
              active.set('flipY', !active.flipY);
            }
            active.setCoords();
            state.canvas.requestRenderAll();
          } finally {
            hudWriting = false;
          }
          fireModified();
          renderHudBody();
        });
      }
    );
  }

  function highlightHudSwatch(color) {
    if (!dom.hudBody) return;
    var lower = color ? String(color).toLowerCase() : '';
    Array.prototype.forEach.call(
      dom.hudBody.querySelectorAll('[data-hud-swatch]'),
      function (sw) {
        sw.classList.toggle('is-active',
          sw.dataset.hudSwatch.toLowerCase() === lower);
      }
    );
  }

  // Refresh the HUD numeric inputs in response to canvas-driven changes
  // (drag / scale / rotate). Skips inputs that the user is currently
  // focused on — so typing in W doesn't get clobbered while another
  // dimension updates.
  function refreshHudInputsFromCanvas() {
    if (!dom.hudBody || hudWriting) return;
    var active = state.canvas ? state.canvas.getActiveObject() : null;
    if (!active || active.type === 'activeSelection' || active.ibRole === 'background') {
      return;
    }
    var focused = document.activeElement;
    function setField(name, value) {
      var inp = dom.hudBody.querySelector('input[data-hud-field="' + name + '"]');
      if (!inp || inp === focused) return;
      hudRendering = true;
      try { inp.value = String(value); } finally { hudRendering = false; }
    }
    var dims = hudDimensions(active);
    setField('x', Math.round(active.left || 0));
    setField('y', Math.round(active.top || 0));
    setField('w', dims.w);
    setField('h', dims.h);
    var rot = Math.round(((active.angle || 0) % 360 + 360) % 360);
    if (rot > 180) rot -= 360;
    setField('rot', rot);
    setField('z', state.canvas.getObjects().indexOf(active));
    var op = Math.round((active.opacity == null ? 1 : active.opacity) * 100);
    setField('opacity', op);
    var disp = dom.hudBody.querySelector('[data-hud-field-display="opacity"]');
    if (disp) disp.textContent = op + '%';
  }

  function wireHudDragAndButtons() {
    if (!dom.inspectorHud) return;

    // Close: hides the HUD and persists the state (mirroring the title-
    // bar HUD button's "off" state).
    if (dom.hudClose) {
      dom.hudClose.addEventListener('click', function (e) {
        e.stopPropagation();
        if (!state.uiState) return;
        state.uiState.hudVisible = false;
        applyHudVisibility(false);
        saveUiState();
      });
    }

    // Minimize: collapse to header-only.
    if (dom.hudMinimize) {
      dom.hudMinimize.addEventListener('click', function (e) {
        e.stopPropagation();
        dom.inspectorHud.classList.toggle('is-minimized');
      });
    }

    // Header drag.
    if (!dom.hudHeader) return;
    var drag = null;
    dom.hudHeader.addEventListener('mousedown', function (e) {
      // Ignore drags that start on the icon buttons.
      if (e.target.closest('.ib-hud-icon-btn')) return;
      var hudRect = dom.inspectorHud.getBoundingClientRect();
      var areaRect = dom.canvasArea.getBoundingClientRect();
      drag = {
        startX: e.clientX,
        startY: e.clientY,
        // Offset of mouse within HUD, in canvas-area coordinates.
        offsetX: e.clientX - hudRect.left,
        offsetY: e.clientY - hudRect.top,
        areaRect: areaRect,
        hudW: hudRect.width,
        hudH: hudRect.height,
      };
      dom.hudHeader.classList.add('is-dragging');
      document.body.style.userSelect = 'none';
      e.preventDefault();
    });

    document.addEventListener('mousemove', function (e) {
      if (!drag) return;
      var x = e.clientX - drag.areaRect.left - drag.offsetX;
      var y = e.clientY - drag.areaRect.top - drag.offsetY;
      // Clamp so the HUD stays fully inside the canvas area.
      x = Math.max(0, Math.min(drag.areaRect.width - drag.hudW, x));
      y = Math.max(0, Math.min(drag.areaRect.height - drag.hudH, y));
      dom.inspectorHud.style.left = x + 'px';
      dom.inspectorHud.style.top = y + 'px';
      dom.inspectorHud.style.right = '';
      // Debounce the persist write.
      debounce('hud-pos-save', 200, function () {
        if (!state.uiState) return;
        state.uiState.hudPos = { x: x, y: y };
        saveUiState();
      });
    });

    document.addEventListener('mouseup', function () {
      if (!drag) return;
      drag = null;
      dom.hudHeader.classList.remove('is-dragging');
      document.body.style.userSelect = '';
    });
  }

  // ====================================================================
  // 24c. IMAGE-CONTEXT TOOLBAR — pill above selected images
  // ====================================================================
  //
  // Visible only when the active selection is a single non-background
  // image. Anchored to the image's top-centre and re-positioned on
  // every `object:moving|scaling|rotating` so it tracks the image.
  // Buttons: crop-rect, crop-circle, cutout (AI bg-remove), outline
  // toggle, greyscale toggle, rotate-90, delete.

  // imgly lazy-load — null until first cutout click. Module promise so
  // concurrent clicks don't double-load.
  var imglyModulePromise = null;

  function loadImgly() {
    if (imglyModulePromise) return imglyModulePromise;
    imglyModulePromise = import(IMGLY_BG_REMOVAL_URL).catch(function (err) {
      imglyModulePromise = null;
      throw err;
    });
    return imglyModulePromise;
  }

  function activeImageOrNull() {
    if (!state.canvas) return null;
    var active = state.canvas.getActiveObject();
    if (!active) return null;
    if (active.type === 'activeSelection') return null;
    if (active.ibRole === 'background') return null;
    var isImg = (active.type === 'image') ||
      (active.isType && active.isType('image'));
    return isImg ? active : null;
  }

  function buildImageToolbar() {
    if (!dom.imageToolbar || dom.imageToolbar.dataset.built === '1') return;
    dom.imageToolbar.innerHTML =
      '<button type="button" class="ib-image-toolbar-btn" data-img-action="crop-rect"' +
      '        title="Rectangular crop"><i class="fas fa-crop"></i></button>' +
      '<button type="button" class="ib-image-toolbar-btn" data-img-action="crop-circle"' +
      '        title="Circular crop"><i class="fas fa-circle"></i></button>' +
      '<span class="ib-image-toolbar-sep"></span>' +
      '<button type="button" class="ib-image-toolbar-btn" data-img-action="cutout"' +
      '        title="Remove background"><i class="fas fa-wand-magic-sparkles"></i></button>' +
      '<button type="button" class="ib-image-toolbar-btn" data-img-action="outline"' +
      '        title="Toggle outline"><i class="fas fa-border-style"></i></button>' +
      '<button type="button" class="ib-image-toolbar-btn" data-img-action="greyscale"' +
      '        title="Toggle greyscale"><i class="fas fa-circle-half-stroke"></i></button>' +
      '<span class="ib-image-toolbar-sep"></span>' +
      '<button type="button" class="ib-image-toolbar-btn" data-img-action="rotate"' +
      '        title="Rotate 90&deg;"><i class="fas fa-rotate-right"></i></button>' +
      '<button type="button" class="ib-image-toolbar-btn" data-img-action="aspect-reset"' +
      '        title="Reset aspect ratio"><i class="fas fa-expand"></i></button>' +
      '<button type="button" class="ib-image-toolbar-btn is-danger" data-img-action="delete"' +
      '        title="Delete image"><i class="fas fa-trash"></i></button>' +
      // Crop-mode-only confirm/cancel — hidden by default; revealed by
      // the .is-crop-mode class on the toolbar.
      '<button type="button" class="ib-image-toolbar-btn is-crop-only is-confirm"' +
      '        data-img-action="crop-apply" title="Apply crop (Enter)">' +
      '<i class="fas fa-check"></i></button>' +
      '<button type="button" class="ib-image-toolbar-btn is-crop-only is-danger"' +
      '        data-img-action="crop-cancel" title="Cancel crop (Esc)">' +
      '<i class="fas fa-xmark"></i></button>' +
      '<span class="ib-image-toolbar-progress" data-img-progress hidden></span>';
    dom.imageToolbar.dataset.built = '1';

    Array.prototype.forEach.call(
      dom.imageToolbar.querySelectorAll('[data-img-action]'),
      function (btn) {
        btn.addEventListener('click', function () {
          var img = activeImageOrNull();
          if (!img) return;
          runImageToolbarAction(btn.dataset.imgAction, img, btn);
        });
      }
    );
  }

  function setImageToolbarProgress(msg, kind) {
    var el = dom.imageToolbar && dom.imageToolbar.querySelector('[data-img-progress]');
    if (!el) return;
    if (!msg) {
      el.hidden = true;
      el.textContent = '';
      el.classList.remove('is-error');
      return;
    }
    el.hidden = false;
    el.textContent = msg;
    el.classList.toggle('is-error', kind === 'error');
  }

  function reflectImageToolbarState(img) {
    if (!dom.imageToolbar || !img) return;
    // Reflect: outline (has stroke), greyscale (has Grayscale filter).
    var hasOutline = !!(img.stroke && img.strokeWidth);
    var hasGrey = Array.isArray(img.filters) && img.filters.some(function (f) {
      return f && (f.type === 'Grayscale' ||
        (fabric.filters && f instanceof fabric.filters.Grayscale));
    });
    var hasCrop = !!img.clipPath;
    var outlineBtn = dom.imageToolbar.querySelector('[data-img-action="outline"]');
    var greyBtn = dom.imageToolbar.querySelector('[data-img-action="greyscale"]');
    var rectBtn = dom.imageToolbar.querySelector('[data-img-action="crop-rect"]');
    var circleBtn = dom.imageToolbar.querySelector('[data-img-action="crop-circle"]');
    var aspectBtn = dom.imageToolbar.querySelector('[data-img-action="aspect-reset"]');
    if (outlineBtn) outlineBtn.classList.toggle('is-active', hasOutline);
    if (greyBtn) greyBtn.classList.toggle('is-active', hasGrey);
    if (rectBtn) rectBtn.classList.toggle('is-active',
      hasCrop && img.clipPath && img.clipPath.type === 'rect');
    if (circleBtn) circleBtn.classList.toggle('is-active',
      hasCrop && img.clipPath && img.clipPath.type === 'circle');
    // Aspect-reset needs the underlying HTMLImageElement's natural
    // dimensions; if the image hasn't finished loading or is cross-origin
    // tainted we can't read them, so disable the button.
    if (aspectBtn) {
      var dims = imageNaturalDims(img);
      aspectBtn.disabled = !dims;
      aspectBtn.classList.toggle('is-disabled', !dims);
    }
  }

  // ---- Interactive crop ------------------------------------------------
  //
  // Enter crop mode: the active image becomes non-selectable, a Fabric
  // Rect overlay with black corner handles appears over it, and the
  // toolbar swaps to Apply / Cancel. Dragging the rect handles defines
  // the crop region; Apply sets the image's clipPath to that region;
  // Cancel discards. Esc / Enter shortcuts mirror the buttons.
  //
  // The rect's bounds are in canvas-pixel space; clipPath lives in the
  // image's local coordinate space (relative to its centre, in the
  // image's natural-pixel units), so we divide by scaleX / scaleY when
  // committing.

  function enterCropMode(imgObj) {
    if (!state.canvas || !imgObj) return;
    if (state.cropping) return;
    var bound = imgObj.getBoundingRect(true, true);  // canvas coords
    var overlay = new fabric.Rect({
      left: bound.left,
      top: bound.top,
      width: bound.width,
      height: bound.height,
      originX: 'left',
      originY: 'top',
      fill: 'rgba(255,255,255,0)',
      stroke: '#000',
      strokeWidth: 1,
      strokeUniform: true,
      strokeDashArray: [5, 4],
      cornerColor: '#000',
      cornerStrokeColor: '#000',
      transparentCorners: false,
      cornerSize: 10,
      borderColor: '#000',
      borderDashArray: [5, 4],
      hasRotatingPoint: false,
      lockRotation: true,
      lockScalingFlip: true,
      // Tag so it survives Fabric mutations / wouldn't be persisted in
      // canvas_json on the off-chance the toJSON allowlist catches it.
      ibRole: 'crop-overlay',
    });
    state.cropping = { image: imgObj, overlay: overlay };
    imgObj.selectable = false;
    imgObj.evented = false;
    state.canvas.add(overlay);
    state.canvas.setActiveObject(overlay);
    if (dom.imageToolbar) {
      dom.imageToolbar.classList.add('is-crop-mode');
      positionImageToolbar(overlay);
    }
    state.canvas.requestRenderAll();
  }

  function exitCropMode(apply) {
    if (!state.cropping) return;
    var imgObj = state.cropping.image;
    var overlay = state.cropping.overlay;

    if (apply) {
      var oBound = overlay.getBoundingRect(true, true);  // canvas coords
      var sx = imgObj.scaleX || 1;
      var sy = imgObj.scaleY || 1;
      // imgObj has originX/Y = 'center', so .left / .top is the image's
      // visual centre on the canvas.
      var oCx = oBound.left + oBound.width / 2;
      var oCy = oBound.top + oBound.height / 2;
      var localCx = (oCx - imgObj.left) / sx;
      var localCy = (oCy - imgObj.top) / sy;
      var localW = oBound.width / sx;
      var localH = oBound.height / sy;
      imgObj.set('clipPath', new fabric.Rect({
        left: localCx,
        top: localCy,
        width: localW,
        height: localH,
        originX: 'center',
        originY: 'center',
      }));
      imgObj.dirty = true;
    }

    state.canvas.remove(overlay);
    imgObj.selectable = true;
    imgObj.evented = true;
    state.canvas.setActiveObject(imgObj);
    state.cropping = null;
    if (dom.imageToolbar) {
      dom.imageToolbar.classList.remove('is-crop-mode');
    }
    state.canvas.requestRenderAll();
    if (apply) {
      state.canvas.fire('object:modified', { target: imgObj });
    }
    reflectImageToolbarState(imgObj);
    positionImageToolbar(imgObj);
  }

  // Returns { w, h } from the underlying HTMLImageElement, or null if
  // the natural dimensions can't be read (still loading, cross-origin
  // without anonymous CORS, etc.).
  function imageNaturalDims(img) {
    if (!img) return null;
    var el = img._originalElement || img._element;
    if (!el) return null;
    var nw = el.naturalWidth || 0;
    var nh = el.naturalHeight || 0;
    if (!nw || !nh) return null;
    return { w: nw, h: nh };
  }

  function positionImageToolbar(img) {
    if (!dom.imageToolbar || !dom.canvasArea || !state.canvas || !img) return;
    // The image's bounding rect in SCENE coords. We then run that
    // through the canvas viewport transform to get canvas-element
    // pixel coords, and offset by the canvas element's position
    // within the canvas-area (where the toolbar is positioned absolute).
    var br;
    try { br = img.getBoundingRect(); }
    catch (_) { return; }
    if (!br) return;

    var vt = state.canvas.viewportTransform || [1, 0, 0, 1, 0, 0];
    // Scene → canvas-element CSS pixels.
    var topCx = (br.left + br.width / 2) * vt[0] + vt[4];
    var topCy = br.top * vt[3] + vt[5];

    var canvasEl = state.canvas.lowerCanvasEl;
    var canvasRect = canvasEl.getBoundingClientRect();
    var areaRect = dom.canvasArea.getBoundingClientRect();
    var x = (canvasRect.left - areaRect.left) + topCx;
    var y = (canvasRect.top - areaRect.top) + topCy - 10; // 10 px gap

    var tbRect = dom.imageToolbar.getBoundingClientRect();
    var tbH = tbRect.height || 36;
    if (y < tbH) y = tbH;

    dom.imageToolbar.style.left = x + 'px';
    dom.imageToolbar.style.top = y + 'px';
  }

  function refreshImageToolbar() {
    if (!dom.imageToolbar) return;
    var img = activeImageOrNull();
    if (!img) {
      dom.imageToolbar.hidden = true;
      setImageToolbarProgress(null);
      return;
    }
    buildImageToolbar();
    dom.imageToolbar.hidden = false;
    reflectImageToolbarState(img);
    positionImageToolbar(img);
  }

  function runImageToolbarAction(action, img, btn) {
    if (!state.canvas) return;
    if (action === 'delete') {
      state.canvas.remove(img);
      state.canvas.discardActiveObject();
      state.canvas.requestRenderAll();
      refreshImageToolbar();
      renderHudBody();
      return;
    }
    if (action === 'rotate') {
      img.set('angle', (((img.angle || 0) + 90) % 360 + 360) % 360);
      img.setCoords();
      state.canvas.requestRenderAll();
      state.canvas.fire('object:modified', { target: img });
      positionImageToolbar(img);
      return;
    }
    if (action === 'aspect-reset') {
      // Reset aspect ratio leaves the image's *width* on screen unchanged
      // and adjusts the height to restore the natural w/h ratio, undoing
      // any uneven stretch the user applied via the HUD W/H or by dragging
      // a side handle. Hidden / disabled for the locked background.
      if (img.ibRole === 'background') return;
      var nat = imageNaturalDims(img);
      if (!nat) {
        setImageToolbarProgress("Image dimensions not available — try again.", 'error');
        setTimeout(function () { setImageToolbarProgress(null); }, 3000);
        return;
      }
      // For Fabric Images whose width/height equal the natural
      // dimensions, this collapses to scaleY = scaleX. The general form
      // here handles the rare case where Fabric's `width`/`height` were
      // overridden (e.g. by a crop).
      var sx = img.scaleX || 1;
      var newScaleY = sx * (nat.w / (img.width || nat.w)) *
        ((img.height || nat.h) / nat.h);
      img.set('scaleY', newScaleY);
      img.setCoords();
      state.canvas.requestRenderAll();
      state.canvas.fire('object:modified', { target: img });
      positionImageToolbar(img);
      return;
    }
    if (action === 'crop-rect') {
      // If a rect clip is already applied, clicking removes it.
      // Otherwise enter interactive crop mode: a draggable overlay
      // appears over the image, the image becomes non-selectable, and
      // the toolbar swaps to Apply / Cancel.
      if (state.cropping) return;
      if (img.clipPath && img.clipPath.type === 'rect') {
        img.set('clipPath', null);
        state.canvas.requestRenderAll();
        state.canvas.fire('object:modified', { target: img });
        reflectImageToolbarState(img);
        return;
      }
      enterCropMode(img);
      return;
    }
    if (action === 'crop-apply') {
      exitCropMode(true);
      return;
    }
    if (action === 'crop-cancel') {
      exitCropMode(false);
      return;
    }
    if (action === 'crop-circle') {
      if (img.clipPath && img.clipPath.type === 'circle') {
        img.set('clipPath', null);
      } else {
        var r = Math.min(img.width, img.height) / 2;
        img.set('clipPath', new fabric.Circle({
          radius: r,
          originX: 'center',
          originY: 'center',
        }));
      }
      state.canvas.requestRenderAll();
      state.canvas.fire('object:modified', { target: img });
      reflectImageToolbarState(img);
      return;
    }
    if (action === 'outline') {
      if (img.stroke && img.strokeWidth) {
        img.set({ stroke: null, strokeWidth: 0 });
      } else {
        img.set({ stroke: '#000000', strokeWidth: 4, strokeUniform: true });
      }
      state.canvas.requestRenderAll();
      state.canvas.fire('object:modified', { target: img });
      reflectImageToolbarState(img);
      return;
    }
    if (action === 'greyscale') {
      var hasGrey = Array.isArray(img.filters) && img.filters.some(function (f) {
        return f && (f.type === 'Grayscale' ||
          (fabric.filters && f instanceof fabric.filters.Grayscale));
      });
      if (hasGrey) {
        img.filters = (img.filters || []).filter(function (f) {
          return !(f && (f.type === 'Grayscale' ||
            (fabric.filters && f instanceof fabric.filters.Grayscale)));
        });
      } else {
        img.filters = (img.filters || []).concat([new fabric.filters.Grayscale()]);
      }
      try {
        var p = img.applyFilters();
        // Fabric doesn't auto-fire object:modified after applyFilters —
        // emit so undo + autosave snapshot the new (greyed / un-greyed)
        // state. v6 returns a promise; v5 was synchronous; we tolerate both.
        if (p && typeof p.then === 'function') {
          p.then(function () {
            state.canvas.requestRenderAll();
            state.canvas.fire('object:modified', { target: img });
          });
        } else {
          state.canvas.requestRenderAll();
          state.canvas.fire('object:modified', { target: img });
        }
      } catch (_) {
        state.canvas.requestRenderAll();
        state.canvas.fire('object:modified', { target: img });
      }
      reflectImageToolbarState(img);
      return;
    }
    if (action === 'cutout') {
      runCutout(img, btn);
      return;
    }
  }

  async function runCutout(img, btn) {
    if (!img || !btn) return;
    if (btn.classList.contains('is-loading')) return;
    btn.classList.add('is-loading');
    setImageToolbarProgress('Loading background-removal library…');

    var imgly;
    try {
      imgly = await loadImgly();
    } catch (err) {
      console.warn('imgly load failed', err);
      btn.classList.remove('is-loading');
      setImageToolbarProgress(
        "Couldn't load the background-removal library. Check your connection.",
        'error');
      setTimeout(function () { setImageToolbarProgress(null); }, 4500);
      return;
    }

    setImageToolbarProgress('Removing background…');

    // Get the image's pixel data as a Blob. Use _originalElement (the
    // underlying HTMLImageElement) so we keep the original resolution
    // for the model — the on-canvas display scale is irrelevant.
    var sourceEl = img._originalElement || img._element;
    if (!sourceEl) {
      btn.classList.remove('is-loading');
      setImageToolbarProgress("Couldn't read image pixels.", 'error');
      setTimeout(function () { setImageToolbarProgress(null); }, 4500);
      return;
    }

    var w = sourceEl.naturalWidth || sourceEl.width || img.width;
    var h = sourceEl.naturalHeight || sourceEl.height || img.height;
    var off = document.createElement('canvas');
    off.width = w;
    off.height = h;
    var ctx = off.getContext('2d');
    try {
      ctx.drawImage(sourceEl, 0, 0, w, h);
    } catch (_) {
      btn.classList.remove('is-loading');
      setImageToolbarProgress(
        "Couldn't read image pixels (cross-origin?).", 'error');
      setTimeout(function () { setImageToolbarProgress(null); }, 4500);
      return;
    }

    var inputBlob = await new Promise(function (resolve) {
      off.toBlob(function (b) { resolve(b); }, 'image/png');
    });
    if (!inputBlob) {
      btn.classList.remove('is-loading');
      setImageToolbarProgress("Couldn't encode image for processing.", 'error');
      setTimeout(function () { setImageToolbarProgress(null); }, 4500);
      return;
    }

    var resultBlob;
    try {
      var remove = imgly && (imgly.removeBackground ||
        (imgly.default && imgly.default.removeBackground));
      if (!remove) throw new Error('removeBackground not found');
      resultBlob = await remove(inputBlob);
    } catch (err) {
      console.warn('cutout failed', err);
      btn.classList.remove('is-loading');
      setImageToolbarProgress(
        "Background removal failed. Try a smaller image or check WebGL/WASM support.",
        'error');
      setTimeout(function () { setImageToolbarProgress(null); }, 4500);
      return;
    }

    var resultUrl = URL.createObjectURL(resultBlob);
    try {
      if (img.setSrc && img.setSrc.length >= 1) {
        // Fabric v6: setSrc returns a Promise.
        await img.setSrc(resultUrl, { crossOrigin: 'anonymous' });
      }
      img.dirty = true;
      state.canvas.requestRenderAll();
      // Fabric doesn't auto-fire object:modified after async setSrc — emit
      // so undo + autosave snapshot the new state.
      state.canvas.fire('object:modified', { target: img });
      setImageToolbarProgress('Background removed', null);
      setTimeout(function () { setImageToolbarProgress(null); }, 1500);
    } catch (err) {
      console.warn('setSrc failed', err);
      setImageToolbarProgress("Couldn't apply cutout result.", 'error');
      setTimeout(function () { setImageToolbarProgress(null); }, 4500);
    } finally {
      btn.classList.remove('is-loading');
    }
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
    // The Undo / Redo buttons mirror their Cmd/Ctrl+Z[+Shift] shortcuts:
    // canvas stack first, then step-level. Routing both through the same
    // entrypoints keeps button and shortcut behaviour in lockstep.
    if (dom.undoBtn) dom.undoBtn.addEventListener('click', onUndoShortcut);
    if (dom.redoBtn) dom.redoBtn.addEventListener('click', onRedoShortcut);

    if (dom.rotation) dom.rotation.addEventListener('change', onRotationInputChange);
    if (dom.resetTransforms) dom.resetTransforms.addEventListener('click', onResetTransformsClick);

    if (dom.bringToFront) dom.bringToFront.addEventListener('click', function () { arrangeActive('front'); });
    if (dom.bringForward) dom.bringForward.addEventListener('click', function () { arrangeActive('forward'); });
    if (dom.sendBackward) dom.sendBackward.addEventListener('click', function () { arrangeActive('backward'); });
    if (dom.sendToBack) dom.sendToBack.addEventListener('click', function () { arrangeActive('back'); });

    // Keyboard shortcuts: Cmd/Ctrl+Z = undo (canvas first, then
    // step-level), Cmd/Ctrl+Shift+Z = redo (canvas first, then
    // step-level — symmetric with undo), Delete/Backspace = delete
    // selected canvas object.
    document.addEventListener('keydown', function (e) {
      var tag = (e.target && e.target.tagName) || '';
      var typing = (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT');
      var editingText = state.canvas && state.canvas.getActiveObject() &&
        state.canvas.getActiveObject().isType && state.canvas.getActiveObject().isType('i-text') &&
        state.canvas.getActiveObject().isEditing;
      // Crop mode commits with Enter and bails with Escape. Highest
      // priority so they don't trigger an Undo or step-rename instead.
      if (state.cropping && !typing && !editingText) {
        if (e.key === 'Enter') { e.preventDefault(); exitCropMode(true); return; }
        if (e.key === 'Escape') { e.preventDefault(); exitCropMode(false); return; }
      }
      // Canvas zoom — Cmd/Ctrl + / - / 0 (zoom to fit). Match the
      // schematic editor's shortcut set so muscle memory transfers.
      if ((e.metaKey || e.ctrlKey) && !typing && !editingText) {
        if (e.key === '+' || e.key === '=') {
          e.preventDefault(); zoomCanvasBy(1.25, null); return;
        }
        if (e.key === '-' || e.key === '_') {
          e.preventDefault(); zoomCanvasBy(1 / 1.25, null); return;
        }
        if (e.key === '0') {
          e.preventDefault(); resetCanvasZoom(); return;
        }
      }
      if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'z') {
        if (typing || editingText) return;
        e.preventDefault();
        if (e.shiftKey) onRedoShortcut();
        else onUndoShortcut();
      } else if ((e.key === 'Delete' || e.key === 'Backspace') && !typing && !editingText) {
        if (state.canvas && state.canvas.getActiveObject()) {
          e.preventDefault();
          onDeleteSelected();
        }
      }
    });

    // Zoom buttons branch based on the active step type. On a
    // schematic-type step the embedded iframe owns the canvas — forward
    // the zoom action via postMessage. Otherwise drive the builder's
    // own Fabric canvas.
    if (dom.zoomIn)    dom.zoomIn.addEventListener('click', function () { zoomActiveCanvasBy(1.25); });
    if (dom.zoomOut)   dom.zoomOut.addEventListener('click', function () { zoomActiveCanvasBy(1 / 1.25); });
    if (dom.zoomReset) dom.zoomReset.addEventListener('click', resetActiveCanvasZoom);
    if (dom.canvasFrame) {
      dom.canvasFrame.addEventListener('wheel', function (e) {
        if (!(e.ctrlKey || e.metaKey)) return;
        if (!state.canvas) return;
        e.preventDefault();
        var rect = state.canvas.lowerCanvasEl.getBoundingClientRect();
        var focal = { x: e.clientX - rect.left, y: e.clientY - rect.top };
        var factor = e.deltaY < 0 ? 1.1 : 1 / 1.1;
        zoomCanvasBy(factor, focal);
      }, { passive: false });
    }

    // Listen for the embedded schematic editor announcing its zoom
    // level so the title-bar percent stays accurate when the user
    // zooms via the iframe (Ctrl+wheel inside the schematic canvas).
    window.addEventListener('message', function (e) {
      var msg = e && e.data;
      if (!msg || msg.kr_se_event !== 'zoom') return;
      if (currentActiveStepType() !== 'schematic') return;
      if (dom.zoomPct && typeof msg.percent === 'number') {
        dom.zoomPct.textContent = msg.percent + '%';
      }
    });
  }

  // Helpers that branch between "builder's own Fabric canvas" and
  // "embedded schematic editor iframe" based on the active step's
  // type. Keeps the title-bar zoom buttons working uniformly.
  function currentActiveStepType() {
    if (!state.activeStepId) return 'photo';
    var s = state.steps && state.steps.find(function (x) { return x.id === state.activeStepId; });
    return (s && s.step_type) || 'photo';
  }

  function postToSchematicIframe(action) {
    var iframe = document.getElementById('ib-canvas-schematic-iframe');
    if (!iframe || !iframe.contentWindow) return false;
    try {
      iframe.contentWindow.postMessage({ kr_se_action: action }, '*');
      return true;
    } catch (_) { return false; }
  }

  function zoomActiveCanvasBy(factor) {
    if (currentActiveStepType() === 'schematic') {
      postToSchematicIframe(factor >= 1 ? 'zoom-in' : 'zoom-out');
    } else {
      zoomCanvasBy(factor, null);
    }
  }

  function resetActiveCanvasZoom() {
    if (currentActiveStepType() === 'schematic') {
      postToSchematicIframe('zoom-reset');
    } else {
      resetCanvasZoom();
    }
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
    // Fire object:modified so the change goes through the canonical
    // change-event pipe (snapshot + autosave). Without this the audit
    // would have two divergent paths for "object mutated", which makes
    // future maintenance easier to get wrong.
    state.canvas.fire('object:modified', { target: active });
  }

  // ====================================================================
  // 26. INIT FLOW
  // ====================================================================

  /**
   * Size the workspace to exactly fill from its current top offset down
   * to the viewport bottom. The CSS fallback uses calc(100vh - 220px),
   * but the actual chrome stack (top_bar + navigation + searchbar +
   * nav_projects) varies by viewport width — measure it instead.
   *
   * Symptom of getting this wrong: the filmstrip at the bottom of the
   * workspace's grid gets pushed below the fold and looks "missing".
   */
  function adjustWorkspaceHeight() {
    var ws = document.getElementById('ib-workspace');
    if (!ws) return;
    var top = ws.getBoundingClientRect().top;
    // Workspace fills exactly from its top offset to the viewport
    // bottom. No floor: on tight viewports we'd rather a cramped canvas
    // with the filmstrip visible than a roomy one that overflows the
    // viewport and triggers a body scrollbar (or worse, hides the
    // filmstrip entirely). Body has overflow: hidden on this page, so
    // any positive value here always fits without scrolling.
    var avail = Math.max(120, window.innerHeight - top);
    ws.style.height = avail + 'px';
  }

  async function init() {
    bindDom();

    // Size the workspace before first paint of the editor content.
    adjustWorkspaceHeight();
    window.addEventListener('resize', adjustWorkspaceHeight);
    // Page chrome (top_bar / nav / searchbar) can shift after lazy-loaded
    // images settle, so re-measure once everything is loaded.
    if (document.readyState !== 'complete') {
      window.addEventListener('load', adjustWorkspaceHeight, { once: true });
    }

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
    // B3: step-type picker + per-type field handlers. Lives next to
    // wireStepFields because it shares the autosave + step-undo paths.
    wireStepTypePane();
    wireExportMenu();
    wireRail();
    wireSecondaryResize();
    wirePhotosPane();
    wireBomPane();
    wireFilmstrip();
    wireSettings();
    wireHudToggle();
    wireHudDragAndButtons();
    buildImageToolbar();

    // Re-position the image-context toolbar on window resize / secondary
    // pane drag — both change the canvas frame's CSS layout.
    window.addEventListener('resize', function () {
      var img = activeImageOrNull();
      if (img) positionImageToolbar(img);
      // Also re-clamp HUD position so it doesn't end up off-screen.
      applyHudPosition();
    });

    // Apply persisted UI state (density / grid / pane / filmstrip / etc.).
    applyUiStateToDom();

    // Render the steps list and load the first step's canvas.
    state.activeStepId = state.steps[0].id;
    if (dom.stepTitle) dom.stepTitle.value = state.steps[0].title || '';
    if (dom.stepDescription) dom.stepDescription.value = state.steps[0].description || '';
    renderFilmstrip();
    await loadCanvasFromStep(state.steps[0]);
    // Generate filmstrip thumbnails for every step in the background —
    // doesn't block the canvas opening, and updates each tile when the
    // off-screen render completes. Bypasses if Fabric isn't loaded yet
    // (impossible at this point, but defensive).
    setTimeout(function () { rebuildAllStepThumbs(); }, 0);
    // B3: apply the initial step's type to the picker pane + canvas area.
    syncStepTypePane();

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
