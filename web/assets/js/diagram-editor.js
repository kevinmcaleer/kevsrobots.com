// The diagram tool (issue #270): a Canva-lite editor for annotated photos
// and assembly illustrations, full screen over the project editor.
//
// It's a Fabric.js canvas, the engine the instruction builder uses
// (instruction-builder.js), with its layers pane ported: photos from the
// gallery or an upload, shapes, arrows, text, layers (reorder, rename,
// show/hide, lock, group), align and distribute, snap to grid, crop,
// rotate, background removal, undo/redo and templates. Saving stores the
// canvas JSON (the source; never flattened) with an SVG and a PNG of it
// (PUT /api/projects/{id}/diagrams/{did}); the document shows the SVG.
(function () {
  const API = 'https://projects.kevsrobots.com';
  const FABRIC_URL = 'https://cdn.jsdelivr.net/npm/fabric@6.4.0/dist/index.min.js';
  // Loaded the first time someone removes a background. Its model (tens
  // of MB) comes from IMG.LY's CDN and the browser caches it.
  const BG_REMOVAL_URL = 'https://cdn.jsdelivr.net/npm/@imgly/background-removal@1.4.5/+esm';
  const W = 1200;
  const H = 800;
  const GRID = 10;
  const EXTRA = ['name', 'lockedLayer'];
  const FONTS = [
    ['Sans', 'ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, sans-serif'],
    ['Serif', 'ui-serif, Iowan Old Style, Georgia, Cambria, Times New Roman, serif'],
    ['Mono', 'ui-monospace, SF Mono, Cascadia Code, Menlo, Consolas, monospace'],
  ];
  const INK = '#141413';
  const ACCENT = '#c96442';

  function esc(s) { const d = document.createElement('div'); d.textContent = s == null ? '' : String(s); return d.innerHTML; }

  let fabricLoading = null;
  function loadFabric() {
    if (window.fabric) return Promise.resolve(window.fabric);
    if (!fabricLoading) {
      fabricLoading = new Promise((resolve, reject) => {
        const s = document.createElement('script');
        s.src = (window.DiagramEditorConfig && window.DiagramEditorConfig.fabricUrl) || FABRIC_URL;
        s.onload = () => resolve(window.fabric);
        s.onerror = () => { fabricLoading = null; reject(new Error('Could not load the diagram editor')); };
        document.head.appendChild(s);
      });
    }
    return fabricLoading;
  }

  let bgLoading = null;
  function loadBackgroundRemoval() {
    if (!bgLoading) {
      const url = (window.DiagramEditorConfig && window.DiagramEditorConfig.bgRemovalUrl) || BG_REMOVAL_URL;
      bgLoading = import(url).catch(e => { bgLoading = null; throw e; });
    }
    return bgLoading;
  }

  async function toDataUrl(url) {
    if (/^data:/.test(url)) return url;
    const resp = await fetch(url);
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    const blob = await resp.blob();
    return new Promise((resolve, reject) => {
      const r = new FileReader();
      r.onload = () => resolve(r.result);
      r.onerror = () => reject(r.error);
      r.readAsDataURL(blob);
    });
  }

  // ------------------------------------------------------------ shapes

  const ARROW_PATH = 'M 0 0 L 160 0 M 144 -12 L 160 0 L 144 12';

  function make(F, kind, at) {
    const base = { left: at.x, top: at.y };
    switch (kind) {
      case 'rect': return new F.Rect({ ...base, width: 180, height: 120, rx: 8, ry: 8, fill: '#f0eee6', stroke: INK, strokeWidth: 2, name: 'Rectangle' });
      case 'ellipse': return new F.Ellipse({ ...base, rx: 90, ry: 60, fill: '#f0eee6', stroke: INK, strokeWidth: 2, name: 'Ellipse' });
      case 'line': return new F.Line([at.x, at.y, at.x + 180, at.y], { stroke: INK, strokeWidth: 3, strokeLineCap: 'round', name: 'Line' });
      case 'arrow': return new F.Path(ARROW_PATH, { ...base, fill: '', stroke: ACCENT, strokeWidth: 4, strokeLineCap: 'round', strokeLineJoin: 'round', name: 'Arrow' });
      case 'polygon': return new F.Polygon([{ x: 60, y: 0 }, { x: 120, y: 104 }, { x: 0, y: 104 }], { ...base, fill: '#f0eee6', stroke: INK, strokeWidth: 2, name: 'Triangle' });
      case 'text': return new F.Textbox('Label', { ...base, width: 220, fontSize: 32, fontFamily: FONTS[0][1], fill: INK, name: 'Text' });
      default: return null;
    }
  }

  // Templates: starting canvases, as objects to add.
  const TEMPLATES = [
    { id: 'blank', label: 'Blank', detail: 'An empty canvas' },
    { id: 'annotated', label: 'Annotated photo', detail: 'A photo with arrows and labels' },
    { id: 'exploded', label: 'Exploded view', detail: 'Parts stacked along an axis' },
    { id: 'before-after', label: 'Before/after', detail: 'Two frames side by side' },
  ];

  function templateObjects(F, id) {
    const text = (s, x, y, o) => new F.Textbox(s, { left: x, top: y, width: 300, fontSize: 30, fontFamily: FONTS[0][1], fill: INK, name: s, ...(o || {}) });
    const frame = (x, y, w, h, name) => new F.Rect({ left: x, top: y, width: w, height: h, rx: 12, ry: 12, fill: '#f0eee6', stroke: '#cfc9b8', strokeWidth: 2, strokeDashArray: [10, 8], name });
    const arrow = (x, y, angle) => new F.Path(ARROW_PATH, { left: x, top: y, angle, fill: '', stroke: ACCENT, strokeWidth: 4, strokeLineCap: 'round', strokeLineJoin: 'round', name: 'Arrow' });
    switch (id) {
      case 'annotated':
        return [
          frame(200, 140, 600, 520, 'Photo goes here'),
          text('Add a photo, then drag it here', 320, 380, { fill: '#6b6a63', fontSize: 24, width: 360, textAlign: 'center' }),
          arrow(860, 240, 180), text('This part', 880, 200),
          arrow(860, 520, 180), text('That part', 880, 480),
        ];
      case 'exploded': {
        const out = [new F.Line([600, 60, 600, 740], { stroke: '#b8b3a3', strokeWidth: 2, strokeDashArray: [12, 10], name: 'Axis' })];
        ['Top', 'Middle', 'Base'].forEach((label, i) => {
          out.push(new F.Rect({ left: 420, top: 110 + i * 220, width: 360, height: 110, rx: 10, ry: 10, fill: '#f0eee6', stroke: INK, strokeWidth: 2, name: label + ' part' }));
          out.push(text(label, 820, 145 + i * 220));
        });
        return out;
      }
      case 'before-after':
        return [
          text('Before', 60, 40, { fontSize: 40, width: 500 }), frame(60, 110, 520, 600, 'Before photo'),
          text('After', 620, 40, { fontSize: 40, width: 500 }), frame(620, 110, 520, 600, 'After photo'),
        ];
      default:
        return [];
    }
  }

  function kindLabel(o) {
    const t = String(o.type || '').toLowerCase();
    if (t === 'image') return { icon: 'fa-image', label: 'Photo' };
    if (t === 'textbox' || t === 'i-text' || t === 'text') return { icon: 'fa-font', label: 'Text' };
    if (t === 'group') return { icon: 'fa-object-group', label: 'Group' };
    if (t === 'path') return { icon: 'fa-arrow-right-long', label: 'Arrow' };
    if (t === 'line') return { icon: 'fa-minus', label: 'Line' };
    if (t === 'ellipse' || t === 'circle') return { icon: 'fa-circle', label: 'Ellipse' };
    if (t === 'polygon') return { icon: 'fa-play', label: 'Polygon' };
    return { icon: 'fa-square', label: 'Rectangle' };
  }

  // ------------------------------------------------------------ the editor

  // Opens diagram `diagramId` of the project. ctx:
  //   projectId               the project's id
  //   diagramId               the diagram to edit
  //   template                for a new one, a TEMPLATES id to start from
  //   apiFetch(url, opts)     fetch with the signed-in user's token
  //   listImages()            the project gallery: [{ url, title }]
  //   uploadImage(blob, name) adds to the gallery, resolving to { url }
  //   placeImage(diagram)     puts or refreshes the diagram in the document
  //   onClose(saved)          after it closes; saved says whether it ever saved
  async function open(ctx) {
    const base = API + '/api/projects/' + ctx.projectId + '/diagrams/' + ctx.diagramId;
    const overlay = document.createElement('div');
    overlay.className = 'diagram-editor';
    overlay.setAttribute('role', 'dialog');
    overlay.setAttribute('aria-modal', 'true');
    overlay.setAttribute('aria-label', 'Diagram');
    const tool = (t, icon, label) => `<button type="button" class="diagram-tool" data-tool="${t}" title="${label}" aria-label="${label}"><i class="fas ${icon}"></i></button>`;
    overlay.innerHTML = `
      <div class="diagram-bar">
        <i class="fas fa-shapes text-muted"></i>
        <input type="text" class="form-control form-control-sm diagram-title" maxlength="200" aria-label="Diagram title">
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="undo" title="Undo (Ctrl+Z)"><i class="fas fa-rotate-left"></i></button>
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="redo" title="Redo (Ctrl+Shift+Z)"><i class="fas fa-rotate-right"></i></button>
        <label class="form-check form-switch small mb-0 ms-1"><input class="form-check-input" type="checkbox" data-act="snap" checked> Snap to grid</label>
        <div class="dropdown">
          <button type="button" class="btn btn-sm btn-outline-secondary dropdown-toggle" data-bs-toggle="dropdown" aria-expanded="false">Export</button>
          <ul class="dropdown-menu">
            <li><button type="button" class="dropdown-item" data-export="svg">SVG</button></li>
            <li><button type="button" class="dropdown-item" data-export="png1">PNG 1×</button></li>
            <li><button type="button" class="dropdown-item" data-export="png2">PNG 2×</button></li>
          </ul>
        </div>
        <span class="diagram-status small text-muted" role="status"></span>
        <span class="ms-auto"></span>
        <button type="button" class="btn btn-sm btn-primary" data-act="save"><i class="fas fa-save me-1"></i>Save</button>
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="close">Close</button>
      </div>
      <div class="diagram-body">
        <div class="diagram-tools" role="toolbar" aria-label="Add">
          ${tool('photo', 'fa-image', 'Photo')}
          ${tool('rect', 'fa-square', 'Rectangle')}
          ${tool('ellipse', 'fa-circle', 'Ellipse')}
          ${tool('line', 'fa-minus', 'Line')}
          ${tool('arrow', 'fa-arrow-right-long', 'Arrow')}
          ${tool('polygon', 'fa-play', 'Triangle')}
          ${tool('text', 'fa-font', 'Text')}
        </div>
        <div class="diagram-stage"><div class="diagram-artboard"><canvas></canvas><div class="diagram-grid"></div></div></div>
        <aside class="diagram-panel">
          <section data-panel="props" hidden>
            <h6>Style</h6>
            <div class="diagram-row" data-for="fill"><label>Fill</label><input type="color" data-prop="fill"><label class="small"><input type="checkbox" data-prop="nofill"> None</label></div>
            <div class="diagram-row" data-for="stroke"><label>Line</label><input type="color" data-prop="stroke"><input type="number" class="form-control form-control-sm" data-prop="strokeWidth" min="0" max="60" step="1" title="Line width"></div>
            <div class="diagram-row"><label>Opacity</label><input type="range" class="form-range" data-prop="opacity" min="0.05" max="1" step="0.05"></div>
            <div data-for="text">
              <div class="diagram-row"><label>Font</label><select class="form-select form-select-sm" data-prop="fontFamily">${FONTS.map(([l, v]) => `<option value="${esc(v)}">${l}</option>`).join('')}</select></div>
              <div class="diagram-row"><label>Size</label><input type="number" class="form-control form-control-sm" data-prop="fontSize" min="6" max="400" step="1"><button type="button" class="btn btn-sm btn-outline-secondary" data-prop="bold" title="Bold"><i class="fas fa-bold"></i></button></div>
              <div class="diagram-row"><label>Align</label>
                <div class="btn-group btn-group-sm">${['left', 'center', 'right'].map(a => `<button type="button" class="btn btn-outline-secondary" data-align-text="${a}" title="${a}"><i class="fas fa-align-${a}"></i></button>`).join('')}</div>
              </div>
            </div>
          </section>
          <section data-panel="arrange" hidden>
            <h6>Arrange</h6>
            <div class="diagram-row diagram-grid-buttons">
              ${[['left', 'fa-align-left'], ['hcenter', 'fa-align-center'], ['right', 'fa-align-right'], ['top', 'fa-arrow-up'], ['vcenter', 'fa-grip-lines'], ['bottom', 'fa-arrow-down']].map(([a, i]) => `<button type="button" class="btn btn-sm btn-outline-secondary" data-align="${a}" title="Align ${a}"><i class="fas ${i}"></i></button>`).join('')}
            </div>
            <div class="diagram-row">
              <button type="button" class="btn btn-sm btn-outline-secondary" data-distribute="h" title="Distribute across">⇿</button>
              <button type="button" class="btn btn-sm btn-outline-secondary" data-distribute="v" title="Distribute down">⇳</button>
              <button type="button" class="btn btn-sm btn-outline-secondary" data-act="rotl" title="Rotate left"><i class="fas fa-rotate-left"></i> 90°</button>
              <button type="button" class="btn btn-sm btn-outline-secondary" data-act="rotr" title="Rotate right"><i class="fas fa-rotate-right"></i> 90°</button>
            </div>
            <div class="diagram-row">
              <button type="button" class="btn btn-sm btn-outline-secondary" data-act="group" title="Group (Ctrl+G)">Group</button>
              <button type="button" class="btn btn-sm btn-outline-secondary" data-act="ungroup" title="Ungroup (Ctrl+Shift+G)">Ungroup</button>
              <button type="button" class="btn btn-sm btn-outline-secondary" data-act="duplicate" title="Duplicate (Ctrl+D)">Duplicate</button>
              <button type="button" class="btn btn-sm btn-outline-danger" data-act="delete" title="Delete"><i class="fas fa-trash"></i></button>
            </div>
          </section>
          <section data-panel="image" hidden>
            <h6>Photo</h6>
            <label class="form-check small"><input class="form-check-input" type="checkbox" data-act="aspect" checked> Keep proportions</label>
            <div class="diagram-row">
              <button type="button" class="btn btn-sm btn-outline-secondary" data-act="crop"><i class="fas fa-crop-simple me-1"></i><span>Crop</span></button>
              <button type="button" class="btn btn-sm btn-outline-secondary" data-act="cutout"><i class="fas fa-wand-magic-sparkles me-1"></i>Remove background</button>
            </div>
          </section>
          <section>
            <h6>Layers</h6>
            <ul class="diagram-layers"></ul>
          </section>
        </aside>
      </div>
      <div class="diagram-photos" hidden>
        <div class="diagram-photos-head"><strong>Add a photo</strong><button type="button" class="btn-close" aria-label="Close"></button></div>
        <div class="diagram-photos-grid"></div>
        <label class="btn btn-sm btn-outline-primary mt-2"><i class="fas fa-upload me-1"></i>Upload…<input type="file" accept="image/*" hidden></label>
      </div>`;
    document.body.appendChild(overlay);
    document.body.classList.add('diagram-editor-open');
    const $ = s => overlay.querySelector(s);
    const $$ = s => [...overlay.querySelectorAll(s)];
    const setStatus = (text, bad) => {
      const el = $('.diagram-status');
      el.textContent = text || '';
      el.className = 'diagram-status small ' + (bad ? 'text-danger' : 'text-muted');
    };

    let F;
    let canvas;
    let record;
    let saved = false;
    let dirty = false;
    let closed = false;
    try {
      const [fab, resp] = await Promise.all([loadFabric(), ctx.apiFetch(base)]);
      F = fab;
      if (!resp.ok) throw new Error('Could not open the diagram (HTTP ' + resp.status + ')');
      record = await resp.json();
      saved = !!record.svg_url;
    } catch (e) {
      $('.diagram-stage').innerHTML = '<div class="alert alert-danger m-3">' + esc(e.message || e) + '</div>';
      $('[data-act=close]').addEventListener('click', () => { overlay.remove(); document.body.classList.remove('diagram-editor-open'); });
      return;
    }
    $('.diagram-title').value = record.title || 'Diagram';

    // --- canvas, fitted to the stage
    const el = $('canvas');
    canvas = new F.Canvas(el, { width: W, height: H, backgroundColor: '#ffffff', preserveObjectStacking: true, uniformScaling: true });
    let zoom = 1;
    function fit() {
      const stage = $('.diagram-stage');
      zoom = Math.max(0.2, Math.min(1.5, (stage.clientWidth - 48) / W, (stage.clientHeight - 48) / H));
      canvas.setDimensions({ width: Math.round(W * zoom), height: Math.round(H * zoom) });
      canvas.setZoom(zoom);
      const grid = $('.diagram-grid');
      grid.style.backgroundSize = (GRID * zoom * 2) + 'px ' + (GRID * zoom * 2) + 'px';
      canvas.requestRenderAll();
    }
    const ro = new ResizeObserver(fit);
    ro.observe($('.diagram-stage'));

    // Locked layers can't be picked on the canvas; they still are in the list.
    function applyLock(o) {
      o.selectable = !o.lockedLayer;
      o.evented = !o.lockedLayer;
    }
    function aspectControls(o, keep) {
      if (!o || !o.setControlsVisibility) return;
      o.setControlsVisibility({ mt: !keep, mb: !keep, ml: !keep, mr: !keep });
    }

    // --- history
    const past = [];
    const future = [];
    let restoring = false;
    let pending = 0;
    // The canvas as saved: never the crop box, which is only a tool.
    const canvasJson = () => {
      const json = canvas.toObject(EXTRA);
      json.objects = json.objects.filter(o => o.name !== '__crop');
      return json;
    };
    const snapshot = () => JSON.stringify(canvasJson());
    let current = null;
    function changed() {
      if (restoring) return;
      dirty = true;
      setStatus('Not saved');
      clearTimeout(pending);
      pending = setTimeout(() => {
        const next = snapshot();
        if (next === current) return;
        if (current) past.push(current);
        if (past.length > 60) past.shift();
        future.length = 0;
        current = next;
      }, 120);
      renderLayers();
    }
    async function restore(json) {
      restoring = true;
      await canvas.loadFromJSON(JSON.parse(json));
      canvas.getObjects().forEach(o => { applyLock(o); if (o.type === 'image') aspectControls(o, true); });
      canvas.backgroundColor = canvas.backgroundColor || '#ffffff';
      canvas.requestRenderAll();
      restoring = false;
      current = json;
      dirty = true;
      renderLayers();
      syncPanels();
    }
    async function undo() {
      if (cropping) endCrop(false);
      clearTimeout(pending);
      if (!past.length) return;
      future.push(current);
      await restore(past.pop());
    }
    async function redo() {
      if (cropping) endCrop(false);
      if (!future.length) return;
      past.push(current);
      await restore(future.pop());
    }

    // --- load
    restoring = true;
    const data = record.data || {};
    if (Array.isArray(data.objects) && data.objects.length) {
      await canvas.loadFromJSON(data);
      canvas.getObjects().forEach(o => { applyLock(o); if (o.type === 'image') aspectControls(o, true); });
    } else if (ctx.template) {
      templateObjects(F, ctx.template).forEach(o => canvas.add(o));
    }
    canvas.backgroundColor = '#ffffff';
    restoring = false;
    current = snapshot();
    fit();

    canvas.on('object:added', changed);
    canvas.on('object:removed', changed);
    canvas.on('object:modified', changed);
    canvas.on('text:changed', changed);
    // Selecting only re-marks the list, so a row being double-clicked to
    // rename isn't redrawn under the pointer.
    canvas.on('selection:created', () => { syncPanels(); markLayers(); });
    canvas.on('selection:updated', () => { syncPanels(); markLayers(); });
    canvas.on('selection:cleared', () => { syncPanels(); markLayers(); });
    canvas.on('object:moving', e => {
      if (!$('[data-act=snap]').checked) return;
      const t = e.target;
      t.set({ left: Math.round(t.left / GRID) * GRID, top: Math.round(t.top / GRID) * GRID });
    });
    $('.diagram-grid').hidden = !$('[data-act=snap]').checked;
    $('[data-act=snap]').addEventListener('change', e => { $('.diagram-grid').hidden = !e.target.checked; });

    const active = () => canvas.getActiveObject();
    const selected = () => canvas.getActiveObjects();
    const center = () => ({ x: W / 2 - 90 + Math.round(Math.random() * 6) * GRID, y: H / 2 - 60 + Math.round(Math.random() * 6) * GRID });

    function addObject(o) {
      canvas.add(o);
      canvas.setActiveObject(o);
      canvas.requestRenderAll();
    }

    // --- tools
    $$('[data-tool]').forEach(b => b.addEventListener('click', () => {
      const t = b.dataset.tool;
      if (t === 'photo') { openPhotos(); return; }
      const o = make(F, t, center());
      if (o) addObject(o);
      if (t === 'text' && o) { o.enterEditing(); o.selectAll(); }
    }));

    async function addPhoto(url, title) {
      setStatus('Adding the photo…');
      try {
        const img = await F.FabricImage.fromURL(url, { crossOrigin: 'anonymous' });
        const scale = Math.min(1, (W * 0.6) / img.width, (H * 0.6) / img.height);
        img.set({ left: (W - img.width * scale) / 2, top: (H - img.height * scale) / 2, scaleX: scale, scaleY: scale, name: title || 'Photo' });
        aspectControls(img, true);
        addObject(img);
        setStatus('');
      } catch (e) {
        setStatus('That photo could not be loaded.', true);
      }
    }

    function closePhotos() { $('.diagram-photos').hidden = true; }
    async function openPhotos() {
      const pane = $('.diagram-photos');
      pane.hidden = false;
      const grid = $('.diagram-photos-grid');
      grid.innerHTML = '<div class="small text-muted">Loading the gallery…</div>';
      const images = await ctx.listImages().catch(() => []);
      grid.innerHTML = images.length
        ? images.map((im, i) => `<button type="button" data-i="${i}" title="${esc(im.title || '')}"><img src="${esc(im.url)}" alt="${esc(im.title || '')}" loading="lazy"></button>`).join('')
        : '<div class="small text-muted">No photos in the gallery yet. Upload one.</div>';
      grid.querySelectorAll('button').forEach(b => b.addEventListener('click', () => {
        const im = images[+b.dataset.i];
        closePhotos();
        addPhoto(im.url, im.title);
      }));
    }
    $('.diagram-photos .btn-close').addEventListener('click', closePhotos);
    $('.diagram-photos input[type=file]').addEventListener('change', async e => {
      const file = e.target.files && e.target.files[0];
      e.target.value = '';
      if (!file) return;
      closePhotos();
      setStatus('Uploading ' + file.name + '…');
      try {
        const up = await ctx.uploadImage(file, file.name);
        addPhoto(up.url, file.name.replace(/\.[^.]+$/, ''));
      } catch (err) {
        setStatus('Upload failed: ' + (err.message || err), true);
      }
    });

    // --- properties
    const kindOf = o => String(o && o.type || '').toLowerCase();
    const isText = o => ['textbox', 'i-text', 'text'].includes(kindOf(o));
    function hex(c, fallback) {
      if (typeof c !== 'string' || !c) return fallback;
      if (/^#[0-9a-f]{6}$/i.test(c)) return c;
      const m = c.match(/rgba?\((\d+),\s*(\d+),\s*(\d+)/);
      return m ? '#' + [m[1], m[2], m[3]].map(n => (+n).toString(16).padStart(2, '0')).join('') : fallback;
    }
    function syncPanels() {
      const objs = selected();
      const o = objs.length === 1 ? objs[0] : null;
      $('[data-panel=props]').hidden = !objs.length;
      $('[data-panel=arrange]').hidden = !objs.length;
      // While cropping, the crop box is what's selected; Apply stays on screen.
      $('[data-panel=image]').hidden = !(cropping || (o && kindOf(o) === 'image'));
      if (cropping) { $('[data-panel=props]').hidden = true; $('[data-panel=arrange]').hidden = true; return; }
      if (!objs.length) return;
      const first = o || objs[0];
      const text = isText(first);
      $('[data-for=text]').hidden = !text;
      $('[data-for=fill]').hidden = kindOf(first) === 'image' || kindOf(first) === 'line' || kindOf(first) === 'path';
      $('[data-prop=fill]').value = hex(first.fill, '#f0eee6');
      $('[data-prop=nofill]').checked = !first.fill || first.fill === 'transparent';
      $('[data-prop=stroke]').value = hex(first.stroke, INK);
      $('[data-prop=strokeWidth]').value = first.strokeWidth || 0;
      $('[data-prop=opacity]').value = first.opacity == null ? 1 : first.opacity;
      if (text) {
        $('[data-prop=fontFamily]').value = first.fontFamily;
        $('[data-prop=fontSize]').value = Math.round(first.fontSize);
        $('[data-prop=bold]').classList.toggle('active', first.fontWeight === 'bold');
        $$('[data-align-text]').forEach(b => b.classList.toggle('active', b.dataset.alignText === (first.textAlign || 'left')));
        // A text's colour is its fill.
        $('[data-for=fill]').hidden = false;
      }
      if (o && kindOf(o) === 'image') {
        $('[data-act=aspect]').checked = !o.isControlVisible || !o.isControlVisible('mt');
      }
    }
    function setProp(name, value) {
      selected().forEach(o => {
        o.set(name, value);
        if (isText(o) && name === 'fontSize') o.initDimensions && o.initDimensions();
        o.setCoords();
      });
      canvas.requestRenderAll();
      changed();
    }
    $('[data-prop=fill]').addEventListener('input', e => { $('[data-prop=nofill]').checked = false; setProp('fill', e.target.value); });
    $('[data-prop=nofill]').addEventListener('change', e => setProp('fill', e.target.checked ? '' : $('[data-prop=fill]').value));
    $('[data-prop=stroke]').addEventListener('input', e => setProp('stroke', e.target.value));
    $('[data-prop=strokeWidth]').addEventListener('input', e => setProp('strokeWidth', Math.max(0, +e.target.value || 0)));
    $('[data-prop=opacity]').addEventListener('input', e => setProp('opacity', +e.target.value));
    $('[data-prop=fontFamily]').addEventListener('change', e => setProp('fontFamily', e.target.value));
    $('[data-prop=fontSize]').addEventListener('input', e => setProp('fontSize', Math.max(6, +e.target.value || 6)));
    $('[data-prop=bold]').addEventListener('click', () => {
      const bold = !(active() && active().fontWeight === 'bold');
      setProp('fontWeight', bold ? 'bold' : 'normal');
      syncPanels();
    });
    $$('[data-align-text]').forEach(b => b.addEventListener('click', () => { setProp('textAlign', b.dataset.alignText); syncPanels(); }));

    // --- arrange
    // Each object's box on the artboard, then a move by (dx, dy).
    function boxes(objs) {
      return objs.map(o => ({ o, r: o.getBoundingRect() }));
    }
    function moveBy(o, dx, dy) {
      o.set({ left: o.left + dx, top: o.top + dy });
      o.setCoords();
    }
    function withPlainSelection(fn) {
      const objs = selected();
      if (!objs.length) return;
      canvas.discardActiveObject();
      fn(objs);
      if (objs.length > 1) canvas.setActiveObject(new F.ActiveSelection(objs, { canvas }));
      else canvas.setActiveObject(objs[0]);
      canvas.requestRenderAll();
      changed();
    }
    $$('[data-align]').forEach(b => b.addEventListener('click', () => withPlainSelection(objs => {
      const bs = boxes(objs);
      // One object lines up with the artboard; several with each other.
      const area = objs.length > 1
        ? bs.reduce((a, { r }) => ({ l: Math.min(a.l, r.left), t: Math.min(a.t, r.top), r: Math.max(a.r, r.left + r.width), b: Math.max(a.b, r.top + r.height) }), { l: Infinity, t: Infinity, r: -Infinity, b: -Infinity })
        : { l: 0, t: 0, r: W, b: H };
      bs.forEach(({ o, r }) => {
        const a = b.dataset.align;
        if (a === 'left') moveBy(o, area.l - r.left, 0);
        if (a === 'right') moveBy(o, area.r - (r.left + r.width), 0);
        if (a === 'hcenter') moveBy(o, (area.l + area.r) / 2 - (r.left + r.width / 2), 0);
        if (a === 'top') moveBy(o, 0, area.t - r.top);
        if (a === 'bottom') moveBy(o, 0, area.b - (r.top + r.height));
        if (a === 'vcenter') moveBy(o, 0, (area.t + area.b) / 2 - (r.top + r.height / 2));
      });
    })));
    $$('[data-distribute]').forEach(b => b.addEventListener('click', () => withPlainSelection(objs => {
      if (objs.length < 3) { setStatus('Select three or more to distribute.'); return; }
      const across = b.dataset.distribute === 'h';
      const bs = boxes(objs).sort((x, y) => (across ? x.r.left - y.r.left : x.r.top - y.r.top));
      const size = r => (across ? r.width : r.height);
      const start = across ? bs[0].r.left : bs[0].r.top;
      const last = bs[bs.length - 1].r;
      const end = across ? last.left + last.width : last.top + last.height;
      const gap = (end - start - bs.reduce((n, { r }) => n + size(r), 0)) / (bs.length - 1);
      let at = start;
      bs.forEach(({ o, r }) => {
        const cur = across ? r.left : r.top;
        if (across) moveBy(o, at - cur, 0); else moveBy(o, 0, at - cur);
        at += size(r) + gap;
      });
    })));
    function rotate(by) {
      selected().forEach(o => { o.rotate(((o.angle || 0) + by + 360) % 360); o.setCoords(); });
      canvas.requestRenderAll();
      changed();
    }
    $('[data-act=rotl]').addEventListener('click', () => rotate(-90));
    $('[data-act=rotr]').addEventListener('click', () => rotate(90));

    function removeSelected() {
      const objs = selected().filter(o => !o.lockedLayer);
      if (!objs.length) return;
      canvas.discardActiveObject();
      objs.forEach(o => canvas.remove(o));
      canvas.requestRenderAll();
    }
    async function duplicate() {
      const objs = selected();
      if (!objs.length) return;
      canvas.discardActiveObject();
      const copies = [];
      for (const o of objs) {
        const c = await o.clone(EXTRA);
        c.set({ left: o.left + 20, top: o.top + 20, name: (o.name || kindLabel(o).label) + ' copy' });
        canvas.add(c);
        copies.push(c);
      }
      canvas.setActiveObject(copies.length > 1 ? new F.ActiveSelection(copies, { canvas }) : copies[0]);
      canvas.requestRenderAll();
    }
    function group() {
      const sel = active();
      if (!sel || kindOf(sel) !== 'activeselection') { setStatus('Select two or more to group.'); return; }
      const objs = sel.getObjects();
      canvas.discardActiveObject();
      objs.forEach(o => canvas.remove(o));
      const g = new F.Group(objs, { name: 'Group' });
      addObject(g);
    }
    function ungroup() {
      const g = active();
      if (!g || kindOf(g) !== 'group') return;
      canvas.discardActiveObject();
      const items = g.removeAll();
      canvas.remove(g);
      items.forEach(o => { canvas.add(o); o.setCoords(); });
      canvas.setActiveObject(new F.ActiveSelection(items, { canvas }));
      canvas.requestRenderAll();
    }
    $('[data-act=delete]').addEventListener('click', removeSelected);
    $('[data-act=duplicate]').addEventListener('click', duplicate);
    $('[data-act=group]').addEventListener('click', group);
    $('[data-act=ungroup]').addEventListener('click', ungroup);

    // --- photo: proportions, crop, background
    $('[data-act=aspect]').addEventListener('change', e => {
      const o = active();
      aspectControls(o, e.target.checked);
      canvas.uniformScaling = e.target.checked;
      canvas.requestRenderAll();
    });
    let cropRect = null;
    let cropping = null;
    function endCrop(apply) {
      const img = cropping;
      const r = cropRect;
      cropping = null;
      cropRect = null;
      $('[data-act=crop] span').textContent = 'Crop';
      if (!img || !r) return;
      restoring = true;
      canvas.remove(r);
      restoring = false;
      if (apply) {
        const sx = img.scaleX;
        const sy = img.scaleY;
        const el = img.getElement();
        const natW = el.naturalWidth || el.width;
        const natH = el.naturalHeight || el.height;
        const rb = r.getBoundingRect();
        const ib = img.getBoundingRect();
        let cropX = (img.cropX || 0) + (rb.left - ib.left) / sx;
        let cropY = (img.cropY || 0) + (rb.top - ib.top) / sy;
        cropX = Math.max(0, cropX);
        cropY = Math.max(0, cropY);
        const width = Math.max(4, Math.min(natW - cropX, rb.width / sx));
        const height = Math.max(4, Math.min(natH - cropY, rb.height / sy));
        img.set({ cropX, cropY, width, height, left: img.left + (cropX - (img.cropX || 0)) * sx, top: img.top + (cropY - (img.cropY || 0)) * sy });
        img.setCoords();
        changed();
      }
      canvas.setActiveObject(img);
      canvas.requestRenderAll();
    }
    $('[data-act=crop]').addEventListener('click', () => {
      if (cropping) { endCrop(true); return; }
      const img = active();
      if (!img || kindOf(img) !== 'image') return;
      if ((img.angle || 0) % 360) { setStatus('Rotate the photo back to 0° to crop it.', true); return; }
      cropping = img;
      const b = img.getBoundingRect();
      restoring = true;
      cropRect = new F.Rect({
        left: b.left + b.width * 0.1, top: b.top + b.height * 0.1, width: b.width * 0.8, height: b.height * 0.8,
        fill: 'rgba(201,100,66,0.08)', stroke: ACCENT, strokeWidth: 2 / zoom, strokeDashArray: [8, 6], strokeUniform: true,
        cornerColor: ACCENT, transparentCorners: false, name: '__crop',
      });
      canvas.add(cropRect);
      restoring = false;
      canvas.setActiveObject(cropRect);
      canvas.requestRenderAll();
      $('[data-act=crop] span').textContent = 'Apply crop';
      setStatus('Drag the box to what you want to keep, then Apply crop. Esc cancels.');
    });

    $('[data-act=cutout]').addEventListener('click', async () => {
      const img = active();
      if (!img || kindOf(img) !== 'image') return;
      const btn = $('[data-act=cutout]');
      btn.disabled = true;
      const t0 = performance.now();
      try {
        setStatus('Loading background removal (the first time downloads its model)…');
        const mod = await loadBackgroundRemoval();
        const removeBackground = mod.removeBackground || (mod.default && mod.default.removeBackground) || mod.default;
        const src = await toDataUrl(img.getSrc());
        const config = Object.assign({
          progress: (key, done, total) => {
            if (/fetch/.test(key) && total) setStatus('Downloading the model… ' + Math.round((done / total) * 100) + '%');
            else if (/compute/.test(key)) setStatus('Removing the background…');
          },
        }, (window.DiagramEditorConfig && window.DiagramEditorConfig.bgRemoval) || {});
        const blob = await removeBackground(src, config);
        setStatus('Saving the cut-out to the gallery…');
        const up = await ctx.uploadImage(blob, (img.name || 'photo') + '-cutout.png');
        const next = await F.FabricImage.fromURL(up.url, { crossOrigin: 'anonymous' });
        next.set({
          left: img.left, top: img.top, scaleX: img.scaleX, scaleY: img.scaleY, angle: img.angle,
          cropX: img.cropX, cropY: img.cropY, width: img.width, height: img.height,
          flipX: img.flipX, flipY: img.flipY, opacity: img.opacity, name: (img.name || 'Photo') + ' (cut-out)',
        });
        aspectControls(next, true);
        const at = canvas.getObjects().indexOf(img);
        canvas.remove(img);
        canvas.add(next);
        canvas.moveObjectTo(next, at);
        canvas.setActiveObject(next);
        canvas.requestRenderAll();
        setStatus('Background removed in ' + ((performance.now() - t0) / 1000).toFixed(1) + ' s. The cut-out is in the gallery.');
      } catch (e) {
        setStatus('Background removal failed: ' + (e.message || e), true);
      } finally {
        btn.disabled = false;
      }
    });

    // --- layers (ported from the instruction builder's pane, #183)
    function renderLayers() {
      const list = $('.diagram-layers');
      const objs = canvas.getObjects().filter(o => o.name !== '__crop');
      if (!objs.length) { list.innerHTML = '<li class="diagram-layers-empty">Nothing here yet.</li>'; return; }
      const on = new Set(selected());
      const all = canvas.getObjects();
      list.innerHTML = objs.slice().reverse().map(o => {
        const k = kindLabel(o);
        const idx = all.indexOf(o);
        return `<li class="${on.has(o) ? 'is-active' : ''}${o.visible === false ? ' is-hidden' : ''}" draggable="true" data-i="${idx}">
          <i class="fas ${k.icon} diagram-layer-icon"></i>
          <span class="diagram-layer-name" title="Double-click to rename">${esc(o.name || k.label)}</span>
          <button type="button" data-layer="visible" title="${o.visible === false ? 'Show' : 'Hide'}"><i class="fas ${o.visible === false ? 'fa-eye-slash' : 'fa-eye'}"></i></button>
          <button type="button" data-layer="lock" title="${o.lockedLayer ? 'Unlock' : 'Lock'}"><i class="fas ${o.lockedLayer ? 'fa-lock' : 'fa-lock-open'}"></i></button>
        </li>`;
      }).join('');
      list.querySelectorAll('li[data-i]').forEach(li => {
        const target = () => canvas.getObjects()[+li.dataset.i];
        li.addEventListener('click', e => {
          const o = target();
          if (!o || e.target.closest('button') || e.target.closest('input')) return;
          if (o.lockedLayer || o.visible === false) return;
          canvas.setActiveObject(o);
          canvas.requestRenderAll();
        });
        li.querySelector('[data-layer=visible]').addEventListener('click', () => {
          const o = target();
          o.visible = o.visible === false;
          if (!o.visible) canvas.discardActiveObject();
          canvas.requestRenderAll();
          changed();
        });
        li.querySelector('[data-layer=lock]').addEventListener('click', () => {
          const o = target();
          o.lockedLayer = !o.lockedLayer;
          applyLock(o);
          if (o.lockedLayer) canvas.discardActiveObject();
          canvas.requestRenderAll();
          changed();
        });
        li.querySelector('.diagram-layer-name').addEventListener('dblclick', () => {
          const o = target();
          const span = li.querySelector('.diagram-layer-name');
          const input = document.createElement('input');
          input.className = 'form-control form-control-sm';
          input.value = o.name || kindLabel(o).label;
          span.replaceWith(input);
          input.focus();
          input.select();
          const done = keep => { if (keep && input.value.trim()) { o.name = input.value.trim().slice(0, 80); changed(); } renderLayers(); };
          input.addEventListener('keydown', e => { if (e.key === 'Enter') done(true); if (e.key === 'Escape') done(false); e.stopPropagation(); });
          input.addEventListener('blur', () => done(true));
        });
        li.addEventListener('dragstart', e => {
          e.dataTransfer.effectAllowed = 'move';
          try { e.dataTransfer.setData('text/x-diagram-layer', li.dataset.i); } catch (_) {}
          li.classList.add('is-dragging');
        });
        li.addEventListener('dragend', () => li.classList.remove('is-dragging'));
        li.addEventListener('dragover', e => { e.preventDefault(); li.classList.add('is-dragover'); });
        li.addEventListener('dragleave', () => li.classList.remove('is-dragover'));
        li.addEventListener('drop', e => {
          e.preventDefault();
          li.classList.remove('is-dragover');
          const from = parseInt(e.dataTransfer.getData('text/x-diagram-layer'), 10);
          const to = +li.dataset.i;
          if (isNaN(from) || from === to) return;
          moveLayer(from, to);
        });
      });
    }
    function markLayers() {
      const on = new Set(selected());
      const all = canvas.getObjects();
      $$('.diagram-layers li[data-i]').forEach(li => li.classList.toggle('is-active', on.has(all[+li.dataset.i])));
    }
    function moveLayer(from, to) {
      const o = canvas.getObjects()[from];
      if (!o) return;
      canvas.moveObjectTo(o, to);
      canvas.requestRenderAll();
      changed();
    }
    renderLayers();

    // --- keyboard
    function typing(e) {
      const t = e.target;
      if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT' || t.isContentEditable)) return true;
      const a = active();
      return !!(a && a.isEditing);
    }
    function onKey(e) {
      if (closed || document.querySelector('.modal.show')) return;
      if (e.key === 'Escape') {
        if (cropping) { endCrop(false); e.preventDefault(); return; }
        if (!$('.diagram-photos').hidden) { closePhotos(); e.preventDefault(); return; }
        if (typing(e)) return;
        if (active()) { canvas.discardActiveObject(); canvas.requestRenderAll(); } else close();
        e.preventDefault();
        return;
      }
      if (typing(e)) return;
      const mod = e.ctrlKey || e.metaKey;
      const k = e.key.toLowerCase();
      if (mod && k === 'z') { e.preventDefault(); if (e.shiftKey) redo(); else undo(); return; }
      if (mod && k === 'y') { e.preventDefault(); redo(); return; }
      if (mod && k === 'd') { e.preventDefault(); duplicate(); return; }
      if (mod && k === 'g') { e.preventDefault(); if (e.shiftKey) ungroup(); else group(); return; }
      if (mod && k === 's') { e.preventDefault(); save(); return; }
      if (e.key === 'Delete' || e.key === 'Backspace') { if (selected().length) { e.preventDefault(); removeSelected(); } return; }
      const nudge = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] }[e.key];
      if (nudge && selected().length) {
        e.preventDefault();
        const step = e.shiftKey ? GRID : 1;
        const a = active();
        a.set({ left: a.left + nudge[0] * step, top: a.top + nudge[1] * step });
        a.setCoords();
        canvas.requestRenderAll();
        canvas.fire('object:modified', { target: a });
      }
    }
    document.addEventListener('keydown', onKey, true);
    $('[data-act=undo]').addEventListener('click', undo);
    $('[data-act=redo]').addEventListener('click', redo);

    // --- export
    // The artboard at its own size, whatever the zoom on screen.
    function atFullSize(fn) {
      if (cropping) endCrop(false);
      const sel = active();
      canvas.discardActiveObject();
      canvas.setZoom(1);
      canvas.setDimensions({ width: W, height: H });
      try { return fn(); } finally {
        fit();
        if (sel) canvas.setActiveObject(sel);
        canvas.requestRenderAll();
      }
    }
    // An SVG that stands alone: every photo read into a data: URL, since
    // an SVG shown through <img> loads nothing from outside.
    async function exportSvg() {
      const svg = atFullSize(() => canvas.toSVG({ width: W, height: H, viewBox: { x: 0, y: 0, width: W, height: H } }));
      const doc = new DOMParser().parseFromString(svg, 'image/svg+xml');
      const seen = new Map();
      await Promise.all([...doc.querySelectorAll('image')].map(async im => {
        const href = im.getAttribute('href') || im.getAttribute('xlink:href');
        if (!href || href.startsWith('data:')) return;
        if (!seen.has(href)) seen.set(href, toDataUrl(href).catch(() => null));
        const data = await seen.get(href);
        if (data) { im.setAttribute('href', data); im.removeAttribute('xlink:href'); }
      }));
      return new XMLSerializer().serializeToString(doc.documentElement);
    }
    function exportPng(scale) {
      return atFullSize(() => canvas.toDataURL({ format: 'png', multiplier: scale || 1 }));
    }
    function download(name, href) {
      const a = document.createElement('a');
      a.href = href;
      a.download = name;
      document.body.appendChild(a);
      a.click();
      a.remove();
    }
    const fileBase = () => ($('.diagram-title').value.trim() || 'diagram').replace(/[^\w.-]+/g, '-').toLowerCase();
    $$('[data-export]').forEach(b => b.addEventListener('click', async () => {
      try {
        if (b.dataset.export === 'svg') {
          const url = URL.createObjectURL(new Blob([await exportSvg()], { type: 'image/svg+xml' }));
          download(fileBase() + '.svg', url);
          setTimeout(() => URL.revokeObjectURL(url), 1000);
        } else {
          download(fileBase() + (b.dataset.export === 'png2' ? '@2x' : '') + '.png', exportPng(b.dataset.export === 'png2' ? 2 : 1));
        }
      } catch (e) {
        setStatus('Export failed: ' + (e.message || e) + '. A photo from another site can stop the PNG export.', true);
      }
    }));

    // --- save / close
    $('.diagram-title').addEventListener('input', () => { dirty = true; setStatus('Not saved'); });
    async function save() {
      const btn = $('[data-act=save]');
      btn.disabled = true;
      setStatus('Saving…');
      try {
        if (cropping) endCrop(true);
        const body = {
          title: $('.diagram-title').value.trim() || 'Diagram',
          data: Object.assign(canvasJson(), { width: W, height: H }),
          svg: await exportSvg(),
        };
        try { body.png = exportPng(1); } catch (_) { /* a tainted canvas: the SVG still saves */ }
        const resp = await ctx.apiFetch(base, { method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
        if (!resp.ok) {
          let detail = 'Could not save (HTTP ' + resp.status + ').';
          try { const j = await resp.json(); if (typeof j.detail === 'string') detail = j.detail; } catch (_) {}
          setStatus(detail, true);
          return;
        }
        record = await resp.json();
        saved = true;
        dirty = false;
        ctx.placeImage(record);
        setStatus('Saved. The diagram in the document is up to date.');
      } catch (e) {
        setStatus(e.message || String(e), true);
      } finally {
        btn.disabled = false;
      }
    }
    function close() {
      if (closed) return;
      if (dirty && !window.confirm('Close without saving the diagram?')) return;
      closed = true;
      ro.disconnect();
      document.removeEventListener('keydown', onKey, true);
      canvas.dispose();
      overlay.remove();
      document.body.classList.remove('diagram-editor-open');
      if (ctx.onClose) ctx.onClose(saved);
    }
    $('[data-act=save]').addEventListener('click', save);
    $('[data-act=close]').addEventListener('click', close);

    // For tests and the console: the live canvas.
    overlay.diagram = { canvas, save, undo, redo, exportSvg, exportPng };
    return overlay.diagram;
  }

  window.DiagramEditor = { open, TEMPLATES, loadFabric };
})();
