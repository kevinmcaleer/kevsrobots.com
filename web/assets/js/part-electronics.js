// The Electronics tab of /parts/edit.html (issue #269): Snakie's Part
// Editor, mounted over one of the part's photos. Pins, pin types and
// capabilities, the outline and the derived schematic symbol are saved as
// the part's definition (PUT /api/parts/{slug}/definition, a revision like
// any other edit). A photo picked or changed in the editor is uploaded as a
// new part photo, which needs a licence. The part's 3-D model is uploaded
// here too.
(function () {
  const API = 'https://projects.kevsrobots.com';

  const LICENCES = [
    ['', 'Licence for a new photo…'],
    ['own', 'My own photo'],
    ['cc-by', 'CC BY (credit the source)'],
    ['permission-granted:adafruit', 'Adafruit (permission granted)'],
    ['permission-granted:pimoroni', 'Pimoroni (permission granted)'],
    ['permission-granted:digikey', 'DigiKey (permission granted)'],
  ];

  function esc(s) { const d = document.createElement('div'); d.textContent = s == null ? '' : String(s); return d.innerHTML; }

  async function errorOf(resp, fallback) {
    try { const j = await resp.json(); if (j && j.detail) return typeof j.detail === 'string' ? j.detail : fallback; } catch (_) {}
    return fallback;
  }

  // A part with no pins yet starts at its photo's shape, not Snakie's tall
  // default outline (which stretched the photo to fit). A width in mm is
  // kept and the height follows the photo. Once it has pins the outline
  // is the one they were placed on, so it's left alone.
  function hasPins(def) {
    return (def.headers || []).some(h => (h.pins || []).length > 0) || (def.connectors || []).some(c => (c.pins || []).length > 0);
  }
  function fitToPhoto(def) {
    if (!def.imageData || hasPins(def)) return def;
    return new Promise(resolve => {
      const img = new Image();
      img.onload = () => {
        const w = img.naturalWidth, h = img.naturalHeight;
        if (!(w > 0 && h > 0)) { resolve(def); return; }
        const aspect = w / h;
        const width = def.dimensions && def.dimensions.width;
        const out = Object.assign({}, def, { aspect });
        if (width > 0) out.dimensions = Object.assign({}, def.dimensions, { width, height: Math.round(width / aspect * 10) / 10 });
        else delete out.dimensions;
        resolve(out);
      };
      img.onerror = () => resolve(def);
      img.src = def.imageData;
    });
  }

  // Draws the tab into `root` for `part` (GET /api/parts/{slug}).
  // `onSaved(part)` gets the part after each save.
  function create(root, part, opts) {
    opts = opts || {};
    let current = part;
    let photo = SnakieHost.pinPhoto(part);
    let mounted = null;

    root.innerHTML = `
      <div class="d-flex flex-wrap align-items-center gap-2 mb-2">
        <span class="small text-muted me-1">Pins sit on:</span>
        <div class="d-flex flex-wrap gap-1" data-el="photos"></div>
        <select class="form-select form-select-sm w-auto ms-auto" data-el="licence" aria-label="Licence for a new photo">
          ${LICENCES.map(([v, l]) => `<option value="${esc(v)}">${esc(l)}</option>`).join('')}
        </select>
        <input type="text" class="form-control form-control-sm w-auto" data-el="source" maxlength="200" placeholder="Source (for CC BY)" aria-label="Photo source">
      </div>
      <p class="form-text mt-0 mb-2">Choose the photo to place the pins over, or import a new one in the editor. A new photo is added to the part's photos when you save, so pick its licence first.</p>
      <div class="part-electronics-editor" data-el="editor"><div class="text-center py-5"><div class="spinner-border text-primary" role="status"></div></div></div>
      <div class="d-flex flex-wrap align-items-center gap-2 mt-3">
        <strong class="small"><i class="fas fa-cube me-1"></i> 3-D model</strong>
        <span class="small text-muted" data-el="mesh-status"></span>
        <input type="file" class="form-control form-control-sm w-auto" data-el="mesh-file" accept=".stl,.glb,.obj">
        <button type="button" class="btn btn-sm btn-outline-primary" data-el="mesh-upload"><i class="fas fa-upload me-1"></i>Upload</button>
        <button type="button" class="btn btn-sm btn-outline-danger d-none" data-el="mesh-remove">Remove</button>
      </div>
      <div class="small mt-2" data-el="status" role="status"></div>`;
    const el = name => root.querySelector('[data-el="' + name + '"]');

    function setStatus(text, ok) {
      const s = el('status');
      s.textContent = text || '';
      s.className = 'small mt-2 ' + (ok ? 'text-success' : 'text-danger');
    }

    function renderPhotos() {
      const photos = current.photos || [];
      el('photos').innerHTML = photos.length
        ? photos.map(p => `
          <button type="button" class="part-electronics-photo${photo && p.id === photo.id ? ' is-active' : ''}" data-id="${p.id}" title="${esc(p.title || 'Photo ' + p.id)}">
            <img src="${esc(SnakieHost.photoUrl(p))}" alt="${esc(p.title || '')}" loading="lazy">
          </button>`).join('')
        : '<span class="small text-muted fst-italic">No photos yet: import one in the editor.</span>';
      el('photos').querySelectorAll('button').forEach(b => b.addEventListener('click', () => {
        const next = photos.find(p => String(p.id) === b.dataset.id);
        if (!next || (photo && next.id === photo.id)) return;
        if (mounted && !window.confirm('Switch photos? Pins you haven’t saved are lost.')) return;
        photo = next;
        renderPhotos();
        mount();
      }));
    }

    function renderMesh() {
      el('mesh-status').textContent = current.has_mesh
        ? 'This part has a model (' + String(current.mesh_format || '').toUpperCase() + '). Upload another to replace it.'
        : 'None yet. STL, GLB or OBJ.';
      el('mesh-remove').classList.toggle('d-none', !current.has_mesh);
    }

    async function uploadPhoto(blob) {
      const licence = el('licence').value;
      if (!licence) return { error: 'Choose a licence for the new photo, above the editor, then save again.' };
      const type = blob.type || 'image/png';
      const ext = type.split('/')[1] === 'jpeg' ? 'jpg' : (type.split('/')[1] || 'png').replace('svg+xml', 'svg');
      const fd = new FormData();
      fd.append('file', blob, 'pinout.' + ext);
      fd.append('title', 'Pinout photo');
      fd.append('licence', licence);
      const source = el('source').value.trim();
      if (source) fd.append('source', source);
      const resp = await ProjectAuth.apiFetch(API + '/api/parts/' + encodeURIComponent(current.slug) + '/photos', { method: 'POST', body: fd });
      if (!resp.ok) return { error: await errorOf(resp, 'The photo didn’t upload (HTTP ' + resp.status + ').') };
      return { photo: await resp.json() };
    }

    async function save(def, image) {
      setStatus('');
      let photoId = photo ? photo.id : null;
      if (image) {
        const up = await uploadPhoto(image);
        if (up.error) return { ok: false, error: up.error };
        photoId = up.photo.id;
      }
      const body = Object.assign({}, def, { id: current.slug, name: current.name });
      delete body.imageData;
      delete body.image;
      if (photoId != null) body.photo_id = photoId;
      else delete body.photo_id;
      const resp = await ProjectAuth.apiFetch(API + '/api/parts/' + encodeURIComponent(current.slug) + '/definition', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      if (resp.status === 400 && !image) return { ok: true }; // nothing changed
      if (resp.status === 403) return { ok: false, error: 'Your account needs to be 14 days old to edit parts.' };
      if (!resp.ok) return { ok: false, error: await errorOf(resp, 'Could not save (HTTP ' + resp.status + ').') };
      current = await resp.json();
      if (image) photo = (current.photos || []).find(p => p.id === photoId) || photo;
      renderPhotos();
      setStatus('Saved. The change is in the part’s revision history.', true);
      if (opts.onSaved) opts.onSaved(current);
      return { ok: true };
    }

    async function mount() {
      const box = el('editor');
      if (mounted) { mounted.unmount(); mounted = null; }
      try {
        const SP = await SnakieHost.load();
        const def = await fitToPhoto(await SnakieHost.partFor(current, { photo, inline: true }));
        mounted = SP.mountPartEditor(box, {
          part: def,
          libraryId: 'kevsrobots',
          libraryName: 'kevsrobots.com parts',
          onSave: save,
          onClose: () => { if (opts.onClose) opts.onClose(); },
        });
      } catch (e) {
        box.innerHTML = '<div class="alert alert-danger m-3">' + esc(e.message || e) + '</div>';
      }
    }

    el('mesh-upload').addEventListener('click', async () => {
      setStatus('');
      const file = el('mesh-file').files && el('mesh-file').files[0];
      if (!file) { setStatus('Choose an STL, GLB or OBJ file.'); return; }
      const fd = new FormData();
      fd.append('file', file);
      const resp = await ProjectAuth.apiFetch(API + '/api/parts/' + encodeURIComponent(current.slug) + '/mesh', { method: 'POST', body: fd });
      if (!resp.ok) { setStatus(await errorOf(resp, 'Upload failed (HTTP ' + resp.status + ').')); return; }
      current = await resp.json();
      el('mesh-file').value = '';
      renderMesh();
      setStatus('Model uploaded.', true);
      if (opts.onSaved) opts.onSaved(current);
    });
    el('mesh-remove').addEventListener('click', async () => {
      if (!window.confirm('Remove the 3-D model from this part? An older revision can bring it back.')) return;
      const resp = await ProjectAuth.apiFetch(API + '/api/parts/' + encodeURIComponent(current.slug) + '/mesh', { method: 'DELETE' });
      if (!resp.ok) { setStatus(await errorOf(resp, 'Could not remove (HTTP ' + resp.status + ').')); return; }
      current = await resp.json();
      renderMesh();
      setStatus('Model removed.', true);
      if (opts.onSaved) opts.onSaved(current);
    });

    renderPhotos();
    renderMesh();
    mount();
    return {
      unmount() { if (mounted) mounted.unmount(); mounted = null; },
    };
  }

  window.PartElectronics = { create };
})();
