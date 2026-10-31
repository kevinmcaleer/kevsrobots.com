// The project's electronics workspace (issue #268): Snakie's wiring canvas,
// full screen over the project editor.
//
// Its parts come from the Bill of Materials: every catalog part the BOM
// table links is in the workspace's parts list, the microcontroller among
// them is the board, and a part added here (from the catalog) gets a row
// in the table. Saving stores the wiring (PUT /api/projects/{id}/wiring,
// a Snakie RobotDefinition) with an SVG of the canvas, which the document
// shows as an image under its Wiring heading. The link carries ?v= so the
// image refreshes after each save; the Markdown stays portable.
(function () {
  const API = 'https://projects.kevsrobots.com';
  const LIB = 'kevsrobots';

  function esc(s) { const d = document.createElement('div'); d.textContent = s == null ? '' : String(s); return d.innerHTML; }

  function svgUrl(projectId) {
    return API + '/api/projects/' + projectId + '/wiring.svg';
  }

  // The wiring image's link in `markdown`, as { index, length, url }.
  const IMAGE_RE = /!\[([^\]]*)\]\((https:\/\/projects\.kevsrobots\.com\/api\/projects\/(\d+)\/wiring\.svg(?:\?v=\d+)?)\)/;
  function findImage(markdown) {
    const m = IMAGE_RE.exec(markdown);
    return m ? { index: m.index, length: m[0].length, alt: m[1], url: m[2] } : null;
  }

  function isWiringSrc(src) {
    return /\/api\/projects\/\d+\/wiring\.svg/.test(src || '');
  }

  // A catalog part as the workspace's PartDefinition, or null.
  const defs = new Map(); // slug -> Promise<PartDefinition|null>
  function definition(slug) {
    if (!defs.has(slug)) {
      defs.set(slug, (async () => {
        const resp = await fetch(API + '/api/parts/' + encodeURIComponent(slug));
        if (!resp.ok) return null;
        return SnakieHost.partFor(await resp.json());
      })().catch(() => null));
    }
    return defs.get(slug);
  }

  const hasPins = def => (def.headers || []).some(h => (h.pins || []).length) || (def.connectors || []).length > 0;
  const isBoard = def => String(def.family || '').trim().toLowerCase() === 'microcontroller';

  function download(name, blob) {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = name;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }

  // The bundle, the saved wiring ({ data } or null) and the library: the
  // BOM's catalog parts that have electronics, plus any placed part the BOM
  // no longer links.
  async function partsOf(slugs) {
    const out = [];
    for (const slug of slugs) {
      const def = await definition(slug);
      if (def && hasPins(def)) out.push(def);
    }
    return out;
  }
  async function loadSaved(ctx, projectId) {
    const SP = await SnakieHost.load();
    const [saved, parts] = await Promise.all([
      ctx.apiFetch(API + '/api/projects/' + projectId + '/wiring').then(r => (r.ok ? r.json() : null)).catch(() => null),
      partsOf(ctx.bomSlugs()),
    ]);
    const robotParts = saved && saved.data && Array.isArray(saved.data.parts) ? saved.data.parts : [];
    const extra = robotParts.map(p => p.part).filter(slug => !parts.some(d => d.id === slug));
    parts.push(...await partsOf([...new Set(extra)]));
    return { SP, saved, library: { id: LIB, name: 'This project', parts } };
  }

  // The pinout table the saved wiring makes now (PinoutSync.fromGenerated),
  // without opening the workspace: the canvas is mounted out of sight just
  // long enough to ask it. '' when nothing is wired, null with no wiring.
  // ctx: { projectId, bomSlugs(), apiFetch() }.
  async function pinoutFor(ctx) {
    const { SP, saved, library } = await loadSaved(ctx, ctx.projectId);
    if (!saved || !saved.data) return null;
    const box = document.createElement('div');
    box.setAttribute('aria-hidden', 'true');
    box.style.cssText = 'position:fixed;left:-10000px;top:0;width:800px;height:600px;visibility:hidden;pointer-events:none';
    document.body.appendChild(box);
    try {
      const handle = SP.mountWiring(box, { robot: saved.data, libraries: [library], onChange: () => {} });
      try { return PinoutSync.fromGenerated(handle.pinoutMarkdown()); } finally { handle.unmount(); }
    } finally {
      box.remove();
    }
  }

  // Opens the workspace. ctx:
  //   projectId()           the project's id, saving it first if it's new
  //   markdown()            the document's text
  //   bomSlugs()            the catalog slugs the BOM table links, in order
  //   addBomRows(parts)     adds rows for [{ name, slug }]
  //   removeBomRows(slugs)  removes the rows linking these slugs
  //   placeImage(url)       puts (or refreshes) the wiring image in the text
  //   insertPinout(block)   puts the pinout table in the text (after the
  //                         wiring image), or refreshes the one there; false
  //                         when the writer said no to losing their edits
  //   hasPinout()           whether the text has a pinout table
  //   wiringSaved(block)    the pinout table the just-saved wiring makes;
  //                         returns { stale } for the table in the text
  //   apiFetch(url, opts)   fetch with the signed-in user's token
  async function open(ctx) {
    const projectId = await ctx.projectId();
    if (!projectId) return;
    const overlay = document.createElement('div');
    overlay.className = 'project-wiring';
    overlay.setAttribute('role', 'dialog');
    overlay.setAttribute('aria-modal', 'true');
    overlay.setAttribute('aria-label', 'Wiring');
    overlay.innerHTML = `
      <div class="project-wiring-bar">
        <strong class="me-2"><i class="fas fa-plug me-1"></i> Wiring</strong>
        <div class="btn-group btn-group-sm" role="group" aria-label="View">
          <button type="button" class="btn btn-outline-secondary active" data-view="lifelike">Life-like</button>
          <button type="button" class="btn btn-outline-secondary" data-view="schematic">Schematic</button>
        </div>
        <button type="button" class="btn btn-sm btn-outline-primary" data-act="add"><i class="fas fa-plus me-1"></i>Add parts</button>
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="pinout"><i class="fas fa-table me-1"></i>Insert pinout table</button>
        <div class="dropdown">
          <button type="button" class="btn btn-sm btn-outline-secondary dropdown-toggle" data-bs-toggle="dropdown" aria-expanded="false">Export</button>
          <ul class="dropdown-menu">
            <li><button type="button" class="dropdown-item" data-act="svg">SVG</button></li>
            <li><button type="button" class="dropdown-item" data-act="png">PNG</button></li>
          </ul>
        </div>
        <span class="project-wiring-status small text-muted ms-2" role="status"></span>
        <span class="ms-auto"></span>
        <button type="button" class="btn btn-sm btn-primary" data-act="save"><i class="fas fa-save me-1"></i>Save</button>
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="close">Close</button>
      </div>
      <div class="project-wiring-warnings" hidden></div>
      <div class="project-wiring-canvas"><div class="text-center py-5"><div class="spinner-border text-primary" role="status"></div></div></div>`;
    document.body.appendChild(overlay);
    document.body.classList.add('project-wiring-open');
    const $ = sel => overlay.querySelector(sel);
    const status = (text, bad) => {
      const s = $('.project-wiring-status');
      s.textContent = text || '';
      s.className = 'project-wiring-status small ms-2 ' + (bad ? 'text-danger' : 'text-muted');
    };

    let handle = null;
    let dirty = false;
    let closed = false;
    let library = { id: LIB, name: 'This project', parts: [] };
    let startSlugs = new Set();

    function close() {
      if (closed) return;
      if (dirty && !window.confirm('Close without saving the wiring?')) return;
      closed = true;
      if (handle) handle.unmount();
      overlay.remove();
      document.body.classList.remove('project-wiring-open');
      document.removeEventListener('keydown', onKey, true);
    }
    function onKey(e) {
      if (e.key === 'Escape' && !document.querySelector('.parts-catalog-overlay')) { e.stopPropagation(); close(); }
    }
    document.addEventListener('keydown', onKey, true);

    function showWarnings() {
      const box = $('.project-wiring-warnings');
      const list = handle ? handle.warnings() : [];
      box.hidden = !list.length;
      box.innerHTML = list.map(w => `<div class="project-wiring-warning is-${esc(w.kind)}"><i class="fas ${w.kind === 'short' || w.kind === 'rails' ? 'fa-bolt' : 'fa-triangle-exclamation'} me-1"></i>${esc(w.message)}</div>`).join('');
    }

    try {
      const loaded = await loadSaved(ctx, projectId);
      const SP = loaded.SP;
      const saved = loaded.saved;
      library = loaded.library;
      const parts = library.parts;

      let robot = saved && saved.data;
      const fresh = !robot;
      if (fresh) {
        const board = parts.find(isBoard);
        robot = { board: board ? board.id : undefined, parts: [], connections: [] };
      }
      startSlugs = new Set((robot.parts || []).map(p => p.part));
      const canvas = $('.project-wiring-canvas');
      canvas.innerHTML = '';
      handle = SP.mountWiring(canvas, {
        robot,
        libraries: [library],
        onChange: () => { dirty = true; status('Not saved'); showWarnings(); },
      });
      // A new wiring starts with every BOM part that isn't the board.
      if (fresh) {
        const place = parts.filter(d => d.id !== robot.board);
        if (place.length) handle.addParts(place.map(part => ({ libraryId: LIB, part })));
        dirty = false;
        status(parts.length ? '' : 'Link parts in the Bill of Materials, or add them here.');
      }
      showWarnings();
    } catch (e) {
      $('.project-wiring-canvas').innerHTML = '<div class="alert alert-danger m-3">' + esc(e.message || e) + '</div>';
      return;
    }

    overlay.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => {
      overlay.querySelectorAll('[data-view]').forEach(x => x.classList.toggle('active', x === b));
      handle.setView(b.dataset.view);
    }));

    $('[data-act=close]').addEventListener('click', close);

    $('[data-act=add]').addEventListener('click', async e => {
      const picked = await PartsCatalog.open({
        api: API, title: 'Add parts to the wiring', addLabel: 'Add to wiring', origin: e.currentTarget,
      });
      if (!picked.length) return;
      const defsPicked = [];
      const noPins = [];
      for (const p of picked) {
        const def = await definition(p.slug);
        if (def && hasPins(def)) defsPicked.push(def);
        else noPins.push(p.name);
      }
      if (defsPicked.length) {
        library = { ...library, parts: library.parts.concat(defsPicked.filter(d => !library.parts.some(x => x.id === d.id))) };
        handle.setLibraries([library]);
        handle.addParts(defsPicked.map(part => ({ libraryId: LIB, part })));
      }
      // Into the Bill of Materials too, pins or not.
      ctx.addBomRows(picked.map(p => ({ name: p.name, slug: p.slug })));
      status(noPins.length ? noPins.join(', ') + (noPins.length > 1 ? ' have' : ' has') + ' no pins yet, so ' + (noPins.length > 1 ? 'they were' : 'it was') + ' only added to the Bill of Materials.' : 'Added. Not saved');
    });

    const pinoutBtn = $('[data-act=pinout]');
    const pinoutLabel = () => {
      pinoutBtn.innerHTML = '<i class="fas fa-table me-1"></i>' + (ctx.hasPinout() ? 'Update pinout table' : 'Insert pinout table');
    };
    pinoutLabel();
    pinoutBtn.addEventListener('click', () => {
      // Just the table: it goes under the document's own Wiring heading.
      const block = PinoutSync.fromGenerated(handle.pinoutMarkdown());
      if (!block) { status('Wire something first: the pinout table lists the board pins in use.', true); return; }
      const had = ctx.hasPinout();
      if (ctx.insertPinout(block) === false) return;
      status(had ? 'Pinout table updated in the document.' : 'Pinout table inserted into the document.');
      pinoutLabel();
    });

    $('[data-act=svg]').addEventListener('click', async () => {
      download('wiring.svg', new Blob([await handle.exportSvg()], { type: 'image/svg+xml' }));
    });
    $('[data-act=png]').addEventListener('click', async () => {
      try { download('wiring.png', await handle.exportPng(2)); } catch (err) { status(err.message || String(err), true); }
    });

    $('[data-act=save]').addEventListener('click', async () => {
      const btn = $('[data-act=save]');
      btn.disabled = true;
      status('Saving…');
      try {
        const data = handle.robot();
        const svg = await handle.exportSvg();
        const resp = await ctx.apiFetch(API + '/api/projects/' + projectId + '/wiring', {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ data, svg }),
        });
        if (!resp.ok) {
          let detail = 'Could not save (HTTP ' + resp.status + ').';
          try { const j = await resp.json(); if (j.detail) detail = typeof j.detail === 'string' ? j.detail : detail; } catch (_) {}
          status(detail, true);
          return;
        }
        dirty = false;
        ctx.placeImage(svgUrl(projectId) + '?v=' + Date.now());
        // Parts taken out of the wiring: offer to take them off the BOM.
        const now = new Set((data.parts || []).map(p => p.part));
        const gone = [...startSlugs].filter(s => !now.has(s) && ctx.bomSlugs().includes(s));
        if (gone.length) {
          const names = gone.map(s => (library.parts.find(d => d.id === s) || { name: s }).name);
          if (window.confirm('Remove ' + names.join(', ') + ' from the Bill of Materials too?')) ctx.removeBomRows(gone);
        }
        startSlugs = now;
        const pin = ctx.hasPinout() ? ctx.wiringSaved(PinoutSync.fromGenerated(handle.pinoutMarkdown())) : null;
        status(pin && pin.stale
          ? 'Saved. The pinout table in the document no longer matches: Update pinout table to refresh it.'
          : 'Saved. The diagram in the document is up to date.', pin && pin.stale);
      } catch (err) {
        status(err.message || String(err), true);
      } finally {
        btn.disabled = false;
      }
    });
  }

  window.ProjectWiring = { open, pinoutFor, findImage, isWiringSrc, svgUrl };
})();
