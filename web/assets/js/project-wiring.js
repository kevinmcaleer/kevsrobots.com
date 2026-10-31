// The project's electronics workspace (issue #268): Snakie's wiring canvas,
// full screen over the project editor.
//
// Its parts come from the Bill of Materials. A new wiring starts with the
// BOM's catalog parts that have pins, and its microcontroller as the board
// (no board at all when the BOM has none). The Bill of Materials panel
// lists every row: unlinked rows and parts with no pins yet go on the
// canvas as placeholder cards (wiring-bom.js), and a row can be linked to
// a catalog part from there. A part added here (from the catalog) gets a
// row in the table. Saving stores the wiring (PUT /api/projects/{id}/wiring,
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

  const { BOM_LIB, hasPins, isBoard } = WiringBom;

  // The server keeps only PNG/JPEG/GIF/WebP pictures in a saved diagram
  // (svg_safety.clean_svg), so the placeholder cards, which are SVG, are
  // drawn into PNGs first.
  const pngs = new Map();
  function svgToPng(src) {
    if (!pngs.has(src)) {
      pngs.set(src, new Promise(resolve => {
        const img = new Image();
        img.onload = () => {
          const c = document.createElement('canvas');
          c.width = (img.naturalWidth || 300) * 2;
          c.height = (img.naturalHeight || 200) * 2;
          c.getContext('2d').drawImage(img, 0, 0, c.width, c.height);
          try { resolve(c.toDataURL('image/png')); } catch (_) { resolve(null); }
        };
        img.onerror = () => resolve(null);
        img.src = src;
      }));
    }
    return pngs.get(src);
  }
  async function rasterizeSvgImages(svgText) {
    const re = /(href=")(data:image\/svg\+xml[^"]*)"/g;
    const srcs = [...new Set([...svgText.matchAll(re)].map(m => m[2].replace(/&amp;/g, '&')))];
    if (!srcs.length) return svgText;
    const out = new Map();
    await Promise.all(srcs.map(async src => out.set(src, await svgToPng(src))));
    return svgText.replace(re, (all, pre, src) => {
      const png = out.get(src.replace(/&amp;/g, '&'));
      return png ? pre + png + '"' : all;
    });
  }

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
  // BOM's catalog parts, pins or not (one without pins is drawn as a card
  // until it has some), plus any placed part the BOM no longer links.
  async function partsOf(slugs) {
    const out = [];
    for (const slug of slugs) {
      const def = await definition(slug);
      if (def) out.push(WiringBom.withFallbackImage(def));
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
    const extra = robotParts.filter(p => p.lib !== BOM_LIB).map(p => p.part).filter(slug => !parts.some(d => d.id === slug));
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
  //   bomRows()             every BOM row as [{ name, slug|null }], in order
  //   linkBomRow(name, part) links the unlinked row `name` to part { name, slug } in the
  //                         BOM table; false when the table was left alone
  //   createPart(name, origin)  opens the "New part" dialog for the row `name`;
  //                         resolves to the part { name, slug, existing }, or null
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
  // where: where it opens. { block, onForcedClose(fn) }: in the document,
  // in the editor's host block (MarkdownEditor openBlock: { element,
  // close() }), with a button to go full screen and back; the host calls
  // the fn onForcedClose gives it when the editor closes the block itself.
  // Without a block it opens full screen, as it used to.
  async function open(ctx, where) {
    const block = where && where.block ? where.block : null;
    const projectId = await ctx.projectId();
    if (!projectId) { if (block) block.close(); return; }
    // The workspace. In the page it sits in the block; full screen, it
    // moves into an overlay on the body (the same element, so the canvas
    // keeps everything), and back.
    const overlay = document.createElement('div');
    overlay.className = 'project-wiring';
    overlay.setAttribute('aria-label', 'Wiring');
    let fullOverlay = null;
    let mode = block ? 'inline' : 'full';
    overlay.innerHTML = `
      <div class="project-wiring-bar">
        <strong class="me-2"><i class="fas fa-plug me-1"></i> Wiring</strong>
        <div class="btn-group btn-group-sm" role="group" aria-label="View">
          <button type="button" class="btn btn-outline-secondary active" data-view="lifelike">Life-like</button>
          <button type="button" class="btn btn-outline-secondary" data-view="schematic">Schematic</button>
        </div>
        <button type="button" class="btn btn-sm btn-outline-primary" data-act="add"><i class="fas fa-plus me-1"></i>Add parts</button>
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="bom" aria-pressed="false"><i class="fas fa-list me-1"></i><span class="pw-full-label">Bill of Materials</span><span class="pw-inline-label">BOM</span> <span class="badge text-bg-light" data-bom-count></span></button>
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="pinout"><i class="fas fa-table me-1"></i>Insert pinout table</button>
        <div class="dropdown">
          <button type="button" class="btn btn-sm btn-outline-secondary dropdown-toggle" data-bs-toggle="dropdown" data-bs-popper-config='{"strategy":"fixed"}' aria-expanded="false">Export</button>
          <ul class="dropdown-menu">
            <li><button type="button" class="dropdown-item" data-act="svg">SVG</button></li>
            <li><button type="button" class="dropdown-item" data-act="png">PNG</button></li>
          </ul>
        </div>
        <span class="project-wiring-status small text-muted ms-2" role="status"></span>
        <span class="ms-auto"></span>
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="mode"${block ? '' : ' hidden'}></button>
        <button type="button" class="btn btn-sm btn-primary" data-act="save"><i class="fas fa-save me-1"></i>Save</button>
        <button type="button" class="btn btn-sm btn-outline-secondary" data-act="close">Close</button>
      </div>
      <div class="project-wiring-warnings" hidden></div>
      <div class="project-wiring-body">
        <div class="project-wiring-canvas"><div class="text-center py-5"><div class="spinner-border text-primary" role="status"></div></div></div>
        <aside class="project-wiring-bom" aria-label="Bill of Materials"></aside>
      </div>`;
    // The Bill of Materials panel: shown full screen, hidden in the page
    // (the column is narrow), until the writer says otherwise in that mode.
    const bomShown = { inline: false, full: true };
    function applyBom() {
      const on = bomShown[mode];
      overlay.classList.toggle('bom-hidden', !on);
      const b = overlay.querySelector('[data-act=bom]');
      b.classList.toggle('active', on);
      b.setAttribute('aria-pressed', String(on));
    }
    function setMode(next) {
      if (!block) next = 'full';
      mode = next;
      overlay.classList.toggle('is-inline', mode === 'inline');
      overlay.classList.toggle('is-full', mode === 'full');
      const btn = overlay.querySelector('[data-act=mode]');
      btn.innerHTML = mode === 'full'
        ? '<i class="fas fa-compress me-1"></i>Back to page'
        : '<i class="fas fa-expand me-1"></i>Full screen';
      btn.title = mode === 'full' ? 'Back to the page (Esc)' : 'Full screen';
      if (mode === 'full') {
        if (!fullOverlay) {
          fullOverlay = document.createElement('div');
          fullOverlay.className = 'project-wiring-overlay';
          fullOverlay.setAttribute('role', 'dialog');
          fullOverlay.setAttribute('aria-modal', 'true');
          fullOverlay.setAttribute('aria-label', 'Wiring');
        }
        fullOverlay.appendChild(overlay);
        document.body.appendChild(fullOverlay);
        document.body.classList.add('project-wiring-open');
      } else {
        block.element.appendChild(overlay);
        if (fullOverlay) fullOverlay.remove();
        document.body.classList.remove('project-wiring-open');
      }
      applyBom();
    }
    setMode(mode);
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
    let bomLibrary = { id: BOM_LIB, name: 'Bill of Materials', parts: [] };
    let startSlugs = new Set();
    const libraries = () => [library, bomLibrary];

    // Takes the workspace away, with no asking: also what happens when the
    // editor closes the block itself (another one opened, or it went).
    function teardown() {
      if (closed) return;
      closed = true;
      if (handle) handle.unmount();
      overlay.remove();
      if (fullOverlay) fullOverlay.remove();
      document.body.classList.remove('project-wiring-open');
      document.removeEventListener('keydown', onKey, true);
      window.removeEventListener('focus', onFocus);
      if (block) block.close();
    }
    function close() {
      if (closed) return;
      if (dirty && !window.confirm('Close without saving the wiring?')) return;
      teardown();
    }
    if (where && where.onForcedClose) where.onForcedClose(teardown);
    // Esc goes back to the page from full screen (or closes the full
    // screen only workspace). In the page, Esc is the editor's.
    function onKey(e) {
      if (e.key !== 'Escape' || mode !== 'full' || document.querySelector('.parts-catalog-overlay')) return;
      if (overlay.querySelector('.dropdown-menu.show')) return;
      e.stopPropagation();
      if (block) setMode('inline'); else close();
    }
    document.addEventListener('keydown', onKey, true);
    $('[data-act=close]').addEventListener('click', close);
    $('[data-act=mode]').addEventListener('click', () => setMode(mode === 'full' ? 'inline' : 'full'));

    function showWarnings() {
      const box = $('.project-wiring-warnings');
      // One line each: a part with two GND pins (both QWIIC ports, say)
      // would say the same thing twice.
      const list = handle ? handle.warnings().filter((w, i, all) => all.findIndex(x => x.message === w.message) === i) : [];
      box.hidden = !list.length;
      box.innerHTML = list.map(w => `<div class="project-wiring-warning is-${esc(w.kind)}"><i class="fas ${w.kind === 'short' || w.kind === 'rails' ? 'fa-bolt' : 'fa-triangle-exclamation'} me-1"></i>${esc(w.message)}</div>`).join('');
    }

    const defOf = slug => library.parts.find(d => d.id === slug) || null;

    // Placeholders: one per unlinked row, and one for each placed
    // placeholder whose row has gone (it keeps its label).
    function syncPlaceholders(robot) {
      const want = new Map();
      ctx.bomRows().filter(r => !r.slug).forEach(r => want.set(WiringBom.placeholderId(r.name), r.name));
      (robot.parts || []).filter(p => p.lib === BOM_LIB && !want.has(p.part)).forEach(p => want.set(p.part, p.label || p.part));
      bomLibrary = { id: BOM_LIB, name: 'Bill of Materials', parts: [...want].map(([id, name]) => Object.assign(WiringBom.placeholderPart(name), { id })) };
    }

    function markDirty(text) { dirty = true; status(text || 'Not saved'); showWarnings(); drawBom(); }

    try {
      const loaded = await loadSaved(ctx, projectId);
      const SP = loaded.SP;
      const saved = loaded.saved;
      library = loaded.library;
      const parts = library.parts;

      let robot = saved && saved.data;
      const fresh = !robot;
      if (fresh) {
        // The board is the BOM's microcontroller. None in the BOM, no board.
        const board = parts.find(d => isBoard(d) && hasPins(d));
        robot = { parts: [], connections: [] };
        if (board) robot.board = board.id;
      }
      // Placeholders whose rows were linked since: the real part, in place.
      let relinked = 0;
      for (const l of WiringBom.linkedSince(robot, ctx.bomRows())) {
        if (!defOf(l.slug)) { const d = await definition(l.slug); if (d) library.parts.push(WiringBom.withFallbackImage(d)); }
        robot = WiringBom.relink(robot, BOM_LIB, l.from, LIB, l.slug, (defOf(l.slug) || {}).name);
        relinked++;
      }
      syncPlaceholders(robot);
      startSlugs = new Set((robot.parts || []).map(p => p.part));
      const canvas = $('.project-wiring-canvas');
      canvas.innerHTML = '';
      handle = SP.mountWiring(canvas, {
        robot,
        libraries: libraries(),
        emptyHint: 'Add parts from the Bill of Materials list, or with Add parts.',
        onChange: () => markDirty(),
      });
      // A new wiring starts with the BOM's parts that have pins.
      if (fresh) {
        const place = parts.filter(d => d.id !== robot.board && hasPins(d));
        if (place.length) handle.addParts(place.map(part => ({ libraryId: LIB, part })));
        dirty = false;
        status(ctx.bomRows().length ? '' : 'Add parts here, and they go in the Bill of Materials too.');
      }
      if (relinked) markDirty(relinked + (relinked === 1 ? ' placeholder was' : ' placeholders were') + ' swapped for the part linked in the Bill of Materials. Not saved');
      showWarnings();
      drawBom();
      window.addEventListener('focus', onFocus);
    } catch (e) {
      $('.project-wiring-canvas').innerHTML = '<div class="alert alert-danger m-3">' + esc(e.message || e) + '</div>';
      return;
    }

    // --- The Bill of Materials panel ---

    function addToLibrary(defsIn) {
      const fresh = defsIn.filter(d => d && !library.parts.some(x => x.id === d.id));
      const changed = defsIn.filter(d => d && library.parts.some(x => x.id === d.id && x !== d));
      if (!fresh.length && !changed.length) return;
      library = { ...library, parts: library.parts.map(x => changed.find(d => d.id === x.id) || x).concat(fresh) };
      handle.setLibraries(libraries());
    }

    // Puts parts on the canvas. A microcontroller with pins becomes the
    // board when there isn't one.
    function place(items) {
      const robot = handle.robot();
      let boardFree = !robot.board;
      const parts = [];
      for (const { lib, def } of items) {
        if (boardFree && lib === LIB && isBoard(def) && hasPins(def)) { handle.setBoard(def.id); boardFree = false; continue; }
        parts.push({ libraryId: lib, part: def });
      }
      if (parts.length) handle.addParts(parts);
    }

    function entryDef(e) {
      return e.lib === BOM_LIB ? bomLibrary.parts.find(d => d.id === e.part) : defOf(e.slug);
    }

    function addEntries(list) {
      const items = list.map(e => ({ lib: e.lib, def: entryDef(e) })).filter(i => i.def);
      if (items.length) place(items);
    }

    // Links the unlinked row `e` to a catalog part, and swaps its
    // placeholder for that part where it stands.
    async function linkEntry(e, part) {
      if (!part || !part.slug) return;
      status('Linking…');
      const inTable = ctx.linkBomRow(e.name, part);
      const def = await definition(part.slug);
      if (def) addToLibrary([WiringBom.withFallbackImage(def)]);
      const before = handle.robot();
      let robot = WiringBom.relink(before, BOM_LIB, e.part, LIB, part.slug, def ? def.name : part.name);
      if (robot !== before && def && isBoard(def) && hasPins(def) && !robot.board) {
        // The board isn't a placed part: the placeholder makes way for it.
        robot = { ...robot, parts: robot.parts.filter(p => !(p.lib === LIB && p.part === part.slug)) };
        handle.setRobot(robot);
        handle.setBoard(def.id);
      } else if (robot !== before) {
        handle.setRobot(robot);
      }
      syncPlaceholders(handle.robot());
      handle.setLibraries(libraries());
      const pins = def && hasPins(def);
      markDirty('Linked “' + e.name + '” to ' + (def ? def.name : part.name) + '.' +
        (inTable ? ' The Bill of Materials table links it too.' : ' The Bill of Materials table was left as it is: its row links a shop page and has no Notes column to move that link to. Link the row in the table by hand.') +
        (pins ? '' : ' It has no pins in the catalog yet: use Add pins.') + ' Not saved');
    }

    async function pickLink(e, origin) {
      const picked = await PartsCatalog.open({
        api: API, query: e.name, origin,
        title: 'Link “' + e.name + '” to a catalog part', addLabel: 'Link',
      });
      if (picked.length) await linkEntry(e, picked[0]);
    }

    async function newPart(e, origin) {
      try {
        const part = await ctx.createPart(e.name, origin);
        if (!part) return;
        await linkEntry(e, part);
        if (!part.existing) status('“' + part.name + '” is linked to its new catalog part. Use Add pins when you want to wire it up.');
      } catch (err) {
        status(err.message || String(err), true);
      }
    }

    function pinsUrl(slug) {
      return '/parts/edit.html?slug=' + encodeURIComponent(slug) + '&tab=electronics';
    }

    // Back from adding a part's pins in another tab: read the pin-less
    // parts again, and redraw the ones that have pins now.
    let refreshing = false;
    async function onFocus() {
      if (refreshing || closed || !handle) return;
      const stale = library.parts.filter(d => !hasPins(d)).map(d => d.id);
      if (!stale.length) return;
      refreshing = true;
      try {
        stale.forEach(slug => defs.delete(slug));
        const fresh = (await partsOf(stale)).filter(hasPins);
        if (fresh.length) {
          addToLibrary(fresh);
          drawBom();
          status(fresh.map(d => d.name).join(', ') + (fresh.length > 1 ? ' have' : ' has') + ' pins now.');
        }
      } finally {
        refreshing = false;
      }
    }

    function drawBom() {
      const box = $('.project-wiring-bom');
      if (!box || !handle) return;
      const robot = handle.robot();
      const rows = ctx.bomRows();
      const list = WiringBom.entries(rows, robot, new Map(library.parts.map(d => [d.id, d])));
      const count = overlay.querySelector('[data-bom-count]');
      if (count) count.textContent = list.length ? list.filter(e => e.onCanvas).length + '/' + list.length : '';
      const waiting = list.filter(e => !e.onCanvas);
      const thumb = e => {
        const d = entryDef(e);
        const src = d && d.imageData && (e.linked || e.lib === BOM_LIB) ? d.imageData : '';
        return src ? '<img src="' + esc(src) + '" alt="">' : '<i class="fas fa-microchip" aria-hidden="true"></i>';
      };
      const state = e => {
        if (e.isTheBoard) return '<span class="wb-state is-board"><i class="fas fa-check"></i> The board</span>';
        const placed = e.onCanvas ? '<i class="fas fa-check"></i> On the canvas' : '';
        const why = !e.linked ? 'not linked' : !e.def ? 'not in the catalog' : !e.pins ? 'no pins yet' : '';
        if (!why) return placed ? '<span class="wb-state is-placed">' + placed + '</span>' : '';
        const cls = e.linked && e.def ? 'is-nopins' : 'is-unlinked';
        return '<span class="wb-state ' + cls + '">' + (placed ? placed + ', ' + why : why.charAt(0).toUpperCase() + why.slice(1)) + '</span>';
      };
      const acts = (e, i) => {
        const out = [];
        if (!e.onCanvas && (e.def || !e.linked)) {
          const asBoard = e.board && e.pins && !robot.board;
          out.push('<button type="button" class="btn btn-sm btn-outline-primary" data-bom-add="' + i + '">' + (asBoard ? 'Add as board' : 'Add') + '</button>');
        }
        if (!e.linked) {
          out.push('<button type="button" class="btn btn-sm btn-outline-secondary" data-bom-link="' + i + '"><i class="fas fa-link me-1"></i>Link…</button>');
          out.push('<button type="button" class="btn btn-sm btn-outline-secondary" data-bom-new="' + i + '" title="Not in the catalog? Add it as a new part, with pictures and links" aria-label="Add “' + esc(e.name) + '” to the catalog as a new part"><i class="fas fa-plus"></i></button>');
        } else if (e.def && !e.pins) {
          out.push('<a class="btn btn-sm btn-outline-secondary" target="_blank" rel="noopener" href="' + esc(pinsUrl(e.slug)) + '" title="Opens the part’s electronics editor in a new tab"><i class="fas fa-up-right-from-square me-1"></i>Add pins</a>');
        }
        return out.join('');
      };
      // The board, when it isn't one of the BOM's parts (an old wiring's
      // Pico, say): it can be taken away.
      const strayBoard = robot.board && !list.some(e => e.isTheBoard)
        ? '<li class="wb-row"><span class="wb-thumb"><i class="fas fa-microchip" aria-hidden="true"></i></span>' +
          '<span class="wb-text"><span class="wb-name">' + esc((defOf(robot.board) || {}).name || (robot.board === 'pico2w' ? 'Raspberry Pi Pico 2 W' : robot.board)) + '</span>' +
          '<span class="wb-state is-nopins">The board, but not in the Bill of Materials</span></span>' +
          '<span class="wb-acts"><button type="button" class="btn btn-sm btn-outline-danger" data-bom-unboard>Remove board</button></span></li>'
        : '';
      const unlinked = list.filter(e => !e.linked).length;
      box.innerHTML =
        '<div class="wb-head"><strong>Bill of Materials</strong>' +
        (waiting.length > 1 ? '<button type="button" class="btn btn-sm btn-link p-0 ms-auto" data-bom-all>Add all ' + waiting.length + '</button>' : '') +
        '</div>' +
        (list.length || strayBoard
          ? '<ul class="wb-list">' + strayBoard + list.map((e, i) =>
              '<li class="wb-row' + (e.onCanvas ? ' is-placed' : '') + '"><span class="wb-thumb">' + thumb(e) + '</span>' +
              '<span class="wb-text"><span class="wb-name">' + esc(e.name) + '</span>' + state(e) + '</span>' +
              '<span class="wb-acts">' + acts(e, i) + '</span></li>').join('') + '</ul>'
          : '<p class="wb-empty">The Bill of Materials is empty. Parts you add here go into it.</p>') +
        (unlinked ? '<p class="wb-foot"><strong>Link…</strong> a row to its part in the catalog to give it pins and a photo. Not in the catalog? <i class="fas fa-plus"></i> adds it as a new part, for everyone to use.</p>' : '');
      box.querySelectorAll('[data-bom-add]').forEach(b => b.addEventListener('click', () => addEntries([list[+b.dataset.bomAdd]])));
      box.querySelectorAll('[data-bom-link]').forEach(b => b.addEventListener('click', ev => pickLink(list[+b.dataset.bomLink], ev.currentTarget)));
      const all = box.querySelector('[data-bom-all]');
      if (all) all.addEventListener('click', () => addEntries(waiting.filter(e => e.def || !e.linked)));
      const unboard = box.querySelector('[data-bom-unboard]');
      if (unboard) unboard.addEventListener('click', () => {
        if (window.confirm('Take the board off the wiring? Its wires go too.')) handle.setBoard(null);
      });
      box.querySelectorAll('[data-bom-new]').forEach(b => b.addEventListener('click', ev => newPart(list[+b.dataset.bomNew], ev.currentTarget)));
    }

    overlay.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => {
      overlay.querySelectorAll('[data-view]').forEach(x => x.classList.toggle('active', x === b));
      handle.setView(b.dataset.view);
    }));

    $('[data-act=bom]').addEventListener('click', () => {
      bomShown[mode] = !bomShown[mode];
      applyBom();
    });

    $('[data-act=add]').addEventListener('click', async e => {
      const picked = await PartsCatalog.open({
        api: API, title: 'Add parts to the wiring', addLabel: 'Add to wiring', origin: e.currentTarget,
      });
      if (!picked.length) return;
      const defsPicked = [];
      const noPins = [];
      for (const p of picked) {
        const def = await definition(p.slug);
        if (!def) continue;
        defsPicked.push(WiringBom.withFallbackImage(def));
        if (!hasPins(def)) noPins.push(p.name);
      }
      // Into the Bill of Materials too.
      ctx.addBomRows(picked.map(p => ({ name: p.name, slug: p.slug })));
      if (defsPicked.length) {
        addToLibrary(defsPicked);
        place(defsPicked.map(def => ({ lib: LIB, def })));
      }
      status(noPins.length ? noPins.join(', ') + (noPins.length > 1 ? ' have' : ' has') + ' no pins in the catalog yet, so ' + (noPins.length > 1 ? 'they are cards' : 'it is a card') + ' you can\u2019t wire. Not saved' : 'Added. Not saved');
    });

    const pinoutBtn = $('[data-act=pinout]');
    const pinoutLabel = () => {
      const verb = ctx.hasPinout() ? 'Update' : 'Insert';
      pinoutBtn.innerHTML = '<i class="fas fa-table me-1"></i><span class="pw-full-label">' + verb + ' pinout table</span><span class="pw-inline-label">Pinout</span>';
      pinoutBtn.title = verb + ' pinout table';
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
        const svg = await rasterizeSvgImages(await handle.exportSvg());
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
        drawBom();
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
