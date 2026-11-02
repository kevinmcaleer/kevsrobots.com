// Files & Models: a project's 3D models as a filmstrip gallery, in the
// editor and on the project page. One model large, drawn in 3D so it can be
// dragged round (its picture shows until the model loads, and a button
// opens it full screen), and a strip of pictures of the other models under
// it that swap it when clicked. Under the
// strip sit the shown model's name, description and print settings,
// which the page fills in (the editor's are editable).
//
// ProjectModels.render(el, models, {
//   thumbUrl(f), canView(f), openViewer(f), details(f) -> html, selectedId,
//   modelUrl(f) (the model's bytes, for the big 3D preview; without it the
//   big preview stays a picture),
//   fetch(url) (for colouring pictures; defaults to ProjectAuth.apiFetch),
// })
// returns { select(id), selectedId() }.
//
// ProjectModels.renderTable(el, models, { thumbUrl, canView, openViewer,
//   fetch, head: html of the <th>s after the picture, cells(f) -> <td>s })
// draws the same models as a table with a small picture on each row.
//
// A model's print details (print_qty, print_colour, filament) show as
// printDetails(f); a model with a colour is drawn in that colour.
(function () {
  'use strict';

  function esc(s) {
    return String(s == null ? '' : s)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
  }

  function formatSize(bytes) {
    const kb = (bytes || 0) / 1024;
    return kb >= 1024 ? (kb / 1024).toFixed(1) + ' MB' : kb.toFixed(1) + ' KB';
  }

  // The badge says "this picture is the model itself", as Printables does.
  const BADGE = '<span class="kr-model-badge" title="Rendered from the 3D model" aria-hidden="true"><i class="fas fa-cube"></i></span>';

  // ---- Colours ---------------------------------------------------------
  // A print colour is a hex value from the colour picker or a word such as
  // "white" or "silk gold". colourHex() turns it into #rrggbb when the
  // browser knows it as a colour (trying the last word, so "silk gold"
  // is gold), or null.
  const FILAMENT_WORDS = { natural: '#efe8d4', clear: null, transparent: null, translucent: null };
  let colourCtx = null;
  function cssHex(word) {
    if (!word) return null;
    if (/^#[0-9a-f]{3}([0-9a-f]{3})?$/i.test(word)) {
      const h = word.slice(1).toLowerCase();
      return '#' + (h.length === 3 ? h.replace(/./g, c => c + c) : h);
    }
    if (!/^[a-z]+$/i.test(word)) return null;
    if (!colourCtx) colourCtx = document.createElement('canvas').getContext('2d');
    if (!colourCtx) return null;
    // An unknown word leaves fillStyle as it was, so test with two bases.
    colourCtx.fillStyle = '#000000';
    colourCtx.fillStyle = word;
    const a = colourCtx.fillStyle;
    colourCtx.fillStyle = '#ffffff';
    colourCtx.fillStyle = word;
    return a === colourCtx.fillStyle && /^#[0-9a-f]{6}$/.test(a) ? a : null;
  }
  function colourHex(value) {
    const v = String(value == null ? '' : value).trim().toLowerCase();
    if (!v) return null;
    const words = v.split(/[\s-]+/);
    const last = words[words.length - 1];
    for (const w of [v.replace(/[\s-]+/g, ''), last]) {
      if (Object.prototype.hasOwnProperty.call(FILAMENT_WORDS, w)) return FILAMENT_WORDS[w];
      const hex = cssHex(w);
      if (hex) return hex;
    }
    return null;
  }

  // A round dot of the colour (a dashed ring when it isn't one we know).
  function swatch(value) {
    const hex = colourHex(value);
    return hex
      ? `<span class="kr-swatch" style="background:${hex}" aria-hidden="true"></span>`
      : '<span class="kr-swatch is-unknown" aria-hidden="true"></span>';
  }

  // Thumbnails are rendered in the site green (model-thumbnails.js). A
  // coloured model is redrawn from that picture: its light and shade kept,
  // its colour swapped. Cached per picture and colour.
  const BASE_LUMA = 0.2126 * 0x3d + 0.7152 * 0x8b + 0.0722 * 0x5a;
  const tinted = new Map();
  function defaultFetch(url) {
    return window.ProjectAuth ? ProjectAuth.apiFetch(url, {}) : fetch(url, { credentials: 'include' });
  }
  function tintUrl(url, hex, fetcher) {
    const key = url + '|' + hex;
    if (!tinted.has(key)) {
      tinted.set(key, (async () => {
        const resp = await (fetcher || defaultFetch)(url);
        if (!resp.ok) throw new Error('thumbnail ' + resp.status);
        const bitmap = await createImageBitmap(await resp.blob());
        const canvas = document.createElement('canvas');
        canvas.width = bitmap.width;
        canvas.height = bitmap.height;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(bitmap, 0, 0);
        const img = ctx.getImageData(0, 0, canvas.width, canvas.height);
        const d = img.data;
        // Very dark colours keep a little light, or a black part would be
        // a silhouette.
        const t = [1, 3, 5].map(i => 34 + parseInt(hex.slice(i, i + 2), 16) * (221 / 255));
        for (let i = 0; i < d.length; i += 4) {
          if (!d[i + 3]) continue;
          const shade = (0.2126 * d[i] + 0.7152 * d[i + 1] + 0.0722 * d[i + 2]) / BASE_LUMA;
          for (let c = 0; c < 3; c++) {
            d[i + c] = shade <= 1
              ? t[c] * shade
              : Math.min(255, t[c] + (255 - t[c]) * (shade - 1) * 0.8);
          }
        }
        ctx.putImageData(img, 0, 0);
        const blob = await new Promise(r => canvas.toBlob(r, 'image/png'));
        return URL.createObjectURL(blob);
      })());
      tinted.get(key).catch(() => tinted.delete(key));
    }
    return tinted.get(key);
  }

  // Colour every picture under `root` that asks for it (data-tint).
  function tintAll(root, fetcher) {
    if (!root || typeof createImageBitmap !== 'function') return;
    root.querySelectorAll('img[data-tint]').forEach(img => {
      const url = img.dataset.src || img.getAttribute('src');
      const hex = img.dataset.tint;
      img.removeAttribute('data-tint');
      img.classList.add('is-tinting');
      tintUrl(url, hex, fetcher).then(u => { img.src = u; })
        .catch(() => {})
        .finally(() => img.classList.remove('is-tinting'));
    });
  }

  function picture(f, opts, cls) {
    if (f.thumbnail_version) {
      const hex = colourHex(f.print_colour);
      return `<img src="${esc(opts.thumbUrl(f))}" alt="${esc(f.filename)}" class="${cls}" loading="lazy"${hex ? ` data-tint="${hex}"` : ''}>${BADGE}`;
    }
    const hex = colourHex(f.print_colour);
    return `<span class="kr-model-placeholder ${cls}" aria-hidden="true"${hex ? ` style="color:${hex};opacity:1"` : ''}><i class="fas fa-cube"></i></span>`;
  }

  // ---- Print details -----------------------------------------------------
  function qtyText(f) { return f.print_qty ? '×' + f.print_qty : ''; }

  // "×2 · ● white · PETG" for the filmstrip's details; '' when none set.
  function printDetails(f) {
    const bits = [];
    if (f.print_qty) bits.push(`<span class="kr-print-chip" title="Print ${f.print_qty}"><i class="fas fa-layer-group" aria-hidden="true"></i> Print ${f.print_qty}</span>`);
    if (f.print_colour) bits.push(`<span class="kr-print-chip" title="Colour">${swatch(f.print_colour)} ${esc(f.print_colour)}</span>`);
    if (f.filament) bits.push(`<span class="kr-print-chip" title="Filament"><i class="fas fa-circle-notch" aria-hidden="true"></i> ${esc(f.filament)}</span>`);
    return bits.length ? `<div class="kr-print-details">${bits.join('')}</div>` : '';
  }

  // ---- Filmstrip / Table switch -----------------------------------------
  const VIEWS = [
    { id: 'filmstrip', icon: 'fa-film', label: 'Filmstrip' },
    { id: 'table', icon: 'fa-table', label: 'Table' },
  ];
  function viewToggle(current, title) {
    return `<div class="kr-models-views" role="group" aria-label="${esc(title || 'Show models as')}">${VIEWS.map(v => `
      <button type="button" class="kr-models-view${v.id === current ? ' is-active' : ''}" data-models-view="${v.id}"
        aria-pressed="${v.id === current}" title="${v.label}"><i class="fas ${v.icon}" aria-hidden="true"></i><span>${v.label}</span></button>`).join('')}</div>`;
  }
  function normaliseView(v) { return v === 'table' ? 'table' : 'filmstrip'; }

  // ---- Table -------------------------------------------------------------
  function renderTable(el, models, opts) {
    if (!el) return;
    stopInline(el);
    if (!models.length) { el.innerHTML = ''; return; }
    el.innerHTML = `<div class="kr-models-table-wrap"><table class="table table-hover mb-0 kr-models-table">
      <thead><tr><th class="kr-models-table-pic"><span class="visually-hidden">Picture</span></th>${opts.head}</tr></thead>
      <tbody>${models.map(f => {
        const viewable = opts.canView(f);
        return `<tr data-model-id="${f.id}">
          <td class="kr-models-table-pic">
            <button type="button" class="kr-models-table-thumb" ${viewable ? '' : 'disabled'}
              title="${viewable ? 'Open ' + esc(f.filename) + ' in 3D' : esc(f.filename)}">${picture(f, opts, 'kr-models-table-img')}</button>
          </td>${opts.cells(f)}</tr>`;
      }).join('')}</tbody></table></div>`;
    el.querySelectorAll('.kr-models-table-thumb:not([disabled])').forEach(b => {
      b.addEventListener('click', () => {
        const f = models.find(m => m.id === Number(b.closest('tr').dataset.modelId));
        if (f) opts.openViewer(f);
      });
    });
    tintAll(el, opts.fetch);
  }

  // The big preview's live 3D model, one per gallery element. Stopped
  // (its WebGL context given back) whenever the gallery redraws or the
  // selection moves on: browsers allow only a handful of contexts.
  function stopInline(el) {
    if (el && el._krModels3d) { el._krModels3d(); el._krModels3d = null; }
  }

  // Load f's model into `hero` once it scrolls near the screen, then swap
  // the picture for it. Returns a stop function.
  function startInline(hero, f, opts) {
    const MT = window.ModelThumbnails;
    if (!MT || !MT.mountViewer || !opts.modelUrl) return () => {};
    let mounted = null;
    let stopped = false;
    const begin = () => {
      if (stopped || mounted) return;
      hero.classList.add('is-loading-3d');
      mounted = MT.mountViewer(hero, { url: opts.modelUrl(f), colour: colourHex(f.print_colour), inline: true });
      mounted.ready.then(() => {
        if (stopped) return;
        hero.classList.remove('is-loading-3d');
        hero.classList.add('is-3d');
      }, () => {
        // No WebGL, or the file wouldn't load: the picture stays.
        hero.classList.remove('is-loading-3d');
      });
    };
    let io = null;
    if (window.IntersectionObserver) {
      io = new IntersectionObserver(entries => {
        if (entries.some(e => e.isIntersecting)) { io.disconnect(); io = null; begin(); }
      }, { rootMargin: '200px' });
      io.observe(hero);
    } else {
      begin();
    }
    return () => {
      stopped = true;
      if (io) io.disconnect();
      if (mounted) mounted.destroy();
    };
  }

  function render(el, models, opts) {
    if (!el) return null;
    stopInline(el);
    if (!models.length) { el.innerHTML = ''; return null; }
    let current = models.find(f => f.id === opts.selectedId) || models[0];

    el.innerHTML = `
      <div class="kr-models" data-count="${models.length}">
        <div class="kr-models-stage"></div>
        <div class="kr-models-strip" role="listbox" aria-label="Models">
          ${models.map(f => `
            <button type="button" class="kr-models-thumb" role="option" data-model-id="${f.id}" title="${esc(f.filename)}">
              <span class="kr-models-thumb-pic">${picture(f, opts, 'kr-models-thumb-img')}</span>
              <span class="kr-models-thumb-name">${esc(f.filename)}</span>
            </button>`).join('')}
        </div>
        <div class="kr-models-details"></div>
      </div>`;

    const stage = el.querySelector('.kr-models-stage');
    const details = el.querySelector('.kr-models-details');
    const strip = el.querySelector('.kr-models-strip');

    function show(f, focus) {
      current = f;
      const viewable = opts.canView(f);
      stopInline(el);
      const live = viewable && !!opts.modelUrl && !!(window.ModelThumbnails && ModelThumbnails.mountViewer);
      stage.innerHTML = `
        <div class="kr-models-hero${live ? ' is-live' : ''}${viewable && !live ? ' is-clickable' : ''}"
          ${live ? `role="img" aria-label="${esc(f.filename)} in 3D, drag to turn"` : ''}
          title="${live ? '' : viewable ? 'Open ' + esc(f.filename) + ' in 3D' : esc(f.filename)}">
          ${picture(f, opts, 'kr-models-hero-img')}
        </div>
        ${live ? `<span class="kr-models-hint" aria-hidden="true"><i class="fas fa-sync-alt"></i> Drag to turn</span>` : ''}
        ${viewable ? `<button type="button" class="kr-models-open${live ? ' is-icon' : ''}" title="Open ${esc(f.filename)} full screen" aria-label="Open ${esc(f.filename)} full screen">${live ? '<i class="fas fa-expand" aria-hidden="true"></i>' : '<i class="fas fa-cube" aria-hidden="true"></i> Open 3D viewer'}</button>` : ''}`;
      if (live) el._krModels3d = startInline(stage.querySelector('.kr-models-hero'), f, opts);
      details.innerHTML = opts.details(f);
      tintAll(stage, opts.fetch);
      strip.querySelectorAll('.kr-models-thumb').forEach(b => {
        const on = Number(b.dataset.modelId) === f.id;
        b.classList.toggle('is-active', on);
        b.setAttribute('aria-selected', on ? 'true' : 'false');
        b.tabIndex = on ? 0 : -1;
        if (on && focus) { b.focus(); b.scrollIntoView({ block: 'nearest', inline: 'nearest' }); }
      });
    }

    stage.addEventListener('click', e => {
      if (e.target.closest('.kr-models-hero.is-clickable, .kr-models-open')) opts.openViewer(current);
    });
    strip.addEventListener('click', e => {
      const b = e.target.closest('.kr-models-thumb');
      const f = b && models.find(m => m.id === Number(b.dataset.modelId));
      if (f) show(f);
    });
    strip.addEventListener('keydown', e => {
      const step = { ArrowRight: 1, ArrowDown: 1, ArrowLeft: -1, ArrowUp: -1 }[e.key];
      if (!step) return;
      e.preventDefault();
      const i = models.indexOf(current);
      show(models[(i + step + models.length) % models.length], true);
    });

    show(current);
    tintAll(strip, opts.fetch);
    return {
      select(id) { const f = models.find(m => m.id === id); if (f) show(f); },
      selectedId() { return current.id; },
    };
  }

  window.ProjectModels = {
    render, renderTable, formatSize, esc, colourHex, swatch, printDetails, qtyText,
    viewToggle, normaliseView, tintAll,
  };
})();
