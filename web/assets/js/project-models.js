// Files & Models: a project's 3D models as a filmstrip gallery, in the
// editor and on the project page. One model large, with a cube badge on a
// picture rendered from the model and an Open 3D viewer button, and a
// strip of the other models under it that swap it when clicked. Under the
// strip sit the shown model's name, description and print settings,
// which the page fills in (the editor's are editable).
//
// ProjectModels.render(el, models, {
//   thumbUrl(f), canView(f), openViewer(f), details(f) -> html, selectedId,
// })
// returns { select(id), selectedId() }.
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

  function picture(f, opts, cls) {
    if (f.thumbnail_version) {
      return `<img src="${esc(opts.thumbUrl(f))}" alt="${esc(f.filename)}" class="${cls}" loading="lazy">${BADGE}`;
    }
    return `<span class="kr-model-placeholder ${cls}" aria-hidden="true"><i class="fas fa-cube"></i></span>`;
  }

  function render(el, models, opts) {
    if (!el) return null;
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
      stage.innerHTML = `
        <button type="button" class="kr-models-hero" ${viewable ? '' : 'disabled'}
          title="${viewable ? 'Open ' + esc(f.filename) + ' in 3D' : esc(f.filename)}">
          ${picture(f, opts, 'kr-models-hero-img')}
        </button>
        ${viewable ? '<button type="button" class="kr-models-open"><i class="fas fa-cube" aria-hidden="true"></i> Open 3D viewer</button>' : ''}`;
      details.innerHTML = opts.details(f);
      strip.querySelectorAll('.kr-models-thumb').forEach(b => {
        const on = Number(b.dataset.modelId) === f.id;
        b.classList.toggle('is-active', on);
        b.setAttribute('aria-selected', on ? 'true' : 'false');
        b.tabIndex = on ? 0 : -1;
        if (on && focus) { b.focus(); b.scrollIntoView({ block: 'nearest', inline: 'nearest' }); }
      });
    }

    stage.addEventListener('click', e => {
      if (e.target.closest('.kr-models-hero:not([disabled]), .kr-models-open')) opts.openViewer(current);
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
    return {
      select(id) { const f = models.find(m => m.id === id); if (f) show(f); },
      selectedId() { return current.id; },
    };
  }

  window.ProjectModels = { render, formatSize, esc };
})();
