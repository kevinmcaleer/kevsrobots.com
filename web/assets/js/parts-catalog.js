// The visual parts catalog (issue #265): a shelf per category, every part
// a card with its photo, facet filters and, when picking, multi-select
// with one "Add" action. /parts/ shows it as the catalog, and the project
// editor's /part picker opens it full screen to add Bill of Materials rows.
//
// It follows Snakie's PartCatalog (shelves of cards) and part-facets.ts
// (the filter rules) until Snakie's embeddable bundle can be mounted here
// instead. Data is GET /api/parts/_catalog, loaded once; filtering is
// local. facets() and applyFilters() are pure, tested in
// scripts/tests/parts-catalog.test.js.
(function (root) {
  const API = 'https://projects.kevsrobots.com';
  const UNSET = '—'; // a part with no family or manufacturer
  const SHELF_PAGE = 24; // cards drawn per shelf before "Show all"
  const STATUSES = ['verified', 'draft', 'disputed'];

  // ------------------------------------------------------------ filters

  const familyOf = p => (p.family || '').trim() || UNSET;
  const manufacturerOf = p => (p.manufacturer || '').trim() || UNSET;
  const categoryOf = p => (p.category || '').trim() || 'Other parts';
  // ?category=single-board-computer matches "Single Board Computer".
  const slugish = s => String(s).toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '');
  const statusOf = p => (STATUSES.includes(p.status) ? p.status : 'draft');
  const tagsOf = p => (p.tags || []).map(t => String(t).trim().toLowerCase()).filter(Boolean);

  function emptySelection() {
    return { families: new Set(), manufacturers: new Set(), tags: new Set(), statuses: new Set(), priced: false, category: null };
  }

  function matchesQuery(p, query) {
    const q = String(query || '').trim().toLowerCase();
    if (!q) return true;
    return [p.name, p.sku, p.mpn, p.manufacturer, p.family, p.category].concat(p.tags || [])
      .filter(Boolean).join(' ').toLowerCase().includes(q);
  }

  // OR within a facet, AND across facets. `except` skips one axis, so a
  // facet counts its own values against the other filters only.
  function matchesSelection(p, sel, except) {
    if (sel.category && slugish(categoryOf(p)) !== slugish(sel.category)) return false;
    if (except !== 'families' && sel.families.size && !sel.families.has(familyOf(p))) return false;
    if (except !== 'manufacturers' && sel.manufacturers.size && !sel.manufacturers.has(manufacturerOf(p))) return false;
    if (except !== 'statuses' && sel.statuses.size && !sel.statuses.has(statusOf(p))) return false;
    if (except !== 'tags' && sel.tags.size) {
      const t = tagsOf(p);
      if (![...sel.tags].some(x => t.includes(x))) return false;
    }
    if (except !== 'priced' && sel.priced && !p.has_price) return false;
    return true;
  }

  function applyFilters(parts, sel, query) {
    return parts.filter(p => matchesQuery(p, query) && matchesSelection(p, sel));
  }

  function count(parts, keyOf, sel, axis, query) {
    const counts = new Map();
    parts.forEach(p => {
      if (!matchesQuery(p, query) || !matchesSelection(p, sel, axis)) return;
      [].concat(keyOf(p)).forEach(k => counts.set(k, (counts.get(k) || 0) + 1));
    });
    return counts;
  }

  // The values each facet offers, with how many parts each would leave.
  // Every value any part has is listed (with 0 when the other filters rule
  // it out) so the panel doesn't jump about. Tags that restate a family or
  // manufacturer are dropped, and those on one part go last (`rare`).
  function facets(parts, sel, query) {
    const list = (all, counts, chosen) => [...all].map(value => ({
      value, count: counts.get(value) || 0, selected: chosen.has(value),
    }));
    const byCount = (a, b) => b.count - a.count || a.value.localeCompare(b.value);
    const families = new Set(parts.map(familyOf));
    const manufacturers = new Set(parts.map(manufacturerOf));
    const restated = new Set([...families, ...manufacturers].map(v => v.toLowerCase()));
    const tagTotals = new Map();
    parts.forEach(p => tagsOf(p).forEach(t => tagTotals.set(t, (tagTotals.get(t) || 0) + 1)));
    const tags = [...tagTotals.keys()].filter(t => !restated.has(t));
    const tagCounts = count(parts, tagsOf, sel, 'tags', query);
    return {
      families: list(families, count(parts, familyOf, sel, 'families', query), sel.families).sort(byCount),
      manufacturers: list(manufacturers, count(parts, manufacturerOf, sel, 'manufacturers', query), sel.manufacturers).sort(byCount),
      statuses: STATUSES.map(value => ({
        value,
        count: count(parts, statusOf, sel, 'statuses', query).get(value) || 0,
        selected: sel.statuses.has(value),
      })),
      tags: tags.map(value => ({
        value, count: tagCounts.get(value) || 0, selected: sel.tags.has(value), rare: tagTotals.get(value) < 2,
      })).sort((a, b) => (a.rare - b.rare) || byCount(a, b)),
      priced: parts.filter(p => p.has_price && matchesQuery(p, query) && matchesSelection(p, sel, 'priced')).length,
    };
  }

  // Shelves in the order a reader scans them: the biggest first, the
  // catch-all last. [{ category, parts }]
  function shelves(parts) {
    const by = new Map();
    parts.forEach(p => {
      const c = categoryOf(p);
      if (!by.has(c)) by.set(c, []);
      by.get(c).push(p);
    });
    return [...by].map(([category, list]) => ({ category, parts: list }))
      .sort((a, b) => (a.category === 'Other parts') - (b.category === 'Other parts') ||
        b.parts.length - a.parts.length || a.category.localeCompare(b.category));
  }

  // ------------------------------------------------------------ drawing

  function el(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text != null) e.textContent = text;
    return e;
  }

  function coverSrc(p, api) {
    return p.cover_url ? (/^https?:\/\//.test(p.cover_url) ? p.cover_url : api + p.cover_url) : null;
  }

  const STATUS_LABEL = { verified: 'Verified', draft: 'Draft', disputed: 'Disputed' };

  let uid = 0;

  // Draws the catalog into `host`.
  //   api           the projects API base
  //   pick          true: cards are checkboxes and an Add bar appears
  //   addLabel      the Add button's words ("Add to Bill of Materials")
  //   onAddMany(parts)  the chosen parts, when Add is pressed
  //   initial       { category, family, query } to start from (the URL's)
  // Resolves to { setQuery, focus } once the parts have loaded.
  async function mount(host, opts) {
    opts = opts || {};
    const api = opts.api || API;
    const id = 'pc' + (++uid);
    const sel = emptySelection();
    const chosen = new Map(); // slug -> part, in the order picked
    const expanded = new Set(); // shelves showing every card
    let tagsOpen = false;
    let query = (opts.initial && opts.initial.query) || '';
    if (opts.initial && opts.initial.category) sel.category = opts.initial.category;
    if (opts.initial && opts.initial.family) sel.families.add(opts.initial.family);

    host.classList.add('parts-catalog');
    if (opts.pick) host.classList.add('parts-catalog--pick');
    host.innerHTML = '';
    const bar = el('div', 'parts-catalog-search');
    const search = el('input', 'form-control');
    search.type = 'search';
    search.placeholder = 'Search parts by name, part number, maker or tag…';
    search.setAttribute('aria-label', 'Search parts');
    search.value = query;
    bar.appendChild(search);
    const body = el('div', 'parts-catalog-body');
    const side = el('aside', 'parts-catalog-facets');
    side.setAttribute('aria-label', 'Filters');
    const main = el('div', 'parts-catalog-shelves');
    const status = el('p', 'parts-catalog-status');
    status.setAttribute('role', 'status');
    status.setAttribute('aria-live', 'polite');
    body.append(side, main);
    host.append(bar, status, body);
    let addBar = null;
    let addBtn = null;
    if (opts.pick) {
      addBar = el('div', 'parts-catalog-addbar');
      const summary = el('span', 'parts-catalog-chosen');
      addBtn = el('button', 'btn btn-success', opts.addLabel || 'Add');
      addBtn.type = 'button';
      const clear = el('button', 'btn btn-link btn-sm', 'Clear');
      clear.type = 'button';
      clear.addEventListener('click', () => {
        chosen.clear();
        main.querySelectorAll('.parts-catalog-check:checked').forEach(b => { b.checked = false; b.closest('.parts-catalog-card').classList.remove('is-chosen'); });
        drawAddBar();
      });
      addBtn.addEventListener('click', () => {
        if (chosen.size && opts.onAddMany) opts.onAddMany([...chosen.values()]);
      });
      addBar.append(summary, clear, addBtn);
      host.appendChild(addBar);
    }

    status.textContent = 'Loading the parts catalog…';
    let parts = [];
    try {
      const resp = await fetch(api + '/api/parts/_catalog');
      if (!resp.ok) throw new Error(String(resp.status));
      parts = await resp.json();
    } catch (_) {
      status.textContent = 'The parts catalog couldn’t be loaded. Try again in a moment.';
      return { setQuery() {}, focus() { search.focus(); } };
    }

    // --- facets panel
    function facetGroup(title, values, axis, labelOf) {
      // One value for every part (no part has a family, say) filters nothing.
      if (!values.length || (axis !== 'statuses' && axis !== 'tags' && values.length < 2 && !values.some(v => v.selected))) return null;
      const group = el('fieldset', 'parts-catalog-facet');
      group.appendChild(el('legend', null, title));
      values.forEach(v => {
        const label = el('label', 'parts-catalog-facet-value' + (v.count ? '' : ' is-empty') + (v.rare ? ' is-rare' : ''));
        const box = el('input');
        box.type = 'checkbox';
        box.checked = v.selected;
        box.dataset.axis = axis;
        box.dataset.value = v.value;
        label.append(box, el('span', null, labelOf ? labelOf(v.value) : v.value), el('small', null, String(v.count)));
        group.appendChild(label);
      });
      return group;
    }

    function drawFacets() {
      const f = facets(parts, sel, query);
      side.replaceChildren();
      if (sel.category) {
        const chip = el('button', 'parts-catalog-chip', sel.category + ' ×');
        chip.type = 'button';
        chip.setAttribute('aria-label', 'Show every category, not just ' + sel.category);
        chip.addEventListener('click', () => { sel.category = null; draw(); });
        side.appendChild(chip);
      }
      const priced = el('label', 'parts-catalog-facet-value parts-catalog-priced');
      const box = el('input');
      box.type = 'checkbox';
      box.checked = sel.priced;
      box.dataset.axis = 'priced';
      priced.append(box, el('span', null, 'Has a price'), el('small', null, String(f.priced)));
      side.appendChild(priced);
      [
        facetGroup('Type', f.families, 'families'),
        facetGroup('Manufacturer', f.manufacturers, 'manufacturers'),
        facetGroup('Status', f.statuses, 'statuses', v => STATUS_LABEL[v]),
      ].forEach(g => g && side.appendChild(g));
      const common = f.tags.filter(t => !t.rare || t.selected);
      const rare = f.tags.filter(t => t.rare && !t.selected);
      const tags = facetGroup('Tags', tagsOpen ? common.concat(rare) : common, 'tags');
      if (tags) {
        if (rare.length) {
          const more = el('button', 'btn btn-link btn-sm p-0', tagsOpen ? 'Fewer tags' : rare.length + ' more tags');
          more.type = 'button';
          more.setAttribute('aria-expanded', String(tagsOpen));
          more.addEventListener('click', () => { tagsOpen = !tagsOpen; drawFacets(); });
          tags.appendChild(more);
        }
        side.appendChild(tags);
      }
    }

    side.addEventListener('change', e => {
      const box = e.target;
      if (!box.dataset || !box.dataset.axis) return;
      if (box.dataset.axis === 'priced') sel.priced = box.checked;
      else if (box.checked) sel[box.dataset.axis].add(box.dataset.value);
      else sel[box.dataset.axis].delete(box.dataset.value);
      draw();
    });

    // --- cards
    function card(p) {
      const src = coverSrc(p, api);
      const maker = (p.manufacturer || '').trim();
      const node = el(opts.pick ? 'label' : 'a', 'parts-catalog-card');
      node.dataset.slug = p.slug;
      const describe = [p.name, maker, STATUS_LABEL[statusOf(p)], p.has_price ? 'has a price' : '']
        .filter(Boolean).join(', ');
      if (opts.pick) {
        const box = el('input', 'parts-catalog-check');
        box.type = 'checkbox';
        box.checked = chosen.has(p.slug);
        box.dataset.slug = p.slug;
        box.setAttribute('aria-label', describe);
        node.appendChild(box);
        if (box.checked) node.classList.add('is-chosen');
      } else {
        node.href = '/parts/view.html?slug=' + encodeURIComponent(p.slug);
        node.setAttribute('aria-label', describe);
      }
      const photo = el('span', 'parts-catalog-photo');
      if (src) {
        const img = el('img');
        img.src = src;
        img.alt = '';
        img.loading = 'lazy';
        img.decoding = 'async';
        photo.appendChild(img);
      } else {
        photo.classList.add('is-empty');
      }
      node.appendChild(photo);
      const text = el('span', 'parts-catalog-text');
      text.append(el('span', 'parts-catalog-name', p.name));
      if (maker) text.append(el('span', 'parts-catalog-maker', maker));
      node.appendChild(text);
      if (statusOf(p) !== 'draft') node.appendChild(el('span', 'parts-catalog-badge is-' + statusOf(p), STATUS_LABEL[statusOf(p)]));
      return node;
    }

    function draw() {
      drawFacets();
      const shown = applyFilters(parts, sel, query);
      status.textContent = shown.length === parts.length
        ? parts.length + (parts.length === 1 ? ' part' : ' parts')
        : shown.length + ' of ' + parts.length + ' parts';
      main.replaceChildren();
      if (!shown.length) {
        main.appendChild(el('p', 'parts-catalog-empty', 'No parts match. Try fewer filters, or another word.'));
      }
      shelves(shown).forEach((shelf, n) => {
        const section = el('section', 'parts-catalog-shelf');
        const headId = id + '-shelf-' + n;
        const head = el('h2', null, shelf.category);
        head.id = headId;
        head.appendChild(el('small', null, String(shelf.parts.length)));
        section.setAttribute('aria-labelledby', headId);
        const grid = el('div', 'parts-catalog-grid');
        grid.setAttribute('role', 'list');
        const all = expanded.has(shelf.category) || shelf.parts.length <= SHELF_PAGE + 4;
        (all ? shelf.parts : shelf.parts.slice(0, SHELF_PAGE)).forEach(p => {
          const item = el('div', 'parts-catalog-item');
          item.setAttribute('role', 'listitem');
          item.appendChild(card(p));
          grid.appendChild(item);
        });
        section.append(head, grid);
        if (!all) {
          const more = el('button', 'btn btn-outline-secondary btn-sm parts-catalog-more',
            'Show all ' + shelf.parts.length + ' ' + shelf.category.toLowerCase());
          more.type = 'button';
          more.addEventListener('click', () => { expanded.add(shelf.category); draw(); });
          section.appendChild(more);
        }
        main.appendChild(section);
      });
      drawAddBar();
    }

    function drawAddBar() {
      if (!addBar) return;
      addBar.querySelector('.parts-catalog-chosen').textContent = chosen.size
        ? chosen.size + (chosen.size === 1 ? ' part chosen' : ' parts chosen')
        : 'Choose parts to add';
      addBtn.disabled = !chosen.size;
    }

    main.addEventListener('change', e => {
      const slug = e.target.dataset && e.target.dataset.slug;
      if (!slug) return;
      const p = parts.find(x => x.slug === slug);
      if (e.target.checked && p) chosen.set(slug, p);
      else chosen.delete(slug);
      e.target.closest('.parts-catalog-card').classList.toggle('is-chosen', e.target.checked);
      drawAddBar();
    });

    // Arrow keys move between cards along the grid; Tab still leaves it.
    main.addEventListener('keydown', e => {
      if (!/^Arrow(Left|Right|Up|Down)$/.test(e.key)) return;
      const focusables = [...main.querySelectorAll(opts.pick ? '.parts-catalog-check' : 'a.parts-catalog-card')];
      const i = focusables.indexOf(document.activeElement);
      if (i < 0) return;
      e.preventDefault();
      let next = i + (e.key === 'ArrowRight' ? 1 : e.key === 'ArrowLeft' ? -1 : 0);
      if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
        // The card above or below: the nearest one in the next row.
        const r = focusables[i].getBoundingClientRect();
        const down = e.key === 'ArrowDown';
        let best = -1;
        let bestD = Infinity;
        focusables.forEach((f, j) => {
          const q = f.getBoundingClientRect();
          if (down ? q.top <= r.top + 2 : q.top >= r.top - 2) return;
          const d = Math.abs(q.top - r.top) * 10 + Math.abs(q.left - r.left);
          if (d < bestD) { bestD = d; best = j; }
        });
        next = best < 0 ? i : best;
      }
      if (focusables[next]) focusables[next].focus();
    });

    let timer = 0;
    search.addEventListener('input', () => {
      clearTimeout(timer);
      timer = setTimeout(() => { query = search.value; draw(); }, 120);
    });

    draw();
    return {
      setQuery(q) { search.value = q; query = q; draw(); },
      focus() { search.focus(); },
    };
  }

  // The catalog full screen over the page, growing from `origin` (an
  // element or a rect) when given. Resolves to the parts added, or [] when
  // closed without adding.
  function open(opts) {
    opts = opts || {};
    return new Promise(resolve => {
      const back = document.activeElement;
      const overlay = el('div', 'parts-catalog-overlay');
      const dialog = el('div', 'parts-catalog-dialog');
      dialog.setAttribute('role', 'dialog');
      dialog.setAttribute('aria-modal', 'true');
      dialog.setAttribute('aria-label', 'Parts catalog');
      const head = el('div', 'parts-catalog-dialog-head');
      head.appendChild(el('h2', null, opts.title || 'Parts catalog'));
      const close = el('button', 'btn-close');
      close.type = 'button';
      close.setAttribute('aria-label', 'Close the catalog');
      head.appendChild(close);
      const host = el('div');
      dialog.append(head, host);
      overlay.appendChild(dialog);
      const rect = opts.origin && (opts.origin.getBoundingClientRect ? opts.origin.getBoundingClientRect() : opts.origin);
      if (rect) dialog.style.transformOrigin = (rect.left + rect.width / 2) + 'px ' + (rect.top + rect.height / 2) + 'px';
      document.body.appendChild(overlay);
      document.body.classList.add('parts-catalog-open');
      requestAnimationFrame(() => overlay.classList.add('is-open'));

      let done = false;
      function finish(added) {
        if (done) return;
        done = true;
        overlay.remove();
        document.body.classList.remove('parts-catalog-open');
        document.removeEventListener('keydown', onKey, true);
        if (back && back.focus) back.focus();
        resolve(added);
      }
      function onKey(e) {
        if (e.key === 'Escape') { e.preventDefault(); e.stopPropagation(); finish([]); }
        else if (e.key === 'Tab') {
          // Keep focus in the dialog.
          const f = [...dialog.querySelectorAll('input, button, a[href]')].filter(x => !x.disabled && x.offsetParent);
          if (!f.length) return;
          if (e.shiftKey && document.activeElement === f[0]) { e.preventDefault(); f[f.length - 1].focus(); }
          else if (!e.shiftKey && document.activeElement === f[f.length - 1]) { e.preventDefault(); f[0].focus(); }
        }
      }
      document.addEventListener('keydown', onKey, true);
      close.addEventListener('click', () => finish([]));
      overlay.addEventListener('mousedown', e => { if (e.target === overlay) finish([]); });
      mount(host, {
        api: opts.api, pick: true, addLabel: opts.addLabel, initial: { query: opts.query || '' },
        onAddMany: parts => finish(parts),
      }).then(c => c.focus());
    });
  }

  const api = { mount, open, facets, applyFilters, shelves, emptySelection };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.PartsCatalog = api;
})(typeof window !== 'undefined' ? window : globalThis);
