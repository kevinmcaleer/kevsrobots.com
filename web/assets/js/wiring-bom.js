// The Bill of Materials side of the wiring workspace (project-wiring.js).
//
// Every row of the project's Bill of Materials can go on the wiring
// canvas. A row linked to a catalog part that has pins comes in as that
// part. A row linked to a part with no pins yet, or not linked at all,
// comes in as a placeholder card with the row's name, so the diagram
// still shows it. Linking a row (here or in the BOM table) swaps the
// placeholder for the real part where it stands.
//
// Pure helpers, tested in scripts/tests/wiring-bom.test.js; the panel
// itself is drawn by project-wiring.js.
(function (root) {
  // The library placeholders for unlinked rows live in. Catalog parts are
  // in 'kevsrobots' (project-wiring.js's LIB).
  const BOM_LIB = 'kevsrobots-bom';

  const slugify = s => String(s || '').toLowerCase().normalize('NFKD').replace(/[̀-ͯ]/g, '')
    .replace(/[^a-z0-9]+/g, '-').replace(/^-+|-+$/g, '').slice(0, 60) || 'part';

  // The placeholder part id for an unlinked row named `name`.
  function placeholderId(name) {
    return 'row-' + slugify(name);
  }

  const hasPins = def => !!def && ((def.headers || []).some(h => (h.pins || []).length > 0) ||
    (def.connectors || []).some(c => (c.pins || []).length > 0));
  const isBoard = def => !!def && String(def.family || '').trim().toLowerCase() === 'microcontroller';

  function xml(s) {
    return String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&apos;' }[c]));
  }

  // The name in at most two lines of about `width` characters.
  function wrap(name, width) {
    const words = String(name).trim().split(/\s+/);
    const lines = [''];
    for (const w of words) {
      const cur = lines[lines.length - 1];
      if (!cur) lines[lines.length - 1] = w;
      else if ((cur + ' ' + w).length <= width) lines[lines.length - 1] = cur + ' ' + w;
      else if (lines.length < 2) lines.push(w);
      else { lines[1] += ' ' + w; break; }
    }
    return lines.map(l => (l.length > width ? l.slice(0, width - 1) + '…' : l));
  }

  // The card drawn for a part with nothing better to show: the name, and
  // why there are no pins. `kind` is 'unlinked' or 'nopins'.
  function placeholderImage(name, kind) {
    const lines = wrap(name, 20);
    const note = kind === 'nopins' ? 'No pins in the catalog yet' : 'Not linked to a catalog part';
    const top = lines.length > 1 ? 82 : 98;
    const text = lines.map((l, i) =>
      `<text x="150" y="${top + i * 32}" font-family="Plus Jakarta Sans, Helvetica, Arial, sans-serif" font-size="26" font-weight="700" text-anchor="middle" fill="#3b3628">${xml(l)}</text>`).join('');
    const svg = '<svg xmlns="http://www.w3.org/2000/svg" width="300" height="200" viewBox="0 0 300 200">' +
      '<rect x="4" y="4" width="292" height="192" rx="14" fill="#f6f1e4" stroke="#8a7f66" stroke-width="4" stroke-dasharray="14 10"/>' +
      text +
      `<text x="150" y="${top + lines.length * 32 + 14}" font-family="Plus Jakarta Sans, Helvetica, Arial, sans-serif" font-size="17" text-anchor="middle" fill="#7a705a">${xml(note)}</text>` +
      '</svg>';
    return 'data:image/svg+xml;charset=utf-8,' + encodeURIComponent(svg);
  }

  // The PartDefinition for an unlinked row: no pins, the card as its photo.
  function placeholderPart(name) {
    return {
      id: placeholderId(name),
      name: String(name),
      description: 'A Bill of Materials row that isn’t linked to a catalog part yet.',
      headers: [],
      aspect: 1.5,
      dimensions: { width: 30, height: 20 },
      pcbColor: '#8a7f66',
      imageData: placeholderImage(name, 'unlinked'),
    };
  }

  // A catalog part with no pins and no photo gets the card as its photo.
  function withFallbackImage(def) {
    if (!def || hasPins(def) || def.imageData) return def;
    return Object.assign({}, def, {
      imageData: placeholderImage(def.name || def.id, 'nopins'),
      aspect: def.aspect || 1.5,
      dimensions: def.dimensions || { width: 30, height: 20 },
    });
  }

  // A part with no pins drawn at its photo's shape. With nothing else to
  // go on, the canvas fits a picture to the part's outline (Snakie's
  // default is a tall 0.6), which squashed product photos. The photo's own
  // width/height (`w`, `h`, in pixels) gives the outline; a real width in
  // millimetres is kept and the height follows it. Parts with pins are
  // left alone: their pins were placed over the outline as it is.
  function fitToPhoto(def, w, h) {
    if (!def || hasPins(def) || !(w > 0) || !(h > 0)) return def;
    const aspect = w / h;
    const out = Object.assign({}, def, { aspect });
    const width = def.dimensions && def.dimensions.width;
    if (width > 0) out.dimensions = Object.assign({}, def.dimensions, { width, height: Math.round(width / aspect * 10) / 10 });
    else delete out.dimensions;
    return out;
  }

  // The rows the panel lists, one per distinct part, in BOM order:
  //   { name, slug, lib, part, def, linked, pins, board, isTheBoard,
  //     qty, placed, left, onCanvas }
  // `rows` is [{ name, slug, qty }] from the BOM table (qty defaults to 1;
  // two rows for the same part add up), `defs` a Map of slug ->
  // PartDefinition (or null when the catalog doesn't have it).
  // `placed` counts that part on the canvas (the board counts as one) and
  // `left` how many more the Qty allows. `onCanvas` is whether the row is
  // done: every one of a part with pins, since each gets wired; for a card
  // (unlinked, or no pins yet) one is enough, though more can go on.
  function entries(rows, robot, defs) {
    const placedCount = new Map();
    for (const p of robot.parts || []) {
      const key = p.lib + '/' + p.part;
      placedCount.set(key, (placedCount.get(key) || 0) + 1);
    }
    const byKey = new Map();
    const out = [];
    for (const r of rows) {
      const def = r.slug ? defs.get(r.slug) || null : null;
      const linked = !!r.slug;
      const lib = linked ? 'kevsrobots' : BOM_LIB;
      const part = linked ? r.slug : placeholderId(r.name);
      const key = lib + '/' + part;
      const qty = Math.max(1, parseInt(r.qty, 10) || 1);
      if (byKey.has(key)) { byKey.get(key).qty += qty; continue; }
      const isTheBoard = linked && robot.board === r.slug;
      const e = {
        name: r.name,
        slug: r.slug || null,
        lib,
        part,
        def,
        linked,
        pins: hasPins(def),
        board: isBoard(def),
        isTheBoard,
        qty,
        placed: (placedCount.get(key) || 0) + (isTheBoard ? 1 : 0),
      };
      byKey.set(key, e);
      out.push(e);
    }
    for (const e of out) {
      e.left = Math.max(0, e.qty - e.placed);
      e.onCanvas = e.pins ? e.placed >= e.qty : e.placed > 0;
    }
    return out;
  }

  // The labels for the next `count` copies of `lib/part` on the canvas,
  // named `name`: "Servo" for the only one, "Servo 2", "Servo 3"… after
  // that, skipping numbers already used. When a second copy goes on, a
  // first copy still called plain "Servo" becomes "Servo 1", so the
  // pinout table can tell them apart. { robot, labels }: `robot` is the
  // same object when nothing was renamed.
  function copyLabels(robot, lib, part, name, count) {
    const mine = (robot.parts || []).filter(p => p.lib === lib && p.part === part);
    const base = String(name || part).trim();
    if (!mine.length && count === 1) return { robot, labels: [base] };
    const re = new RegExp('^' + base.replace(/[.*+?^${}()|[\]\\]/g, '\\$&') + ' (\\d+)$');
    const used = new Set();
    let next = robot;
    let plain = null;
    for (const p of mine) {
      const m = String(p.label || '').match(re);
      if (m) used.add(+m[1]);
      else if (!p.label || p.label === base) plain = plain || p;
    }
    if (plain && !used.has(1)) {
      used.add(1);
      next = Object.assign({}, robot, { parts: robot.parts.map(p => (p === plain ? Object.assign({}, p, { label: base + ' 1' }) : p)) });
    }
    const labels = [];
    let n = 1;
    while (labels.length < count) {
      if (!used.has(n)) { used.add(n); labels.push(base + ' ' + n); }
      n++;
    }
    return { robot: next, labels };
  }

  // The wiring with every placed `fromLib/fromPart` swapped for
  // `toLib/toPart`, where it stands. Wires on the old part's pins go: a
  // placeholder has none, and a new part's pins are its own. With a
  // `label`, one copy is called that and several "label 1", "label 2"…
  function relink(robot, fromLib, fromPart, toLib, toPart, label) {
    const ids = new Set();
    const count = (robot.parts || []).filter(p => p.lib === fromLib && p.part === fromPart).length;
    const parts = (robot.parts || []).map(p => {
      if (p.lib !== fromLib || p.part !== fromPart) return p;
      ids.add(p.id);
      const next = Object.assign({}, p, { lib: toLib, part: toPart });
      if (label) next.label = count > 1 ? label + ' ' + ids.size : label;
      return next;
    });
    if (!ids.size) return robot;
    const onPart = end => ids.has(String(end).split('.')[0]);
    const connections = (robot.connections || []).filter(c => !onPart(c.from) && !onPart(c.to));
    return Object.assign({}, robot, { parts, connections });
  }

  // Parts with pins the canvas has more of than the Bill of Materials'
  // Qty: [{ slug, name, qty, placed }], to offer raising the Qty.
  function overQty(list) {
    return list.filter(e => e.linked && e.pins && e.placed > e.qty)
      .map(e => ({ slug: e.slug, name: e.name, qty: e.qty, placed: e.placed }));
  }

  // Placeholders on the canvas whose row has since been linked (in the BOM
  // table, say): [{ from, slug, name }], matched by the row's name.
  function linkedSince(robot, rows) {
    const bySlugName = new Map();
    for (const r of rows) if (r.slug) bySlugName.set(placeholderId(r.name), r);
    const unlinked = new Set(rows.filter(r => !r.slug).map(r => placeholderId(r.name)));
    const out = [];
    const seen = new Set();
    for (const p of robot.parts || []) {
      if (p.lib !== BOM_LIB || seen.has(p.part) || unlinked.has(p.part)) continue;
      const r = bySlugName.get(p.part);
      if (r) { seen.add(p.part); out.push({ from: p.part, slug: r.slug, name: r.name }); }
    }
    return out;
  }

  const api = { BOM_LIB, placeholderId, placeholderPart, placeholderImage, withFallbackImage, fitToPhoto, hasPins, isBoard, entries, copyLabels, relink, overQty, linkedSince };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.WiringBom = api;
})(typeof window !== 'undefined' ? window : globalThis);
