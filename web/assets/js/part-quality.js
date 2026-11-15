// How complete a catalog part is, and which catalog part a Bill of
// Materials row is most likely meant to be.
//
// A part is "complete" when it has the three things that make it work
// everywhere on the site: a 3-D model (3D scenes), pins (wiring and the
// pinout table) and a curated wiring picture (the photo its pins were
// placed over). GET /api/parts/_catalog says which a part has (has_mesh,
// pin_count, curated_photo). The parts catalog shows them as marks on each
// card, and the wiring workspace's Bill of Materials panel uses suggest()
// to point a row at a more complete part with a matching name: a
// home-made "SG90 Servo" at the catalog's "Tower Pro SG90".
//
// Pure helpers, tested in scripts/tests/part-quality.test.js.
(function (root) {
  // Words that say what kind of thing a part is rather than which one.
  const STOP = new Set(['a', 'an', 'the', 'and', 'for', 'with', 'of', 'x', 'pcs', 'pc', 'pack', 'set', 'kit', 'mini', 'micro', 'small', 'module', 'board', 'breakout']);

  // A part's quality from its catalog entry:
  //   { model, pins, photo, verified, score (0–3), complete, missing: [...] }
  function quality(p) {
    const q = {
      model: !!(p && p.has_mesh),
      pins: !!(p && p.pin_count > 0),
      photo: !!(p && p.curated_photo),
      verified: !!(p && p.status === 'verified'),
    };
    q.score = (q.model ? 1 : 0) + (q.pins ? 1 : 0) + (q.photo ? 1 : 0);
    q.complete = q.score === 3;
    q.missing = [q.model ? null : '3D model', q.pins ? null : 'pins', q.photo ? null : 'picture'].filter(Boolean);
    return q;
  }

  // What it has, in words: "a 3D model, pins and a curated picture".
  function describe(p) {
    const q = quality(p);
    const has = [q.model ? 'a 3D model' : null, q.pins ? (p.pin_count + (p.pin_count === 1 ? ' pin' : ' pins')) : null, q.photo ? 'a curated picture' : null].filter(Boolean);
    if (!has.length) return 'no 3D model, pins or curated picture yet';
    return has.length === 1 ? has[0] : has.slice(0, -1).join(', ') + ' and ' + has[has.length - 1];
  }

  // The marks a card or row shows: [{ key, label, on }].
  function marks(p) {
    const q = quality(p);
    return [
      { key: 'model', label: q.model ? 'Has a 3D model' : 'No 3D model yet', on: q.model },
      { key: 'pins', label: q.pins ? 'Has pins (' + p.pin_count + ')' : 'No pins yet', on: q.pins },
      { key: 'photo', label: q.photo ? 'Has a curated wiring picture' : 'No curated wiring picture yet', on: q.photo },
    ];
  }

  // Ranks better parts first: more of the three, verified, then more used.
  function compare(a, b) {
    const qa = quality(a);
    const qb = quality(b);
    return qb.score - qa.score || (qb.verified - qa.verified) || ((b.usage_count || 0) - (a.usage_count || 0));
  }

  // A name's words: lowercase, "SG-90" as "sg90", without the generic
  // ones or a count ("x2").
  function words(name) {
    const text = String(name || '').toLowerCase().normalize('NFKD').replace(/[̀-ͯ]/g, '')
      .replace(/([a-z])[-\s]+(?=\d)/g, '$1');
    return [...new Set(text.split(/[^a-z0-9.]+/).map(w => w.replace(/^\.+|\.+$/g, '')).filter(w => w && !STOP.has(w) && !/^(x\d+|\d+x|\d+pcs)$/.test(w)))];
  }

  // A model number: letters and digits together ("sg90", "bno055",
  // "mg996r") or a long number ("18650"). Two parts sharing one are very
  // likely the same thing.
  const isModel = w => (/[a-z]/.test(w) && /\d/.test(w) && w.length >= 3) || /^\d{4,}$/.test(w);

  // How alike two names are, 0–1. Names sharing a model number score at
  // least 0.6; names whose model numbers differ ("SG90" and "MG90S") 0.
  // Otherwise the share of words in common.
  function similarity(a, b) {
    const wa = words(a);
    const wb = words(b);
    if (!wa.length || !wb.length) return 0;
    const ma = wa.filter(isModel);
    const mb = wb.filter(isModel);
    const shared = wa.filter(w => wb.includes(w));
    const all = new Set([...wa, ...wb]).size;
    const overlap = shared.length / all;
    if (ma.length && mb.length) {
      const sharedModels = ma.filter(m => mb.includes(m));
      if (!sharedModels.length) return 0;
      return Math.min(1, 0.6 + 0.4 * overlap);
    }
    return overlap;
  }

  // A more complete catalog part for the Bill of Materials row `row`
  // ({ name, slug }), or null. `parts` is the catalog list. The row's own
  // part (by slug) is what it has to beat: more of the three marks, or as
  // many and verified when it isn't. A row with no part has to be matched
  // closely, as Link does it in one press. Disputed parts aren't offered.
  //   -> { part, similarity, quality }
  function suggest(row, parts, opts) {
    opts = opts || {};
    const min = opts.min != null ? opts.min : 0.6;
    const current = row.slug ? parts.find(p => p.slug === row.slug) || null : null;
    const names = [row.name, current && current.name].filter(Boolean);
    const cq = current ? quality(current) : null;
    let best = null;
    for (const p of parts) {
      if (!p || p.slug === row.slug || p.status === 'disputed' || String(p.status || '').startsWith('merged')) continue;
      const q = quality(p);
      if (!q.score) continue;
      if (cq && !(q.score > cq.score || (q.score === cq.score && q.verified && !cq.verified))) continue;
      const sim = Math.max(...names.map(n => similarity(n, p.name)));
      if (sim < min) continue;
      const pick = { part: p, similarity: sim, quality: q };
      if (!best || compare(best.part, p) > 0 || (compare(best.part, p) === 0 && sim > best.similarity)) best = pick;
    }
    return best;
  }

  // What to search the catalog for to find a row's part: its model
  // numbers when it has any ("SG90 Servo" -> "sg90", which finds "Tower Pro
  // SG90" too), else the name as it is.
  function searchQuery(name) {
    const models = words(name).filter(isModel);
    return models.length ? models.join(' ') : String(name || '').trim();
  }

  const api = { quality, describe, marks, compare, words, similarity, suggest, searchQuery };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.PartQuality = api;
})(typeof window !== 'undefined' ? window : globalThis);
