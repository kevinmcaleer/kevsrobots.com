/**
 * Project versions in the browser (epic #433): the small pieces the editor
 * and the project page share.
 *
 *  - ProjectVersions.dialog()   an in-page confirm, with an optional one-line field
 *  - ProjectVersions.date()     "2 Nov 2026" / "2 Nov 2026 14:02" from an API time
 *  - ProjectVersions.label()    what a version row is called in the drop-down
 *  - ProjectVersions.diff()     a word diff of two texts (Myers), no library
 *  - ProjectVersions.summary()  "+120 words, −35 words, 2 pictures added …"
 *  - ProjectVersions.compare()  the compare dialog (#441)
 *
 * Nothing here calls a paid service: comparing is all done on this page.
 */
(function () {
  'use strict';

  function esc(text) {
    return String(text == null ? '' : text)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  // API times are UTC without a zone ("2026-11-02T14:02:11.123").
  function parseTime(value) {
    if (!value) return null;
    const s = String(value);
    return new Date(/[zZ]|[+-]\d\d:?\d\d$/.test(s) ? s : s + 'Z');
  }

  function date(value, withTime) {
    const d = parseTime(value);
    if (!d || isNaN(d)) return '';
    const opts = { day: 'numeric', month: 'short', year: 'numeric' };
    if (withTime) { opts.hour = '2-digit'; opts.minute = '2-digit'; }
    return d.toLocaleString(undefined, opts);
  }

  // The drop-down's words for one version: "v3 · Live", "v2", or
  // "Unpublished v5 · saved before restoring v1".
  function label(v) {
    if (v.kind === 'unpublished') {
      const why = v.note && /^Saved before restoring v\d+$/.test(v.note)
        ? v.note.replace(/^Saved/, 'saved') : 'saved copy';
      return 'Unpublished v' + v.number + ' \u00b7 ' + why;
    }
    return 'v' + v.number + (v.is_live ? ' \u00b7 Live' : '');
  }

  // A version's short name in sentences: "v3", "unpublished v5".
  function name(v) {
    return (v.kind === 'unpublished' ? 'unpublished v' : 'v') + v.number;
  }

  // --- Dialog --------------------------------------------------------------

  // A small modal, so nothing relies on window.prompt. Resolves to
  // { ok, value }: value is the text field's (when there is one).
  function dialog(opts) {
    return new Promise(resolve => {
      const back = document.createElement('div');
      back.className = 'kr-version-dialog-back';
      const box = document.createElement('div');
      box.className = 'kr-version-dialog' + (opts.wide ? ' kr-version-dialog--wide' : '');
      box.setAttribute('role', 'dialog');
      box.setAttribute('aria-modal', 'true');
      box.innerHTML =
        '<h2 class="kr-version-dialog-title"></h2>' +
        '<div class="kr-version-dialog-body"></div>' +
        (opts.input
          ? '<label class="kr-version-dialog-field"><span></span>' +
            '<input type="text" class="form-control form-control-sm"></label>'
          : '') +
        '<div class="kr-version-dialog-buttons">' +
        (opts.cancel === false ? '' : '<button type="button" class="btn btn-sm btn-outline-secondary" data-act="cancel"></button>') +
        (opts.confirm ? '<button type="button" class="btn btn-sm" data-act="ok"></button>' : '') +
        '</div>';
      box.querySelector('.kr-version-dialog-title').textContent = opts.title || '';
      const body = box.querySelector('.kr-version-dialog-body');
      if (opts.html) body.innerHTML = opts.html;
      else body.textContent = opts.body || '';
      const input = box.querySelector('input');
      if (input) {
        box.querySelector('.kr-version-dialog-field span').textContent = opts.input.label || '';
        input.placeholder = opts.input.placeholder || '';
        if (opts.input.maxlength) input.maxLength = opts.input.maxlength;
        input.value = opts.input.value || '';
      }
      const cancel = box.querySelector('[data-act="cancel"]');
      if (cancel) cancel.textContent = opts.cancel || 'Cancel';
      const ok = box.querySelector('[data-act="ok"]');
      if (ok) {
        ok.textContent = opts.confirm;
        ok.classList.add(opts.danger ? 'btn-danger' : 'kr-btn-publish');
      }
      back.appendChild(box);
      document.body.appendChild(back);
      const previous = document.activeElement;
      (input || ok || cancel).focus();

      function done(result) {
        document.removeEventListener('keydown', onKey, true);
        back.remove();
        if (previous && previous.focus) previous.focus();
        resolve(result);
      }
      function onKey(e) {
        if (e.key === 'Escape') { e.preventDefault(); done({ ok: false }); }
        else if (e.key === 'Enter' && ok && (e.target === input || e.target === ok)) {
          e.preventDefault(); done({ ok: true, value: input ? input.value.trim() : undefined });
        }
      }
      document.addEventListener('keydown', onKey, true);
      back.addEventListener('mousedown', e => { if (e.target === back) done({ ok: false }); });
      if (cancel) cancel.addEventListener('click', () => done({ ok: false }));
      if (ok) ok.addEventListener('click', () => done({ ok: true, value: input ? input.value.trim() : undefined }));
      if (opts.onOpen) opts.onOpen(box);
    });
  }

  // --- Diff ----------------------------------------------------------------

  // Myers' O((N+M)D) diff of two arrays. Returns [op, item] pairs with op
  // '=' (in both), '-' (only in a) or '+' (only in b).
  function diffArrays(a, b) {
    const n = a.length, m = b.length, max = n + m;
    const v = new Map([[1, 0]]);
    const trace = [];
    outer:
    for (let d = 0; d <= max; d++) {
      trace.push(new Map(v));
      for (let k = -d; k <= d; k += 2) {
        let x = (k === -d || (k !== d && (v.get(k - 1) || 0) < (v.get(k + 1) || 0)))
          ? (v.get(k + 1) || 0) : (v.get(k - 1) || 0) + 1;
        let y = x - k;
        while (x < n && y < m && a[x] === b[y]) { x++; y++; }
        v.set(k, x);
        if (x >= n && y >= m) break outer;
      }
    }
    const out = [];
    let x = n, y = m;
    for (let d = trace.length - 1; d >= 0 && (x > 0 || y > 0); d--) {
      const vd = trace[d];
      const k = x - y;
      const prevK = (k === -d || (k !== d && (vd.get(k - 1) || 0) < (vd.get(k + 1) || 0))) ? k + 1 : k - 1;
      const prevX = vd.get(prevK) || 0;
      const prevY = prevX - prevK;
      while (x > prevX && y > prevY) { out.push(['=', a[x - 1]]); x--; y--; }
      if (d > 0) {
        if (x === prevX) out.push(['+', b[y - 1]]);
        else out.push(['-', a[x - 1]]);
      }
      x = prevX; y = prevY;
    }
    return out.reverse();
  }

  // Words and the spaces between them, so the diff can be put back together.
  function tokens(text) {
    return String(text || '').match(/\s+|[^\s]+/g) || [];
  }

  // A word diff of two texts. Lines are compared first (cheap even for a
  // long write-up), then the words of each changed stretch.
  function diff(before, after) {
    const lines = diffArrays(String(before || '').split('\n'), String(after || '').split('\n'));
    const out = [];
    let removed = [], added = [];
    function flush() {
      if (!removed.length && !added.length) return;
      const words = diffArrays(tokens(removed.join('\n')), tokens(added.join('\n')));
      words.forEach(w => out.push(w));
      out.push(['=', '\n']);
      removed = []; added = [];
    }
    lines.forEach(([op, line]) => {
      if (op === '-') removed.push(line);
      else if (op === '+') added.push(line);
      else { flush(); out.push(['=', line + '\n']); }
    });
    flush();
    // Merge neighbours with the same op.
    const merged = [];
    out.forEach(([op, t]) => {
      const last = merged[merged.length - 1];
      if (last && last[0] === op) last[1] += t;
      else merged.push([op, t]);
    });
    return merged;
  }

  function countWords(text) {
    return (String(text || '').match(/[^\s]+/g) || []).length;
  }

  // --- Summary of two snapshots -------------------------------------------

  const FIELDS = [
    ['title', 'Title'],
    ['short_description', 'Description'],
    ['difficulty', 'Difficulty'],
    ['estimated_minutes', 'Build time'],
    ['tags', 'Tags'],
  ];

  function same(a, b) { return JSON.stringify(a) === JSON.stringify(b); }

  function summary(before, after) {
    before = before || {}; after = after || {};
    const parts = [];
    let plus = 0, minus = 0;
    diff(before.content_md, after.content_md).forEach(([op, t]) => {
      if (op === '+') plus += countWords(t);
      if (op === '-') minus += countWords(t);
    });
    if (plus) parts.push('+' + plus + (plus === 1 ? ' word' : ' words'));
    if (minus) parts.push('−' + minus + (minus === 1 ? ' word' : ' words'));
    const ids = s => new Set((s.images || []).map(i => i.id));
    const was = ids(before), now = ids(after);
    const addedPics = [...now].filter(i => !was.has(i)).length;
    const removedPics = [...was].filter(i => !now.has(i)).length;
    if (addedPics) parts.push(addedPics + (addedPics === 1 ? ' picture added' : ' pictures added'));
    if (removedPics) parts.push(removedPics + (removedPics === 1 ? ' picture removed' : ' pictures removed'));
    const wiring = s => s.wiring ? s.wiring.data : null;
    if (!same(wiring(before), wiring(after))) parts.push('wiring changed');
    const diagrams = s => Object.fromEntries(Object.entries(s.diagrams || {}).map(([k, d]) => [k, d.data]));
    if (!same(diagrams(before), diagrams(after))) parts.push('diagrams changed');
    const row = r => [r.name, r.quantity, r.unit_cost, r.currency_code, r.part_id].join('\u0001');
    const bomA = (before.bom || []).map(row), bomB = (after.bom || []).map(row);
    let bomChanged = 0;
    diffArrays(bomA, bomB).forEach(([op]) => { if (op !== '=') bomChanged++; });
    // A changed row is one removal and one addition: count it once.
    const changedRows = Math.max(
      bomA.filter(r => !bomB.includes(r)).length,
      bomB.filter(r => !bomA.includes(r)).length,
    );
    if (bomChanged) parts.push('BOM: ' + changedRows + (changedRows === 1 ? ' row changed' : ' rows changed'));
    const fileIds = s => (s.files || []).map(f => f.id).sort().join(',');
    if (fileIds(before) !== fileIds(after)) parts.push('files changed');
    const meta = FIELDS
      .filter(([key]) => !same(key === 'tags' ? [...(before.tags || [])].sort() : before[key],
                               key === 'tags' ? [...(after.tags || [])].sort() : after[key]))
      .map(([key, name]) => ({
        name,
        before: key === 'tags' ? (before.tags || []).join(', ') : before[key],
        after: key === 'tags' ? (after.tags || []).join(', ') : after[key],
      }));
    return { parts, meta, text: parts.length ? parts.join(', ') : (meta.length ? '' : 'No changes') };
  }

  // --- Compare dialog (#441) -----------------------------------------------

  // before / after: { label, snapshot } (a snapshot is what the API keeps
  // for a version; the working copy is shaped the same way).
  function compare(before, after) {
    const s = summary(before.snapshot, after.snapshot);
    const pieces = diff(before.snapshot.content_md, after.snapshot.content_md);
    const side = which => pieces
      .filter(([op]) => op === '=' || op === which)
      .map(([op, t]) => op === '=' ? esc(t) : '<span class="kr-diff-' + (op === '+' ? 'add' : 'del') + '">' + esc(t) + '</span>')
      .join('');
    const meta = s.meta.length
      ? '<table class="kr-compare-meta"><tbody>' + s.meta.map(m =>
          '<tr><th>' + esc(m.name) + '</th><td><span class="kr-diff-del">' + esc(m.before == null || m.before === '' ? '(none)' : m.before) +
          '</span></td><td><span class="kr-diff-add">' + esc(m.after == null || m.after === '' ? '(none)' : m.after) + '</span></td></tr>'
        ).join('') + '</tbody></table>'
      : '';
    const note = after.note ? '<p class="kr-compare-note">“' + esc(after.note) + '”</p>' : '';
    const html =
      '<p class="kr-compare-summary">' + esc(s.text) + '</p>' + note + meta +
      '<div class="kr-compare">' +
        '<div class="kr-compare-col"><h3>' + esc(before.label) + '</h3><pre>' + side('-') + '</pre></div>' +
        '<div class="kr-compare-col"><h3>' + esc(after.label) + '</h3><pre>' + side('+') + '</pre></div>' +
      '</div>';
    return dialog({
      title: 'Compare ' + before.label + ' with ' + after.label,
      html, wide: true, cancel: 'Close',
    });
  }

  window.ProjectVersions = { dialog, date, label, name, diff, diffArrays, summary, compare, parseTime, esc };
})();
