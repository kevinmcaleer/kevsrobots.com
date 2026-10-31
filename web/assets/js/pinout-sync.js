// The pinout table that the wiring workspace puts in a project's Markdown,
// and whether it still matches the wiring diagram.
//
// "Insert pinout table" writes a hidden marker line above the table:
//
//   <!-- pinout from the wiring diagram: 1a2b3c4d -->
//   | MCU Pin | Part | Part Pin | Net |
//   ...
//
// The number is a hash of the table as it was made. Hashing the table the
// wiring makes now tells whether the diagram changed since (stale); hashing
// the table in the text tells whether the writer edited it (edited). The
// marker is an HTML comment, so the published page doesn't show it.
// A pinout table from before the marker (no marker line) is found by its
// MCU Pin | Part | Part Pin | Net header; whether it was edited is unknown.
//
// Pure functions, no DOM (tests: node --test scripts/tests/*.test.js).
(function (root) {
  const MARKER_RE = /^<!--\s*pinout\b[^>]*?\b([0-9a-f]{8})\s*-->\s*$/i;
  const MCU_HEADER = ['mcu pin', 'part', 'part pin', 'net'];
  const OTHER_RE = /^#{2,6}\s+other connections\s*#*\s*$/i;
  const isTableLine = line => /^\s*\|/.test(line);
  const isBlank = line => !line.trim();

  // A table line's cells, trimmed (an escaped \| stays in its cell).
  function cells(line) {
    const inner = line.trim().replace(/^\|/, '').replace(/(?<!\\)\|$/, '');
    return inner.split(/(?<!\\)\|/).map(c => c.trim());
  }

  // The generated Markdown (handle.pinoutMarkdown()) without its "# … —
  // Pinouts" heading: the table(s) alone, or '' when nothing is wired.
  function fromGenerated(md) {
    const block = String(md || '').replace(/^\s*#{1,6} [^\n]*\n+/, '').trim();
    return isTableLine(block) ? block : '';
  }

  // The block as compared: one line per table row with its cells trimmed, so
  // the editor padding or re-spacing a table isn't an edit.
  function normalize(block) {
    return String(block || '').split('\n').filter(l => !isBlank(l)).map(line => {
      if (!isTableLine(line)) return line.trim().replace(/\s+/g, ' ');
      return cells(line).map(c => (/^:?-{3,}:?$/.test(c) ? c.replace(/-+/, '---') : c.replace(/\s+/g, ' '))).join('|');
    }).join('\n');
  }

  // FNV-1a, 32 bits, as 8 hex digits.
  function hash(block) {
    const s = normalize(block);
    let h = 0x811c9dc5;
    for (let i = 0; i < s.length; i++) {
      h ^= s.charCodeAt(i);
      h = Math.imul(h, 0x01000193) >>> 0;
    }
    return h.toString(16).padStart(8, '0');
  }

  function marker(block) {
    return '<!-- pinout from the wiring diagram: ' + hash(block) + ' -->';
  }

  // What "Insert pinout table" puts in the text: the marker and the block.
  function withMarker(block) {
    return marker(block) + '\n' + String(block).trim();
  }

  // The lines of a pinout block starting at line `start` (a table line):
  // the table, then an "## Other connections" heading and its table when
  // they follow. Returns the index of the block's last line.
  function blockEnd(lines, start) {
    let end = start;
    while (end + 1 < lines.length && isTableLine(lines[end + 1])) end++;
    let i = end + 1;
    while (i < lines.length && isBlank(lines[i])) i++;
    if (i < lines.length && OTHER_RE.test(lines[i].trim())) {
      let j = i + 1;
      while (j < lines.length && isBlank(lines[j])) j++;
      if (j < lines.length && isTableLine(lines[j])) {
        end = j;
        while (end + 1 < lines.length && isTableLine(lines[end + 1])) end++;
      }
    }
    return end;
  }

  // The pinout table in `markdown`: { from, to, hash, block, header }, where
  // from/to cover the marker line (when there is one) to the block's end,
  // hash is the marker's (null without one), block is the table text and
  // header its first row's cells. Null when there's none. Skips fenced code.
  function find(markdown) {
    const text = String(markdown || '');
    const lines = text.split('\n');
    const starts = [];
    let at = 0;
    for (const line of lines) { starts.push(at); at += line.length + 1; }
    const span = (fromLine, firstTable, markHash) => {
      const end = blockEnd(lines, firstTable);
      const to = starts[end] + lines[end].length;
      return {
        from: starts[fromLine], to, hash: markHash,
        block: text.slice(starts[firstTable], to),
        header: cells(lines[firstTable]),
      };
    };
    let fence = null;
    let unmarked = null;
    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      const f = line.match(/^\s{0,3}(`{3,}|~{3,})/);
      if (fence) { if (f && f[1][0] === fence[0] && f[1].length >= fence.length) fence = null; continue; }
      if (f) { fence = f[1]; continue; }
      const m = line.trim().match(MARKER_RE);
      if (m) {
        if (i + 1 < lines.length && isTableLine(lines[i + 1])) return span(i, i + 1, m[1].toLowerCase());
        continue;
      }
      if (!unmarked && isTableLine(line) && (i === 0 || !isTableLine(lines[i - 1])) &&
          cells(line).map(c => c.replace(/[*_`]/g, '').toLowerCase()).join('|') === MCU_HEADER.join('|')) {
        unmarked = span(i, i, null);
      }
    }
    return unmarked;
  }

  // How the table in the text stands against `fresh`, the block the wiring
  // makes now: { stale, edited }. edited is null for a table without a marker.
  function compare(found, fresh) {
    const now = hash(found.block);
    const made = found.hash || now;
    return {
      stale: hash(fresh) !== made,
      edited: found.hash ? now !== found.hash : null,
    };
  }

  // Whether a comment (its text, <!-- to -->) is the marker.
  function isMarker(comment) {
    return MARKER_RE.test(String(comment || '').trim());
  }

  const api = { isMarker, cells, fromGenerated, normalize, hash, marker, withMarker, find, compare, MCU_HEADER };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.PinoutSync = api;
})(typeof window !== 'undefined' ? window : globalThis);
