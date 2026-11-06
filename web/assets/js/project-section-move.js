// Moving a whole section of a project's Markdown, for dragging headings in
// the editor's "On this page" (project-toc.js).
//
// ProjectSectionMove.move(markdown, headings, fromLine, toLine, after)
//   headings  ProjectSections.headings(markdown): [{ level, line }], lines
//             from 0, outside code fences and cards.
//   Moves the section headed on `fromLine` (its heading and everything
//   under it, up to the next heading at its level or above) to before the
//   section headed on `toLine`, or after it when `after`. Sections keep one
//   blank line between them.
// Returns { doc, line } (`line`: where the moved heading now is), or null
// when there's nothing to move.
(function (root) {
  function sectionRange(headings, at, total) {
    const h = headings[at];
    const next = headings.slice(at + 1).find(n => n.level <= h.level);
    return [h.line, next ? next.line : total];
  }

  const blank = line => !line.trim();

  function move(markdown, headings, fromLine, toLine, after) {
    const lines = String(markdown || '').split('\n');
    const from = headings.findIndex(h => h.line === fromLine);
    const to = headings.findIndex(h => h.line === toLine);
    if (from < 0 || to < 0 || from === to) return null;

    const [s0, s1] = sectionRange(headings, from, lines.length);
    const [t0, t1] = sectionRange(headings, to, lines.length);
    const at = after ? t1 : t0;
    // Into itself, or to where it already is.
    if (at >= s0 && at <= s1) return null;

    const chunk = lines.slice(s0, s1);
    while (chunk.length && blank(chunk[chunk.length - 1])) chunk.pop();
    const rest = lines.slice(0, s0).concat(lines.slice(s1));
    let p = at > s1 ? at - (s1 - s0) : at;

    const insert = chunk.slice();
    if (p > 0 && !blank(rest[p - 1])) insert.unshift('');
    if (p < rest.length && !blank(rest[p])) insert.push('');
    // The section was last: what's now last loses the blank lines it had
    // before the moved section, so the end of the document stays the same.
    rest.splice(p, 0, ...insert);
    const endedWithNewline = lines.length > 1 && lines[lines.length - 1] === '';
    while (rest.length > 1 && blank(rest[rest.length - 1])) rest.pop();
    if (endedWithNewline) rest.push('');

    const line = p + (insert[0] === '' ? 1 : 0);
    return { doc: rest.join('\n'), line };
  }

  const api = { move };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.ProjectSectionMove = api;
})(typeof window !== 'undefined' ? window : this);
