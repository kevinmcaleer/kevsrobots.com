// Table column widths in a project's Markdown, kept in plain GFM the way
// Pandoc does it: each column's share of the dashes on the delimiter row is
// its share of the table's width (colons don't count).
//
//   | Part | Notes |
//   | ----- | --------------- |      <- 5 and 15 dashes: 25% and 75%
//
// Equal dashes (`| --- | --- |`, as most tables are) set nothing. The editor
// writes the dashes when a column edge is dragged (markdown-studio's bundle);
// the view page renders with marked, so this adds the same <colgroup> and
// `has-widths` class to marked's tables. Keep the rule in step with
// markdown-studio's packages/markdown-editor/src/tableWidths.ts. Load it after
// marked and before anything calls marked.parse.
(function () {
  // The dash count of each cell on a delimiter line, or null if it isn't one.
  function delimiterDashes(line) {
    let text = line.replace(/^\s*(?:>\s?)*/, '').trim();
    if (text.startsWith('|')) text = text.slice(1);
    if (text.endsWith('|')) text = text.slice(0, -1);
    const cells = text.split('|').map(cell => cell.trim());
    if (!cells.length || !cells.every(cell => /^:?-+:?$/.test(cell))) return null;
    return cells.map(cell => cell.replace(/:/g, '').length);
  }

  // Each column's width in percent, or null when the dash counts are all the same.
  function columnWidths(dashes) {
    if (!dashes || dashes.length < 2 || dashes.every(n => n === dashes[0])) return null;
    const total = dashes.reduce((sum, n) => sum + n, 0);
    return dashes.map(n => (n / total) * 100);
  }

  function colgroup(widths) {
    return '<colgroup>' + widths.map(w => '<col style="width: ' + +w.toFixed(2) + '%">').join('') + '</colgroup>';
  }

  // The widths a marked table token's delimiter row sets, or null.
  function tokenWidths(token) {
    const line = String(token.raw || '').split('\n').filter(l => l.trim())[1];
    const dashes = line ? delimiterDashes(line) : null;
    if (!dashes || dashes.length !== (token.header || []).length) return null;
    return columnWidths(dashes);
  }

  if (typeof marked === 'undefined' || !marked.use || !marked.Renderer) return;
  const defaultTable = marked.Renderer.prototype.table;
  marked.use({
    renderer: {
      table(token) {
        const html = defaultTable.call(this, token);
        const widths = tokenWidths(token);
        if (!widths) return html;
        return html.replace(/^<table>/, '<table class="has-widths">' + colgroup(widths));
      },
    },
  });

  window.ProjectTableWidths = { delimiterDashes, columnWidths };
})();
