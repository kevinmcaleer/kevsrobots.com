// Headings in table cells: a Markdown table cell only holds inline text, so
// a cell that starts with #'s is drawn as a heading of that level instead,
// such as a heading over each column of an image grid.
//
//   | ## Open Duck Mini | ## Microduck |
//   |---|---|
//   | ![Open Duck Mini](/odm.jpg) | ![Microduck](/md.jpg) |
//
// The editor draws these itself (markdown-studio's bundle, cellHeadings.ts,
// as <h2 class="cell-heading">); the view page renders with marked, so
// render() wraps the cell's content the same way afterwards. Keep the rule
// in step with cellHeading() there. They're left out of the page's
// contents and heading links (view.html skips .cell-heading).
(function () {
  const OPEN = /^\s*(#{1,6})[ \t]+/;
  const CLOSE = /(?:[ \t]+#+)?[ \t]*$/;

  function heading(cell) {
    const first = cell.firstChild;
    const last = cell.lastChild;
    if (!first || first.nodeType !== 3 || !last || last.nodeType !== 3) return;
    const match = OPEN.exec(first.nodeValue);
    if (!match) return;
    first.nodeValue = first.nodeValue.slice(match[0].length);
    last.nodeValue = last.nodeValue.replace(CLOSE, '');
    if (!cell.textContent.trim()) {
      first.nodeValue = match[0] + first.nodeValue;
      return;
    }
    const h = document.createElement('h' + match[1].length);
    h.className = 'cell-heading';
    while (cell.firstChild) h.appendChild(cell.firstChild);
    cell.appendChild(h);
  }

  // Draws each table cell under `root` that starts with #'s as a heading.
  function render(root) {
    root.querySelectorAll('table th, table td').forEach(heading);
  }

  window.ProjectCellHeadings = { render: render };
})();
