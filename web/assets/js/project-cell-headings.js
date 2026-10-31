// Lines and headings in table cells: a Markdown table cell only holds inline
// text, so a table used for layout (picture cards with a title and a few
// words under each picture) gets two helpers:
//
// - `<br>` starts a new line in the cell (Shift+Enter in the editor types it).
// - A line that starts with #'s is drawn as a heading of that level. A
//   heading straight after a picture starts its own line, `<br>` or not.
//
//   | ## Open Duck Mini | ## Microduck |
//   |---|---|
//   | ![Open Duck Mini](/odm.jpg)<br>## Open Duck Mini<br>By Antoine | ![Microduck](/md.jpg) ## Microduck |
//
// The editor draws these itself (markdown-studio's bundle, cellHeadings.ts'
// cellLines, as <h2 class="cell-heading">); the view page renders with
// marked, so render() rebuilds each cell the same way afterwards. Keep the
// rule in step with cellLines() there. They're left out of the page's
// contents and heading links (view.html skips .cell-heading).
(function () {
  const OPEN = /^\s*(#{1,6})[ \t]+/;
  const CLOSE = /(?:[ \t]+#+)?[ \t]*$/;

  function isBreak(node) {
    return node.nodeType === 1 && node.tagName === 'BR';
  }

  function isPicture(node) {
    return !!node && node.nodeType === 1 && (node.tagName === 'IMG' || !!node.querySelector('img'));
  }

  // The previous node that isn't blank text.
  function previous(nodes, i) {
    for (let j = i - 1; j >= 0; j--) {
      const node = nodes[j];
      if (node.nodeType !== 3 || node.nodeValue.trim()) return node;
    }
    return null;
  }

  // A line's heading level: #'s then a space, with something after them.
  function levelOf(line) {
    const first = line[0];
    if (!first || first.nodeType !== 3) return 0;
    const match = OPEN.exec(first.nodeValue);
    if (!match) return 0;
    const rest = first.nodeValue.slice(match[0].length).trim();
    return rest || line.length > 1 ? match[1].length : 0;
  }

  function lines(cell) {
    const nodes = Array.from(cell.childNodes);
    const out = [[]];
    nodes.forEach((node, i) => {
      if (isBreak(node)) {
        out.push([]);
        return;
      }
      if (node.nodeType === 3 && OPEN.test(node.nodeValue) && node.nodeValue.trim().replace(/^#+/, '') && isPicture(previous(nodes, i))) {
        out.push([]);
      }
      // Leading blank text isn't part of a line.
      const line = out[out.length - 1];
      if (!line.length && node.nodeType === 3 && !node.nodeValue.trim()) return;
      line.push(node);
    });
    return out;
  }

  function render(cell) {
    const split = lines(cell);
    if (!split.some(levelOf)) return;
    const frag = document.createDocumentFragment();
    let lastHeading = true;
    split.forEach((line, index) => {
      const level = levelOf(line);
      const first = line[0];
      const last = line[line.length - 1];
      if (level) {
        first.nodeValue = first.nodeValue.replace(OPEN, '');
        if (last.nodeType === 3) last.nodeValue = last.nodeValue.replace(CLOSE, '');
        const h = document.createElement('h' + level);
        h.className = 'cell-heading';
        line.forEach(node => h.appendChild(node));
        frag.appendChild(h);
        lastHeading = true;
      } else {
        if (first && first.nodeType === 3) first.nodeValue = first.nodeValue.replace(/^\s+/, '');
        if (last && last.nodeType === 3) last.nodeValue = last.nodeValue.replace(/\s+$/, '');
        if (index > 0 && !lastHeading) frag.appendChild(document.createElement('br'));
        line.forEach(node => frag.appendChild(node));
        lastHeading = false;
      }
    });
    cell.replaceChildren(frag);
  }

  // Splits each table cell under `root` into lines and draws the ones that
  // start with #'s as headings.
  function renderAll(root) {
    root.querySelectorAll('table th, table td').forEach(render);
  }

  window.ProjectCellHeadings = { render: renderAll };
})();
