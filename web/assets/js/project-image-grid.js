// Image grids in a project's Markdown: a table whose cells hold images is a
// layout, so it shows as a borderless grid of image cards, one column per
// table column.
//
//   | | | |
//   |---|---|---|
//   | ![Base](/base.jpg) | ![Lid](/lid.jpg) The lid | ![Buttons](/buttons.jpg) |
//
// Text beside an image is its card's caption. An empty header row is hidden;
// a header row of plain text labels the columns. The editor marks these
// tables itself (markdown-studio's bundle); the view page renders with
// marked, so render() adds the same classes afterwards. The styles are
// markdown-studio's image-grid.css, vendored beside the editor bundle by
// scripts/update-markdown-editor.sh. Keep isImageGrid in step with
// markdown-studio's packages/markdown-editor/src/imageGrid.ts. Run it after
// ProjectImageSizes.render, so a `{size}` isn't taken for a caption.
(function () {
  function hasImage(cell) {
    return !!cell.querySelector('img');
  }

  function isEmpty(cell) {
    return !hasImage(cell) && !cell.textContent.trim();
  }

  // Has an image, and every filled cell holds one, leaving out a header row
  // with no images in it (its column labels).
  function isImageGrid(table) {
    const rows = Array.from(table.rows);
    if (!rows.some(tr => Array.from(tr.cells).some(hasImage))) return false;
    const head = table.tHead ? Array.from(table.tHead.rows) : [];
    const headHasImage = head.some(tr => Array.from(tr.cells).some(hasImage));
    const checked = headHasImage ? rows : rows.filter(tr => !head.includes(tr));
    return checked.every(tr => Array.from(tr.cells).every(cell => isEmpty(cell) || hasImage(cell)));
  }

  function grid(table) {
    if (!isImageGrid(table)) return;
    table.classList.add('image-grid');
    const head = table.tHead ? Array.from(table.tHead.querySelectorAll('th, td')) : [];
    if (head.length && head.every(isEmpty)) table.classList.add('image-grid-nohead');
    table.querySelectorAll('th, td').forEach(cell => {
      if (hasImage(cell)) cell.classList.add('image-card');
    });
  }

  // Turns each table of images under `root` into a grid of image cards.
  function render(root) {
    root.querySelectorAll('table').forEach(grid);
  }

  window.ProjectImageGrid = { render: render, isImageGrid: isImageGrid };
})();
