// Image sizes in a project's Markdown: a `{size}` straight after an image.
//
//   ![Chassis](/chassis.jpg){small}
//
// full is the width of the text, half half the size (two on a line sit side
// by side), cover the site's .cover banner (main.scss) and small a quarter
// of the size. The editor's preview renders them itself (markdown-studio's
// bundle); the view page renders with marked, which leaves the `{size}` as
// text after the image, so render() turns it into the image's class
// afterwards. The styles are markdown-studio's image-sizes.css, vendored
// beside the editor bundle by scripts/update-markdown-editor.sh. Keep SIZES
// in step with imageSizes in markdown-studio's
// packages/markdown-editor/src/imageSize.ts.
(function () {
  const SIZES = ['full', 'half', 'cover', 'small'];
  const SIZE_RE = new RegExp('^\\{(' + SIZES.join('|') + ')\\}');

  function size(img) {
    const next = img.nextSibling;
    if (!next || next.nodeType !== Node.TEXT_NODE) return;
    const match = SIZE_RE.exec(next.nodeValue);
    if (!match) return;
    next.nodeValue = next.nodeValue.slice(match[0].length);
    if (!next.nodeValue) next.remove();
    img.classList.add('image-' + match[1]);
    if (match[1] === 'cover') img.classList.add('cover');
  }

  // Sizes each image under `root` that has a `{size}` after it.
  function render(root) {
    root.querySelectorAll('img').forEach(size);
  }

  window.ProjectImageSizes = { render: render, SIZES: SIZES };
})();
