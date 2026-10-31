// Image captions in a project's Markdown: a picture on a line of its own
// shows its title (the alt text) as a caption beneath it.
//
//   ![BDX Droids at Disney](/bdx.jpg){half}
//
// The editor's live view and preview caption pictures themselves
// (markdown-studio's imageCaption.ts). The view page renders with marked, so
// render() wraps each lone picture in the same figure afterwards:
// <figure class="image-figure image-figure-half"><img><figcaption>. Its
// styles are in markdown-studio's image-sizes.css, vendored beside the
// editor bundle. Keep caption() in step with imageCaption() there: titles
// that look made from a file name (IMG_2041, robot-arm, chassis, duck.jpg,
// hero) aren't shown. Run it after ProjectImageSizes and ProjectGallery.
(function () {
  const PLACEHOLDERS = /^(image|img|picture|pic|photo|screenshot|hero|banner|alt|untitled)$/i;
  const SIZES = ['full', 'half', 'cover', 'small'];

  // The caption for alt text `alt`, or null when it has none worth showing.
  function caption(alt) {
    const text = (alt || '').trim();
    if (!text || PLACEHOLDERS.test(text)) return null;
    if (/\.(png|jpe?g|gif|webp|svg|avif|heic|bmp|tiff?)$/i.test(text)) return null;
    if (!/\s/.test(text) && (/[-_\d]/.test(text) || text === text.toLowerCase())) return null;
    return text;
  }

  // Captions each lone picture under `root`; tables, galleries and cards keep their own look.
  function render(root) {
    root.querySelectorAll('p').forEach(function (p) {
      if (p.children.length !== 1 || p.firstElementChild.tagName !== 'IMG') return;
      if (p.textContent.trim()) return;
      if (p.closest('table, .md-gallery, .md-cards, .md-videos, figure')) return;
      const img = p.firstElementChild;
      const text = caption(img.getAttribute('alt'));
      if (!text) return;
      const size = SIZES.find(function (s) { return img.classList.contains('image-' + s); });
      const figure = document.createElement('figure');
      figure.className = 'image-figure' + (size ? ' image-figure-' + size : '');
      const figcaption = document.createElement('figcaption');
      figcaption.textContent = text;
      figure.append(img, figcaption);
      p.replaceWith(figure);
    });
  }

  window.ProjectImageCaptions = { render: render, caption: caption };
})();
