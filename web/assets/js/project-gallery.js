// Galleries in a project's Markdown: the image lines between `::: gallery`
// and `:::` show as one gallery, in the order they're written.
//
//   ::: gallery filmstrip
//   ![Front](/front.jpg)
//   ![Wiring](/wiring.jpg "Before the lid went on")
//   :::
//
// Layouts: filmstrip (the first picture large, thumbnails swap it), bento
// (tiles, the first one big) or lightbox (even thumbnails). Any picture opens
// in the shared ImageViewer. The editor draws these itself (markdown-studio's
// bundle); the view page renders with marked, so this file adds a marked
// extension that writes the same HTML. The styles are markdown-studio's
// gallery.css, vendored beside the editor bundle by
// scripts/update-markdown-editor.sh. Keep it in step with markdown-studio's
// packages/markdown-editor/src/gallery.ts.
(function () {
  const LAYOUTS = ['filmstrip', 'bento', 'lightbox'];
  const OPEN = /^[ \t]{0,3}:::[ \t]*gallery\b[ \t]*([\w-]*)[ \t]*$/i;
  const CLOSE = /^[ \t]{0,3}:::[ \t]*$/;
  const IMAGE = /!\[((?:\\.|[^\]\\])*)\]\(\s*(<[^>\n]*>|[^\s)]+)(?:\s+(?:"([^"]*)"|'([^']*)'))?\s*\)/g;

  function layoutOf(line) {
    const m = OPEN.exec(line);
    if (!m) return null;
    const word = m[1].toLowerCase();
    return LAYOUTS.includes(word) ? word : 'filmstrip';
  }

  function unescapeMarkdown(text) {
    return text.replace(/\\([\\`*_{}[\]()#+\-.!|"'<>~])/g, '$1');
  }

  // The images in a gallery's lines, in order: { src, alt, title }.
  function images(body) {
    const out = [];
    for (const m of String(body).matchAll(IMAGE)) {
      const src = m[2].replace(/^<|>$/g, '').trim();
      if (src) out.push({ src, alt: unescapeMarkdown(m[1]).trim(), title: (m[3] || m[4] || '').trim() });
    }
    return out;
  }

  function esc(text) {
    return String(text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  function safeUrl(src) {
    const url = src.trim();
    if (/^(javascript|vbscript|file):/i.test(url.replace(/[\s\u0000-\u001f]/g, ''))) return null;
    if (/^data:/i.test(url) && !/^data:image\/(png|gif|jpe?g|webp|avif);/i.test(url)) return null;
    return url;
  }

  // The same markup as gallery.ts' galleryHtml.
  function html(layout, list) {
    const safe = list.map(i => Object.assign({}, i, { src: safeUrl(i.src) })).filter(i => i.src);
    const caption = i => i.title || i.alt;
    const items = safe.map((i, n) => {
      const title = caption(i);
      return '<a class="md-gallery-item' + (layout === 'filmstrip' && n === 0 ? ' is-active' : '') + '" href="' + esc(i.src) +
        '" target="_blank" rel="noopener noreferrer" data-gallery-index="' + n + '"' + (title ? ' title="' + esc(title) + '"' : '') + '>' +
        '<img src="' + esc(i.src) + '" alt="' + esc(i.alt) + '" loading="lazy"></a>';
    }).join('');
    let hero = '';
    if (layout === 'filmstrip' && safe.length) {
      const first = safe[0];
      const title = caption(first);
      hero = '<figure class="md-gallery-hero"><img src="' + esc(first.src) + '" alt="' + esc(first.alt) + '">' +
        '<figcaption' + (title ? '' : ' hidden') + '>' + esc(title) + '</figcaption></figure>';
    }
    return '<div class="md-gallery md-gallery-' + layout + (safe.length ? '' : ' md-gallery-empty') + '" data-gallery-layout="' + layout +
      '" data-gallery-count="' + safe.length + '">' + hero + '<div class="md-gallery-items">' + items + '</div></div>';
  }

  // marked: a closed `::: gallery` block at the start of `src`, or nothing.
  const extension = {
    name: 'gallery',
    level: 'block',
    start(src) {
      const m = /^[ \t]{0,3}:::[ \t]*gallery\b/im.exec(src);
      return m ? m.index : undefined;
    },
    tokenizer(src) {
      const lines = src.split('\n');
      const layout = layoutOf(lines[0]);
      if (!layout) return undefined;
      for (let i = 1; i < lines.length; i++) {
        if (OPEN.test(lines[i])) return undefined;
        if (CLOSE.test(lines[i])) {
          const raw = lines.slice(0, i + 1).join('\n') + (i + 1 < lines.length ? '\n' : '');
          return { type: 'gallery', raw, layout, images: images(lines.slice(1, i).join('\n')) };
        }
      }
      return undefined;
    },
    renderer(token) {
      return html(token.layout, token.images) + '\n';
    },
  };

  if (window.marked && typeof window.marked.use === 'function') window.marked.use({ extensions: [extension] });

  // The filmstrip swap, and the ImageViewer for a click on a picture.
  function wire(gallery) {
    if (gallery.dataset.galleryWired) return;
    gallery.dataset.galleryWired = '1';
    const links = Array.from(gallery.querySelectorAll('.md-gallery-item'));
    const shown = links.map(a => {
      const img = a.querySelector('img');
      return { url: img ? img.getAttribute('src') : a.getAttribute('href'), alt: (img && img.getAttribute('alt')) || a.title || '' };
    });
    const hero = gallery.querySelector('.md-gallery-hero');
    const viewer = typeof ImageViewer !== 'undefined' ? ImageViewer : null;
    let current = 0;
    links.forEach((a, i) => {
      a.addEventListener('click', e => {
        if (e.button !== 0 || e.metaKey || e.ctrlKey || e.shiftKey) return;
        if (hero) {
          e.preventDefault();
          current = i;
          const img = hero.querySelector('img');
          const cap = hero.querySelector('figcaption');
          img.src = shown[i].url;
          img.alt = shown[i].alt;
          if (cap) {
            cap.textContent = a.title || '';
            cap.hidden = !a.title;
          }
          links.forEach(other => other.classList.toggle('is-active', other === a));
        } else if (viewer) {
          e.preventDefault();
          viewer.open(shown, i);
        }
      });
    });
    if (hero && viewer) {
      hero.classList.add('md-gallery-zoomable');
      hero.addEventListener('click', () => viewer.open(shown, current));
    }
  }

  // Wires every gallery under `root` (after marked has rendered it).
  function render(root) {
    root.querySelectorAll('.md-gallery').forEach(wire);
  }

  window.ProjectGallery = { render, images, html, layoutOf, extension, layouts: LAYOUTS };
})();
