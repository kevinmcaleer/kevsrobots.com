// Galleries in a project's Markdown: the image lines between `::: gallery`
// and `:::` show as one gallery, in the order they're written.
//
//   ::: gallery filmstrip
//   ![Front](/front.jpg)
//   ![Wiring](/wiring.jpg "Before the lid went on")
//   :::
//
// Layouts: filmstrip (the first picture large, thumbnails swap it), bento
// (tiles, the first one big), lightbox (even thumbnails) or carousel (one
// picture at a time that moves on by itself, with countdown dots and a pause
// button). Any picture opens in the shared ImageViewer. The editor draws these itself (markdown-studio's
// bundle); the view page renders with marked, so this file adds a marked
// extension that writes the same HTML. The styles are markdown-studio's
// gallery.css, vendored beside the editor bundle by
// scripts/update-markdown-editor.sh. Keep it in step with markdown-studio's
// packages/markdown-editor/src/gallery.ts.
(function () {
  const LAYOUTS = ['filmstrip', 'bento', 'lightbox', 'carousel'];
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
    // Its title, or alt text someone wrote: not one made from a file name
    // (project-image-captions.js' rule, as gallery.ts uses imageCaption).
    const captions = window.ProjectImageCaptions;
    const caption = i => i.title || (captions ? captions.caption(i.alt) || '' : i.alt);
    const items = safe.map((i, n) => {
      const title = caption(i);
      return '<a class="md-gallery-item' + (layout === 'filmstrip' && n === 0 ? ' is-active' : '') + '" href="' + esc(i.src) +
        '" target="_blank" rel="noopener noreferrer" data-gallery-index="' + n + '"' + (title ? ' title="' + esc(title) + '"' : '') + '>' +
        '<img src="' + esc(i.src) + '" alt="' + esc(i.alt) + '" loading="lazy"></a>';
    }).join('');
    if (layout === 'carousel') return carouselHtml(safe, caption);
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

  const PAUSE_ICON = '<svg class="md-gallery-icon-pause" viewBox="0 0 24 24" aria-hidden="true"><rect x="6" y="4.5" width="4" height="15" rx="1.2"/><rect x="14" y="4.5" width="4" height="15" rx="1.2"/></svg>';
  const PLAY_ICON = '<svg class="md-gallery-icon-play" viewBox="0 0 24 24" aria-hidden="true"><path d="M8 4.8v14.4a1 1 0 0 0 1.5.86l11.6-7.2a1 1 0 0 0 0-1.72L9.5 3.94A1 1 0 0 0 8 4.8z"/></svg>';

  // A carousel: the pictures in a sideways track (each with its caption),
  // then a tray of dots and a pause button. The same markup as gallery.ts'
  // carouselHtml.
  function carouselHtml(list, caption) {
    const items = list.map((i, n) => {
      const title = caption(i);
      return '<a class="md-gallery-item' + (n === 0 ? ' is-active' : '') + '" href="' + esc(i.src) +
        '" target="_blank" rel="noopener noreferrer" data-gallery-index="' + n + '"' + (title ? ' title="' + esc(title) + '"' : '') + '>' +
        '<img src="' + esc(i.src) + '" alt="' + esc(i.alt) + '" loading="' + (n === 0 ? 'eager' : 'lazy') + '">' +
        (title ? '<span class="md-gallery-caption">' + esc(title) + '</span>' : '') + '</a>';
    }).join('');
    const dots = list.map((_, n) =>
      '<button type="button" class="md-gallery-dot' + (n === 0 ? ' is-active' : '') + '" data-gallery-index="' + n +
      '" aria-label="Show picture ' + (n + 1) + ' of ' + list.length + '"' + (n === 0 ? ' aria-current="true"' : '') +
      '><span class="md-gallery-dot-fill"></span></button>').join('');
    const controls = list.length > 1
      ? '<div class="md-gallery-controls"><div class="md-gallery-dots">' + dots + '</div>' +
        '<button type="button" class="md-gallery-play" aria-label="Pause the slideshow">' + PAUSE_ICON + PLAY_ICON + '</button></div>'
      : '';
    return '<div class="md-gallery md-gallery-carousel' + (list.length ? '' : ' md-gallery-empty') + '" data-gallery-layout="carousel" data-gallery-count="' +
      list.length + '"><div class="md-gallery-items">' + items + '</div>' + controls + '</div>';
  }

  // Makes a carousel go, as gallery.ts' wireCarousel: each picture shows for
  // its dot's countdown (a CSS animation, --md-gallery-interval), then the
  // next slides in, round and round. The button pauses and plays; a dot or a
  // swipe shows that picture; it counts down only while on screen, and starts
  // paused for people who ask for less motion.
  function wireCarousel(gallery, links, openAt) {
    const track = gallery.querySelector('.md-gallery-items');
    const dots = Array.from(gallery.querySelectorAll('.md-gallery-dot'));
    const play = gallery.querySelector('.md-gallery-play');
    if (!track || links.length < 2 || !play) return;
    gallery.dataset.galleryWired = '1';
    const reduced = typeof matchMedia === 'function' && matchMedia('(prefers-reduced-motion: reduce)').matches;
    let current = 0;
    let paused = reduced;

    function count(index) {
      dots.forEach(d => d.classList.remove('is-counting'));
      const dot = dots[index];
      if (!dot) return;
      void dot.offsetWidth; // so the animation starts over
      dot.classList.add('is-counting');
    }
    function setPaused(value) {
      paused = value;
      gallery.classList.toggle('is-paused', paused);
      play.setAttribute('aria-label', paused ? 'Play the slideshow' : 'Pause the slideshow');
      play.title = paused ? 'Play' : 'Pause';
      if (!paused && dots[current] && !dots[current].classList.contains('is-counting')) count(current);
    }
    function show(index, scroll) {
      current = (index + links.length) % links.length;
      links.forEach((a, i) => a.classList.toggle('is-active', i === current));
      dots.forEach((d, i) => {
        d.classList.toggle('is-active', i === current);
        if (i === current) d.setAttribute('aria-current', 'true');
        else d.removeAttribute('aria-current');
      });
      if (!paused) count(current);
      else dots.forEach(d => d.classList.remove('is-counting'));
      if (scroll !== false) {
        const left = links[current].offsetLeft - links[0].offsetLeft;
        if (typeof track.scrollTo === 'function') track.scrollTo({ left, behavior: reduced ? 'auto' : 'smooth' });
        else track.scrollLeft = left;
      }
    }

    gallery.addEventListener('animationend', e => {
      if (paused || !e.target.classList || !e.target.classList.contains('md-gallery-dot-fill')) return;
      if (e.target.parentElement === dots[current]) show(current + 1);
    });
    dots.forEach((d, i) => d.addEventListener('click', e => { e.preventDefault(); show(i); }));
    play.addEventListener('click', e => { e.preventDefault(); setPaused(!paused); });
    // A swipe (or scrolling the track) shows the picture it settles on.
    let settle;
    track.addEventListener('scroll', () => {
      clearTimeout(settle);
      settle = setTimeout(() => {
        const at = track.scrollLeft + links[0].offsetLeft;
        let nearest = 0;
        links.forEach((a, i) => {
          if (Math.abs(a.offsetLeft - at) < Math.abs(links[nearest].offsetLeft - at)) nearest = i;
        });
        if (nearest !== current) show(nearest, false);
      }, 120);
    }, { passive: true });
    // A picture opens in the viewer, pausing the slideshow behind it.
    if (openAt) {
      links.forEach((a, i) => a.addEventListener('click', e => {
        if (e.button !== 0 || e.metaKey || e.ctrlKey || e.shiftKey) return;
        e.preventDefault();
        setPaused(true);
        openAt(i);
      }));
    }
    if (typeof IntersectionObserver === 'function') {
      new IntersectionObserver(entries => {
        entries.forEach(entry => gallery.classList.toggle('is-offscreen', !entry.isIntersecting));
      }).observe(gallery);
    }
    setPaused(paused);
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

  // The filmstrip swap, the carousel's slideshow, and the ImageViewer for a
  // click on a picture.
  function wire(gallery) {
    if (gallery.dataset.galleryWired) return;
    gallery.dataset.galleryWired = '1';
    const links = Array.from(gallery.querySelectorAll('.md-gallery-item'));
    const shown = links.map(a => {
      const img = a.querySelector('img');
      return { url: img ? img.getAttribute('src') : a.getAttribute('href'), alt: (img && img.getAttribute('alt')) || a.title || '' };
    });
    const viewer = typeof ImageViewer !== 'undefined' ? ImageViewer : null;
    if (gallery.dataset.galleryLayout === 'carousel') {
      wireCarousel(gallery, links, viewer ? i => viewer.open(shown, i) : null);
      return;
    }
    const hero = gallery.querySelector('.md-gallery-hero');
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
