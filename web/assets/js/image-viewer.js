/**
 * Fullscreen image viewer with gallery strip.
 * Usage: ImageViewer.open(images, startIndex)
 *   images: [{url, alt}]
 *   startIndex: which image to show first
 */
var ImageViewer = (function() {
  var overlay = null;
  var images = [];
  var currentIdx = 0;

  function create() {
    if (overlay) return;
    overlay = document.createElement('div');
    overlay.id = 'image-viewer-overlay';
    overlay.innerHTML =
      '<div class="iv-backdrop"></div>' +
      '<div class="iv-container">' +
        '<div class="iv-header">' +
          '<span class="iv-counter"></span>' +
          '<button class="iv-close" title="Close"><i class="fas fa-times"></i></button>' +
        '</div>' +
        '<div class="iv-main">' +
          '<button class="iv-prev" title="Previous"><i class="fas fa-chevron-left"></i></button>' +
          '<div class="iv-image-wrap"><img class="iv-image" src="" alt=""></div>' +
          '<button class="iv-next" title="Next"><i class="fas fa-chevron-right"></i></button>' +
        '</div>' +
        '<div class="iv-strip"></div>' +
      '</div>';
    document.body.appendChild(overlay);

    overlay.querySelector('.iv-backdrop').addEventListener('click', close);
    overlay.querySelector('.iv-close').addEventListener('click', close);
    overlay.querySelector('.iv-prev').addEventListener('click', prev);
    overlay.querySelector('.iv-next').addEventListener('click', next);

    document.addEventListener('keydown', onKey);
  }

  function onKey(e) {
    if (!overlay || overlay.style.display === 'none') return;
    if (e.key === 'Escape') close();
    if (e.key === 'ArrowLeft') prev();
    if (e.key === 'ArrowRight') next();
  }

  function open(imgs, startIdx) {
    images = imgs || [];
    currentIdx = startIdx || 0;
    if (images.length === 0) return;
    create();
    overlay.style.display = '';
    document.body.style.overflow = 'hidden';
    show(currentIdx);
  }

  function close() {
    if (!overlay) return;
    overlay.style.display = 'none';
    document.body.style.overflow = '';
  }

  function prev() {
    if (images.length <= 1) return;
    currentIdx = (currentIdx - 1 + images.length) % images.length;
    show(currentIdx);
  }

  function next() {
    if (images.length <= 1) return;
    currentIdx = (currentIdx + 1) % images.length;
    show(currentIdx);
  }

  function show(idx) {
    currentIdx = idx;
    var img = overlay.querySelector('.iv-image');
    img.src = images[idx].url;
    img.alt = images[idx].alt || '';

    overlay.querySelector('.iv-counter').textContent = (idx + 1) + ' / ' + images.length;

    // Prev/next visibility
    overlay.querySelector('.iv-prev').style.visibility = images.length > 1 ? '' : 'hidden';
    overlay.querySelector('.iv-next').style.visibility = images.length > 1 ? '' : 'hidden';

    // Gallery strip
    var strip = overlay.querySelector('.iv-strip');
    strip.innerHTML = images.map(function(im, i) {
      return '<div class="iv-thumb' + (i === idx ? ' iv-active' : '') + '" data-idx="' + i + '">' +
        '<img src="' + im.url + '" alt="' + (im.alt || '') + '">' +
      '</div>';
    }).join('');

    strip.querySelectorAll('.iv-thumb').forEach(function(thumb) {
      thumb.addEventListener('click', function() {
        show(parseInt(thumb.dataset.idx));
      });
    });

    // Scroll active thumb into view
    var active = strip.querySelector('.iv-active');
    if (active) active.scrollIntoView({ behavior: 'smooth', inline: 'center', block: 'nearest' });
  }

  return { open: open, close: close };
})();
