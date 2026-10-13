/**
 * Behaviour for the `article` layout:
 *  - builds the "On this page" list from the article's h2 headings
 *  - shows it while the article body is on screen and highlights the current section
 *  - opens photos in the shared ImageViewer when clicked
 */
(function () {
  var prose = document.querySelector('.kr-prose');
  if (!prose) return;

  // ----- On this page -----
  var toc = document.querySelector('.kr-toc');
  var headings = Array.prototype.slice.call(prose.querySelectorAll(':scope > h2[id], .kr-band-inner > h2[id]'));

  if (toc && headings.length >= 2) {
    var list = toc.querySelector('ol');
    var links = headings.map(function (h) {
      var li = document.createElement('li');
      var a = document.createElement('a');
      a.href = '#' + h.id;
      a.textContent = h.textContent.trim();
      li.appendChild(a);
      list.appendChild(li);
      return a;
    });

    if ('IntersectionObserver' in window) {
      // Show the list only once the hero and spec strip have scrolled away,
      // and hide it again when the reader reaches the end of the article.
      var top = [document.querySelector('.kr-hero'), document.querySelector('.kr-specs')].filter(Boolean);
      var topInView = new Set(top);
      var proseInView = false;
      var update = function () { toc.classList.toggle('is-visible', proseInView && topInView.size === 0); };
      var topWatch = new IntersectionObserver(function (entries) {
        entries.forEach(function (e) { if (e.isIntersecting) topInView.add(e.target); else topInView.delete(e.target); });
        update();
      });
      top.forEach(function (el) { topWatch.observe(el); });
      new IntersectionObserver(function (entries) {
        proseInView = entries[entries.length - 1].isIntersecting;
        update();
      }, { rootMargin: '0px 0px -30% 0px' }).observe(prose);

      var current = null;
      var spy = new IntersectionObserver(function (entries) {
        entries.forEach(function (e) {
          if (!e.isIntersecting) return;
          var i = headings.indexOf(e.target);
          if (current) current.classList.remove('is-active');
          current = links[i];
          current.classList.add('is-active');
        });
      }, { rootMargin: '0px 0px -70% 0px' });
      headings.forEach(function (h) { spy.observe(h); });
    } else {
      toc.classList.add('is-visible');
    }
  } else if (toc) {
    toc.remove();
  }

  // ----- Photo viewer -----
  if (typeof ImageViewer === 'undefined') return;
  var photos = Array.prototype.slice.call(prose.querySelectorAll('img')).filter(function (img) {
    return !img.closest('a') && !img.closest('.kr-card');
  });
  var set = photos.map(function (img) { return { url: img.currentSrc || img.src, alt: img.alt }; });
  photos.forEach(function (img, i) {
    img.classList.add('kr-zoom');
    img.addEventListener('click', function () { ImageViewer.open(set, i); });
  });
})();
