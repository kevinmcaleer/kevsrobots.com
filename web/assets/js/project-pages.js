// Multi-page project write-ups (prototype).
//
// An author splits a long write-up into pages with a line holding only
//
//     <!-- page -->
//
// An HTML comment, so any other Markdown renderer simply ignores it and
// the text reads as one page there. marked keeps it in the rendered HTML
// as a comment node, and this module splits the rendered document at the
// top-level ones.
//
// Without markers a write-up stays one page, unless the caller asks for
// `auto`: then a long write-up is split before its top-level headings,
// keeping each page to a reasonable length.
//
// ProjectPages.paginate(contentEl, { auto, autoWords, minWords, pagerEl, endEl })
//   Wraps the content's pages in <section class="kr-page"> and shows one
//   at a time, with Previous / 1 2 3 / Next and "View as one page" in
//   pagerEl. endEl, if given, is what follows the text (files and models,
//   comments...): it belongs to the last page, so it shows only there
//   and in the one-page view, and links into it open the last page.
//   Returns null when there is only one page, else
//   { count, current(), show(n), pageOf(el), reveal(id), oneView() }.
//
// URLs: #page-2 opens page 2; #<heading-id> opens the page holding that
// heading; #one-page shows the whole write-up. Hash links in the text and
// the "On this page" nav keep working, whichever page they point into.
(function () {
  const MARKER = /^\s*page(?:\s*break)?\s*$/i;
  const ONE_PAGE = 'one-page';

  function isMarker(node) {
    return node.nodeType === Node.COMMENT_NODE && MARKER.test(node.data);
  }

  function words(nodes) {
    let n = 0;
    nodes.forEach(node => {
      const t = node.textContent || '';
      const m = t.match(/\S+/g);
      if (m) n += m.length;
    });
    return n;
  }

  function isHeading(node, level) {
    return node.nodeType === Node.ELEMENT_NODE && node.tagName === 'H' + level &&
      !node.classList.contains('cell-heading');
  }

  // Groups of top-level nodes, one per page, from the author's markers.
  function byMarkers(nodes) {
    if (!nodes.some(isMarker)) return null;
    const pages = [[]];
    nodes.forEach(node => {
      if (isMarker(node)) pages.push([]);
      else pages[pages.length - 1].push(node);
    });
    return pages.filter(p => p.some(n => n.nodeType === Node.ELEMENT_NODE || (n.textContent || '').trim()));
  }

  // Auto: break before top-level headings once a page holds minWords,
  // and only when the whole write-up is longer than autoWords.
  function byHeadings(nodes, autoWords, minWords) {
    if (words(nodes) < autoWords) return null;
    const level = [1, 2].find(l => nodes.filter(n => isHeading(n, l)).length >= 2);
    if (!level) return null;
    const pages = [[]];
    nodes.forEach(node => {
      const page = pages[pages.length - 1];
      if (isHeading(node, level) && words(page) >= minWords) pages.push([node]);
      else page.push(node);
    });
    // A short last page joins the one before it.
    if (pages.length > 1 && words(pages[pages.length - 1]) < minWords / 2) {
      const last = pages.pop();
      pages[pages.length - 1].push(...last);
    }
    return pages;
  }

  // The page's name: its first heading, else "Page n".
  function pageTitle(section, n) {
    const h = section.querySelector('h1, h2, h3');
    if (!h) return 'Page ' + n;
    const anchor = h.querySelector(':scope > .heading-anchor');
    const text = (anchor ? h.textContent.slice(0, -anchor.textContent.length) : h.textContent).trim();
    return text || 'Page ' + n;
  }

  function esc(t) {
    const d = document.createElement('div');
    d.textContent = t == null ? '' : String(t);
    return d.innerHTML;
  }

  function paginate(contentEl, options) {
    const opts = Object.assign({ auto: false, autoWords: 2500, minWords: 600 }, options || {});
    const nodes = Array.from(contentEl.childNodes);
    const groups = byMarkers(nodes) || (opts.auto ? byHeadings(nodes, opts.autoWords, opts.minWords) : null);
    // Drop the markers either way, so a one-page write-up has none left.
    nodes.filter(isMarker).forEach(n => n.remove());
    if (!groups || groups.length < 2) return null;

    const sections = groups.map((group, i) => {
      const s = document.createElement('section');
      s.className = 'kr-page';
      s.dataset.page = String(i + 1);
      s.setAttribute('aria-label', 'Page ' + (i + 1) + ' of ' + groups.length);
      group.forEach(node => s.appendChild(node));
      contentEl.appendChild(s);
      return s;
    });
    contentEl.classList.add('kr-paged');

    const pager = opts.pagerEl || document.createElement('nav');
    pager.classList.add('kr-pager');
    pager.setAttribute('aria-label', 'Pages');
    if (!opts.pagerEl) contentEl.after(pager);

    let current = 1;
    let oneView = false;

    function titles() { return sections.map((s, i) => pageTitle(s, i + 1)); }

    function drawPager() {
      const names = titles();
      if (oneView) {
        pager.innerHTML =
          '<div class="kr-pager-one">' +
            '<span>Showing all ' + sections.length + ' pages</span>' +
            '<a href="#page-' + current + '" data-page="' + current + '">View in pages</a>' +
          '</div>';
        return;
      }
      const prev = current > 1
        ? '<a class="kr-pager-step kr-pager-prev" href="#page-' + (current - 1) + '" data-page="' + (current - 1) + '" rel="prev">' +
            '<span class="kr-pager-dir">‹ Previous</span><span class="kr-pager-name">' + esc(names[current - 2]) + '</span></a>'
        : '<span class="kr-pager-step kr-pager-prev is-empty"></span>';
      const next = current < sections.length
        ? '<a class="kr-pager-step kr-pager-next" href="#page-' + (current + 1) + '" data-page="' + (current + 1) + '" rel="next">' +
            '<span class="kr-pager-dir">Next ›</span><span class="kr-pager-name">' + esc(names[current]) + '</span></a>'
        : '<span class="kr-pager-step kr-pager-next is-empty"></span>';
      const numbers = sections.map((s, i) => {
        const n = i + 1;
        return n === current
          ? '<span class="kr-pager-num is-current" aria-current="page" title="' + esc(names[i]) + '">' + n + '</span>'
          : '<a class="kr-pager-num" href="#page-' + n + '" data-page="' + n + '" title="' + esc(names[i]) + '">' + n + '</a>';
      }).join('');
      pager.innerHTML =
        '<div class="kr-pager-row">' + prev +
          '<div class="kr-pager-nums"><span class="kr-pager-of">Page</span>' + numbers + '</div>' +
          next + '</div>' +
        '<div class="kr-pager-all"><a href="#' + ONE_PAGE + '" data-one-page="1">View as one page</a></div>';
    }

    function apply() {
      sections.forEach((s, i) => { s.hidden = !oneView && i + 1 !== current; });
      if (opts.endEl) opts.endEl.hidden = !oneView && current !== sections.length;
      contentEl.classList.toggle('kr-paged-all', oneView);
      drawPager();
      contentEl.dispatchEvent(new CustomEvent('kr-page-change', { detail: { page: current, oneView } }));
    }

    function scrollToContent() {
      const top = contentEl.getBoundingClientRect().top + window.scrollY - 90;
      if (window.scrollY > top) window.scrollTo({ top });
    }

    function show(n, opts2) {
      const o = opts2 || {};
      current = Math.max(1, Math.min(sections.length, n));
      oneView = false;
      apply();
      if (o.push !== false) history.pushState(null, '', '#page-' + current);
      if (o.scroll !== false) scrollToContent();
    }

    function pageOf(el) {
      if (!el) return null;
      if (opts.endEl && opts.endEl.contains(el)) return sections.length;
      const s = el.closest && el.closest('.kr-page');
      return s && sections.includes(s) ? Number(s.dataset.page) : null;
    }

    // Make an element visible (open its page); true if it is in the text.
    function reveal(id) {
      const el = id && document.getElementById(id);
      const n = pageOf(el);
      if (!n) return !!el;
      if (!oneView && n !== current) { current = n; apply(); }
      return true;
    }

    // Read the hash: #page-n, #one-page, or an element's id.
    function fromHash() {
      let id = '';
      try { id = decodeURIComponent(location.hash.slice(1)); } catch (e) { return; }
      if (!id) { if (oneView || current !== 1) { oneView = false; current = 1; apply(); } return; }
      if (id === ONE_PAGE) { if (!oneView) { oneView = true; apply(); } return; }
      const m = id.match(/^page-(\d+)$/);
      if (m && !document.getElementById(id)) {
        const n = Math.max(1, Math.min(sections.length, Number(m[1])));
        if (oneView || n !== current) { oneView = false; current = n; apply(); }
        return;
      }
      reveal(id);
    }

    pager.addEventListener('click', ev => {
      const a = ev.target.closest('a');
      if (!a) return;
      ev.preventDefault();
      if (a.dataset.onePage) {
        oneView = true;
        apply();
        history.pushState(null, '', '#' + ONE_PAGE);
        return;
      }
      show(Number(a.dataset.page));
    });

    // Links in the text to a heading on another page.
    contentEl.addEventListener('click', ev => {
      const a = ev.target.closest('a[href^="#"]');
      if (!a) return;
      let id;
      try { id = decodeURIComponent(a.getAttribute('href').slice(1)); } catch (e) { return; }
      const target = id && document.getElementById(id);
      const n = pageOf(target);
      if (!n || n === current || oneView) return;
      ev.preventDefault();
      current = n;
      apply();
      history.pushState(null, '', '#' + encodeURIComponent(id));
      target.scrollIntoView();
      window.scrollBy(0, -80);
    });

    window.addEventListener('popstate', fromHash);
    window.addEventListener('hashchange', fromHash);

    fromHash();
    apply();

    return {
      count: sections.length,
      current: () => current,
      isOneView: () => oneView,
      show,
      pageOf,
      reveal,
    };
  }

  window.ProjectPages = { paginate, isMarker };
})();
