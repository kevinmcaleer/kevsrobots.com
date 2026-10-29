// Part links (issue #263).
//
// A Markdown link to a catalog part, [Raspberry Pi Pico](/parts/view.html?slug=pico),
// shows as a chip with the part's photo, in the editor's live view and on
// the project page, and hovering one shows a card with the photo, the
// lowest price and how many projects use it.
//
// The editor's text is redrawn by CodeMirror at will, so the photos are
// CSS rules keyed on the link's target (`data-href` on the live view's link
// text, `href` on links in tables and on the page) rather than elements
// put into the text.
(function () {
  const API = 'https://projects.kevsrobots.com';
  const PATH = '/parts/view.html?slug=';
  const SLUG_RE = /^(?:https?:\/\/(?:www\.)?kevsrobots\.com)?\/parts\/view\.html\?slug=([a-z0-9][a-z0-9-]*)$/i;

  // The slug a link goes to, or null when it isn't a part page.
  function slugOf(href) {
    const m = String(href || '').trim().match(SLUG_RE);
    return m ? m[1].toLowerCase() : null;
  }

  function href(slug) {
    return PATH + encodeURIComponent(slug);
  }

  // What the page links read: a site-relative link means nothing in a
  // README on GitHub, so the export makes part links absolute.
  function absolutize(markdown) {
    return String(markdown || '').replace(/\]\(\/parts\/view\.html\?slug=/g, '](https://www.kevsrobots.com/parts/view.html?slug=');
  }

  function apiUrl(url) {
    return url && url.startsWith('/') ? API + url : url;
  }

  // ------------------------------------------------------------ parts

  const parts = new Map(); // slug -> Promise<PartDetail | null>

  function fetchPart(slug) {
    if (!parts.has(slug)) {
      parts.set(slug, fetch(API + '/api/parts/' + encodeURIComponent(slug))
        .then(resp => (resp.ok ? resp.json() : null))
        .catch(() => null));
    }
    return parts.get(slug);
  }

  function coverOf(part) {
    const photo = part && part.photos && part.photos[0];
    return photo ? apiUrl(photo.url) : null;
  }

  // The cheapest shop that still has it, as { amount, code, name }, in
  // currency `to` (else the reader's, else the shop's own), or null.
  async function cheapest(part, to) {
    const shops = (part.suppliers || []).filter(s =>
      (s.kind || 'shop') === 'shop' && !s.unavailable_at && !s.is_broken && s.unit_cost != null);
    if (!shops.length) return null;
    const cc = window.CurrencyConvert;
    to = to || (cc && await cc.getPreferredCurrency().catch(() => null)) || shops[0].currency_code || 'GBP';
    let best = null;
    for (const s of shops) {
      let amount = Number(s.unit_cost);
      let code = s.currency_code || to;
      if (cc && code !== to) {
        const c = await cc.convert(amount, code, to);
        if (c) { amount = c.amount; code = to; }
      }
      if (code === to && (!best || amount < best.amount)) best = { amount, code, name: s.name };
    }
    return best;
  }

  // The same, for reading: "£3.90 at Pimoroni".
  async function lowestPrice(part) {
    const best = await cheapest(part);
    if (!best) return null;
    const cc = window.CurrencyConvert;
    const price = cc ? cc.formatPrice(best.amount, best.code) : best.amount.toFixed(2) + ' ' + best.code;
    return best.name ? price + ' at ' + best.name : price;
  }

  // ------------------------------------------------------------ chips

  let sheet = null;
  const drawn = new Set();

  // Gives the chips for `slug` the part's photo, once it's known.
  function thumb(slug) {
    if (drawn.has(slug)) return;
    drawn.add(slug);
    fetchPart(slug).then(part => {
      const src = coverOf(part);
      if (!src) return;
      if (!sheet) {
        sheet = document.createElement('style');
        sheet.id = 'part-link-thumbs';
        document.head.appendChild(sheet);
      }
      const end = 'slug=' + slug + '"]';
      const url = src.replace(/["\\\n]/g, encodeURIComponent);
      sheet.textContent += [
        '.cm-md-link[data-href*="' + PATH + '"][data-href$="' + end,
        '.cm-content a[href*="' + PATH + '"][href$="' + end,
        'a.part-link[href$="' + end,
      ].map(s => s + '::before').join(',') + '{background-image:url("' + url + '")}\n';
    });
  }

  // Every part link in `markdown` gets its photo.
  function thumbsFor(markdown) {
    const re = /\]\(((?:https?:\/\/(?:www\.)?kevsrobots\.com)?\/parts\/view\.html\?slug=[a-z0-9-]+)\)/gi;
    for (const m of String(markdown || '').matchAll(re)) thumb(slugOf(m[1]));
  }

  // The project page: part links in `root` become chips.
  function decorate(root) {
    root.querySelectorAll('a[href]').forEach(a => {
      const slug = slugOf(a.getAttribute('href'));
      if (!slug || a.querySelector('img')) return; // an image that links to a part stays an image
      a.classList.add('part-link');
      a.setAttribute('href', href(slug));
      thumb(slug);
    });
    attachHover(root);
  }

  // ------------------------------------------------------------ hover card

  let card = null;
  let showing = null; // the element the card is for
  let timer = 0;

  // The part link under `target`, as { el, slug }.
  function linkAt(target) {
    const el = target && target.closest && target.closest('a[href], .cm-md-link[data-href]');
    if (!el) return null;
    const slug = slugOf(el.getAttribute('data-href') || el.getAttribute('href'));
    return slug ? { el, slug } : null;
  }

  function hideCard() {
    clearTimeout(timer);
    showing = null;
    if (card) card.hidden = true;
  }

  async function showCard(el, slug) {
    showing = el;
    if (!card) {
      card = document.createElement('div');
      card.className = 'part-link-card';
      card.setAttribute('role', 'tooltip');
      card.addEventListener('mouseleave', hideCard);
      document.body.appendChild(card);
    }
    const part = await fetchPart(slug);
    if (showing !== el) return;
    card.replaceChildren();
    if (!part) {
      card.textContent = 'This part isn’t in the catalog any more.';
    } else {
      const src = coverOf(part);
      if (src) {
        const img = document.createElement('img');
        img.src = src;
        img.alt = '';
        card.appendChild(img);
      }
      const body = document.createElement('div');
      const name = document.createElement('strong');
      name.textContent = part.name;
      body.appendChild(name);
      const add = text => {
        const line = document.createElement('div');
        line.className = 'small';
        line.textContent = text;
        body.appendChild(line);
        return line;
      };
      if (part.category) add(part.category).classList.add('text-muted');
      const price = add('…');
      add('Used in ' + part.usage_count + (part.usage_count === 1 ? ' project' : ' projects'));
      card.appendChild(body);
      lowestPrice(part).then(p => { price.textContent = p ? 'From ' + p : 'No price yet'; });
    }
    card.hidden = false;
    const r = el.getBoundingClientRect();
    const w = card.offsetWidth;
    const h = card.offsetHeight;
    card.style.left = Math.max(8, Math.min(r.left, window.innerWidth - w - 8)) + 'px';
    card.style.top = (r.bottom + 6 + h > window.innerHeight && r.top > h + 6 ? r.top - h - 6 : r.bottom + 6) + 'px';
  }

  const hovered = new WeakSet();

  // Shows the card over part links in `root` (the page's content, or the
  // editor's text).
  function attachHover(root) {
    if (hovered.has(root)) return;
    hovered.add(root);
    root.addEventListener('mouseover', e => {
      const link = linkAt(e.target);
      if (!link || link.el === showing) return;
      clearTimeout(timer);
      timer = setTimeout(() => showCard(link.el, link.slug), 300);
    });
    root.addEventListener('mouseout', e => {
      const link = linkAt(e.target);
      if (!link || link.el.contains(e.relatedTarget)) return;
      if (card && card.contains(e.relatedTarget)) return;
      hideCard();
    });
    root.addEventListener('mousedown', hideCard);
  }

  window.PartLinks = { slugOf, href, absolutize, fetchPart, coverOf, cheapest, lowestPrice, thumbsFor, decorate, attachHover, linkAt, hideCard };
})();
