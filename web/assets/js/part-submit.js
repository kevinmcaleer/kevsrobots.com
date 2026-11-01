// "New part": add a part to the parts catalog without leaving the page.
//
// Opened from a Bill of Materials row that isn't linked to a catalog part
// yet (its "New part" button in the editor's table, "Create …" in the /part
// picker, and the wiring workspace's "+"). The row fills in what it can
// (name, shop link, price, notes); the writer adds pictures (choose, drop or
// paste) and anything else they know under "More details".
//
// The part goes to POST /api/parts/submit. It waits for review by the
// KevsRobots team before it shows in the catalog for everyone, but the
// writer can link it and use it in their project straight away.
//
//   PartSubmit.open({ api, apiFetch, prefill, origin, searchParts })
//     prefill      { name, shopUrl, price, currency, notes }
//     searchParts  q => catalog parts { name, slug, url, thumb, detail }, to
//                  offer one that's already there instead
//   resolves to { name, slug, url, existing } or null when closed.
//
// payload() and the other pure helpers are tested in
// scripts/tests/part-submit.test.js.
(function (root) {
  const MAX_PHOTOS = 8;
  const MAX_PHOTO_BYTES = 10 * 1024 * 1024;
  const IMAGE_TYPES = /^image\/(png|jpe?g|gif|webp)$/;
  const CURRENCIES = ['GBP', 'USD', 'EUR', 'CAD', 'AUD', 'JPY', 'INR', 'CHF'];
  const SYMBOL = { GBP: '£', USD: '$', EUR: '€', CAD: 'C$', AUD: 'A$', JPY: '¥', INR: '₹', CHF: 'CHF' };
  const CATEGORIES = ['Microcontroller', 'Single Board Computer', 'Sensor', 'Motor', 'Servo', 'Motor Driver',
    'Power', 'Battery', 'Display', 'Switch', 'Connector', 'Communication', 'Camera', 'Audio', 'LED', 'Mechanical', 'Other'];

  const isUrl = s => /^https?:\/\/\S+$/i.test(String(s || '').trim());
  const clean = s => {
    const t = String(s == null ? '' : s).trim();
    return t || null;
  };

  // The form's values as the body POST /api/parts/submit takes. Throws an
  // Error with a message for the writer when something can't be sent.
  function payload(v) {
    const name = clean(v.name);
    if (!name || name.length < 2) throw new Error('Give the part a name.');
    const links = [];
    const shop = clean(v.shopUrl);
    if (shop) {
      if (!isUrl(shop)) throw new Error('The shop link should start with https://');
      const price = v.price === '' || v.price == null ? null : Number(String(v.price).replace(/[^0-9.]/g, ''));
      if (price != null && !(price >= 0)) throw new Error('The price should be a number, like 4.50');
      links.push({
        url: shop,
        kind: 'shop',
        name: clean(v.shopName) || hostName(shop),
        unit_cost: price,
        currency_code: price != null ? (v.currency || 'GBP') : null,
      });
    }
    const datasheet = clean(v.datasheetUrl);
    if (datasheet && !isUrl(datasheet)) throw new Error('The datasheet link should start with https://');
    const tags = String(v.tags || '').split(',').map(t => t.trim()).filter(Boolean).slice(0, 20);
    const body = { name, links, tags };
    const fields = {
      description_md: v.description, category: v.category, manufacturer: v.manufacturer,
      mpn: v.mpn, voltage: v.voltage, package: v.package, datasheet_url: datasheet,
    };
    Object.keys(fields).forEach(k => {
      const value = clean(fields[k]);
      if (value) body[k] = value;
    });
    return body;
  }

  // "https://www.pimoroni.com/x" -> "pimoroni.com"
  function hostName(url) {
    try { return new URL(url).hostname.replace(/^www\./, ''); } catch (_) { return null; }
  }

  // The image links in a drop or paste from another web page (text/uri-list
  // or a bare URL), for pictures that are links rather than files.
  function imageLinks(text) {
    return String(text || '').split(/\s+/).filter(s => /^https?:\/\/\S+\.(png|jpe?g|gif|webp)(\?\S*)?$/i.test(s));
  }

  function errorText(body, status) {
    const d = body && body.detail;
    if (typeof d === 'string') return d;
    if (Array.isArray(d) && d[0] && d[0].msg) return d[0].msg;
    if (status === 401) return 'Sign in to add parts to the catalog.';
    return 'The part could not be added (HTTP ' + status + ').';
  }

  // --- the dialog ---------------------------------------------------------

  function el(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text != null) e.textContent = text;
    return e;
  }

  function field(label, input, hint) {
    const box = el('label', 'part-submit-field');
    box.appendChild(el('span', 'part-submit-label', label));
    box.appendChild(input);
    if (hint) box.appendChild(el('span', 'part-submit-hint', hint));
    return box;
  }

  function input(type, name, value, placeholder) {
    const i = el(type === 'textarea' ? 'textarea' : 'input');
    if (type !== 'textarea') i.type = type;
    i.name = name;
    i.className = 'form-control';
    if (value != null) i.value = value;
    if (placeholder) i.placeholder = placeholder;
    return i;
  }

  function open(opts) {
    opts = opts || {};
    const pre = opts.prefill || {};
    const api = opts.api || '';
    const fetcher = opts.apiFetch || ((u, o) => fetch(u, Object.assign({ credentials: 'include' }, o || {})));
    return new Promise(resolve => {
      const back = document.activeElement;
      const overlay = el('div', 'part-submit-overlay');
      const dialog = el('form', 'part-submit-dialog');
      dialog.noValidate = true;
      dialog.setAttribute('role', 'dialog');
      dialog.setAttribute('aria-modal', 'true');
      dialog.setAttribute('aria-labelledby', 'part-submit-title');

      const head = el('div', 'part-submit-head');
      const title = el('h2', null, 'Add a part to the catalog');
      title.id = 'part-submit-title';
      const close = el('button', 'btn-close');
      close.type = 'button';
      close.setAttribute('aria-label', 'Close');
      head.append(title, close);
      const intro = el('p', 'part-submit-intro', 'Fill in what you know. Only the name is needed; everything else helps other makers find and buy it.');

      // Name, and the catalog parts it might already be.
      const name = input('text', 'name', pre.name || '', 'e.g. SG90 micro servo');
      name.required = true;
      name.maxLength = 200;
      const matches = el('div', 'part-submit-matches');

      // Pictures.
      const photos = [];
      const drop = el('div', 'part-submit-drop');
      drop.tabIndex = 0;
      drop.setAttribute('role', 'button');
      drop.setAttribute('aria-label', 'Add pictures: choose, drop or paste them');
      const thumbs = el('div', 'part-submit-thumbs');
      const dropText = el('div', 'part-submit-drop-text');
      dropText.innerHTML = '<i class="fas fa-camera" aria-hidden="true"></i> <strong>Add pictures</strong> <span>drop, paste or click to choose</span>';
      drop.append(thumbs, dropText);
      const file = el('input');
      file.type = 'file';
      file.accept = 'image/png,image/jpeg,image/gif,image/webp';
      file.multiple = true;
      file.hidden = true;
      const licence = el('select', 'form-select form-select-sm');
      licence.name = 'licence';
      [['own', 'I took these pictures'], ['cc-by', 'Someone else’s, shared under CC BY']].forEach(([v, label]) => {
        const o = el('option', null, label);
        o.value = v;
        licence.appendChild(o);
      });
      const credit = input('text', 'credit', '', 'Who to credit, e.g. a link to the maker’s page');
      const licenceRow = el('div', 'part-submit-licence');
      licenceRow.append(licence, credit);
      licenceRow.hidden = true;

      // Shop link and price.
      const shop = input('url', 'shopUrl', pre.shopUrl || '', 'https://');
      const price = input('text', 'price', pre.price != null ? Number(pre.price).toFixed(pre.currency === 'JPY' ? 0 : 2) : '', '0.00');
      price.inputMode = 'decimal';
      const currency = el('select', 'form-select');
      currency.name = 'currency';
      currency.setAttribute('aria-label', 'Currency');
      CURRENCIES.forEach(code => {
        const o = el('option', null, SYMBOL[code]);
        o.title = code;
        o.value = code;
        currency.appendChild(o);
      });
      currency.value = CURRENCIES.includes(pre.currency) ? pre.currency : 'GBP';
      const priceBox = el('div', 'part-submit-price');
      priceBox.append(currency, price);
      const shopRow = el('div', 'part-submit-row');
      shopRow.append(field('Where to buy it', shop), field('Price', priceBox));

      // More details.
      const more = el('details', 'part-submit-more');
      more.appendChild(el('summary', null, 'More details (optional)'));
      const description = input('textarea', 'description', pre.notes || '', 'What it is and what it’s good for');
      description.rows = 3;
      const category = input('text', 'category', '', 'e.g. Sensor');
      const list = el('datalist');
      list.id = 'part-submit-categories-' + Math.random().toString(36).slice(2);
      category.setAttribute('list', list.id);
      CATEGORIES.forEach(c => { const o = el('option'); o.value = c; list.appendChild(o); });
      const datasheet = input('url', 'datasheetUrl', '', 'https://');
      const manufacturer = input('text', 'manufacturer', '', 'e.g. Tower Pro');
      const mpn = input('text', 'mpn', '', 'e.g. SG90');
      const voltage = input('text', 'voltage', '', 'e.g. 4.8–6V');
      const pack = input('text', 'package', '', 'e.g. module');
      pack.maxLength = 20;
      const tags = input('text', 'tags', '', 'servo, 9g, hobby');
      const grid = el('div', 'part-submit-grid');
      grid.append(
        field('Category', category), field('Datasheet', datasheet),
        field('Manufacturer', manufacturer), field('Part number', mpn),
        field('Voltage', voltage), field('Package', pack),
      );
      more.append(field('Description', description), grid, field('Tags', tags, 'Separate them with commas'), list);
      if (pre.notes) more.open = true;

      const note = el('p', 'part-submit-note');
      note.innerHTML = '<i class="fas fa-shield-halved" aria-hidden="true"></i> New parts are checked by the KevsRobots team before they show in the catalog for everyone. You can use yours in this project straight away.';
      const status = el('div', 'part-submit-status');
      status.setAttribute('role', 'status');
      const foot = el('div', 'part-submit-foot');
      const cancel = el('button', 'btn btn-outline-secondary', 'Cancel');
      cancel.type = 'button';
      const save = el('button', 'btn btn-primary', 'Add part');
      save.type = 'submit';
      foot.append(cancel, save);

      dialog.append(head, intro, field('Name', name), matches, el('span', 'part-submit-label', 'Pictures'), drop, file, licenceRow, shopRow, more, note, status, foot);
      overlay.appendChild(dialog);
      const rect = opts.origin && (opts.origin.getBoundingClientRect ? opts.origin.getBoundingClientRect() : opts.origin);
      if (rect) dialog.style.transformOrigin = (rect.left + rect.width / 2) + 'px ' + (rect.top + rect.height / 2) + 'px';
      document.body.appendChild(overlay);
      document.body.classList.add('part-submit-open');
      requestAnimationFrame(() => overlay.classList.add('is-open'));
      setTimeout(() => { (pre.name ? (pre.shopUrl ? drop : shop) : name).focus(); }, 60);

      let done = false;
      let busy = false;
      let made = null;
      function finish(result) {
        if (done) return;
        done = true;
        photos.forEach(p => p.preview && URL.revokeObjectURL(p.preview));
        overlay.remove();
        document.body.classList.remove('part-submit-open');
        document.removeEventListener('keydown', onKey, true);
        if (back && back.focus) back.focus();
        resolve(result);
      }
      function onKey(e) {
        if (e.key === 'Escape' && !busy) { e.preventDefault(); e.stopPropagation(); finish(made); }
        else if (e.key === 'Tab') {
          const f = [...dialog.querySelectorAll('input:not([hidden]), select, textarea, button, summary, [tabindex="0"]')].filter(x => !x.disabled && x.offsetParent);
          if (!f.length) return;
          if (e.shiftKey && document.activeElement === f[0]) { e.preventDefault(); f[f.length - 1].focus(); }
          else if (!e.shiftKey && document.activeElement === f[f.length - 1]) { e.preventDefault(); f[0].focus(); }
        }
      }
      document.addEventListener('keydown', onKey, true);
      // Once the part is made, closing the dialog any way still hands it back.
      close.addEventListener('click', () => !busy && finish(made));
      cancel.addEventListener('click', () => !busy && finish(made));
      overlay.addEventListener('mousedown', e => { if (e.target === overlay && !busy) finish(made); });

      function say(text, bad) {
        status.textContent = text || '';
        status.classList.toggle('is-error', !!bad);
      }

      // --- pictures
      function drawThumbs() {
        thumbs.innerHTML = '';
        photos.forEach((p, i) => {
          const t = el('div', 'part-submit-thumb');
          const img = el('img');
          img.alt = '';
          img.src = p.preview || p.url;
          const x = el('button', 'part-submit-thumb-x');
          x.type = 'button';
          x.setAttribute('aria-label', 'Remove this picture');
          x.innerHTML = '&times;';
          x.addEventListener('click', e => {
            e.stopPropagation();
            if (p.preview) URL.revokeObjectURL(p.preview);
            photos.splice(i, 1);
            drawThumbs();
          });
          t.append(img, x);
          thumbs.appendChild(t);
        });
        drop.classList.toggle('has-photos', photos.length > 0);
        dropText.querySelector('strong').textContent = photos.length ? 'Add more' : 'Add pictures';
        licenceRow.hidden = !photos.some(p => p.file);
        credit.hidden = licence.value !== 'cc-by';
      }
      function addFiles(files) {
        let skipped = 0;
        [...files].forEach(f => {
          if (photos.length >= MAX_PHOTOS || !IMAGE_TYPES.test(f.type) || f.size > MAX_PHOTO_BYTES) { skipped++; return; }
          photos.push({ file: f, preview: URL.createObjectURL(f) });
        });
        drawThumbs();
        say(skipped ? 'Some pictures were left out: PNG, JPEG, GIF or WebP up to 10 MB, ' + MAX_PHOTOS + ' at most.' : '', !!skipped);
      }
      function addLinks(urls) {
        urls.forEach(u => { if (photos.length < MAX_PHOTOS && !photos.some(p => p.url === u)) photos.push({ url: u }); });
        drawThumbs();
      }
      licence.addEventListener('change', drawThumbs);
      drop.addEventListener('click', () => file.click());
      drop.addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); file.click(); } });
      file.addEventListener('change', () => { addFiles(file.files); file.value = ''; });
      drop.addEventListener('dragover', e => { e.preventDefault(); drop.classList.add('is-over'); });
      drop.addEventListener('dragleave', () => drop.classList.remove('is-over'));
      drop.addEventListener('drop', e => {
        e.preventDefault();
        drop.classList.remove('is-over');
        const dt = e.dataTransfer;
        if (dt.files && dt.files.length) addFiles(dt.files);
        else addLinks(imageLinks(dt.getData('text/uri-list') || dt.getData('text/plain')));
      });
      // Pasting a picture anywhere in the dialog adds it (text pastes as text).
      dialog.addEventListener('paste', e => {
        const files = [...(e.clipboardData && e.clipboardData.files || [])].filter(f => IMAGE_TYPES.test(f.type));
        if (files.length) { e.preventDefault(); addFiles(files); return; }
        if (document.activeElement === drop) {
          const links = imageLinks(e.clipboardData && e.clipboardData.getData('text/plain'));
          if (links.length) { e.preventDefault(); addLinks(links); }
        }
      });

      // --- parts already in the catalog
      let searchToken = 0;
      async function findMatches() {
        const q = name.value.trim();
        const token = ++searchToken;
        if (!opts.searchParts || q.length < 3) { matches.innerHTML = ''; return; }
        let found = [];
        try { found = await opts.searchParts(q); } catch (_) { found = []; }
        if (token !== searchToken) return;
        matches.innerHTML = '';
        found = found.filter(p => p.slug).slice(0, 3);
        if (!found.length) return;
        matches.appendChild(el('div', 'part-submit-matches-title', 'Already in the catalog? Use it instead:'));
        found.forEach(p => {
          const b = el('button', 'part-submit-match');
          b.type = 'button';
          if (p.thumb) { const img = el('img'); img.src = p.thumb; img.alt = ''; b.appendChild(img); }
          b.appendChild(el('span', 'part-submit-match-name', p.name));
          if (p.detail) b.appendChild(el('span', 'part-submit-match-detail', p.detail));
          b.appendChild(el('span', 'part-submit-match-use', 'Use this'));
          b.addEventListener('click', () => finish({ name: p.name, slug: p.slug, url: p.url, existing: true }));
          matches.appendChild(b);
        });
      }
      let timer = null;
      name.addEventListener('input', () => { clearTimeout(timer); timer = setTimeout(findMatches, 250); });
      findMatches();

      // --- after it's added: pins and a 3-D model
      function showNext(part, failed) {
        const next = el('div', 'part-submit-next');
        next.appendChild(el('p', 'part-submit-next-title', '“' + part.name + '” is added.'));
        next.appendChild(el('p', 'part-submit-next-text', failed.length
          ? failed.length + ' of the pictures could not be uploaded (' + failed[0] + '). You can add them on the part’s page.'
          : 'Next, place its pins on the picture and say what each one does, and add a 3-D model if you have one. That lets everyone wire it up, here and in Snakie.'));
        next.classList.toggle('is-error', !!failed.length);
        const go = el('a', 'btn btn-outline-primary part-submit-next-go');
        go.href = electronicsUrl(part.slug);
        go.target = '_blank';
        go.rel = 'noopener';
        go.innerHTML = '<i class="fas fa-microchip me-1" aria-hidden="true"></i> Add pins and a 3-D model';
        go.addEventListener('click', () => setTimeout(() => finish(made), 0));
        next.appendChild(go);
        [...dialog.children].forEach(c => { if (c !== head && c !== foot) c.style.display = 'none'; });
        dialog.insertBefore(next, foot);
        save.disabled = false;
        save.textContent = 'Done';
        cancel.hidden = true;
        title.textContent = 'Part added';
        go.focus();
      }

      // --- send it
      dialog.addEventListener('submit', async e => {
        e.preventDefault();
        if (made) { finish(made); return; }
        if (busy) return;
        let body;
        try {
          body = payload({
            name: name.value, shopUrl: shop.value, price: price.value, currency: currency.value,
            description: description.value, category: category.value, datasheetUrl: datasheet.value,
            manufacturer: manufacturer.value, mpn: mpn.value, voltage: voltage.value, package: pack.value, tags: tags.value,
          });
        } catch (err) {
          say(err.message, true);
          if (!name.value.trim()) name.focus();
          else if (/datasheet/.test(err.message)) { more.open = true; datasheet.focus(); }
          else if (/shop/.test(err.message)) shop.focus();
          else if (/price/.test(err.message)) price.focus();
          return;
        }
        busy = true;
        save.disabled = cancel.disabled = true;
        save.innerHTML = '<span class="spinner-border spinner-border-sm me-1" aria-hidden="true"></span> Adding…';
        say('');
        try {
          const send = () => fetcher(api + '/api/parts/submit', {
            method: 'POST', credentials: 'include',
            headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
          });
          let resp = await send();
          if (resp.status === 403 && root.TermsGate && await root.TermsGate.handleResponse(resp)) resp = await send();
          const part = await resp.json().catch(() => ({}));
          if (!resp.ok) throw new Error(errorText(part, resp.status));
          const result = { name: part.name, slug: part.slug, url: '/parts/view.html?slug=' + encodeURIComponent(part.slug), existing: false, status: part.status };
          const failed = [];
          for (let i = 0; i < photos.length; i++) {
            const p = photos[i];
            say('Uploading picture ' + (i + 1) + ' of ' + photos.length + '…');
            try {
              let r;
              if (p.file) {
                const fd = new FormData();
                fd.append('file', p.file, p.file.name || 'picture.png');
                fd.append('licence', licence.value);
                if (licence.value === 'cc-by' && credit.value.trim()) fd.append('source', credit.value.trim());
                r = await fetcher(api + '/api/parts/' + encodeURIComponent(part.slug) + '/photos', { method: 'POST', credentials: 'include', body: fd });
              } else {
                r = await fetcher(api + '/api/parts/' + encodeURIComponent(part.slug) + '/photos/link', {
                  method: 'POST', credentials: 'include',
                  headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ external_url: p.url }),
                });
              }
              if (!r.ok) failed.push(errorText(await r.json().catch(() => ({})), r.status));
            } catch (err) {
              failed.push(err.message || String(err));
            }
          }
          // The part is made. Next: its pins and 3-D model, which Snakie's
          // Part Editor on the part's Electronics tab does (in a new tab, so
          // the project stays open). Say which pictures didn't make it.
          busy = false;
          made = result;
          showNext(part, failed);
        } catch (err) {
          busy = false;
          save.disabled = cancel.disabled = false;
          save.textContent = 'Add part';
          say(err.message || String(err), true);
        }
      });
    });
  }

  // The part's Electronics tab: Snakie's Part Editor (pins, pin types, the
  // symbol) and its 3-D model upload.
  function electronicsUrl(slug) {
    return '/parts/edit.html?slug=' + encodeURIComponent(slug) + '&tab=electronics';
  }

  const api = { open, payload, imageLinks, hostName, errorText, electronicsUrl };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.PartSubmit = api;
})(typeof window !== 'undefined' ? window : globalThis);
