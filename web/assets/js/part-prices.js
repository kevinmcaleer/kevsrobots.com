// Vendors, price history, technical information and alternatives on the
// part page (issues #202 and #267).
//
// PartPrices.render(part, { api, signedIn, reload }) fills:
//   #part-vendors-section   every shop: name, country, price in the chosen
//                           currency, last checked, status, and a "Mark
//                           unavailable" / "Clear" action for signed-in users
//   #part-price-chart       price over time per shop, as inline SVG
//   #part-techinfo-section  datasheet, driver, support and docs links
//   #part-alternatives-section  related parts, when a shop is unavailable
//
// Prices are converted with CurrencyConvert (the projects API's FX rates).
// The currency picker is remembered per browser.
(function () {
  'use strict';

  const CURRENCIES = ['GBP', 'USD', 'EUR', 'JPY', 'AUD', 'CAD'];
  const STORE_KEY = 'kr-part-currency';
  const COLOURS = ['#2e7d4f', '#c98a1b', '#3b6fb6', '#b5473a', '#7b5ea7', '#4a8f8f'];
  const KIND_LABELS = { datasheet: 'Datasheet', driver: 'Driver / library', support: 'Support', docs: 'Documentation' };
  const KIND_ICONS = { datasheet: 'fa-file-pdf', driver: 'fa-code', support: 'fa-life-ring', docs: 'fa-book' };

  function esc(t) {
    const d = document.createElement('div');
    d.textContent = t == null ? '' : String(t);
    return d.innerHTML;
  }

  function flag(code) {
    if (!code || !/^[A-Za-z]{2}$/.test(code)) return '';
    const up = code.toUpperCase();
    return String.fromCodePoint(0x1F1E6 + up.charCodeAt(0) - 65, 0x1F1E6 + up.charCodeAt(1) - 65);
  }

  function storedCurrency() {
    try { return localStorage.getItem(STORE_KEY); } catch (e) { return null; }
  }
  function storeCurrency(code) {
    try { localStorage.setItem(STORE_KEY, code); } catch (e) { /* private mode */ }
  }

  // Rates are fetched once per currency pair and reused for every price.
  const rates = new Map();
  function rate(from, to) {
    if (!from || !to || from === to) return Promise.resolve(1);
    const key = from + '>' + to;
    if (!rates.has(key)) {
      rates.set(key, CurrencyConvert.convert(1, from, to).then(r => (r && r.rate) || null));
    }
    return rates.get(key);
  }
  async function convert(amount, from, to) {
    if (amount == null) return null;
    const r = await rate(from, to);
    return r == null ? null : { amount: amount * r, code: to };
  }
  function money(amount, code) {
    return CurrencyConvert.formatPrice(amount, code);
  }
  async function priceHtml(amount, from, to) {
    const conv = await convert(amount, from, to);
    if (!conv) return esc(money(amount, from));
    if (from && from !== to) {
      return `<span title="${esc(money(amount, from))}">${esc(money(conv.amount, to))}</span>`;
    }
    return esc(money(conv.amount, to));
  }

  function available(s) { return !s.is_broken && !s.unavailable_at; }

  function statusHtml(s) {
    if (s.unavailable_at) {
      const who = s.unavailable_by ? ' by ' + esc(s.unavailable_by) : '';
      const note = s.unavailable_note ? ': ' + esc(s.unavailable_note) : '';
      return `<span class="badge bg-warning text-dark" title="Reported${who}${note}"><i class="fas fa-ban me-1"></i>Unavailable</span>`
        + (s.unavailable_note ? `<div class="small text-muted">${esc(s.unavailable_note)}</div>` : '');
    }
    if (s.is_broken) return '<span class="badge bg-danger"><i class="fas fa-unlink me-1"></i>Broken link</span>';
    if (s.last_status === 'ok') return '<span class="badge bg-success-subtle text-success">OK</span>';
    return '<span class="badge bg-light text-muted">Not checked yet</span>';
  }

  async function renderVendors(part, shops, currency, opts) {
    const rows = await Promise.all(shops.map(async s => {
      let price = '<span class="text-muted">—</span>';
      if (available(s) && s.unit_cost != null) {
        price = await priceHtml(s.unit_cost, s.currency_code, currency);
      } else if (s.last_price != null || s.unit_cost != null) {
        // Broken or unavailable: the last good price, greyed.
        const amount = s.last_price != null ? s.last_price : s.unit_cost;
        const code = s.last_price != null ? s.last_price_currency : s.currency_code;
        price = `<span class="text-muted text-decoration-line-through" title="Last recorded price">${await priceHtml(amount, code, currency)}</span>`;
      }
      let action = '';
      if (opts.signedIn) {
        action = s.unavailable_at
          ? `<button type="button" class="btn btn-sm btn-link p-0" data-clear-unavailable="${s.id}">Clear</button>`
          : `<button type="button" class="btn btn-sm btn-link p-0 text-muted" data-mark-unavailable="${s.id}">Mark unavailable</button>`;
      }
      return `<tr>
        <td><a href="${esc(s.url)}" target="_blank" rel="noopener">${esc(s.name || new URL(s.url, location.href).hostname)}</a>
          ${s.country_code ? `<span class="ms-1" title="${esc(s.country_code)}">${flag(s.country_code)}</span>` : ''}</td>
        <td class="text-end">${price}</td>
        <td class="small text-muted">${s.last_checked_at ? new Date(s.last_checked_at).toLocaleDateString() : ''}</td>
        <td>${statusHtml(s)}</td>
        <td class="text-end">${action}</td>
      </tr>`;
    }));
    document.getElementById('part-vendors').innerHTML = `
      <div class="table-responsive"><table class="table table-sm align-middle mb-2">
        <thead><tr><th>Vendor</th><th class="text-end">Price</th><th>Last checked</th><th>Status</th><th></th></tr></thead>
        <tbody>${rows.join('')}</tbody>
      </table></div>`;
  }

  // Price over time, one line per shop, in the chosen currency.
  async function renderChart(history, currency) {
    const el = document.getElementById('part-price-chart');
    const series = [];
    for (const [i, h] of history.entries()) {
      const pts = [];
      for (const p of h.points) {
        const conv = await convert(p.unit_cost, p.currency_code, currency);
        if (conv) pts.push({ t: new Date(p.recorded_at).getTime(), v: conv.amount });
      }
      if (pts.length) series.push({ name: h.name || h.url, colour: COLOURS[i % COLOURS.length], pts });
    }
    const all = series.flatMap(s => s.pts);
    if (all.length < 2) { el.innerHTML = ''; return; }
    const W = 640, H = 220, L = 56, R = 12, T = 12, B = 28;
    const now = Date.now();
    let t0 = Math.min(...all.map(p => p.t));
    let t1 = Math.max(now, ...all.map(p => p.t));
    if (t1 === t0) t0 -= 86400000;
    let v0 = Math.min(...all.map(p => p.v));
    let v1 = Math.max(...all.map(p => p.v));
    if (v1 === v0) { v0 *= 0.9; v1 *= 1.1; }
    const pad = (v1 - v0) * 0.1;
    v0 = Math.max(0, v0 - pad); v1 += pad;
    const x = t => L + (t - t0) / (t1 - t0) * (W - L - R);
    const y = v => T + (1 - (v - v0) / (v1 - v0)) * (H - T - B);
    const ticks = [v0, (v0 + v1) / 2, v1];
    const day = t => new Date(t).toLocaleDateString(undefined, { day: 'numeric', month: 'short', year: '2-digit' });
    const lines = series.map(s => {
      // A price holds until the next one: draw steps, carried on to today.
      const steps = [];
      s.pts.forEach((p, i) => {
        if (i) steps.push(`${x(p.t).toFixed(1)},${y(s.pts[i - 1].v).toFixed(1)}`);
        steps.push(`${x(p.t).toFixed(1)},${y(p.v).toFixed(1)}`);
      });
      steps.push(`${x(now).toFixed(1)},${y(s.pts[s.pts.length - 1].v).toFixed(1)}`);
      const dots = s.pts.map(p => `<circle cx="${x(p.t).toFixed(1)}" cy="${y(p.v).toFixed(1)}" r="3" fill="${s.colour}"><title>${esc(s.name)}: ${esc(money(p.v, currency))} on ${esc(day(p.t))}</title></circle>`).join('');
      return `<polyline points="${steps.join(' ')}" fill="none" stroke="${s.colour}" stroke-width="2"/>${dots}`;
    }).join('');
    el.innerHTML = `
      <h6 class="mt-3 mb-1">Price history</h6>
      <svg viewBox="0 0 ${W} ${H}" class="w-100" role="img" aria-label="Price over time by vendor, in ${currency}" style="max-height:260px">
        ${ticks.map(v => `<line x1="${L}" x2="${W - R}" y1="${y(v)}" y2="${y(v)}" stroke="currentColor" stroke-opacity=".12"/>
          <text x="${L - 6}" y="${y(v) + 4}" text-anchor="end" font-size="11" fill="currentColor" fill-opacity=".6">${esc(money(v, currency))}</text>`).join('')}
        <text x="${L}" y="${H - 8}" font-size="11" fill="currentColor" fill-opacity=".6">${esc(day(t0))}</text>
        <text x="${W - R}" y="${H - 8}" text-anchor="end" font-size="11" fill="currentColor" fill-opacity=".6">today</text>
        ${lines}
      </svg>
      <div class="small">${series.map(s => `<span class="me-3"><span style="display:inline-block;width:10px;height:10px;border-radius:2px;background:${s.colour}"></span> ${esc(s.name)}</span>`).join('')}</div>`;
  }

  function renderTechInfo(links) {
    const sec = document.getElementById('part-techinfo-section');
    if (!links.length) { sec.classList.add('d-none'); return; }
    document.getElementById('part-techinfo').innerHTML = links.map(l => `
      <li class="list-group-item">
        <i class="fas ${KIND_ICONS[l.kind] || 'fa-link'} me-2 text-muted"></i>
        <a href="${esc(l.url)}" target="_blank" rel="noopener">${esc(l.name || KIND_LABELS[l.kind] || l.url)}</a>
        <span class="badge bg-light text-dark ms-2">${esc(KIND_LABELS[l.kind] || l.kind)}</span>
      </li>`).join('');
    sec.classList.remove('d-none');
  }

  function renderAlternatives(part, shops) {
    const sec = document.getElementById('part-alternatives-section');
    const related = part.related_parts || [];
    if (!related.length || !shops.some(s => !available(s))) { sec.classList.add('d-none'); return; }
    document.getElementById('part-alternatives').innerHTML = related.map(r =>
      `<a class="btn btn-sm btn-outline-secondary me-2 mb-2" href="/parts/view.html?slug=${encodeURIComponent(r.slug)}">${esc(r.name)}</a>`).join('');
    sec.classList.remove('d-none');
  }

  async function render(part, opts) {
    const suppliers = part.suppliers || [];
    const shops = suppliers.filter(s => (s.kind || 'shop') === 'shop');
    renderTechInfo(suppliers.filter(s => s.kind && s.kind !== 'shop'));
    renderAlternatives(part, shops);
    const sec = document.getElementById('part-vendors-section');
    if (!shops.length) { sec.classList.add('d-none'); return; }
    sec.classList.remove('d-none');

    const select = document.getElementById('part-currency');
    let currency = storedCurrency();
    if (!CURRENCIES.includes(currency)) {
      currency = (await CurrencyConvert.getPreferredCurrency())
        || (shops.find(s => s.currency_code) || {}).currency_code || 'GBP';
    }
    if (!CURRENCIES.includes(currency)) currency = 'GBP';
    select.innerHTML = CURRENCIES.map(c => `<option value="${c}"${c === currency ? ' selected' : ''}>${c}</option>`).join('');

    let history = [];
    try {
      const resp = await fetch(opts.api + '/api/parts/' + encodeURIComponent(part.slug) + '/price-history', { credentials: 'include' });
      if (resp.ok) history = await resp.json();
    } catch (e) { /* the chart is optional */ }

    const draw = async () => {
      await renderVendors(part, shops, currency, opts);
      await renderChart(history, currency);
    };
    select.onchange = () => { currency = select.value; storeCurrency(currency); draw(); };
    await draw();

    document.getElementById('part-vendors').onclick = async e => {
      const mark = e.target.closest('[data-mark-unavailable]');
      const clear = e.target.closest('[data-clear-unavailable]');
      if (!mark && !clear) return;
      const id = (mark || clear).dataset[mark ? 'markUnavailable' : 'clearUnavailable'];
      const url = opts.api + '/api/parts/' + encodeURIComponent(part.slug) + '/suppliers/' + id + '/unavailable';
      let resp;
      if (mark) {
        const note = window.prompt ? window.prompt('What’s wrong? (out of stock, discontinued…) Optional.', '') : '';
        if (note === null) return;
        resp = await ProjectAuth.apiFetch(url, {
          method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ note }),
        });
      } else {
        resp = await ProjectAuth.apiFetch(url, { method: 'DELETE' });
      }
      if (!resp.ok) {
        let detail = 'HTTP ' + resp.status;
        try { detail = (await resp.json()).detail || detail; } catch (err) {}
        alert('Could not update this vendor: ' + detail);
        return;
      }
      const updated = await resp.json();
      part.suppliers = updated.suppliers;
      render(part, opts);
    };
  }

  window.PartPrices = { render };
})();
