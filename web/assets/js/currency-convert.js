// Issue #150 — display-only currency conversion helper.
//
// Two responsibilities:
//   1) Probe the logged-in user's preferred_currency once per page
//      (returns null for guests or users who haven't set one).
//   2) Format a (amount, currency_code) pair, optionally appending a
//      converted-price tooltip when the preference differs.
//
// Graceful-degradation contract:
//   * No currency_code on the row -> show the bare amount with no
//     currency symbol. We never *guess* the currency.
//   * Preferred currency matches the row's currency -> no conversion.
//   * /api/fx/convert returns rate=null -> show the source amount
//     only, no parenthetical converted price.
//   * Logged out, or profile fetch fails -> no conversion attempted.
//
// This file is independent of #149's data model: rows that lack a
// currency_code column will simply fall through to "show bare
// amount" without breaking the page.

(function (global) {
  'use strict';

  var PROJECTS_API = 'https://projects.kevsrobots.com';

  var SYMBOLS = {
    GBP: '£',     // £
    USD: '$',
    EUR: '€',     // €
    JPY: '¥',     // ¥
    AUD: 'A$',
    CAD: 'C$'
  };

  // In-page cache of /api/fx/convert responses, keyed by from|to|amount.
  // Page lifetimes are short so we don't worry about eviction.
  var convertCache = new Map();

  var preferredPromise = null;

  // Probe the user's profile for preferred_currency. Returns null for
  // guests, network errors, or when no preference is set.
  function getPreferredCurrency() {
    if (preferredPromise) return preferredPromise;
    preferredPromise = (async function () {
      try {
        // ProjectAuth.apiFetch attaches the Bearer token when present.
        // Falls back to plain fetch (no auth) when the helper is absent.
        var url = PROJECTS_API + '/api/users/me/profile';
        var resp;
        if (global.ProjectAuth && typeof global.ProjectAuth.apiFetch === 'function') {
          resp = await global.ProjectAuth.apiFetch(url);
        } else {
          resp = await fetch(url);
        }
        if (!resp || !resp.ok) return null;
        var body = await resp.json();
        var p = body && body.preferred_currency;
        return (typeof p === 'string' && p.length === 3) ? p.toUpperCase() : null;
      } catch (_) {
        return null;
      }
    })();
    return preferredPromise;
  }

  function symbolFor(code) {
    if (!code) return '';
    var c = String(code).toUpperCase();
    return SYMBOLS[c] || (c + ' ');
  }

  // Format an amount + ISO currency code, e.g. "£12.50".
  // Returns the bare number if `code` is missing — we never invent a symbol.
  function formatPrice(amount, code) {
    if (amount == null) return '-';
    var n = Number(amount);
    if (!isFinite(n)) return '-';
    if (!code) return n.toFixed(2);
    var sym = symbolFor(code);
    if (code === 'JPY') {
      // JPY conventionally has no decimal places.
      return sym + Math.round(n).toLocaleString();
    }
    return sym + n.toFixed(2);
  }

  function cacheKey(from, to, amount) {
    return from + '|' + to + '|' + Number(amount).toFixed(4);
  }

  async function convert(amount, from, to) {
    if (amount == null || !from || !to) return null;
    if (from === to) return { amount: Number(amount), rate: 1, fetched_at: null };
    var key = cacheKey(from, to, amount);
    if (convertCache.has(key)) return convertCache.get(key);
    var promise = (async function () {
      try {
        var qs = '?from=' + encodeURIComponent(from) +
                 '&to=' + encodeURIComponent(to) +
                 '&amount=' + encodeURIComponent(amount);
        var resp = await fetch(PROJECTS_API + '/api/fx/convert' + qs);
        if (!resp.ok) return null;
        var body = await resp.json();
        // Rate may be null when the upstream is cold + unreachable.
        if (body == null || body.rate == null || body.amount == null) return null;
        return body;
      } catch (_) {
        return null;
      }
    })();
    convertCache.set(key, promise);
    return promise;
  }

  // Build a price cell. If a preferred currency is set and differs
  // from the row's currency, return the source price + converted
  // suffix; otherwise return the source price alone.
  //
  // Returns an HTML string. Safe to drop into innerHTML — both
  // numbers come from JSON, the currency code is a 3-letter
  // alpha-only value we already validated server-side.
  async function renderPrice(amount, currencyCode, opts) {
    if (amount == null) return '-';
    var bare = formatPrice(amount, currencyCode);
    if (!currencyCode) {
      // No currency on the row -> display as-is, no conversion attempt.
      return bare;
    }
    var preferred = await getPreferredCurrency();
    if (!preferred || preferred === currencyCode) {
      return bare;
    }
    var conv = await convert(amount, currencyCode, preferred);
    if (!conv) return bare;
    var converted = formatPrice(conv.amount, preferred);
    var tooltip = 'Converted from ' + bare;
    if (conv.fetched_at) {
      tooltip += ' (rate fetched ' + new Date(conv.fetched_at).toLocaleDateString() + ')';
    }
    return bare + ' <small class="text-muted" title="' + tooltip + '">(≈ ' + converted + ')</small>';
  }

  global.CurrencyConvert = {
    getPreferredCurrency: getPreferredCurrency,
    formatPrice: formatPrice,
    convert: convert,
    renderPrice: renderPrice,
    symbolFor: symbolFor
  };
})(window);
