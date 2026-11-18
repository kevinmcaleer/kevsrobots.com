/*
 * "Open in Snakie" on lesson code (kevsrobots.com #581, Learn → Snakie).
 * =============================================================================
 *
 * Every Python code block in a lesson gets an "Open in Snakie" link under it.
 * It opens app.snakie.org with the block's code in a new tab, the same way the
 * Snakie code embed's own button does (kevinmcaleer/Snakie#1287): the code is
 * deflated, base64url-encoded and carried in the hash as `#src=…`, so it never
 * reaches a server log. `?name=` carries a file name for the tab (lesson
 * slug plus a number) for when Snakie reads it; today it opens as main.py.
 *
 * Snakie boxes ({% include snakie.html %}) are left alone: the embed has its
 * own "Open in Snakie" that also carries the reader's edits.
 *
 * Desktop: once the Snakie desktop app registers `snakie://` (Snakie epic
 * kevinmcaleer/Snakie#1310), setting localStorage "kr.snakie.target" to
 * "desktop" sends these links to `snakie://open?…#src=…` instead. Until then
 * the web app is the only target.
 */
(function () {
  'use strict';

  var APP = 'https://app.snakie.org/';
  var SELECTOR = [
    'div.language-python pre code',
    'div.language-py pre code',
    'div.language-micropython pre code',
    'pre > code.language-python',
    'pre > code.language-py',
    'pre > code.language-micropython'
  ].join(',');
  // A program that long would make a URL some browsers refuse.
  var MAX_PAYLOAD = 30000;

  var SNAKIE_ICON = '<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true"><path d="M4 3h6a3 3 0 0 1 0 6H6a3 3 0 0 0 0 6h6" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/><circle cx="11.5" cy="5.5" r=".9" fill="currentColor"/></svg>';

  function target() {
    try {
      return window.localStorage.getItem('kr.snakie.target') === 'desktop' ? 'desktop' : 'web';
    } catch (e) {
      return 'web';
    }
  }

  function toBase64Url(bytes) {
    var bin = '';
    for (var i = 0; i < bytes.length; i++) bin += String.fromCharCode(bytes[i]);
    return btoa(bin).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
  }

  // Raw deflate, which is what Snakie's decoder (fflate inflateSync) reads.
  function encode(code) {
    var stream = new Blob([code]).stream().pipeThrough(new CompressionStream('deflate-raw'));
    return new Response(stream).arrayBuffer().then(function (buf) {
      return toBase64Url(new Uint8Array(buf));
    });
  }

  function snakieUrl(name, payload) {
    var query = 'name=' + encodeURIComponent(name);
    var hash = '#src=' + payload;
    if (target() === 'desktop') return 'snakie://open?' + query + hash;
    return APP + '?' + query + hash;
  }

  function baseName() {
    var last = window.location.pathname.replace(/\/+$/, '').split('/').pop() || 'lesson';
    last = last.replace(/\.html?$/, '').replace(/[^A-Za-z0-9_-]+/g, '_');
    return last || 'lesson';
  }

  function addLink(code, name) {
    var text = code.textContent.replace(/\n$/, '');
    if (!text.trim()) return Promise.resolve();
    // The copy button wraps the <pre>; put the link after that wrapper.
    var pre = code.parentElement;
    var anchor = pre.parentElement && pre.parentElement.classList.contains('code-copy-wrap') ? pre.parentElement : pre;
    return encode(text).then(function (payload) {
      if (payload.length > MAX_PAYLOAD) return;
      var bar = document.createElement('div');
      bar.className = 'kr-open-snakie-bar';
      var link = document.createElement('a');
      link.className = 'kr-open-snakie';
      link.href = snakieUrl(name, payload);
      link.target = '_blank';
      link.rel = 'noopener';
      link.title = 'Open this code in Snakie, the kevsrobots Python editor';
      link.innerHTML = SNAKIE_ICON + '<span>Open in Snakie</span>';
      bar.appendChild(link);
      anchor.parentNode.insertBefore(bar, anchor.nextSibling);
    });
  }

  function attach(root) {
    if (typeof CompressionStream === 'undefined') return;
    var blocks = Array.prototype.slice.call((root || document).querySelectorAll(SELECTOR));
    var seen = [];
    var base = baseName();
    var n = 0;
    blocks.forEach(function (code) {
      if (seen.indexOf(code) !== -1 || code.closest('.snakie-box') || code.dataset.krSnakie) return;
      seen.push(code);
      code.dataset.krSnakie = '1';
      n += 1;
      var name = base + (n > 1 ? '_' + n : '') + '.py';
      addLink(code, name).catch(function (err) {
        console.warn('Open in Snakie: could not encode a code block', err);
      });
    });
  }

  window.OpenInSnakie = { attach: attach };

  // After code-copy.js, which also runs on DOMContentLoaded and wraps the <pre>.
  function start() { setTimeout(function () { attach(document); }, 0); }
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', start);
  } else {
    start();
  }
})();
