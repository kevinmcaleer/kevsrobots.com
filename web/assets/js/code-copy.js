/*
 * Copy-to-clipboard buttons on code blocks.
 *
 * Every page (via _includes/code_copy.html) gets a button at the top right of
 * each <pre><code> block once the page has loaded. Pages that render Markdown
 * later in the browser (projects/view.html) call CodeCopy.attach(root) after
 * their content is in the DOM. Mermaid blocks are skipped: they become
 * diagrams, and there's nothing useful to copy.
 *
 * The <pre> is wrapped in a .code-copy-wrap so the button stays in the corner
 * when a long line scrolls the block sideways.
 */
(function () {
  'use strict';

  var COPY_ICON = '<svg viewBox="0 0 16 16" width="16" height="16" aria-hidden="true"><path d="M5.5 5.5h8v8h-8zM10.5 5.5v-3h-8v8h3"/></svg>';
  var DONE_ICON = '<svg viewBox="0 0 16 16" width="16" height="16" aria-hidden="true"><path d="M3 8.5l3 3 7-7"/></svg>';

  function copyText(text) {
    if (window.isSecureContext && navigator.clipboard) {
      return navigator.clipboard.writeText(text).catch(function () { return legacyCopy(text); });
    }
    return legacyCopy(text);
  }

  // navigator.clipboard is missing outside a secure context (http:// dev servers).
  function legacyCopy(text) {
    return new Promise(function (resolve, reject) {
      var area = document.createElement('textarea');
      area.value = text;
      area.setAttribute('readonly', '');
      area.style.position = 'fixed';
      area.style.opacity = '0';
      document.body.appendChild(area);
      area.select();
      var ok = false;
      try { ok = document.execCommand('copy'); } catch (e) { ok = false; }
      area.remove();
      if (ok) { resolve(); } else { reject(new Error('copy failed')); }
    });
  }

  function setState(button, label, icon) {
    button.innerHTML = icon;
    button.title = label;
    button.setAttribute('aria-label', label);
  }

  function addButton(code) {
    var pre = code.parentElement;
    if (!pre || pre.dataset.codeCopy) return;
    if (code.closest('.language-mermaid')) return;
    pre.dataset.codeCopy = '1';

    var wrap = document.createElement('div');
    wrap.className = 'code-copy-wrap';
    pre.parentNode.insertBefore(wrap, pre);
    wrap.appendChild(pre);

    var button = document.createElement('button');
    button.type = 'button';
    button.className = 'copy-code-button';
    setState(button, 'Copy code', COPY_ICON);
    wrap.appendChild(button);

    var reset;
    button.addEventListener('click', function () {
      copyText(code.textContent.replace(/\n$/, '')).then(function () {
        button.classList.add('copied');
        setState(button, 'Copied!', DONE_ICON);
      }, function (err) {
        console.error('Copy failed:', err);
        setState(button, 'Couldn’t copy', COPY_ICON);
      });
      clearTimeout(reset);
      reset = setTimeout(function () {
        button.classList.remove('copied');
        setState(button, 'Copy code', COPY_ICON);
      }, 2000);
    });
  }

  function attach(root) {
    (root || document).querySelectorAll('pre > code').forEach(addButton);
  }

  window.CodeCopy = { attach: attach };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function () { attach(document); });
  } else {
    attach(document);
  }
})();
