/**
 * Feedback widget — issue #138.
 * Floats a "Feedback" pill on /projects/* and /parts/* pages for
 * authenticated users. Opens a slide-up panel that POSTs to the
 * Projects API. Backend implementation lives in a separate repo;
 * spec at web/feedback/API_SPEC.md.
 */
(function () {
  'use strict';

  if (window.__krFeedbackInited) return;
  window.__krFeedbackInited = true;

  const API = 'https://projects.kevsrobots.com';
  const MIN_MESSAGE_LEN = 10;
  const MAX_SCREENSHOT_BYTES = 4 * 1024 * 1024;
  const PILL_REHIDE_MS = 10 * 1000;
  const TOAST_MS = 3000;
  const SUCCESS_CLOSE_MS = 1500;

  // The widget needs ProjectAuth.apiFetch. If it's not loaded yet (e.g.
  // pages that don't include project-auth.js), bail silently — feedback
  // is opt-in and never blocks the page.
  function authFetch(url, opts) {
    if (window.ProjectAuth && typeof window.ProjectAuth.apiFetch === 'function') {
      return window.ProjectAuth.apiFetch(url, opts);
    }
    opts = opts || {};
    opts.credentials = 'include';
    return fetch(url, opts);
  }

  function esc(s) {
    const d = document.createElement('div');
    d.textContent = s == null ? '' : String(s);
    return d.innerHTML;
  }

  function isValidEmail(s) {
    return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(s);
  }

  // Probe auth. Primary: /api/auth/me (per spec). Fallback: /api/projects/my/list
  // (which exists today and 401s for guests). Returns { authed, email }.
  async function probeAuth() {
    try {
      const r = await authFetch(API + '/api/auth/me');
      if (r.ok) {
        let data = null;
        try { data = await r.json(); } catch (e) {}
        return { authed: true, email: (data && data.email) || '' };
      }
      if (r.status === 401 || r.status === 403) return { authed: false, email: '' };
    } catch (e) {}
    try {
      const r2 = await authFetch(API + '/api/projects/my/list');
      if (r2.ok) return { authed: true, email: '' };
    } catch (e) {}
    return { authed: false, email: '' };
  }

  const SENTIMENTS = [
    { key: 'love',  emoji: '\u{1F60D}', label: 'Love' },
    { key: 'like',  emoji: '\u{1F642}', label: 'Like' },
    { key: 'issue', emoji: '\u{1F61E}', label: 'Issue' },
    { key: 'idea',  emoji: '\u{1F4A1}', label: 'Idea' }
  ];

  function buildMarkup(root) {
    root.innerHTML =
      '<button type="button" class="kr-feedback-pill" id="kr-feedback-pill"' +
      ' aria-haspopup="dialog" aria-controls="kr-feedback-panel" aria-expanded="false"' +
      ' aria-label="Send feedback">' +
      '<i class="fas fa-comment-dots me-2" aria-hidden="true"></i>Feedback' +
      '</button>' +
      '<div class="kr-feedback-backdrop" id="kr-feedback-backdrop" hidden></div>' +
      '<div class="kr-feedback-panel" id="kr-feedback-panel" role="dialog"' +
      ' aria-modal="true" aria-labelledby="kr-feedback-title" hidden>' +
        '<div class="kr-feedback-header">' +
          '<div>' +
            '<h3 id="kr-feedback-title">Send Feedback</h3>' +
            '<div class="kr-subtitle">Help us improve this page.</div>' +
          '</div>' +
          '<button type="button" class="kr-feedback-close" aria-label="Close feedback">&times;</button>' +
        '</div>' +
        '<div class="kr-feedback-body" id="kr-feedback-body">' +
          '<div class="kr-sentiment-row" role="radiogroup" aria-label="Feedback type">' +
            SENTIMENTS.map(function (s, i) {
              return '<button type="button" class="kr-sentiment" role="radio"' +
                ' aria-checked="false" data-sentiment="' + s.key + '"' +
                ' tabindex="' + (i === 0 ? '0' : '-1') + '">' +
                '<span class="kr-emoji" aria-hidden="true">' + s.emoji + '</span>' +
                '<span>' + s.label + '</span>' +
                '</button>';
            }).join('') +
          '</div>' +
          '<label for="kr-feedback-message">Your message</label>' +
          '<textarea id="kr-feedback-message" rows="4" maxlength="2000"' +
          ' placeholder="What\'s on your mind?"></textarea>' +
          '<div class="kr-char-count" id="kr-feedback-charcount">0 / 2000</div>' +
          '<div class="kr-collapsible" id="kr-email-wrap">' +
            '<button type="button" class="kr-collapsible-toggle">' +
              '<span class="kr-caret">&#9656;</span> Want a reply? Add your email' +
            '</button>' +
            '<div class="kr-collapsible-body">' +
              '<label for="kr-feedback-email">Email</label>' +
              '<input type="email" id="kr-feedback-email" placeholder="you@example.com" autocomplete="email">' +
            '</div>' +
          '</div>' +
          '<div class="kr-collapsible" id="kr-screenshot-wrap">' +
            '<button type="button" class="kr-collapsible-toggle">' +
              '<span class="kr-caret">&#9656;</span> Attach a screenshot' +
            '</button>' +
            '<div class="kr-collapsible-body">' +
              '<div class="kr-drop-zone" id="kr-drop-zone" tabindex="0">' +
                '<i class="fas fa-image me-1" aria-hidden="true"></i>' +
                'Drop, paste, or click to choose (PNG/JPG, &le; 4 MB)' +
              '</div>' +
              '<input type="file" id="kr-screenshot-file" accept="image/*" hidden>' +
              '<div class="kr-screenshot-preview" id="kr-screenshot-preview" hidden>' +
                '<img alt="Screenshot preview" id="kr-screenshot-img">' +
                '<button type="button" class="kr-remove" aria-label="Remove screenshot">&times;</button>' +
              '</div>' +
            '</div>' +
          '</div>' +
          '<div class="kr-feedback-error" id="kr-feedback-error" role="alert"></div>' +
        '</div>' +
        '<div class="kr-feedback-success" id="kr-feedback-success" aria-live="polite">' +
          '<i class="fas fa-check-circle" aria-hidden="true"></i>' +
          '<div>Thanks! Sent.</div>' +
        '</div>' +
        '<div class="kr-feedback-footer">' +
          '<button type="button" class="kr-feedback-cancel">Cancel</button>' +
          '<button type="button" class="kr-feedback-send" disabled>Send</button>' +
        '</div>' +
      '</div>' +
      '<div class="kr-feedback-toast" id="kr-feedback-toast" role="status" aria-live="polite"></div>';
  }

  function mount(prefillEmail) {
    const root = document.getElementById('kr-feedback-root');
    if (!root) return;
    buildMarkup(root);

    const pill      = root.querySelector('#kr-feedback-pill');
    const backdrop  = root.querySelector('#kr-feedback-backdrop');
    const panel     = root.querySelector('#kr-feedback-panel');
    const closeBtn  = panel.querySelector('.kr-feedback-close');
    const cancelBtn = panel.querySelector('.kr-feedback-cancel');
    const sendBtn   = panel.querySelector('.kr-feedback-send');
    const errorEl   = panel.querySelector('#kr-feedback-error');
    const successEl = panel.querySelector('#kr-feedback-success');
    const bodyEl    = panel.querySelector('#kr-feedback-body');
    const footerEl  = panel.querySelector('.kr-feedback-footer');
    const messageEl = panel.querySelector('#kr-feedback-message');
    const charEl    = panel.querySelector('#kr-feedback-charcount');
    const sentiments = panel.querySelectorAll('.kr-sentiment');
    const emailEl   = panel.querySelector('#kr-feedback-email');
    const dropZone  = panel.querySelector('#kr-drop-zone');
    const fileInput = panel.querySelector('#kr-screenshot-file');
    const previewWrap = panel.querySelector('#kr-screenshot-preview');
    const previewImg  = panel.querySelector('#kr-screenshot-img');
    const removeBtn   = previewWrap.querySelector('.kr-remove');
    const toastEl   = root.querySelector('#kr-feedback-toast');

    let selectedSentiment = null;
    let screenshotFile = null;
    let panelOpen = false;
    let submitting = false;

    if (prefillEmail) emailEl.value = prefillEmail;

    pill.style.display = 'flex';

    function refreshSend() {
      const hasSent = !!selectedSentiment;
      const longEnough = messageEl.value.trim().length >= MIN_MESSAGE_LEN;
      sendBtn.disabled = submitting || !(hasSent && longEnough);
    }

    function setSentiment(key) {
      selectedSentiment = key;
      sentiments.forEach(function (btn) {
        const isMe = btn.getAttribute('data-sentiment') === key;
        btn.classList.toggle('is-selected', isMe);
        btn.setAttribute('aria-checked', isMe ? 'true' : 'false');
        btn.setAttribute('tabindex', isMe ? '0' : '-1');
      });
      refreshSend();
    }

    function updateCharCount() {
      const n = messageEl.value.length;
      charEl.textContent = n + ' / 2000';
      charEl.classList.toggle('is-near', n > 1800 && n < 2000);
      charEl.classList.toggle('is-over', n >= 2000);
    }

    function getFocusable() {
      return Array.prototype.slice.call(
        panel.querySelectorAll(
          'button:not([disabled]), [href], input:not([disabled]), select:not([disabled]),' +
          ' textarea:not([disabled]), [tabindex]:not([tabindex="-1"])'
        )
      ).filter(function (el) { return el.offsetParent !== null || el === document.activeElement; });
    }

    function openPanel() {
      if (panelOpen) return;
      panelOpen = true;
      panel.hidden = false;
      backdrop.hidden = false;
      // Force reflow so the transition runs.
      void panel.offsetHeight;
      panel.classList.add('is-open');
      backdrop.classList.add('is-open');
      pill.setAttribute('aria-expanded', 'true');
      setTimeout(function () {
        const first = panel.querySelector('.kr-sentiment');
        if (first) first.focus();
      }, 30);
    }

    function closePanel(returnFocus) {
      if (!panelOpen) return;
      panelOpen = false;
      panel.classList.remove('is-open');
      backdrop.classList.remove('is-open');
      pill.setAttribute('aria-expanded', 'false');
      setTimeout(function () {
        panel.hidden = true;
        backdrop.hidden = true;
      }, 220);
      if (returnFocus !== false) pill.focus();
    }

    function resetForm() {
      selectedSentiment = null;
      sentiments.forEach(function (btn) {
        btn.classList.remove('is-selected');
        btn.setAttribute('aria-checked', 'false');
      });
      messageEl.value = '';
      updateCharCount();
      emailEl.classList.remove('is-invalid');
      errorEl.classList.remove('is-visible');
      errorEl.textContent = '';
      removeScreenshot();
      successEl.classList.remove('is-visible');
      bodyEl.style.display = '';
      footerEl.style.display = '';
      refreshSend();
    }

    function showToast(msg) {
      toastEl.textContent = msg;
      toastEl.classList.add('is-visible');
      setTimeout(function () {
        toastEl.classList.remove('is-visible');
      }, TOAST_MS);
    }

    function hidePillTemporarily() {
      pill.style.display = 'none';
      setTimeout(function () { pill.style.display = 'flex'; }, PILL_REHIDE_MS);
    }

    function setScreenshot(file) {
      if (!file) return;
      if (!/^image\//.test(file.type)) {
        errorEl.textContent = 'Screenshot must be an image.';
        errorEl.classList.add('is-visible');
        return;
      }
      if (file.size > MAX_SCREENSHOT_BYTES) {
        errorEl.textContent = 'Screenshot is too large (max 4 MB).';
        errorEl.classList.add('is-visible');
        return;
      }
      errorEl.classList.remove('is-visible');
      screenshotFile = file;
      const url = URL.createObjectURL(file);
      previewImg.src = url;
      previewWrap.hidden = false;
      // Make sure the screenshot collapsible is open so the user can see it.
      panel.querySelector('#kr-screenshot-wrap').classList.add('is-open');
    }

    function removeScreenshot() {
      screenshotFile = null;
      previewWrap.hidden = true;
      if (previewImg.src && previewImg.src.indexOf('blob:') === 0) {
        URL.revokeObjectURL(previewImg.src);
      }
      previewImg.removeAttribute('src');
      fileInput.value = '';
    }

    async function submit() {
      if (sendBtn.disabled) return;
      errorEl.classList.remove('is-visible');
      errorEl.textContent = '';

      const message = messageEl.value.trim();
      if (message.length < MIN_MESSAGE_LEN || !selectedSentiment) {
        return;
      }

      const emailVal = emailEl.value.trim();
      if (emailVal && !isValidEmail(emailVal)) {
        emailEl.classList.add('is-invalid');
        errorEl.textContent = 'That email address doesn\'t look right.';
        errorEl.classList.add('is-visible');
        return;
      }
      emailEl.classList.remove('is-invalid');

      submitting = true;
      const originalLabel = sendBtn.textContent;
      sendBtn.disabled = true;
      sendBtn.textContent = 'Sending...';

      const meta = {
        sentiment: selectedSentiment,
        message: message,
        email: emailVal || null,
        page_url: window.location.href,
        referrer: document.referrer || '',
        user_agent: navigator.userAgent,
        viewport: window.innerWidth + 'x' + window.innerHeight
      };

      let resp;
      try {
        if (screenshotFile) {
          const fd = new FormData();
          Object.keys(meta).forEach(function (k) {
            if (meta[k] != null) fd.append(k, meta[k]);
          });
          fd.append('screenshot', screenshotFile, screenshotFile.name || 'screenshot.png');
          resp = await authFetch(API + '/api/feedback', { method: 'POST', body: fd });
        } else {
          resp = await authFetch(API + '/api/feedback', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json', 'Accept': 'application/json' },
            body: JSON.stringify(meta)
          });
        }
      } catch (e) {
        resp = null;
      }

      submitting = false;
      sendBtn.textContent = originalLabel;
      refreshSend();

      if (resp && (resp.status === 200 || resp.status === 201)) {
        bodyEl.style.display = 'none';
        footerEl.style.display = 'none';
        successEl.classList.add('is-visible');
        showToast('Thanks — feedback sent.');
        setTimeout(function () {
          closePanel(true);
          resetForm();
        }, SUCCESS_CLOSE_MS);
        hidePillTemporarily();
      } else {
        errorEl.textContent = "Couldn't send right now — please try again.";
        errorEl.classList.add('is-visible');
      }
    }

    pill.addEventListener('click', function () {
      if (panelOpen) closePanel(true);
      else openPanel();
    });

    closeBtn.addEventListener('click', function () { closePanel(true); });
    cancelBtn.addEventListener('click', function () { closePanel(true); resetForm(); });
    backdrop.addEventListener('click', function () { closePanel(true); });

    sentiments.forEach(function (btn) {
      btn.addEventListener('click', function () {
        setSentiment(btn.getAttribute('data-sentiment'));
      });
      btn.addEventListener('keydown', function (e) {
        if (e.key === 'ArrowRight' || e.key === 'ArrowLeft') {
          e.preventDefault();
          const list = Array.prototype.slice.call(sentiments);
          const idx = list.indexOf(btn);
          const next = e.key === 'ArrowRight'
            ? list[(idx + 1) % list.length]
            : list[(idx - 1 + list.length) % list.length];
          next.focus();
        }
        if (e.key === ' ' || e.key === 'Enter') {
          e.preventDefault();
          setSentiment(btn.getAttribute('data-sentiment'));
        }
      });
    });

    messageEl.addEventListener('input', function () {
      updateCharCount();
      refreshSend();
    });

    panel.querySelectorAll('.kr-collapsible-toggle').forEach(function (toggle) {
      toggle.addEventListener('click', function () {
        toggle.parentElement.classList.toggle('is-open');
      });
    });

    dropZone.addEventListener('click', function () { fileInput.click(); });
    dropZone.addEventListener('keydown', function (e) {
      if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); fileInput.click(); }
    });
    ['dragenter', 'dragover'].forEach(function (ev) {
      dropZone.addEventListener(ev, function (e) {
        e.preventDefault();
        dropZone.classList.add('is-drag');
      });
    });
    ['dragleave', 'drop'].forEach(function (ev) {
      dropZone.addEventListener(ev, function (e) {
        e.preventDefault();
        dropZone.classList.remove('is-drag');
      });
    });
    dropZone.addEventListener('drop', function (e) {
      const f = e.dataTransfer && e.dataTransfer.files && e.dataTransfer.files[0];
      if (f) setScreenshot(f);
    });
    fileInput.addEventListener('change', function () {
      const f = fileInput.files && fileInput.files[0];
      if (f) setScreenshot(f);
    });
    removeBtn.addEventListener('click', removeScreenshot);

    // Listen for paste events anywhere inside the panel.
    panel.addEventListener('paste', function (e) {
      if (!panelOpen) return;
      const items = e.clipboardData && e.clipboardData.items;
      if (!items) return;
      for (let i = 0; i < items.length; i++) {
        if (items[i].kind === 'file' && items[i].type.indexOf('image/') === 0) {
          const f = items[i].getAsFile();
          if (f) {
            setScreenshot(f);
            e.preventDefault();
            break;
          }
        }
      }
    });

    sendBtn.addEventListener('click', submit);

    document.addEventListener('keydown', function (e) {
      if (!panelOpen) return;
      if (e.key === 'Escape') {
        e.preventDefault();
        closePanel(true);
        return;
      }
      if ((e.metaKey || e.ctrlKey) && e.key === 'Enter') {
        e.preventDefault();
        submit();
        return;
      }
      if (e.key === 'Tab') {
        // Focus trap.
        const list = getFocusable();
        if (!list.length) return;
        const first = list[0];
        const last = list[list.length - 1];
        if (e.shiftKey && document.activeElement === first) {
          e.preventDefault();
          last.focus();
        } else if (!e.shiftKey && document.activeElement === last) {
          e.preventDefault();
          first.focus();
        }
      }
    });

    updateCharCount();
  }

  function init() {
    if (!document.getElementById('kr-feedback-root')) return;
    probeAuth().then(function (r) {
      if (r.authed) mount(r.email);
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
