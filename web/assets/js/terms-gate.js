/**
 * TermsGate — frontend helper for the Projects Hub T&Cs acceptance gate.
 *
 * Surfaces three things:
 *   - TermsGate.check() — Promise<bool>. true iff the logged-in user has
 *     accepted the CURRENT version of the terms (as reported by the
 *     server in /api/auth/me).
 *   - TermsGate.showModal() — Promise<bool>. Renders the acceptance modal,
 *     resolves true when the user accepts, false on cancel / close.
 *   - TermsGate.guard(fn) — convenience wrapper. If the user has accepted,
 *     calls fn() and returns its result. Otherwise pops the modal first and
 *     only calls fn() on accept.
 *
 * The modal markup is injected into <body> on first call so pages don't
 * need any per-page HTML. The modal fetches the rendered T&Cs page at
 * /projects-hub-terms/ via fetch() and inlines the <main> content into a
 * scrollable preview, with a "Read full terms in new tab" link as a
 * fallback for screen-readers and "just give me the whole page" users.
 *
 * Depends on ProjectAuth.apiFetch (web/assets/js/project-auth.js) being
 * loaded first so the dev-token shim works in local development. Falls
 * back to plain fetch with ``credentials: include`` if ProjectAuth is
 * not present.
 */
(function () {
  'use strict';

  var PROJECTS_API = 'https://projects.kevsrobots.com';
  var TERMS_URL = '/projects-hub-terms/';
  var MODAL_ID = 'terms-gate-modal';

  // Cached /api/auth/me payload — invalidated after each accept.
  var _meCache = null;

  function _apiFetch(url, opts) {
    if (typeof ProjectAuth !== 'undefined' && ProjectAuth.apiFetch) {
      return ProjectAuth.apiFetch(url, opts);
    }
    opts = opts || {};
    opts.credentials = 'include';
    return fetch(url, opts);
  }

  function _fetchMe(force) {
    if (_meCache && !force) {
      return Promise.resolve(_meCache);
    }
    return _apiFetch(PROJECTS_API + '/api/auth/me', { method: 'GET' })
      .then(function (r) {
        if (!r.ok) {
          // 401 / 403 — surface as "not logged in / not accepted".
          return { _httpStatus: r.status };
        }
        return r.json();
      })
      .then(function (body) {
        _meCache = body;
        return body;
      })
      .catch(function () {
        return { _httpStatus: 0 };
      });
  }

  /**
   * Return true iff the user's stored terms_accepted_version matches the
   * server's current_terms_version.
   */
  function check() {
    return _fetchMe(false).then(function (me) {
      if (!me || me._httpStatus) return false;
      if (!me.terms_accepted_version) return false;
      if (!me.terms_accepted_at) return false;
      return me.terms_accepted_version === me.current_terms_version;
    });
  }

  /**
   * Build the modal DOM (once) and return its Bootstrap instance.
   */
  function _ensureModal() {
    var existing = document.getElementById(MODAL_ID);
    if (existing) return existing;

    var html =
      '<div class="modal fade" id="' + MODAL_ID + '" tabindex="-1" ' +
      '  aria-labelledby="' + MODAL_ID + '-label" aria-hidden="true" ' +
      '  data-bs-backdrop="static" data-bs-keyboard="false">' +
      '  <div class="modal-dialog modal-dialog-centered modal-dialog-scrollable modal-lg">' +
      '    <div class="modal-content">' +
      '      <div class="modal-header">' +
      '        <h5 class="modal-title" id="' + MODAL_ID + '-label">' +
      '          <i class="fas fa-file-signature me-2 text-primary"></i>' +
      '          Please review the Projects Hub terms' +
      '        </h5>' +
      '      </div>' +
      '      <div class="modal-body">' +
      '        <p class="text-muted small mb-2">' +
      '          To upload projects, images, files, BOMs, makes, comments, or ' +
      '          parts-catalogue edits, please read and accept the content ' +
      '          terms below. You retain ownership of what you upload — see ' +
      '          the TL;DR at the top of the full page.' +
      '        </p>' +
      '        <div id="' + MODAL_ID + '-preview" class="border rounded p-3 mb-3" ' +
      '          style="max-height:40vh; overflow-y:auto; background:#f8f9fa;">' +
      '          <div class="text-muted"><i class="fas fa-spinner fa-spin me-2"></i>Loading terms…</div>' +
      '        </div>' +
      '        <p class="small mb-3">' +
      '          <a href="' + TERMS_URL + '" target="_blank" rel="noopener">' +
      '            <i class="fas fa-external-link-alt me-1"></i>Open the full terms in a new tab' +
      '          </a>' +
      '          &middot; current version: <span id="' + MODAL_ID + '-version" class="font-monospace">…</span>' +
      '        </p>' +
      '        <div id="' + MODAL_ID + '-error" class="alert alert-danger mt-3 d-none" role="alert"></div>' +
      '      </div>' +
      '      <div class="modal-footer flex-wrap gap-2 align-items-center" style="background:#fff8e1; border-top:2px solid #ffc107;">' +
      '        <div class="form-check me-auto fs-6 mb-0">' +
      '          <input class="form-check-input" type="checkbox" id="terms-accept" style="width:1.25em; height:1.25em;">' +
      '          <label class="form-check-label fw-semibold" for="terms-accept">' +
      '            I have read and agree to the Projects Hub terms' +
      '          </label>' +
      '        </div>' +
      '        <button type="button" class="btn btn-secondary" id="' + MODAL_ID + '-cancel">' +
      '          Cancel' +
      '        </button>' +
      '        <button type="button" class="btn btn-primary" id="' + MODAL_ID + '-confirm" disabled>' +
      '          <span id="' + MODAL_ID + '-confirm-spinner" class="spinner-border spinner-border-sm me-2 d-none" role="status"></span>' +
      '          I agree' +
      '        </button>' +
      '      </div>' +
      '    </div>' +
      '  </div>' +
      '</div>';

    var wrap = document.createElement('div');
    wrap.innerHTML = html;
    var node = wrap.firstElementChild;
    document.body.appendChild(node);

    // Wire the checkbox -> button enable state.
    var box = node.querySelector('#terms-accept');
    var confirm = node.querySelector('#' + MODAL_ID + '-confirm');
    box.addEventListener('change', function () {
      confirm.disabled = !box.checked;
    });

    // Lazy-load the terms preview body.
    fetch(TERMS_URL, { credentials: 'same-origin' })
      .then(function (r) { return r.ok ? r.text() : null; })
      .then(function (html) {
        var target = node.querySelector('#' + MODAL_ID + '-preview');
        if (!html) {
          target.innerHTML =
            '<div class="text-muted">' +
            'Could not load the terms inline. Please use the link above to ' +
            'read them in a new tab.' +
            '</div>';
          return;
        }
        var doc = new DOMParser().parseFromString(html, 'text/html');
        var main = doc.querySelector('main') ||
                   doc.querySelector('.content') ||
                   doc.querySelector('article') ||
                   doc.body;
        // Strip the DRAFT banner from the preview — it appears in full on
        // the linked-out page already.
        target.innerHTML = main.innerHTML;
      })
      .catch(function () {
        var target = node.querySelector('#' + MODAL_ID + '-preview');
        target.innerHTML =
          '<div class="text-muted">Could not load the terms inline.</div>';
      });

    return node;
  }

  function _showError(msg) {
    var node = document.getElementById(MODAL_ID);
    if (!node) return;
    var box = node.querySelector('#' + MODAL_ID + '-error');
    box.textContent = msg;
    box.classList.remove('d-none');
  }

  function _setVersion(version) {
    var el = document.getElementById(MODAL_ID + '-version');
    if (el) el.textContent = version || '?';
  }

  /**
   * Build the "sign in or sign up" modal shown when a logged-out user
   * triggers a gated action. Returns the modal node. Built once and
   * cached. The user picks an auth path; both options redirect with
   * ``return_to`` so they come back to the page they were on.
   */
  function _ensureSigninModal() {
    var id = MODAL_ID + '-signin';
    var existing = document.getElementById(id);
    if (existing) return existing;

    var html =
      '<div class="modal fade" id="' + id + '" tabindex="-1" ' +
      '  aria-labelledby="' + id + '-label" aria-hidden="true" ' +
      '  data-bs-backdrop="static" data-bs-keyboard="false">' +
      '  <div class="modal-dialog modal-dialog-centered">' +
      '    <div class="modal-content">' +
      '      <div class="modal-header">' +
      '        <h5 class="modal-title" id="' + id + '-label">' +
      '          <i class="fas fa-user-lock me-2 text-primary"></i>' +
      '          Sign in to continue' +
      '        </h5>' +
      '      </div>' +
      '      <div class="modal-body">' +
      '        <p class="mb-3">' +
      '          You need a kevsrobots.com account to upload to the ' +
      '          Projects Hub, post comments, or edit parts. It only ' +
      '          takes a moment.' +
      '        </p>' +
      '        <p class="text-muted small mb-0">' +
      '          Already have an account? Sign in. New here? Create one — ' +
      '          you can accept the Projects Hub terms straight after.' +
      '        </p>' +
      '      </div>' +
      '      <div class="modal-footer flex-wrap gap-2">' +
      '        <button type="button" class="btn btn-link me-auto" id="' + id + '-cancel">' +
      '          Not now' +
      '        </button>' +
      '        <button type="button" class="btn btn-outline-primary" id="' + id + '-signup">' +
      '          <i class="fas fa-user-plus me-2"></i>Create account' +
      '        </button>' +
      '        <button type="button" class="btn btn-primary" id="' + id + '-signin">' +
      '          <i class="fas fa-sign-in-alt me-2"></i>Sign in' +
      '        </button>' +
      '      </div>' +
      '    </div>' +
      '  </div>' +
      '</div>';

    var wrap = document.createElement('div');
    wrap.innerHTML = html;
    var node = wrap.firstElementChild;
    document.body.appendChild(node);
    return node;
  }

  /**
   * Show the signin-required modal. Resolves false on cancel; on
   * sign-in / sign-up the page navigates away so the promise never
   * resolves (caller's then() is replaced by full-page nav).
   */
  function _showSigninModal() {
    var node = _ensureSigninModal();
    var modal = null;
    if (window.bootstrap && window.bootstrap.Modal) {
      modal = window.bootstrap.Modal.getOrCreateInstance(node);
      modal.show();
    } else {
      node.classList.add('show');
      node.style.display = 'block';
    }

    var signinBtn = node.querySelector('#' + MODAL_ID + '-signin-signin');
    var signupBtn = node.querySelector('#' + MODAL_ID + '-signin-signup');
    var cancelBtn = node.querySelector('#' + MODAL_ID + '-signin-cancel');

    return new Promise(function (resolve) {
      var returnTo = window.location.pathname + window.location.search +
                     window.location.hash;
      var q = '?return_to=' + encodeURIComponent(returnTo);

      function cleanup(result) {
        signinBtn.removeEventListener('click', onSignin);
        signupBtn.removeEventListener('click', onSignup);
        cancelBtn.removeEventListener('click', onCancel);
        if (modal) {
          modal.hide();
        } else {
          node.classList.remove('show');
          node.style.display = 'none';
        }
        resolve(result);
      }

      function onSignin() { window.location.href = '/login' + q; }
      function onSignup() { window.location.href = '/register' + q; }
      function onCancel() { cleanup(false); }

      signinBtn.addEventListener('click', onSignin);
      signupBtn.addEventListener('click', onSignup);
      cancelBtn.addEventListener('click', onCancel);
    });
  }

  /**
   * Show the modal and resolve on accept or cancel.
   *
   * If the user is logged out (401 from /api/auth/me), the accept-terms
   * modal would only frustrate them — the POST to /accept-terms would
   * return 401 with "Not authenticated". So we pop the sign-in / sign-up
   * modal instead and let them authenticate first.
   */
  function showModal() {
    return _fetchMe(true).then(function (me) {
      if (!me || me._httpStatus === 401 || me._httpStatus === 403) {
        return _showSigninModal();
      }
      var node = _ensureModal();
      _setVersion(me && me.current_terms_version);

      // Reset state.
      var box = node.querySelector('#terms-accept');
      var confirm = node.querySelector('#' + MODAL_ID + '-confirm');
      var cancel = node.querySelector('#' + MODAL_ID + '-cancel');
      var errBox = node.querySelector('#' + MODAL_ID + '-error');
      var spinner = node.querySelector('#' + MODAL_ID + '-confirm-spinner');
      box.checked = false;
      confirm.disabled = true;
      errBox.classList.add('d-none');
      spinner.classList.add('d-none');

      var modal = null;
      if (window.bootstrap && window.bootstrap.Modal) {
        modal = window.bootstrap.Modal.getOrCreateInstance(node);
        modal.show();
      } else {
        // Bootstrap not loaded — fall back to .show class.
        node.classList.add('show');
        node.style.display = 'block';
      }

      return new Promise(function (resolve) {
        function cleanup(result) {
          confirm.removeEventListener('click', onConfirm);
          cancel.removeEventListener('click', onCancel);
          if (modal) {
            modal.hide();
          } else {
            node.classList.remove('show');
            node.style.display = 'none';
          }
          resolve(result);
        }

        function onConfirm() {
          if (!box.checked) return;
          confirm.disabled = true;
          spinner.classList.remove('d-none');
          errBox.classList.add('d-none');

          var version = (me && me.current_terms_version) || '1.0';
          _apiFetch(PROJECTS_API + '/api/users/me/accept-terms', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ version: version }),
          })
            .then(function (r) {
              spinner.classList.add('d-none');
              if (r.ok) {
                // Bust the /me cache so the next check() reflects the
                // acceptance.
                _meCache = null;
                cleanup(true);
                return;
              }
              return r.json().catch(function () { return {}; })
                .then(function (body) {
                  var msg = body && (body.detail || body.message) ||
                            ('Accept failed (' + r.status + ')');
                  if (typeof msg !== 'string') msg = JSON.stringify(msg);
                  _showError(msg);
                  confirm.disabled = false;
                });
            })
            .catch(function () {
              spinner.classList.add('d-none');
              _showError('Network error — please try again.');
              confirm.disabled = false;
            });
        }

        function onCancel() {
          cleanup(false);
        }

        confirm.addEventListener('click', onConfirm);
        cancel.addEventListener('click', onCancel);
      });
    });
  }

  /**
   * Wrap an upload action. If accepted, calls fn() directly. Otherwise
   * pops the modal first and only calls fn() on accept.
   */
  function guard(fn) {
    return check().then(function (accepted) {
      if (accepted) return fn();
      return showModal().then(function (ok) {
        if (ok) return fn();
        return undefined;
      });
    });
  }

  /**
   * Inspect a response that came back from a write endpoint and, if it's
   * a 403 ``terms_not_accepted``, pop the modal. Returns a Promise that
   * resolves to ``true`` if the user accepted and the caller should retry,
   * ``false`` otherwise. Pages without this helper just see a generic 403.
   */
  function handleResponse(response) {
    if (!response || response.status !== 403) return Promise.resolve(false);
    return response
      .clone()
      .json()
      .catch(function () { return null; })
      .then(function (body) {
        var detail = body && body.detail;
        if (!detail || typeof detail !== 'object') return false;
        if (detail.detail !== 'terms_not_accepted') return false;
        return showModal();
      });
  }

  window.TermsGate = {
    check: check,
    showModal: showModal,
    guard: guard,
    handleResponse: handleResponse,
    _clearCache: function () { _meCache = null; },
  };
})();
