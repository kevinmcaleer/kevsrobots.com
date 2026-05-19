/**
 * Badge toast notification helper — issue #106.
 *
 * Pages that mutate state which can award badges (e.g. project create)
 * receive a `newly_awarded_badges` array on the response body. They call
 * BadgeToast.show(badges) and we render a stack of dismissible toasts in
 * the bottom-right corner.
 *
 * The helper is intentionally tiny and vanilla — no Bootstrap-Toast
 * dependency — because the toast container is its own .kr-badge-toast
 * scope in main.scss and we want to avoid pulling Bootstrap's JS toast
 * collection on every page that might award a badge.
 *
 * Public API:
 *   BadgeToast.show([{slug, name, description, icon, tier}, ...])
 *   BadgeToast.fromResponse(responseBody)  // convenience for fetch().json()
 */
(function () {
  'use strict';

  var CONTAINER_ID = 'kr-badge-toast-container';
  var TOAST_LIFESPAN_MS = 6000;

  function ensureContainer() {
    var el = document.getElementById(CONTAINER_ID);
    if (el) return el;
    el = document.createElement('div');
    el.id = CONTAINER_ID;
    document.body.appendChild(el);
    return el;
  }

  function escapeHtml(text) {
    var div = document.createElement('div');
    div.textContent = text == null ? '' : String(text);
    return div.innerHTML;
  }

  function renderToast(badge) {
    var tier = badge.tier || 'single';
    var node = document.createElement('div');
    node.className = 'kr-badge-toast';
    node.setAttribute('role', 'status');
    node.innerHTML =
      '<span class="kr-badge kr-badge--' + tier + ' kr-badge--md">' +
        '<i class="' + escapeHtml(badge.icon || 'fa-solid fa-trophy') + '"></i>' +
      '</span>' +
      '<div class="kr-badge-toast__body">' +
        '<div class="kr-badge-toast__title">You earned a new badge!</div>' +
        '<div class="kr-badge-toast__desc">' +
          '<strong>' + escapeHtml(badge.name) + '</strong>' +
          (badge.description ? ' — ' + escapeHtml(badge.description) : '') +
        '</div>' +
      '</div>';

    // Click to dismiss — gives the user manual control if the toast
    // hangs around while they read it.
    node.addEventListener('click', function () {
      remove(node);
    });

    return node;
  }

  function remove(node) {
    if (!node || !node.parentNode) return;
    node.style.transition = 'opacity 250ms ease-out, transform 250ms ease-out';
    node.style.opacity = '0';
    node.style.transform = 'translateY(8px)';
    setTimeout(function () {
      if (node.parentNode) node.parentNode.removeChild(node);
    }, 260);
  }

  function show(badges) {
    if (!badges || !badges.length) return;
    var container = ensureContainer();
    badges.forEach(function (b) {
      var node = renderToast(b);
      container.appendChild(node);
      setTimeout(function () { remove(node); }, TOAST_LIFESPAN_MS);
    });
  }

  function fromResponse(responseBody) {
    if (!responseBody) return;
    // Server may return the field at the top level (project / make create)
    // or nested under .newly_awarded (POST /api/badges/evaluate/...).
    var newly = responseBody.newly_awarded_badges || responseBody.newly_awarded;
    show(newly || []);
  }

  window.BadgeToast = { show: show, fromResponse: fromResponse };
})();
