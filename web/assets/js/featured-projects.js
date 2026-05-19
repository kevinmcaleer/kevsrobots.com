/* Featured projects + staff picks UI helpers (issue #115).
 *
 * Exposes a small namespace ``FeaturedProjects`` with:
 *   - DEFAULT_API: the projects-api base URL the rest of the projects UI
 *     already points at.
 *   - ribbon(): returns the gold-ribbon HTML fragment for use inside an
 *     image wrapper.
 *   - badge(note): returns the inline "Staff Pick" badge for the view
 *     page header. Encodes ``note`` for the tooltip safely.
 *   - mountCarousel(container, projects): renders a scrollable shelf of
 *     featured cards into a container. Returns true when at least one
 *     card was rendered so the caller can `classList.remove('d-none')`
 *     conditionally — we want the section hidden entirely on a fresh
 *     install with no featured projects.
 *   - load(limit): fetches /api/projects/featured?limit=N as JSON.
 *
 * Keep this dependency-free so the hub can include it before the larger
 * project-search bundle without a forced load order.
 */
(function () {
  'use strict';

  var DEFAULT_API = 'https://projects.kevsrobots.com';

  function esc(text) {
    var d = document.createElement('div');
    d.textContent = text == null ? '' : String(text);
    return d.innerHTML;
  }

  function ribbon(label) {
    var text = label || 'Featured';
    return (
      '<span class="featured-ribbon" aria-label="' + esc(text) + ' project">' +
        '<i class="fas fa-star" aria-hidden="true"></i>' + esc(text) +
      '</span>'
    );
  }

  function badge(note) {
    // Tooltip via Bootstrap's data-bs-toggle. The title attribute is
    // escaped so a malicious note can't break out of the HTML attribute.
    var tooltip = note ? esc(note) : 'Hand-picked by the kevsrobots team';
    return (
      '<span class="staff-pick-badge" ' +
        'data-bs-toggle="tooltip" ' +
        'data-bs-placement="bottom" ' +
        'title="' + tooltip + '">' +
        '<i class="fas fa-star" aria-hidden="true"></i>' +
        'Staff Pick' +
      '</span>'
    );
  }

  function cardThumbStyle(project) {
    if (project && project.cover_image) {
      return 'background-image:url(\'' + esc(project.cover_image) + '\');background-size:cover;background-position:center;';
    }
    // Cheap deterministic gradient based on the project id so cards
    // without a cover image still feel distinct.
    var id = (project && project.id) || 0;
    var hue = (id * 53) % 360;
    return 'background:linear-gradient(135deg,hsl(' + hue + ',70%,55%),hsl(' + ((hue + 60) % 360) + ',70%,45%));';
  }

  function carouselCardHtml(project) {
    // Issue #152: prefer canonical /projects/<owner>/<slug> URL when the
    // FeaturedProjectResponse surfaced a slug.
    var href = (project.slug && project.author_username)
      ? '/projects/' + encodeURIComponent(project.author_username) + '/' + encodeURIComponent(project.slug)
      : '/projects/view.html?id=' + encodeURIComponent(project.id);
    var note = project.featured_note ? '<div class="featured-note">' + esc(project.featured_note) + '</div>' : '';
    var author = project.author_username ? esc(project.author_username) : '';
    return (
      '<div class="featured-carousel-card">' +
        '<a href="' + href + '" class="text-decoration-none">' +
          '<div class="card border-0 shadow-sm card-hover">' +
            ribbon('Featured') +
            '<div class="project-thumb" style="' + cardThumbStyle(project) + '"></div>' +
            '<div class="card-body">' +
              '<div class="fw-bold text-dark text-truncate" title="' + esc(project.title) + '">' + esc(project.title) + '</div>' +
              (author ? '<div class="small text-muted">by ' + author + '</div>' : '') +
              note +
            '</div>' +
          '</div>' +
        '</a>' +
      '</div>'
    );
  }

  function mountCarousel(container, projects) {
    if (!container) return false;
    if (!projects || !projects.length) {
      container.innerHTML = '';
      return false;
    }
    container.innerHTML = projects.map(carouselCardHtml).join('');
    // Activate tooltips inside the rendered cards (best-effort — bootstrap
    // may not be loaded yet on some pages).
    if (window.bootstrap && window.bootstrap.Tooltip) {
      container.querySelectorAll('[data-bs-toggle="tooltip"]').forEach(function (el) {
        new window.bootstrap.Tooltip(el);
      });
    }
    return true;
  }

  function load(opts) {
    opts = opts || {};
    var api = opts.api || DEFAULT_API;
    var limit = opts.limit || 6;
    return fetch(api + '/api/projects/featured?limit=' + encodeURIComponent(limit))
      .then(function (resp) {
        if (!resp.ok) throw new Error('Featured fetch failed: ' + resp.status);
        return resp.json();
      });
  }

  window.FeaturedProjects = {
    DEFAULT_API: DEFAULT_API,
    ribbon: ribbon,
    badge: badge,
    mountCarousel: mountCarousel,
    load: load
  };
})();
