---
title: Projects Hub
description: Discover amazing maker projects from the community
excerpt: Browse, search, and download complete project packages with instructions, code, and files
layout: content
hide_likes_and_comments: true
thanks: false
---

{% include nav_projects.html %}



# {{page.title}}
## {{page.description}}

{{page.excerpt}}

---

<link rel="stylesheet" href="/assets/css/featured-projects.css?v={{ site.time | date: '%s' }}">

<!-- Project Gallery Component -->
<div id="projects-hub">
  <!-- Featured projects carousel (issue #115). Hidden by default; the
       loader removes d-none once it has at least one featured project to
       render — on a fresh install the whole shelf stays out of the DOM
       flow. -->
  <div id="featured-section" class="featured-carousel-wrap d-none">
    <div class="d-flex align-items-center justify-content-between mb-2">
      <h4 class="mb-0"><i class="fas fa-star text-warning me-3"></i> Featured Projects</h4>
      <a href="/projects/staff-picks/" class="small text-decoration-none">Browse Staff Picks <i class="fas fa-arrow-right"></i></a>
    </div>
    <div id="featured-carousel" class="featured-carousel"></div>
  </div>

  <!-- Popular this month -->
  <div id="popular-section" class="mb-4 d-none">
    <h4 class="mb-3"><i class="fas fa-fire text-danger me-2"></i>Popular this month</h4>
    <div id="popular-row" class="row row-cols-2 row-cols-md-3 row-cols-lg-6 g-3"></div>
  </div>

  <!-- Header with Create Button -->
  <div class="row mb-4">
    <div class="col">
      <div class="d-flex justify-content-between align-items-center">
        <h4 class="mb-0">Browse Projects</h4>
        <div>
          <button id="my-projects-btn" class="btn btn-outline-primary me-2 d-none" onclick="toggleMyProjects()">
            <i class="fas fa-folder"></i> My Projects
          </button>
          <a href="/projects/editor.html" class="btn btn-primary">
            <i class="fas fa-plus-circle"></i> Create New Project
          </a>
        </div>
      </div>
    </div>
  </div>

  <!-- Search and Filters -->
  <div class="row mb-4 g-2">
    <div class="col-md-4">
      <input type="text" id="project-search" class="form-control" placeholder="Search projects (opens full search)...">
    </div>
    <div class="col-md-3">
      <select id="difficulty-filter" class="form-select">
        <option value="">All Difficulties</option>
        <option value="beginner">Beginner</option>
        <option value="intermediate">Intermediate</option>
        <option value="advanced">Advanced</option>
      </select>
    </div>
    <div class="col-md-2">
      <input type="text" id="tag-filter" class="form-control" placeholder="Filter by tag...">
    </div>
    <div class="col-md-3">
      <select id="sort-select" class="form-select" aria-label="Sort projects">
        <option value="newest">Newest first</option>
        <option value="downloads_30d">Most downloaded (30d)</option>
      </select>
    </div>
  </div>

  <!-- Loading Spinner -->
  <div id="loading-spinner" class="text-center my-5">
    <div class="spinner-border text-primary" role="status">
      <span class="visually-hidden">Loading projects...</span>
    </div>
  </div>

  <!-- Projects Grid -->
  <div id="projects-grid" class="row row-cols-1 row-cols-sm-2 row-cols-lg-3 row-cols-xl-4 g-4">
  </div>

  <!-- No Projects -->
  <div id="no-projects" class="text-center py-5 d-none">
    <i class="fas fa-folder-open fa-3x text-muted mb-3"></i>
    <h4 class="text-muted">No projects yet</h4>
    <p class="text-muted">Be the first to share a build!</p>
    <a href="/projects/editor.html" class="btn btn-primary">Create a Project</a>
  </div>
</div>

<script src="/assets/js/project-gradient.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/project-auth.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/project-interactions.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/project-search.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/featured-projects.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/badge-toast.js?v={{ site.time | date: '%s' }}"></script>
<script>
(function() {
  const API = 'https://projects.kevsrobots.com';
  const grid = document.getElementById('projects-grid');
  const spinner = document.getElementById('loading-spinner');
  const noProjects = document.getElementById('no-projects');
  const searchInput = document.getElementById('project-search');
  const difficultyFilter = document.getElementById('difficulty-filter');
  const tagFilter = document.getElementById('tag-filter');
  const sortSelect = document.getElementById('sort-select');

  const difficultyColors = ProjectSearch.DIFFICULTY_COLORS;
  const statusBadges = ProjectSearch.STATUS_BADGES;

  let myProjectIds = new Set();
  let showingMine = false;

  // Redirect to the full search page with the user's criteria attached.
  // Issue #133: "search results page should appear once the user types any
  // criteria on the main projects hub page".
  function redirectToSearch() {
    const params = new URLSearchParams();
    if (searchInput.value.trim()) params.set('q', searchInput.value.trim());
    if (difficultyFilter.value) params.append('difficulty', difficultyFilter.value);
    if (tagFilter.value.trim()) params.append('tag', tagFilter.value.trim().toLowerCase());
    const qs = params.toString();
    window.location.href = '/projects/search' + (qs ? '?' + qs : '');
  }

  async function checkAuth() {
    try {
      const resp = await ProjectAuth.apiFetch(API + '/api/projects/my/list');
      if (resp.ok) {
        document.getElementById('my-projects-btn').classList.remove('d-none');
        const myProjects = await resp.json();
        myProjectIds = new Set(myProjects.map(p => p.id));
      }
    } catch (e) {}
  }

  window.toggleMyProjects = function() {
    showingMine = !showingMine;
    const btn = document.getElementById('my-projects-btn');
    if (showingMine) {
      btn.classList.remove('btn-outline-primary');
      btn.classList.add('btn-primary');
    } else {
      btn.classList.remove('btn-primary');
      btn.classList.add('btn-outline-primary');
    }
    loadProjects();
  };

  async function loadProjects() {
    spinner.classList.remove('d-none');
    noProjects.classList.add('d-none');
    grid.innerHTML = '';

    const sortMode = (sortSelect && sortSelect.value) || 'newest';
    let resp;
    try {
      if (sortMode === 'downloads_30d') {
        // Use the popular endpoint for ordering — it already excludes
        // archived projects and returns download_count baked in.
        resp = await fetch(API + '/api/projects/popular?window=30d&limit=100');
      } else {
        // Issue #140: ask the API to boost followed-author projects for
        // logged-in viewers. The flag is harmless for anonymous calls.
        resp = await ProjectAuth.apiFetch(API + '/api/projects?boost_followed=true');
      }
      if (!resp.ok) throw new Error('API error');
      let projects = await resp.json();
      spinner.classList.add('d-none');

      // Issue #133: rank newest-first so that completed projects come first,
      // then by created_at desc, then by updated_at desc as a tiebreaker.
      // The popular endpoint keeps its own ordering.
      if (sortMode !== 'downloads_30d') {
        projects = ProjectSearch.rankHubProjects(projects);
      }

      if (showingMine) {
        projects = projects.filter(p => myProjectIds.has(p.id));
      }

      if (projects.length === 0) {
        noProjects.classList.remove('d-none');
        return;
      }

      // Issue #152: prefer canonical /projects/<owner>/<slug> when the
      // API surfaced a slug; fall back to ?id= for back-compat with any
      // legacy ProjectListItem rows that haven't been backfilled.
      function projectViewUrl(p) {
        if (p && p.slug && p.author_username) {
          return '/projects/' + encodeURIComponent(p.author_username) +
                 '/' + encodeURIComponent(p.slug);
        }
        return '/projects/view.html?id=' + p.id;
      }
      grid.innerHTML = projects.map(p => `
        <div class="col">
          <a href="${myProjectIds.has(p.id) ? '/projects/editor.html?id=' + p.id : projectViewUrl(p)}" class="text-decoration-none">
            <div class="card kr-project-card h-100 border-0 shadow-sm card-hover position-relative">
              ${p.is_featured ? FeaturedProjects.ribbon('Featured') : ''}
              ${projectThumbnail(p)}
              <div class="card-body">
                <h5 class="card-title text-dark">
                  ${p.is_remix ? '<i class="fas fa-code-branch text-muted me-1" title="Remix of another project" data-bs-toggle="tooltip"></i>' : ''}${esc(p.title)}
                </h5>
                <p class="card-text text-muted small kr-card-description">${esc(p.short_description || '')}</p>
                <div class="d-flex flex-wrap gap-1 mb-2">
                  ${statusBadges[p.status] || ''}
                  ${p.difficulty ? `<span class="badge bg-${difficultyColors[p.difficulty] || 'secondary'}">${p.difficulty}</span>` : ''}
                  ${p.estimated_minutes ? `<span class="badge bg-light text-dark"><i class="fas fa-clock"></i> ${p.estimated_minutes}min</span>` : ''}
                  ${p.is_remix ? '<span class="badge bg-light text-muted"><i class="fas fa-code-branch"></i> Remix</span>' : ''}
                </div>
                <div class="d-flex flex-wrap gap-1 kr-card-tags">
                  ${(p.tags || []).slice(0, 4).map(t => `<span class="badge bg-light text-primary">${esc(t)}</span>`).join('')}
                </div>
              </div>
              <div class="card-footer bg-transparent border-0 d-flex justify-content-between align-items-center">
                <small class="text-muted">by <span data-profile-username="${esc(p.author_username)}" class="profile-username-link">${esc(p.author_username)}</span><span class="ms-1 d-none" data-author-gold="${esc(p.author_username)}"></span> &middot; ${new Date(p.created_at).toLocaleDateString()}</small>
                <small class="text-muted d-flex gap-2 align-items-center">
                  <span id="card-makes-${p.id}" class="d-none"><i class="fas fa-hammer"></i> <span data-count></span></span>
                  <span id="card-likes-${p.id}"><i class="far fa-heart"></i> </span>
                  <span title="Total downloads"><i class="fas fa-download"></i> ${(p.download_count || 0)}</span>
                </small>
              </div>
            </div>
          </a>
        </div>
      `).join('');

      // Fetch like counts for visible cards
      projects.forEach(function (p) {
        var url = 'projects/view.html?id=' + p.id;
        ProjectInteractions.getLikeCount(url).then(function (data) {
          var el = document.getElementById('card-likes-' + p.id);
          if (el && data.count > 0) {
            el.innerHTML = '<i class="far fa-heart"></i> ' + data.count;
          }
        });
      });

      // Fetch makes counts for visible cards (issue #107). Best-effort —
      // failures just leave the badge hidden.
      projects.forEach(function (p) {
        fetch(API + '/api/projects/' + p.id + '/makes')
          .then(function (r) { return r.ok ? r.json() : []; })
          .then(function (makes) {
            if (!makes || makes.length === 0) return;
            var wrap = document.getElementById('card-makes-' + p.id);
            if (!wrap) return;
            wrap.querySelector('[data-count]').textContent = makes.length;
            wrap.classList.remove('d-none');
          })
          .catch(function () {});
      });

      // Issue #111: hijack clicks on author-username spans so they
      // navigate to the user's profile instead of the project. Implemented
      // as a click handler rather than a nested <a> because the card is
      // already wrapped in one (invalid HTML).
      grid.querySelectorAll('.profile-username-link').forEach(function (el) {
        el.style.cursor = 'pointer';
        el.classList.add('text-decoration-underline');
        el.addEventListener('click', function (ev) {
          ev.preventDefault();
          ev.stopPropagation();
          var u = el.getAttribute('data-profile-username') || '';
          if (u) window.location.href = '/profile/?u=' + encodeURIComponent(u);
        });
      });

      // Issue #106: project card adornment — when an author has any
      // gold-tier badge, show a small trophy icon next to their name.
      // One fetch per unique author, cached for the page lifetime.
      var uniqueAuthors = Array.from(new Set(projects.map(function (p) { return p.author_username; })));
      uniqueAuthors.forEach(function (author) {
        fetch(API + '/api/users/' + encodeURIComponent(author) + '/badges')
          .then(function (r) { return r.ok ? r.json() : []; })
          .then(function (badges) {
            var goldBadge = (badges || []).find(function (b) { return b.tier === 'gold'; });
            if (!goldBadge) return;
            var slots = document.querySelectorAll('[data-author-gold="' + author.replace(/"/g, '\\"') + '"]');
            slots.forEach(function (slot) {
              slot.innerHTML = '<i class="fa-solid fa-trophy text-warning" title="' + esc(goldBadge.name) + '" data-bs-toggle="tooltip"></i>';
              slot.classList.remove('d-none');
            });
          })
          .catch(function () {});
      });

    } catch (e) {
      console.error('Failed to load projects:', e);
      spinner.classList.add('d-none');
      grid.innerHTML = '<div class="text-center py-5 w-100"><p class="text-muted">Failed to load projects. Please try again later.</p></div>';
    }
  }

  function esc(text) {
    const div = document.createElement('div');
    div.textContent = text || '';
    return div.innerHTML;
  }

  // Typing into the hub's search/tag boxes (or picking a difficulty) jumps
  // to the dedicated search page with the criteria carried in the URL.
  // Sort stays on the hub since it isn't a "criteria".
  let debounce;
  function debouncedRedirect() {
    clearTimeout(debounce);
    debounce = setTimeout(redirectToSearch, 400);
  }

  searchInput.addEventListener('input', debouncedRedirect);
  difficultyFilter.addEventListener('change', redirectToSearch);
  tagFilter.addEventListener('input', debouncedRedirect);
  if (sortSelect) sortSelect.addEventListener('change', loadProjects);

  // Featured carousel (issue #115). One extra API call to
  // /api/projects/featured. Fails closed: any error / empty list leaves
  // the section hidden so the hub looks identical to today on a fresh
  // install with no featured projects yet.
  async function loadFeatured() {
    try {
      const items = await FeaturedProjects.load({ api: API, limit: 6 });
      if (!Array.isArray(items) || items.length === 0) return;
      const rendered = FeaturedProjects.mountCarousel(
        document.getElementById('featured-carousel'),
        items
      );
      if (rendered) {
        document.getElementById('featured-section').classList.remove('d-none');
      }
    } catch (e) {
      // Silent — the carousel is non-critical.
    }
  }

  // Fail-closed loader for the "Popular this month" row at the top.
  // If the API errors or returns nothing, the section stays hidden — no UX
  // damage on a fresh install.
  async function loadPopular() {
    try {
      const resp = await fetch(API + '/api/projects/popular?window=30d&limit=6');
      if (!resp.ok) return;
      const items = await resp.json();
      if (!Array.isArray(items) || items.length === 0) return;
      const row = document.getElementById('popular-row');
      // Issue #152: emit canonical project URL when slug is available.
      function popularUrl(p) {
        if (p && p.slug && p.author_username) {
          return '/projects/' + encodeURIComponent(p.author_username) +
                 '/' + encodeURIComponent(p.slug);
        }
        return '/projects/view.html?id=' + p.id;
      }
      row.innerHTML = items.map(p => `
        <div class="col">
          <a href="${popularUrl(p)}" class="text-decoration-none">
            <div class="card kr-project-card h-100 border-0 shadow-sm card-hover">
              ${projectThumbnail(p)}
              <div class="card-body p-2">
                <div class="small fw-bold text-dark text-truncate" title="${esc(p.title)}">${esc(p.title)}</div>
                <div class="small text-muted">
                  <i class="fas fa-download"></i> ${(p.download_count || 0)}
                </div>
              </div>
            </div>
          </a>
        </div>
      `).join('');
      document.getElementById('popular-section').classList.remove('d-none');
    } catch (e) {
      // Silent — popular row is non-critical.
    }
  }

  checkAuth().then(() => {
    loadProjects();
    loadPopular();
    loadFeatured();
  });
})();
</script>
