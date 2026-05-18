---
title: Search Projects
description: Filter community projects by difficulty, status, parts, tags, author and date
excerpt: Narrow down to exactly the projects you want to see
layout: content
hide_likes_and_comments: true
thanks: false
permalink: /projects/search
---

{% include nav_projects.html %}

# {{page.title}}
## {{page.description}}

{{page.excerpt}}

---

<div id="projects-search">
  <div class="row g-4">
    <!-- Filters column -->
    <aside class="col-lg-3">
      <div class="card shadow-sm">
        <div class="card-body">
          <div class="d-flex justify-content-between align-items-center mb-3">
            <h5 class="card-title mb-0"><i class="fas fa-filter me-1"></i> Filters</h5>
            <button id="clear-filters" type="button" class="btn btn-sm btn-link p-0">Clear</button>
          </div>

          <div class="mb-3">
            <label for="search-q" class="form-label small text-muted">Search</label>
            <input type="search" id="search-q" class="form-control form-control-sm" placeholder="Title or description...">
          </div>

          <div class="mb-3">
            <div class="form-label small text-muted mb-1">Difficulty</div>
            <div id="filter-difficulty" class="d-flex flex-column gap-1"></div>
          </div>

          <div class="mb-3">
            <div class="form-label small text-muted mb-1">Status</div>
            <div id="filter-status" class="d-flex flex-column gap-1"></div>
          </div>

          <div class="mb-3">
            <div class="d-flex justify-content-between align-items-center">
              <div class="form-label small text-muted mb-1">Parts used</div>
              <small class="text-muted">top 10</small>
            </div>
            <div id="filter-parts" class="d-flex flex-column gap-1">
              <div class="text-muted small">Loading...</div>
            </div>
          </div>

          <div class="mb-3">
            <div class="d-flex justify-content-between align-items-center">
              <div class="form-label small text-muted mb-1">Tags</div>
              <small class="text-muted">top 10</small>
            </div>
            <div id="filter-tags" class="d-flex flex-column gap-1">
              <div class="text-muted small">Loading...</div>
            </div>
          </div>

          <div class="mb-3">
            <label for="filter-author" class="form-label small text-muted">Author</label>
            <input type="text" id="filter-author" class="form-control form-control-sm" placeholder="Username">
          </div>

          <div class="mb-3">
            <div class="form-label small text-muted mb-1">Date created</div>
            <div class="row g-1">
              <div class="col-6">
                <input type="date" id="filter-created-from" class="form-control form-control-sm" aria-label="Created from">
              </div>
              <div class="col-6">
                <input type="date" id="filter-created-to" class="form-control form-control-sm" aria-label="Created to">
              </div>
            </div>
          </div>

          <div class="mb-0">
            <div class="form-label small text-muted mb-1">Date last updated</div>
            <div class="row g-1">
              <div class="col-6">
                <input type="date" id="filter-updated-from" class="form-control form-control-sm" aria-label="Updated from">
              </div>
              <div class="col-6">
                <input type="date" id="filter-updated-to" class="form-control form-control-sm" aria-label="Updated to">
              </div>
            </div>
          </div>
        </div>
      </div>
    </aside>

    <!-- Results column -->
    <section class="col-lg-9">
      <div class="d-flex justify-content-between align-items-center mb-3 flex-wrap gap-2">
        <div id="results-count" class="text-muted small">Loading projects...</div>
        <a href="/projects/hub" class="btn btn-sm btn-outline-secondary">
          <i class="fas fa-arrow-left me-1"></i> Back to hub
        </a>
      </div>

      <div id="loading-spinner" class="text-center my-5">
        <div class="spinner-border text-primary" role="status">
          <span class="visually-hidden">Loading projects...</span>
        </div>
      </div>

      <div id="results-grid" class="row row-cols-1 row-cols-sm-2 row-cols-lg-2 row-cols-xl-3 g-4"></div>

      <div id="no-results" class="text-center py-5 d-none">
        <i class="fas fa-folder-open fa-3x text-muted mb-3"></i>
        <h4 class="text-muted">No projects match those filters</h4>
        <p class="text-muted">Try clearing some criteria.</p>
      </div>
    </section>
  </div>
</div>

<script src="/assets/js/project-gradient.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/project-auth.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/project-interactions.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/project-search.js?v={{ site.time | date: '%s' }}"></script>
<script>
(function () {
  const API = 'https://projects.kevsrobots.com';
  const esc = ProjectSearch.esc;

  const DIFFICULTY_OPTIONS = [
    { value: 'beginner', label: 'Beginner' },
    { value: 'intermediate', label: 'Intermediate' },
    { value: 'advanced', label: 'Advanced' }
  ];
  const STATUS_OPTIONS = [
    { value: 'wip', label: '🔨 Work in progress' },
    { value: 'completed', label: '✅ Completed' }
  ];

  // ─── DOM refs ───────────────────────────────────────────────
  const grid = document.getElementById('results-grid');
  const spinner = document.getElementById('loading-spinner');
  const noResults = document.getElementById('no-results');
  const countEl = document.getElementById('results-count');
  const searchInput = document.getElementById('search-q');
  const authorInput = document.getElementById('filter-author');
  const createdFrom = document.getElementById('filter-created-from');
  const createdTo = document.getElementById('filter-created-to');
  const updatedFrom = document.getElementById('filter-updated-from');
  const updatedTo = document.getElementById('filter-updated-to');
  const clearBtn = document.getElementById('clear-filters');

  // ─── State ──────────────────────────────────────────────────
  let allProjects = [];     // full set fetched from API (unfiltered)
  let myProjectIds = new Set();
  const bomCache = new Map(); // project id -> Set<part_slug>
  let bomFetchPromise = null;

  // ─── URL state ──────────────────────────────────────────────
  function readParams() {
    const p = new URLSearchParams(window.location.search);
    return {
      q: p.get('q') || '',
      difficulty: p.getAll('difficulty'),
      status: p.getAll('status'),
      part: p.getAll('part'),
      tag: p.getAll('tag'),
      author: p.get('author') || '',
      created_from: p.get('created_from') || '',
      created_to: p.get('created_to') || '',
      updated_from: p.get('updated_from') || '',
      updated_to: p.get('updated_to') || ''
    };
  }

  function getCheckedValues(containerId) {
    return Array.from(document.querySelectorAll('#' + containerId + ' input[type=checkbox]:checked'))
      .map(function (el) { return el.value; });
  }

  function currentFilters() {
    return {
      q: searchInput.value.trim(),
      difficulty: getCheckedValues('filter-difficulty'),
      status: getCheckedValues('filter-status'),
      part: getCheckedValues('filter-parts'),
      tag: getCheckedValues('filter-tags'),
      author: authorInput.value.trim(),
      created_from: createdFrom.value,
      created_to: createdTo.value,
      updated_from: updatedFrom.value,
      updated_to: updatedTo.value
    };
  }

  function writeParams(filters) {
    const p = new URLSearchParams();
    if (filters.q) p.set('q', filters.q);
    filters.difficulty.forEach(function (v) { p.append('difficulty', v); });
    filters.status.forEach(function (v) { p.append('status', v); });
    filters.part.forEach(function (v) { p.append('part', v); });
    filters.tag.forEach(function (v) { p.append('tag', v); });
    if (filters.author) p.set('author', filters.author);
    if (filters.created_from) p.set('created_from', filters.created_from);
    if (filters.created_to) p.set('created_to', filters.created_to);
    if (filters.updated_from) p.set('updated_from', filters.updated_from);
    if (filters.updated_to) p.set('updated_to', filters.updated_to);
    const qs = p.toString();
    const url = window.location.pathname + (qs ? '?' + qs : '');
    window.history.replaceState({}, '', url);
  }

  // ─── Filter facet builders ──────────────────────────────────
  function renderCheckboxList(containerId, options, checkedValues) {
    const container = document.getElementById(containerId);
    if (options.length === 0) {
      container.innerHTML = '<div class="text-muted small">No options</div>';
      return;
    }
    const checked = new Set(checkedValues || []);
    container.innerHTML = options.map(function (opt) {
      const id = containerId + '-' + opt.value.replace(/[^a-z0-9]+/gi, '-');
      const meta = (opt.count != null)
        ? ' <span class="text-muted small">(' + opt.count + ')</span>'
        : '';
      return (
        '<div class="form-check">' +
          '<input class="form-check-input" type="checkbox" id="' + id + '" value="' + esc(opt.value) + '"' +
            (checked.has(opt.value) ? ' checked' : '') + '>' +
          '<label class="form-check-label small" for="' + id + '">' + esc(opt.label) + meta + '</label>' +
        '</div>'
      );
    }).join('');
    container.querySelectorAll('input[type=checkbox]').forEach(function (cb) {
      cb.addEventListener('change', onFilterChange);
    });
  }

  function computeTopTags(projects, limit) {
    const counts = new Map();
    projects.forEach(function (p) {
      (p.tags || []).forEach(function (t) {
        if (!t) return;
        const key = String(t).toLowerCase();
        counts.set(key, (counts.get(key) || 0) + 1);
      });
    });
    const entries = Array.from(counts.entries());
    entries.sort(function (a, b) { return b[1] - a[1] || a[0].localeCompare(b[0]); });
    return entries.slice(0, limit).map(function (e) {
      return { value: e[0], label: e[0], count: e[1] };
    });
  }

  // ─── Parts ──────────────────────────────────────────────────
  // We can't infer parts from project records alone — they live in the BOM
  // endpoint. The parts catalogue exposes `usage_count`, so we use that to
  // pick the top 10 by overall popularity. Matching projects to parts then
  // needs each project's BOM, fetched on demand.
  async function loadTopParts() {
    try {
      const resp = await fetch(API + '/api/parts?q=&limit=50');
      if (!resp.ok) throw new Error('parts api');
      const parts = await resp.json();
      const usable = (Array.isArray(parts) ? parts : [])
        .filter(function (p) { return p && p.slug && typeof p.usage_count === 'number' && p.usage_count > 0; })
        .sort(function (a, b) { return (b.usage_count || 0) - (a.usage_count || 0); })
        .slice(0, 10)
        .map(function (p) { return { value: p.slug, label: p.name || p.slug, count: p.usage_count }; });
      const checked = readParams().part;
      renderCheckboxList('filter-parts', usable, checked);
      if (usable.length === 0) {
        document.getElementById('filter-parts').innerHTML =
          '<div class="text-muted small">No parts data available</div>';
      }
    } catch (e) {
      document.getElementById('filter-parts').innerHTML =
        '<div class="text-muted small">Parts catalogue unavailable</div>';
    }
  }

  async function ensureBomFor(projectIds) {
    // Fetch BOMs for any ids we haven't cached yet. Returns when all done.
    const toFetch = projectIds.filter(function (id) { return !bomCache.has(id); });
    if (toFetch.length === 0) return;
    await Promise.all(toFetch.map(function (id) {
      return fetch(API + '/api/projects/' + id + '/bom')
        .then(function (r) { return r.ok ? r.json() : []; })
        .then(function (items) {
          const slugs = new Set();
          (items || []).forEach(function (i) {
            if (i && i.part_slug) slugs.add(i.part_slug);
          });
          bomCache.set(id, slugs);
        })
        .catch(function () { bomCache.set(id, new Set()); });
    }));
  }

  // ─── Filtering ──────────────────────────────────────────────
  function matchesNonPart(p, f) {
    if (f.q) {
      const q = f.q.toLowerCase();
      const hay = ((p.title || '') + ' ' + (p.short_description || '')).toLowerCase();
      if (hay.indexOf(q) === -1) return false;
    }
    if (f.difficulty.length && f.difficulty.indexOf(p.difficulty) === -1) return false;
    if (f.status.length && f.status.indexOf(p.status) === -1) return false;
    if (f.tag.length) {
      const pTags = (p.tags || []).map(function (t) { return String(t).toLowerCase(); });
      const wantAny = f.tag.some(function (t) { return pTags.indexOf(t) !== -1; });
      if (!wantAny) return false;
    }
    if (f.author) {
      const a = (p.author_username || '').toLowerCase();
      if (a.indexOf(f.author.toLowerCase()) === -1) return false;
    }
    const created = p.created_at ? new Date(p.created_at).getTime() : 0;
    if (f.created_from && created < new Date(f.created_from).getTime()) return false;
    if (f.created_to) {
      // inclusive end-of-day
      const to = new Date(f.created_to).getTime() + 86400000 - 1;
      if (created > to) return false;
    }
    const updated = p.updated_at ? new Date(p.updated_at).getTime() : created;
    if (f.updated_from && updated < new Date(f.updated_from).getTime()) return false;
    if (f.updated_to) {
      const to = new Date(f.updated_to).getTime() + 86400000 - 1;
      if (updated > to) return false;
    }
    return true;
  }

  function matchesPart(p, f) {
    if (!f.part.length) return true;
    const slugs = bomCache.get(p.id);
    if (!slugs) return false; // not yet known — treat as miss until fetched
    return f.part.some(function (slug) { return slugs.has(slug); });
  }

  async function applyFilters() {
    const f = currentFilters();
    writeParams(f);

    // Stage 1: cheap filters
    let stage = allProjects.filter(function (p) { return matchesNonPart(p, f); });

    // Stage 2: parts. Only fetch BOMs we don't have yet for the narrowed set.
    if (f.part.length) {
      const ids = stage.map(function (p) { return p.id; });
      const myFetch = ensureBomFor(ids);
      bomFetchPromise = myFetch;
      await myFetch;
      // Bail if another search has started since.
      if (bomFetchPromise !== myFetch) return;
      stage = stage.filter(function (p) { return matchesPart(p, f); });
    }

    const ranked = ProjectSearch.rankHubProjects(stage);
    render(ranked);
  }

  function render(projects) {
    spinner.classList.add('d-none');
    noResults.classList.add('d-none');
    grid.innerHTML = '';
    countEl.textContent = 'Showing ' + projects.length + ' of ' + allProjects.length + ' projects';
    if (projects.length === 0) {
      noResults.classList.remove('d-none');
      return;
    }
    grid.innerHTML = projects.map(function (p) {
      return ProjectSearch.cardHtml(p, myProjectIds);
    }).join('');
    ProjectSearch.attachCounts(API, projects);
  }

  // ─── Event wiring ───────────────────────────────────────────
  let debounceTimer = null;
  function onFilterChange() {
    if (debounceTimer) clearTimeout(debounceTimer);
    debounceTimer = setTimeout(applyFilters, 150);
  }
  function onTextChange() {
    if (debounceTimer) clearTimeout(debounceTimer);
    debounceTimer = setTimeout(applyFilters, 250);
  }

  searchInput.addEventListener('input', onTextChange);
  authorInput.addEventListener('input', onTextChange);
  [createdFrom, createdTo, updatedFrom, updatedTo].forEach(function (el) {
    el.addEventListener('change', onFilterChange);
  });

  clearBtn.addEventListener('click', function () {
    searchInput.value = '';
    authorInput.value = '';
    createdFrom.value = '';
    createdTo.value = '';
    updatedFrom.value = '';
    updatedTo.value = '';
    document.querySelectorAll('#projects-search input[type=checkbox]').forEach(function (cb) {
      cb.checked = false;
    });
    applyFilters();
  });

  // ─── Boot ───────────────────────────────────────────────────
  async function checkAuth() {
    try {
      const resp = await ProjectAuth.apiFetch(API + '/api/projects/my/list');
      if (resp.ok) {
        const mine = await resp.json();
        myProjectIds = new Set((mine || []).map(function (p) { return p.id; }));
      }
    } catch (e) {}
  }

  function seedFromUrl() {
    const f = readParams();
    searchInput.value = f.q;
    authorInput.value = f.author;
    createdFrom.value = f.created_from;
    createdTo.value = f.created_to;
    updatedFrom.value = f.updated_from;
    updatedTo.value = f.updated_to;
    renderCheckboxList('filter-difficulty', DIFFICULTY_OPTIONS, f.difficulty);
    renderCheckboxList('filter-status', STATUS_OPTIONS, f.status);
  }

  async function loadAll() {
    spinner.classList.remove('d-none');
    try {
      const resp = await fetch(API + '/api/projects?');
      if (!resp.ok) throw new Error('api');
      allProjects = await resp.json();
    } catch (e) {
      spinner.classList.add('d-none');
      grid.innerHTML = '<div class="text-center py-5 w-100"><p class="text-muted">Failed to load projects. Please try again later.</p></div>';
      return;
    }
    // Build dynamic facets from the loaded set
    const topTags = computeTopTags(allProjects, 10);
    renderCheckboxList('filter-tags', topTags, readParams().tag);
    await loadTopParts();
    // If a parts filter is preselected, prefetch BOMs for the candidate set
    // so the first paint is correct.
    const initial = readParams();
    if (initial.part.length) {
      const ids = allProjects.filter(function (p) { return matchesNonPart(p, currentFilters()); })
        .map(function (p) { return p.id; });
      await ensureBomFor(ids);
    }
    applyFilters();
  }

  seedFromUrl();
  checkAuth().then(loadAll);
})();
</script>
