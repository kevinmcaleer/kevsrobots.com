---
title: User Projects Hub
description: Discover amazing maker projects from the community
excerpt: Browse, search, and download complete project packages with instructions, code, and files
layout: content
hide_likes_and_comments: true
thanks: false
---

# {{page.title}}
## {{page.description}}

{{page.excerpt}}

---

<!-- Project Gallery Component -->
<div id="projects-hub">
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
  <div class="row mb-4">
    <div class="col-md-6">
      <input type="text" id="project-search" class="form-control" placeholder="Search projects by title or tag...">
    </div>
    <div class="col-md-3">
      <select id="difficulty-filter" class="form-select">
        <option value="">All Difficulties</option>
        <option value="beginner">Beginner</option>
        <option value="intermediate">Intermediate</option>
        <option value="advanced">Advanced</option>
      </select>
    </div>
    <div class="col-md-3">
      <input type="text" id="tag-filter" class="form-control" placeholder="Filter by tag...">
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

<script src="/assets/js/project-gradient.js"></script>
<script src="/assets/js/project-auth.js"></script>
<script src="/assets/js/project-interactions.js"></script>
<script>
(function() {
  const API = 'https://projects.kevsrobots.com';
  const grid = document.getElementById('projects-grid');
  const spinner = document.getElementById('loading-spinner');
  const noProjects = document.getElementById('no-projects');
  const searchInput = document.getElementById('project-search');
  const difficultyFilter = document.getElementById('difficulty-filter');
  const tagFilter = document.getElementById('tag-filter');

  const difficultyColors = {
    beginner: 'success',
    intermediate: 'warning',
    advanced: 'danger',
  };

  const statusBadges = {
    wip: '<span class="badge bg-info">🔨 WIP</span>',
    completed: '<span class="badge bg-success">✅ Completed</span>',
  };

  let myProjectIds = new Set();
  let showingMine = false;

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

    const params = new URLSearchParams();
    if (searchInput.value) params.set('search', searchInput.value);
    if (difficultyFilter.value) params.set('difficulty', difficultyFilter.value);
    if (tagFilter.value) params.set('tag', tagFilter.value.toLowerCase());

    try {
      const resp = await fetch(API + '/api/projects?' + params.toString());
      if (!resp.ok) throw new Error('API error');
      let projects = await resp.json();
      spinner.classList.add('d-none');

      if (showingMine) {
        projects = projects.filter(p => myProjectIds.has(p.id));
      }

      if (projects.length === 0) {
        noProjects.classList.remove('d-none');
        return;
      }

      grid.innerHTML = projects.map(p => `
        <div class="col">
          <a href="${myProjectIds.has(p.id) ? '/projects/editor.html?id=' + p.id : '/projects/view.html?id=' + p.id}" class="text-decoration-none">
            <div class="card h-100 border-0 shadow-sm card-hover">
              ${projectThumbnail(p, 200)}
              <div class="card-body">
                <h5 class="card-title text-dark">${esc(p.title)}</h5>
                <p class="card-text text-muted small">${esc(p.short_description || '')}</p>
                <div class="d-flex flex-wrap gap-1 mb-2">
                  ${statusBadges[p.status] || ''}
                  ${p.difficulty ? `<span class="badge bg-${difficultyColors[p.difficulty] || 'secondary'}">${p.difficulty}</span>` : ''}
                  ${p.estimated_minutes ? `<span class="badge bg-light text-dark"><i class="fas fa-clock"></i> ${p.estimated_minutes}min</span>` : ''}
                </div>
                <div class="d-flex flex-wrap gap-1">
                  ${(p.tags || []).slice(0, 4).map(t => `<span class="badge bg-light text-primary">${esc(t)}</span>`).join('')}
                </div>
              </div>
              <div class="card-footer bg-transparent border-0 d-flex justify-content-between align-items-center">
                <small class="text-muted">by ${esc(p.author_username)} &middot; ${new Date(p.created_at).toLocaleDateString()}</small>
                <small class="text-muted" id="card-likes-${p.id}"><i class="far fa-heart"></i> </small>
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

  let debounce;
  function debouncedLoad() {
    clearTimeout(debounce);
    debounce = setTimeout(loadProjects, 300);
  }

  searchInput.addEventListener('input', debouncedLoad);
  difficultyFilter.addEventListener('change', loadProjects);
  tagFilter.addEventListener('input', debouncedLoad);

  checkAuth().then(() => loadProjects());
})();
</script>
