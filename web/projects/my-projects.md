---
title: My Projects
description: Manage your project portfolio
layout: content
---

{% include nav_projects.html %}

# My Projects

<div id="login-required" class="alert alert-warning d-none">
  <h5>Login Required</h5>
  <p>You need to be logged in to view your projects.</p>
  <a href="/login?redirect=/projects/my-projects" class="btn btn-primary">Login</a>
</div>

<div id="projects-container">
  <!-- Header -->
  <div class="d-flex justify-content-between align-items-center mb-4">
    <div>
      <p class="text-muted mb-0">Manage your draft and published projects</p>
    </div>
    <a href="/projects/new" class="btn btn-primary">
      <i class="fas fa-plus-circle me-1"></i> Create New Project
    </a>
  </div>

  <!-- Issue: early adopters' projects all autosaved as "Untitled Project".
       Surface them at the top so the user knows which ones need renaming
       before they go anywhere on the site. JS reveals when the count > 0. -->
  <div id="untitled-alert" class="alert alert-warning d-none align-items-start" role="alert">
    <i class="fas fa-exclamation-triangle me-2 fs-5"></i>
    <div class="flex-grow-1">
      <strong><span id="untitled-count">0</span> of your projects need a title.</strong>
      They're showing up as "Untitled Project" — invisible in search and on the hub. Click <em>Edit</em> on the highlighted cards below to give them a real name.
    </div>
  </div>

  <!-- Filter Tabs -->
  <ul class="nav nav-tabs mb-4" id="statusTabs" role="tablist">
    <li class="nav-item" role="presentation">
      <button class="nav-link active" id="all-tab" data-status="" onclick="filterProjects('')">
        All Projects <span class="badge bg-secondary ms-1" id="all-count">0</span>
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="wip-tab" data-status="wip" onclick="filterProjects('wip')">
        In Progress <span class="badge bg-warning ms-1" id="wip-count">0</span>
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="completed-tab" data-status="completed" onclick="filterProjects('completed')">
        Completed <span class="badge bg-success ms-1" id="completed-count">0</span>
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="archived-tab" data-status="archived" onclick="filterProjects('archived')">
        Archived <span class="badge bg-secondary ms-1" id="archived-count">0</span>
      </button>
    </li>
  </ul>

  <!-- Loading -->
  <div id="loading" class="text-center py-5">
    <div class="spinner-border text-primary" role="status">
      <span class="visually-hidden">Loading...</span>
    </div>
  </div>

  <!-- Projects Grid -->
  <div id="projects-grid" class="row row-cols-1 row-cols-md-2 row-cols-lg-3 g-4 d-none">
    <!-- Project cards will be inserted here -->
  </div>

  <!-- Empty State -->
  <div id="empty-state" class="text-center py-5 d-none">
    <i class="fas fa-folder-open" style="font-size: 4rem; color: #ccc;"></i>
    <h5 class="mt-3">No projects yet</h5>
    <p class="text-muted">Start sharing your maker projects with the community!</p>
    <a href="/projects/new" class="btn btn-primary">Create Your First Project</a>
  </div>

  <!-- Error State -->
  <div id="error-state" class="alert alert-danger d-none">
    <h5>Error</h5>
    <p id="error-message">Failed to load projects</p>
  </div>
</div>

<!-- JavaScript -->
<script src="/assets/js/project-auth.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/project-gradient.js?v={{ site.time | date: '%s' }}"></script>
<script>
// Projects live in projects-api now (was Chatter — the old /api/projects
// endpoint returns 500 and the browser reports "Failed to fetch").
const PROJECTS_API = 'https://projects.kevsrobots.com';
let allProjects = [];
let currentFilter = '';

function authFetch(url, opts) {
  if (window.ProjectAuth && typeof window.ProjectAuth.apiFetch === 'function') {
    return window.ProjectAuth.apiFetch(url, opts);
  }
  opts = opts || {};
  opts.credentials = 'include';
  return fetch(url, opts);
}

// Check authentication and load projects.
// /my/list is self-scoped — 401 if not logged in.
async function checkAuth() {
  try {
    const response = await authFetch(`${PROJECTS_API}/api/projects/my/list`);

    if (response.status === 401 || response.status === 403) {
      document.getElementById('projects-container').classList.add('d-none');
      document.getElementById('login-required').classList.remove('d-none');
      return false;
    }
    if (!response.ok) {
      throw new Error(`Failed to load projects (${response.status})`);
    }

    allProjects = await response.json();
    document.getElementById('loading').classList.add('d-none');
    updateCounts();
    updateUntitledAlert(allProjects);
    displayProjects(allProjects);
    return true;

  } catch (error) {
    console.error('Failed to load my projects:', error);
    document.getElementById('loading').classList.add('d-none');
    const errorState = document.getElementById('error-state');
    errorState.classList.remove('d-none');
    document.getElementById('error-message').textContent = error.message;
    return false;
  }
}

// Update status counts. projects-api uses wip/completed/archived.
function updateCounts() {
  const wipCount = allProjects.filter(p => p.status === 'wip').length;
  const completedCount = allProjects.filter(p => p.status === 'completed').length;
  const archivedCount = allProjects.filter(p => p.status === 'archived').length;

  document.getElementById('all-count').textContent = allProjects.length;
  document.getElementById('wip-count').textContent = wipCount;
  document.getElementById('completed-count').textContent = completedCount;
  document.getElementById('archived-count').textContent = archivedCount;
}

// Filter projects by status
function filterProjects(status) {
  currentFilter = status;

  document.querySelectorAll('#statusTabs button').forEach(btn => {
    btn.classList.remove('active');
  });

  const tabId = status === '' ? 'all-tab' : `${status}-tab`;
  const tabEl = document.getElementById(tabId);
  if (tabEl) tabEl.classList.add('active');

  const filtered = status === ''
    ? allProjects
    : allProjects.filter(p => p.status === status);

  displayProjects(filtered);
}

// Issue: surface projects whose title is empty or still the literal
// "Untitled Project" so the user can see them at a glance. Defaults to
// case-insensitive comparison since the editor lower-cased the check.
function isUntitled(project) {
  const t = (project && project.title ? String(project.title) : '').trim().toLowerCase();
  return !t || t === 'untitled project';
}

function updateUntitledAlert(projects) {
  const alert = document.getElementById('untitled-alert');
  const countEl = document.getElementById('untitled-count');
  if (!alert || !countEl) return;
  const n = projects.filter(isUntitled).length;
  if (n === 0) { alert.classList.add('d-none'); return; }
  countEl.textContent = n;
  alert.classList.remove('d-none');
  alert.classList.add('d-flex');
}

// Display projects in grid
function displayProjects(projects) {
  const grid = document.getElementById('projects-grid');
  const emptyState = document.getElementById('empty-state');

  if (projects.length === 0) {
    grid.classList.add('d-none');
    emptyState.classList.remove('d-none');
    return;
  }

  emptyState.classList.add('d-none');
  grid.classList.remove('d-none');
  grid.innerHTML = '';

  projects.forEach(project => {
    const card = createProjectCard(project);
    grid.appendChild(card);
  });
}

// Create a project card. Field shape matches projects-api ProjectListItem:
// id, title, short_description, difficulty, status, author_username,
// cover_image, tags, created_at, is_remix, download_count.
function createProjectCard(project) {
  const col = document.createElement('div');
  col.className = 'col';

  let statusBadge;
  if (project.status === 'wip') {
    statusBadge = '<span class="badge bg-warning text-dark">In Progress</span>';
  } else if (project.status === 'completed') {
    statusBadge = '<span class="badge bg-success">Completed</span>';
  } else {
    statusBadge = '<span class="badge bg-secondary">Archived</span>';
  }

  // Untitled-project decoration: italicise the title in muted red, swap
  // in a "Needs a title" badge before the status one, and outline the
  // whole card so it's impossible to miss.
  const untitled = isUntitled(project);
  const titleHtml = untitled
    ? `<span class="text-danger fst-italic">${escapeHtml(project.title || 'Untitled Project')}</span>`
    : escapeHtml(project.title);
  const needsTitleBadge = untitled
    ? '<span class="badge bg-danger me-1" title="This project needs a title before it goes anywhere"><i class="fas fa-exclamation-triangle me-1"></i>Needs a title</span>'
    : '';
  const cardClasses = untitled
    ? 'card kr-project-card h-100 shadow-sm border-2 border-danger'
    : 'card kr-project-card h-100 shadow-sm';

  // Show the full description (truncation is handled by CSS line-clamp on
  // .kr-card-description) so we don't double-truncate.
  const description = project.short_description || '';

  const tagsHtml = (project.tags || []).slice(0, 6).map(tag =>
    `<span class="badge bg-secondary me-1">${escapeHtml(tag)}</span>`
  ).join('');

  const date = new Date(project.created_at).toLocaleDateString();

  // Use the shared gradient+initials fallback when no cover image
  // (matches the hub + profile pages). The previous static placeholder
  // URL (via.placeholder.com) returned 404, leaving a broken image icon.
  const thumb = (typeof projectThumbnail === 'function')
    ? projectThumbnail(project)
    : (project.cover_image
        ? `<img src="${escapeHtml(project.cover_image)}" class="card-img-top" loading="lazy" alt="${escapeHtml(project.title)}">`
        : `<div class="card-img-top d-flex align-items-center justify-content-center text-muted" style="background:#f1f3f5;"><i class="fas fa-cube fa-2x"></i></div>`);

  col.innerHTML = `
    <div class="${cardClasses}">
      ${thumb}
      <div class="card-body">
        <div class="d-flex justify-content-between align-items-start mb-2">
          <h5 class="card-title mb-0">${titleHtml}</h5>
          <div class="d-flex flex-wrap gap-1 justify-content-end">${needsTitleBadge}${statusBadge}</div>
        </div>
        <p class="card-text text-muted small kr-card-description">${escapeHtml(description)}</p>
        ${tagsHtml ? `<div class="mb-2 kr-card-tags">${tagsHtml}</div>` : ''}
        <div class="d-flex justify-content-between align-items-center">
          <small class="text-muted">
            <i class="fas fa-download"></i> ${project.download_count || 0}
          </small>
          <small class="text-muted">${date}</small>
        </div>
      </div>
      <div class="card-footer bg-transparent">
        <div class="btn-group w-100" role="group">
          <a href="/projects/editor.html?id=${project.id}" class="btn btn-sm btn-primary">
            <i class="fas fa-pencil-alt"></i> Edit
          </a>
          <a href="${(project.slug && project.author_username) ? '/projects/' + encodeURIComponent(project.author_username) + '/' + encodeURIComponent(project.slug) : '/projects/view.html?id=' + project.id}" class="btn btn-sm btn-outline-primary">
            <i class="fas fa-eye"></i> View
          </a>
          <button class="btn btn-sm btn-outline-danger" onclick="deleteProject(${project.id}, '${escapeHtml(project.title).replace(/'/g, '&#39;')}')">
            <i class="fas fa-trash"></i>
          </button>
        </div>
      </div>
    </div>
  `;

  return col;
}

// Delete project via projects-api.
async function deleteProject(projectId, projectTitle) {
  if (!confirm(`Are you sure you want to delete "${projectTitle}"? This cannot be undone.`)) {
    return;
  }

  try {
    const response = await authFetch(`${PROJECTS_API}/api/projects/${projectId}`, {
      method: 'DELETE'
    });

    if (!response.ok) {
      let detail = `Failed to delete project (${response.status})`;
      try {
        const err = await response.json();
        if (err && err.detail) detail = err.detail;
      } catch (e) {}
      throw new Error(detail);
    }

    // Drop the row from local cache and re-render rather than refetching.
    allProjects = allProjects.filter(p => p.id !== projectId);
    updateCounts();
    filterProjects(currentFilter);
    showToast('Project deleted successfully');

  } catch (error) {
    console.error('Error deleting project:', error);
    alert(error.message);
  }
}

// Utility functions
function escapeHtml(text) {
  if (!text) return '';
  const div = document.createElement('div');
  div.textContent = text;
  return div.innerHTML;
}

function showToast(message) {
  const toast = document.createElement('div');
  toast.className = 'alert alert-success position-fixed top-0 end-0 m-3';
  toast.style.zIndex = '9999';
  toast.textContent = message;
  document.body.appendChild(toast);
  setTimeout(() => toast.remove(), 3000);
}

// Initialize on page load
checkAuth();
</script>

