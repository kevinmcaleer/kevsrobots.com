---
title: My Projects
description: Manage your project portfolio
layout: content
---

# My Projects

<div id="login-required" class="alert alert-warning d-none">
  <h5>Login Required</h5>
  <p>You need to be logged in to view your projects.</p>
  <a href="/login?redirect=/projects/my-projects" class="btn btn-primary">Login</a>
</div>

<div id="projects-container" class="d-none">
  <!-- Header -->
  <div class="d-flex justify-content-between align-items-center mb-4">
    <div>
      <p class="text-muted mb-0">Manage your draft and published projects</p>
    </div>
    <a href="/projects/new" class="btn btn-primary">
      <i class="bi bi-plus-circle"></i> Create New Project
    </a>
  </div>

  <!-- Filter Tabs -->
  <ul class="nav nav-tabs mb-4" id="statusTabs" role="tablist">
    <li class="nav-item" role="presentation">
      <button class="nav-link active" id="all-tab" data-status="" onclick="filterProjects('')">
        All Projects <span class="badge bg-secondary ms-1" id="all-count">0</span>
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="draft-tab" data-status="draft" onclick="filterProjects('draft')">
        Drafts <span class="badge bg-warning ms-1" id="draft-count">0</span>
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="published-tab" data-status="published" onclick="filterProjects('published')">
        Published <span class="badge bg-success ms-1" id="published-count">0</span>
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
    <i class="bi bi-folder2-open" style="font-size: 4rem; color: #ccc;"></i>
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
<script>
const API_BASE = 'https://chatter.kevsrobots.com/api';
let currentUser = null;
let allProjects = [];
let currentFilter = '';

// Check authentication and load user
async function checkAuth() {
  try {
    const response = await fetch(`${API_BASE}/me`, {
      credentials: 'include'
    });

    if (!response.ok) {
      document.getElementById('login-required').classList.remove('d-none');
      return false;
    }

    currentUser = await response.json();
    document.getElementById('projects-container').classList.remove('d-none');
    await loadProjects();
    return true;

  } catch (error) {
    console.error('Auth check failed:', error);
    document.getElementById('login-required').classList.remove('d-none');
    return false;
  }
}

// Load all user's projects
async function loadProjects() {
  const loading = document.getElementById('loading');
  const grid = document.getElementById('projects-grid');
  const errorState = document.getElementById('error-state');

  loading.classList.remove('d-none');
  grid.classList.add('d-none');
  errorState.classList.add('d-none');

  try {
    // Fetch all projects by this user (both draft and published)
    const response = await fetch(`${API_BASE}/projects?author_id=${currentUser.id}&per_page=100`, {
      credentials: 'include'
    });

    if (!response.ok) throw new Error('Failed to load projects');

    const data = await response.json();
    allProjects = data.projects || [];

    // Update counts
    updateCounts();

    // Display projects
    displayProjects(allProjects);

    loading.classList.add('d-none');

  } catch (error) {
    console.error('Error loading projects:', error);
    loading.classList.add('d-none');
    errorState.classList.remove('d-none');
    document.getElementById('error-message').textContent = error.message;
  }
}

// Update status counts
function updateCounts() {
  const draftCount = allProjects.filter(p => p.status === 'draft').length;
  const publishedCount = allProjects.filter(p => p.status === 'published').length;

  document.getElementById('all-count').textContent = allProjects.length;
  document.getElementById('draft-count').textContent = draftCount;
  document.getElementById('published-count').textContent = publishedCount;
}

// Filter projects by status
function filterProjects(status) {
  currentFilter = status;

  // Update active tab
  document.querySelectorAll('#statusTabs button').forEach(btn => {
    btn.classList.remove('active');
  });

  if (status === '') {
    document.getElementById('all-tab').classList.add('active');
  } else if (status === 'draft') {
    document.getElementById('draft-tab').classList.add('active');
  } else if (status === 'published') {
    document.getElementById('published-tab').classList.add('active');
  }

  // Filter and display
  const filtered = status === ''
    ? allProjects
    : allProjects.filter(p => p.status === status);

  displayProjects(filtered);
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

// Create a project card
function createProjectCard(project) {
  const col = document.createElement('div');
  col.className = 'col';

  const isDraft = project.status === 'draft';
  const statusBadge = isDraft
    ? '<span class="badge bg-warning text-dark">Draft</span>'
    : '<span class="badge bg-success">Published</span>';

  const imageUrl = project.primary_image_url || 'https://via.placeholder.com/400x200?text=No+Image';

  const tagsHtml = (project.tags || []).slice(0, 3).map(tag =>
    `<span class="badge bg-secondary me-1">${escapeHtml(tag)}</span>`
  ).join('');

  const date = new Date(project.updated_at || project.created_at).toLocaleDateString();

  col.innerHTML = `
    <div class="card h-100 shadow-sm">
      <img src="${imageUrl}" class="card-img-top" alt="${escapeHtml(project.title)}"
           style="height: 200px; object-fit: cover;">
      <div class="card-body">
        <div class="d-flex justify-content-between align-items-start mb-2">
          <h5 class="card-title mb-0">${escapeHtml(project.title)}</h5>
          ${statusBadge}
        </div>
        <p class="card-text text-muted small">${escapeHtml(project.description.substring(0, 100))}...</p>
        ${tagsHtml ? `<div class="mb-2">${tagsHtml}</div>` : ''}
        <div class="d-flex justify-content-between align-items-center">
          <small class="text-muted">
            <i class="bi bi-eye"></i> ${project.view_count || 0}
            <i class="bi bi-heart ms-2"></i> ${project.like_count || 0}
            <i class="bi bi-download ms-2"></i> ${project.download_count || 0}
          </small>
          <small class="text-muted">${date}</small>
        </div>
      </div>
      <div class="card-footer bg-transparent">
        <div class="btn-group w-100" role="group">
          <a href="/projects/edit?id=${project.id}" class="btn btn-sm btn-primary">
            <i class="bi bi-pencil"></i> Edit
          </a>
          ${!isDraft ? `
            <a href="/projects/view?id=${project.id}" class="btn btn-sm btn-outline-primary">
              <i class="bi bi-eye"></i> View
            </a>
          ` : ''}
          <button class="btn btn-sm btn-outline-danger" onclick="deleteProject(${project.id}, '${escapeHtml(project.title)}')">
            <i class="bi bi-trash"></i>
          </button>
        </div>
      </div>
    </div>
  `;

  return col;
}

// Delete project
async function deleteProject(projectId, projectTitle) {
  if (!confirm(`Are you sure you want to delete "${projectTitle}"? This cannot be undone.`)) {
    return;
  }

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}`, {
      method: 'DELETE',
      credentials: 'include'
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Failed to delete project');
    }

    // Reload projects
    await loadProjects();
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

<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.10.0/font/bootstrap-icons.css">
