---
title: User Projects Hub
description: Discover amazing maker projects from the community
excerpt: Browse, search, and download complete project packages with instructions, code, and files
layout: content
---

# {{page.title}}
## {{page.description}}

{{page.excerpt}}

---

<!-- Project Gallery Component -->
<div id="projects-hub">
  <!-- Search and Filters -->
  <div class="row mb-4">
    <div class="col-md-6">
      <input type="text" id="project-search" class="form-control" placeholder="Search projects by title or tag...">
    </div>
    <div class="col-md-3">
      <select id="sort-select" class="form-select">
        <option value="recent">Most Recent</option>
        <option value="popular">Most Popular</option>
        <option value="most_liked">Most Liked</option>
        <option value="most_viewed">Most Viewed</option>
      </select>
    </div>
    <div class="col-md-3">
      <select id="tag-filter" class="form-select">
        <option value="">All Tags</option>
        <!-- Tags will be populated dynamically -->
      </select>
    </div>
  </div>

  <!-- Loading Spinner -->
  <div id="loading-spinner" class="text-center my-5">
    <div class="spinner-border text-primary" role="status">
      <span class="visually-hidden">Loading projects...</span>
    </div>
  </div>

  <!-- Error Message -->
  <div id="error-message" class="alert alert-danger d-none" role="alert">
    <strong>Error loading projects.</strong> Please try again later.
  </div>

  <!-- Projects Grid -->
  <div id="projects-grid" class="row row-cols-1 row-cols-sm-2 row-cols-md-3 row-cols-lg-4 g-3">
    <!-- Projects will be loaded here dynamically -->
  </div>

  <!-- Pagination -->
  <nav aria-label="Projects pagination" class="mt-4">
    <ul id="pagination" class="pagination justify-content-center">
      <!-- Pagination will be generated dynamically -->
    </ul>
  </nav>
</div>

<!-- JavaScript for Projects Hub -->
<script>
const PROJECTS_API = 'https://chatter.kevsrobots.com/api/projects';
let currentPage = 1;
let currentSort = 'recent';
let currentTag = '';
let currentSearch = '';

// Load projects from API
async function loadProjects(page = 1, sort = 'recent', tag = '', search = '') {
  const spinner = document.getElementById('loading-spinner');
  const errorMsg = document.getElementById('error-message');
  const grid = document.getElementById('projects-grid');

  spinner.classList.remove('d-none');
  errorMsg.classList.add('d-none');
  grid.innerHTML = '';

  try {
    // Build query parameters
    const params = new URLSearchParams({
      page: page,
      per_page: 20,
      sort: sort,
      status_filter: 'published' // Only show published projects
    });

    if (tag) params.append('tag', tag);
    // Note: Search functionality would need to be added to the API

    const response = await fetch(`${PROJECTS_API}?${params}`);
    if (!response.ok) throw new Error('Failed to fetch projects');

    const data = await response.json();

    // Hide spinner
    spinner.classList.add('d-none');

    // Render projects
    if (data.projects && data.projects.length > 0) {
      data.projects.forEach(project => renderProjectCard(project));
      renderPagination(data.page, data.pages);
    } else {
      grid.innerHTML = '<div class="col-12"><p class="text-center text-muted">No projects found.</p></div>';
    }

  } catch (error) {
    console.error('Error loading projects:', error);
    spinner.classList.add('d-none');
    errorMsg.classList.remove('d-none');
  }
}

// Render a project card
function renderProjectCard(project) {
  const grid = document.getElementById('projects-grid');
  const card = document.createElement('div');
  card.className = 'col';

  // Format date
  const date = new Date(project.created_at).toLocaleDateString();

  // Get primary image or placeholder
  const imageUrl = project.primary_image_url || '/assets/img/placeholder-project.jpg';

  // Build tags HTML
  const tagsHtml = project.tags.slice(0, 3).map(tag =>
    `<span class="badge bg-secondary me-1">${tag}</span>`
  ).join('');

  card.innerHTML = `
    <div class="card h-100 shadow-sm">
      <img src="${imageUrl}" class="card-img-top" alt="${project.title}" style="height: 200px; object-fit: cover;">
      <div class="card-body">
        <h5 class="card-title">${escapeHtml(project.title)}</h5>
        <p class="card-text text-muted small">${escapeHtml(project.description.substring(0, 100))}...</p>
        <div class="mb-2">${tagsHtml}</div>
        <div class="d-flex justify-content-between align-items-center">
          <small class="text-muted">
            <i class="bi bi-eye"></i> ${project.view_count}
            <i class="bi bi-heart ms-2"></i> ${project.like_count}
            <i class="bi bi-chat ms-2"></i> ${project.comment_count}
          </small>
          <small class="text-muted">${date}</small>
        </div>
      </div>
      <div class="card-footer bg-transparent">
        <a href="/projects/view/${project.id}" class="btn btn-sm btn-primary w-100">View Project</a>
      </div>
    </div>
  `;

  grid.appendChild(card);
}

// Render pagination
function renderPagination(currentPage, totalPages) {
  const pagination = document.getElementById('pagination');
  pagination.innerHTML = '';

  if (totalPages <= 1) return;

  // Previous button
  const prevLi = document.createElement('li');
  prevLi.className = `page-item ${currentPage === 1 ? 'disabled' : ''}`;
  prevLi.innerHTML = `<a class="page-link" href="#" data-page="${currentPage - 1}">Previous</a>`;
  pagination.appendChild(prevLi);

  // Page numbers (show max 5 pages)
  const startPage = Math.max(1, currentPage - 2);
  const endPage = Math.min(totalPages, startPage + 4);

  for (let i = startPage; i <= endPage; i++) {
    const pageLi = document.createElement('li');
    pageLi.className = `page-item ${i === currentPage ? 'active' : ''}`;
    pageLi.innerHTML = `<a class="page-link" href="#" data-page="${i}">${i}</a>`;
    pagination.appendChild(pageLi);
  }

  // Next button
  const nextLi = document.createElement('li');
  nextLi.className = `page-item ${currentPage === totalPages ? 'disabled' : ''}`;
  nextLi.innerHTML = `<a class="page-link" href="#" data-page="${currentPage + 1}">Next</a>`;
  pagination.appendChild(nextLi);

  // Add click handlers
  pagination.querySelectorAll('a.page-link').forEach(link => {
    link.addEventListener('click', (e) => {
      e.preventDefault();
      const page = parseInt(link.dataset.page);
      if (page >= 1 && page <= totalPages) {
        loadProjects(page, currentSort, currentTag, currentSearch);
        window.scrollTo({ top: 0, behavior: 'smooth' });
      }
    });
  });
}

// Helper function to escape HTML
function escapeHtml(text) {
  const div = document.createElement('div');
  div.textContent = text;
  return div.innerHTML;
}

// Event listeners
document.getElementById('sort-select').addEventListener('change', (e) => {
  currentSort = e.target.value;
  currentPage = 1;
  loadProjects(currentPage, currentSort, currentTag, currentSearch);
});

document.getElementById('tag-filter').addEventListener('change', (e) => {
  currentTag = e.target.value;
  currentPage = 1;
  loadProjects(currentPage, currentSort, currentTag, currentSearch);
});

document.getElementById('project-search').addEventListener('input', (e) => {
  currentSearch = e.target.value;
  // Debounce search
  clearTimeout(window.searchTimeout);
  window.searchTimeout = setTimeout(() => {
    currentPage = 1;
    loadProjects(currentPage, currentSort, currentTag, currentSearch);
  }, 500);
});

// Load projects on page load
loadProjects(currentPage, currentSort, currentTag, currentSearch);
</script>

<!-- Add Bootstrap Icons if not already included -->
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.10.0/font/bootstrap-icons.css">
