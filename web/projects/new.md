---
title: Create New Project
description: Share your maker project with the community
layout: content
---

# {{page.title}}

Create and share your robotics, electronics, or maker project with the kevsrobots.com community.

---

<!-- Project Creation Form -->
<div id="project-form-container">
  <!-- Login Required Message -->
  <div id="login-required" class="alert alert-warning d-none">
    <h5>Login Required</h5>
    <p>You need to be logged in to create a project.</p>
    <a href="/login?redirect=/projects/new" class="btn btn-primary">Login</a>
    <a href="/register?redirect=/projects/new" class="btn btn-outline-primary">Register</a>
  </div>

  <!-- Project Creation Form -->
  <form id="project-form" class="d-none">
    <div class="card shadow-sm">
      <div class="card-header bg-primary text-white">
        <h5 class="mb-0">Project Details</h5>
      </div>
      <div class="card-body">
        <!-- Title -->
        <div class="mb-3">
          <label for="project-title" class="form-label">Project Title *</label>
          <input type="text" class="form-control" id="project-title" required maxlength="255"
                 placeholder="e.g., Arduino Robot Arm">
          <div class="form-text">Choose a clear, descriptive title for your project</div>
        </div>

        <!-- Description -->
        <div class="mb-3">
          <label for="project-description" class="form-label">Short Description *</label>
          <textarea class="form-control" id="project-description" rows="3" required
                    placeholder="Brief overview of your project..."></textarea>
          <div class="form-text">A concise summary that will appear in project listings</div>
        </div>

        <!-- Tags -->
        <div class="mb-3">
          <label for="project-tags" class="form-label">Tags</label>
          <input type="text" class="form-control" id="project-tags"
                 placeholder="e.g., arduino, robotics, 3d-printing">
          <div class="form-text">Comma-separated tags to help others find your project</div>
        </div>

        <!-- Background (Optional) -->
        <div class="mb-3">
          <label for="project-background" class="form-label">Project Background (Optional)</label>
          <textarea class="form-control" id="project-background" rows="5"
                    placeholder="Tell the story behind your project. What inspired you? What problem does it solve? (Markdown supported)"></textarea>
          <div class="form-text">Detailed background information (supports Markdown formatting)</div>
        </div>

        <!-- Code Link (Optional) -->
        <div class="mb-3">
          <label for="project-code-link" class="form-label">Code Repository (Optional)</label>
          <input type="url" class="form-control" id="project-code-link"
                 placeholder="https://github.com/username/project">
          <div class="form-text">Link to GitHub, GitLab, or other code repository</div>
        </div>

        <!-- Save as Draft Note -->
        <div class="alert alert-info">
          <i class="bi bi-info-circle"></i> Your project will be saved as a <strong>draft</strong>.
          You can add steps, files, images, and other details before publishing.
        </div>
      </div>

      <div class="card-footer">
        <button type="submit" class="btn btn-primary" id="create-btn">
          <span class="spinner-border spinner-border-sm d-none" id="create-spinner"></span>
          Create Project
        </button>
        <a href="/projects/hub" class="btn btn-outline-secondary">Cancel</a>
      </div>
    </div>
  </form>

  <!-- Success Message -->
  <div id="success-message" class="alert alert-success d-none">
    <h5>Project Created!</h5>
    <p>Your project has been created as a draft. You can now add more details.</p>
    <a href="#" id="edit-project-link" class="btn btn-primary">Edit Project</a>
    <a href="/projects/hub" class="btn btn-outline-secondary">Back to Projects</a>
  </div>

  <!-- Error Message -->
  <div id="error-message" class="alert alert-danger d-none">
    <h5>Error</h5>
    <p id="error-text">Failed to create project. Please try again.</p>
  </div>
</div>

<!-- JavaScript for Project Creation -->
<script>
const API_BASE = 'https://chatter.kevsrobots.com/api';

// Check if user is logged in
async function checkAuth() {
  try {
    const response = await fetch(`${API_BASE}/me`, {
      credentials: 'include'
    });

    if (response.ok) {
      // User is logged in
      document.getElementById('login-required').classList.add('d-none');
      document.getElementById('project-form').classList.remove('d-none');
      return true;
    } else {
      // Not logged in
      document.getElementById('login-required').classList.remove('d-none');
      document.getElementById('project-form').classList.add('d-none');
      return false;
    }
  } catch (error) {
    console.error('Auth check failed:', error);
    document.getElementById('login-required').classList.remove('d-none');
    return false;
  }
}

// Create project
async function createProject(projectData) {
  const response = await fetch(`${API_BASE}/projects`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include',
    body: JSON.stringify(projectData)
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Failed to create project');
  }

  return await response.json();
}

// Form submission
document.getElementById('project-form').addEventListener('submit', async (e) => {
  e.preventDefault();

  const createBtn = document.getElementById('create-btn');
  const spinner = document.getElementById('create-spinner');
  const errorMsg = document.getElementById('error-message');
  const successMsg = document.getElementById('success-message');

  // Disable button and show spinner
  createBtn.disabled = true;
  spinner.classList.remove('d-none');
  errorMsg.classList.add('d-none');

  try {
    // Parse tags
    const tagsInput = document.getElementById('project-tags').value;
    const tags = tagsInput ? tagsInput.split(',').map(t => t.trim().toLowerCase()).filter(t => t) : [];

    // Build project data
    const projectData = {
      title: document.getElementById('project-title').value,
      description: document.getElementById('project-description').value,
      tags: tags,
      background: document.getElementById('project-background').value || null,
      code_link: document.getElementById('project-code-link').value || null
    };

    // Create project
    const project = await createProject(projectData);

    // Show success message
    document.getElementById('project-form').classList.add('d-none');
    successMsg.classList.remove('d-none');

    // Set edit link
    document.getElementById('edit-project-link').href = `/projects/edit?id=${project.id}`;

    // Scroll to top
    window.scrollTo({ top: 0, behavior: 'smooth' });

  } catch (error) {
    console.error('Error creating project:', error);
    errorMsg.classList.remove('d-none');
    document.getElementById('error-text').textContent = error.message;
    createBtn.disabled = false;
    spinner.classList.add('d-none');
  }
});

// Check auth on page load
checkAuth();
</script>

<!-- Add Bootstrap Icons if not already included -->
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.10.0/font/bootstrap-icons.css">
