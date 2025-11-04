---
title: Edit Project
description: Manage your project details, steps, files, and more
layout: content
---

# Edit Project

<div id="loading-container" class="text-center py-5">
  <div class="spinner-border text-primary" role="status">
    <span class="visually-hidden">Loading...</span>
  </div>
  <p class="mt-3">Loading project...</p>
</div>

<div id="error-container" class="alert alert-danger d-none">
  <h5>Error</h5>
  <p id="error-message">Failed to load project</p>
  <a href="/projects/hub" class="btn btn-outline-secondary">Back to Projects</a>
</div>

<div id="project-container" class="d-none">
  <!-- Project Header -->
  <div class="d-flex justify-content-between align-items-center mb-4">
    <div>
      <h1 id="project-title-display">Project Title</h1>
      <p class="text-muted mb-0">
        <span id="project-status-badge" class="badge bg-secondary">Draft</span>
        <span class="ms-2">Last updated: <span id="last-updated">-</span></span>
      </p>
    </div>
    <div>
      <button type="button" class="btn btn-success" id="publish-btn" style="display: none;">
        <i class="bi bi-cloud-upload"></i> Publish
      </button>
      <a href="/projects/hub" class="btn btn-outline-secondary">
        <i class="bi bi-arrow-left"></i> Back to Projects
      </a>
    </div>
  </div>

  <!-- Navigation Tabs -->
  <ul class="nav nav-tabs mb-4" id="projectTabs" role="tablist">
    <li class="nav-item" role="presentation">
      <button class="nav-link active" id="details-tab" data-bs-toggle="tab" data-bs-target="#details" type="button">
        <i class="bi bi-info-circle"></i> Details
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="steps-tab" data-bs-toggle="tab" data-bs-target="#steps" type="button">
        <i class="bi bi-list-ol"></i> Steps
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="bom-tab" data-bs-toggle="tab" data-bs-target="#bom" type="button">
        <i class="bi bi-cart"></i> Bill of Materials
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="files-tab" data-bs-toggle="tab" data-bs-target="#files" type="button">
        <i class="bi bi-file-earmark-code"></i> Files
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="images-tab" data-bs-toggle="tab" data-bs-target="#images" type="button">
        <i class="bi bi-image"></i> Images
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="components-tab" data-bs-toggle="tab" data-bs-target="#components" type="button">
        <i class="bi bi-cpu"></i> Components
      </button>
    </li>
    <li class="nav-item" role="presentation">
      <button class="nav-link" id="links-tab" data-bs-toggle="tab" data-bs-target="#links" type="button">
        <i class="bi bi-link-45deg"></i> Links & Tools
      </button>
    </li>
  </ul>

  <!-- Tab Content -->
  <div class="tab-content" id="projectTabContent">

    <!-- Details Tab -->
    <div class="tab-pane fade show active" id="details" role="tabpanel">
      <div class="card shadow-sm">
        <div class="card-header bg-primary text-white">
          <h5 class="mb-0">Project Details</h5>
        </div>
        <div class="card-body">
          <form id="details-form">
            <div class="mb-3">
              <label for="title" class="form-label">Title *</label>
              <input type="text" class="form-control" id="title" required>
            </div>
            <div class="mb-3">
              <label for="description" class="form-label">Short Description *</label>
              <textarea class="form-control" id="description" rows="3" required></textarea>
            </div>
            <div class="mb-3">
              <label for="background" class="form-label">Background (Markdown supported)</label>
              <textarea class="form-control" id="background" rows="8"></textarea>
            </div>
            <div class="mb-3">
              <label for="tags" class="form-label">Tags (comma-separated)</label>
              <input type="text" class="form-control" id="tags" placeholder="e.g., arduino, robotics, 3d-printing">
            </div>
            <div class="mb-3">
              <label for="code_link" class="form-label">Code Repository URL</label>
              <input type="url" class="form-control" id="code_link" placeholder="https://github.com/username/project">
            </div>
            <div class="mb-3">
              <label for="difficulty_level" class="form-label">Difficulty Level</label>
              <select class="form-select" id="difficulty_level">
                <option value="">Not specified</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>
            <div class="mb-3">
              <label for="estimated_time" class="form-label">Estimated Time (minutes)</label>
              <input type="number" class="form-control" id="estimated_time" min="0">
            </div>
            <button type="submit" class="btn btn-primary">
              <i class="bi bi-save"></i> Save Details
            </button>
          </form>
        </div>
      </div>
    </div>

    <!-- Steps Tab -->
    <div class="tab-pane fade" id="steps" role="tabpanel">
      <div class="card shadow-sm">
        <div class="card-header bg-primary text-white d-flex justify-content-between align-items-center">
          <h5 class="mb-0">Build Steps</h5>
          <button type="button" class="btn btn-sm btn-light" onclick="showAddStepForm()">
            <i class="bi bi-plus-circle"></i> Add Step
          </button>
        </div>
        <div class="card-body">
          <!-- Add Step Form -->
          <div id="add-step-form" class="mb-4 d-none">
            <h6>Add New Step</h6>
            <form id="step-form">
              <div class="mb-3">
                <label for="step-title" class="form-label">Step Title *</label>
                <input type="text" class="form-control" id="step-title" required>
              </div>
              <div class="mb-3">
                <label for="step-content" class="form-label">Instructions (Markdown supported) *</label>
                <textarea class="form-control" id="step-content" rows="5" required></textarea>
              </div>
              <button type="submit" class="btn btn-primary">Add Step</button>
              <button type="button" class="btn btn-outline-secondary" onclick="hideAddStepForm()">Cancel</button>
            </form>
            <hr>
          </div>

          <!-- Steps List -->
          <div id="steps-list">
            <p class="text-muted">No steps added yet. Click "Add Step" to get started.</p>
          </div>
        </div>
      </div>
    </div>

    <!-- Bill of Materials Tab -->
    <div class="tab-pane fade" id="bom" role="tabpanel">
      <div class="card shadow-sm">
        <div class="card-header bg-primary text-white d-flex justify-content-between align-items-center">
          <h5 class="mb-0">Bill of Materials</h5>
          <button type="button" class="btn btn-sm btn-light" onclick="showAddBomForm()">
            <i class="bi bi-plus-circle"></i> Add Item
          </button>
        </div>
        <div class="card-body">
          <!-- Add BOM Form -->
          <div id="add-bom-form" class="mb-4 d-none">
            <h6>Add BOM Item</h6>
            <form id="bom-form">
              <div class="row">
                <div class="col-md-6 mb-3">
                  <label for="bom-item-name" class="form-label">Item Name *</label>
                  <input type="text" class="form-control" id="bom-item-name" required>
                </div>
                <div class="col-md-3 mb-3">
                  <label for="bom-quantity" class="form-label">Quantity *</label>
                  <input type="number" class="form-control" id="bom-quantity" min="1" value="1" required>
                </div>
                <div class="col-md-3 mb-3">
                  <label for="bom-price" class="form-label">Price (£)</label>
                  <input type="number" class="form-control" id="bom-price" step="0.01" min="0">
                </div>
              </div>
              <div class="mb-3">
                <label for="bom-link" class="form-label">Purchase Link (Optional)</label>
                <input type="url" class="form-control" id="bom-link" placeholder="https://...">
              </div>
              <button type="submit" class="btn btn-primary">Add Item</button>
              <button type="button" class="btn btn-outline-secondary" onclick="hideAddBomForm()">Cancel</button>
            </form>
            <hr>
          </div>

          <!-- BOM List -->
          <div id="bom-list">
            <p class="text-muted">No items in bill of materials yet.</p>
          </div>
        </div>
      </div>
    </div>

    <!-- Files Tab -->
    <div class="tab-pane fade" id="files" role="tabpanel">
      <div class="card shadow-sm">
        <div class="card-header bg-primary text-white">
          <h5 class="mb-0">Project Files</h5>
        </div>
        <div class="card-body">
          <!-- Upload Form -->
          <form id="file-upload-form" class="mb-4">
            <div class="mb-3">
              <label for="file-upload" class="form-label">Upload File</label>
              <input type="file" class="form-control" id="file-upload" required>
              <div class="form-text">
                Allowed: .py, .cpp, .h, .ino, .md, .txt, .pdf, .stl, .obj, .gcode, .json, .xml, .yaml, .yml (Max 25MB)
              </div>
            </div>
            <div class="mb-3">
              <label for="file-title" class="form-label">File Title *</label>
              <input type="text" class="form-control" id="file-title" required placeholder="e.g., Main Arduino Code">
            </div>
            <div class="mb-3">
              <label for="file-description" class="form-label">Description</label>
              <textarea class="form-control" id="file-description" rows="2"></textarea>
            </div>
            <button type="submit" class="btn btn-primary">
              <i class="bi bi-upload"></i> Upload File
            </button>
          </form>

          <!-- Files List -->
          <div id="files-list">
            <p class="text-muted">No files uploaded yet.</p>
          </div>
        </div>
      </div>
    </div>

    <!-- Images Tab -->
    <div class="tab-pane fade" id="images" role="tabpanel">
      <div class="card shadow-sm">
        <div class="card-header bg-primary text-white">
          <h5 class="mb-0">Project Images</h5>
        </div>
        <div class="card-body">
          <!-- Upload Form -->
          <form id="image-upload-form" class="mb-4">
            <div class="mb-3">
              <label for="image-upload" class="form-label">Upload Image</label>
              <input type="file" class="form-control" id="image-upload" accept="image/*" required>
              <div class="form-text">Allowed: .png, .jpg, .jpeg, .gif, .webp, .svg (Max 10MB)</div>
            </div>
            <div class="mb-3">
              <label for="image-caption" class="form-label">Caption</label>
              <input type="text" class="form-control" id="image-caption">
            </div>
            <button type="submit" class="btn btn-primary">
              <i class="bi bi-upload"></i> Upload Image
            </button>
          </form>

          <!-- Images Grid -->
          <div id="images-grid" class="row g-3">
            <p class="text-muted">No images uploaded yet.</p>
          </div>
        </div>
      </div>
    </div>

    <!-- Components Tab -->
    <div class="tab-pane fade" id="components" role="tabpanel">
      <div class="card shadow-sm">
        <div class="card-header bg-primary text-white">
          <h5 class="mb-0">Components Used</h5>
        </div>
        <div class="card-body">
          <p class="text-muted">Component management coming soon...</p>
        </div>
      </div>
    </div>

    <!-- Links & Tools Tab -->
    <div class="tab-pane fade" id="links" role="tabpanel">
      <div class="card shadow-sm">
        <div class="card-header bg-primary text-white">
          <h5 class="mb-0">Related Links & Tools</h5>
        </div>
        <div class="card-body">
          <!-- Add Link Form -->
          <div class="mb-4">
            <h6>Add Link</h6>
            <form id="link-form">
              <div class="row">
                <div class="col-md-6 mb-3">
                  <label for="link-title" class="form-label">Link Title *</label>
                  <input type="text" class="form-control" id="link-title" required>
                </div>
                <div class="col-md-6 mb-3">
                  <label for="link-url" class="form-label">URL *</label>
                  <input type="url" class="form-control" id="link-url" required>
                </div>
              </div>
              <div class="mb-3">
                <label for="link-type" class="form-label">Link Type *</label>
                <select class="form-select" id="link-type" required>
                  <option value="resource">Resource</option>
                  <option value="video">Video</option>
                  <option value="course">Course</option>
                  <option value="article">Article</option>
                  <option value="related_project">Related Project</option>
                </select>
              </div>
              <button type="submit" class="btn btn-primary">Add Link</button>
            </form>
          </div>

          <!-- Links List -->
          <div id="links-list">
            <p class="text-muted">No links added yet.</p>
          </div>
        </div>
      </div>
    </div>

  </div>
</div>

<!-- JavaScript -->
<script>
const API_BASE = 'https://chatter.kevsrobots.com/api';
let projectId = null;
let projectData = null;

// Get project ID from URL query parameter
function getProjectIdFromUrl() {
  const urlParams = new URLSearchParams(window.location.search);
  const id = urlParams.get('id');
  return id ? parseInt(id) : null;
}

// Load project data
async function loadProject() {
  projectId = getProjectIdFromUrl();

  if (!projectId) {
    showError('Invalid project ID');
    return;
  }

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}`, {
      credentials: 'include'
    });

    if (!response.ok) {
      throw new Error('Failed to load project');
    }

    projectData = await response.json();
    displayProject();

    // Load all sections
    await Promise.all([
      loadSteps(),
      loadBom(),
      loadFiles(),
      loadImages(),
      loadLinks()
    ]);

    document.getElementById('loading-container').classList.add('d-none');
    document.getElementById('project-container').classList.remove('d-none');

  } catch (error) {
    console.error('Error loading project:', error);
    showError(error.message);
  }
}

function displayProject() {
  document.getElementById('project-title-display').textContent = projectData.title;

  // Status badge
  const statusBadge = document.getElementById('project-status-badge');
  if (projectData.is_published) {
    statusBadge.textContent = 'Published';
    statusBadge.className = 'badge bg-success';
  } else {
    statusBadge.textContent = 'Draft';
    statusBadge.className = 'badge bg-secondary';
    document.getElementById('publish-btn').style.display = 'inline-block';
  }

  // Last updated
  if (projectData.updated_at) {
    const date = new Date(projectData.updated_at);
    document.getElementById('last-updated').textContent = date.toLocaleDateString();
  }

  // Fill form fields
  document.getElementById('title').value = projectData.title || '';
  document.getElementById('description').value = projectData.description || '';
  document.getElementById('background').value = projectData.background || '';
  document.getElementById('tags').value = projectData.tags ? projectData.tags.join(', ') : '';
  document.getElementById('code_link').value = projectData.code_link || '';
  document.getElementById('difficulty_level').value = projectData.difficulty_level || '';
  document.getElementById('estimated_time').value = projectData.estimated_time || '';
}

// Save project details
document.getElementById('details-form').addEventListener('submit', async (e) => {
  e.preventDefault();

  const tags = document.getElementById('tags').value
    .split(',')
    .map(t => t.trim().toLowerCase())
    .filter(t => t);

  const data = {
    title: document.getElementById('title').value,
    description: document.getElementById('description').value,
    background: document.getElementById('background').value || null,
    tags: tags.length > 0 ? tags : null,
    code_link: document.getElementById('code_link').value || null,
    difficulty_level: document.getElementById('difficulty_level').value || null,
    estimated_time: parseInt(document.getElementById('estimated_time').value) || null
  };

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify(data)
    });

    if (!response.ok) throw new Error('Failed to save');

    projectData = await response.json();
    displayProject();
    showToast('Details saved successfully');
  } catch (error) {
    console.error('Error saving:', error);
    alert('Failed to save details');
  }
});

// Steps functions
async function loadSteps() {
  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/steps`, {
      credentials: 'include'
    });

    if (!response.ok) return;

    const steps = await response.json();
    displaySteps(steps);
  } catch (error) {
    console.error('Error loading steps:', error);
  }
}

function displaySteps(steps) {
  const container = document.getElementById('steps-list');

  if (steps.length === 0) {
    container.innerHTML = '<p class="text-muted">No steps added yet. Click "Add Step" to get started.</p>';
    return;
  }

  container.innerHTML = steps.map((step, index) => `
    <div class="card mb-3">
      <div class="card-body">
        <div class="d-flex justify-content-between align-items-start">
          <div>
            <h6>Step ${step.step_order}: ${step.title}</h6>
            <div class="text-muted small">${escapeHtml(step.content).substring(0, 100)}...</div>
          </div>
          <button class="btn btn-sm btn-outline-danger" onclick="deleteStep(${step.id})">
            <i class="bi bi-trash"></i>
          </button>
        </div>
      </div>
    </div>
  `).join('');
}

function showAddStepForm() {
  document.getElementById('add-step-form').classList.remove('d-none');
}

function hideAddStepForm() {
  document.getElementById('add-step-form').classList.add('d-none');
  document.getElementById('step-form').reset();
}

document.getElementById('step-form').addEventListener('submit', async (e) => {
  e.preventDefault();

  const data = {
    title: document.getElementById('step-title').value,
    content: document.getElementById('step-content').value
  };

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/steps`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify(data)
    });

    if (!response.ok) throw new Error('Failed to add step');

    hideAddStepForm();
    await loadSteps();
    showToast('Step added successfully');
  } catch (error) {
    console.error('Error adding step:', error);
    alert('Failed to add step');
  }
});

async function deleteStep(stepId) {
  if (!confirm('Delete this step?')) return;

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/steps/${stepId}`, {
      method: 'DELETE',
      credentials: 'include'
    });

    if (!response.ok) throw new Error('Failed to delete');

    await loadSteps();
    showToast('Step deleted');
  } catch (error) {
    console.error('Error deleting step:', error);
    alert('Failed to delete step');
  }
}

// BOM functions
async function loadBom() {
  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/bom`, {
      credentials: 'include'
    });

    if (!response.ok) return;

    const items = await response.json();
    displayBom(items);
  } catch (error) {
    console.error('Error loading BOM:', error);
  }
}

function displayBom(items) {
  const container = document.getElementById('bom-list');

  if (items.length === 0) {
    container.innerHTML = '<p class="text-muted">No items in bill of materials yet.</p>';
    return;
  }

  const total = items.reduce((sum, item) => sum + (item.price_cents * item.quantity), 0);

  container.innerHTML = `
    <div class="table-responsive">
      <table class="table">
        <thead>
          <tr>
            <th>Item</th>
            <th>Quantity</th>
            <th>Unit Price</th>
            <th>Total</th>
            <th>Link</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          ${items.map(item => `
            <tr>
              <td>${escapeHtml(item.item_name)}</td>
              <td>${item.quantity}</td>
              <td>£${(item.price_cents / 100).toFixed(2)}</td>
              <td>£${((item.price_cents * item.quantity) / 100).toFixed(2)}</td>
              <td>${item.link ? `<a href="${item.link}" target="_blank"><i class="bi bi-link-45deg"></i></a>` : '-'}</td>
              <td>
                <button class="btn btn-sm btn-outline-danger" onclick="deleteBomItem(${item.id})">
                  <i class="bi bi-trash"></i>
                </button>
              </td>
            </tr>
          `).join('')}
        </tbody>
        <tfoot>
          <tr class="fw-bold">
            <td colspan="3">Total</td>
            <td>£${(total / 100).toFixed(2)}</td>
            <td colspan="2"></td>
          </tr>
        </tfoot>
      </table>
    </div>
  `;
}

function showAddBomForm() {
  document.getElementById('add-bom-form').classList.remove('d-none');
}

function hideAddBomForm() {
  document.getElementById('add-bom-form').classList.add('d-none');
  document.getElementById('bom-form').reset();
}

document.getElementById('bom-form').addEventListener('submit', async (e) => {
  e.preventDefault();

  const price = parseFloat(document.getElementById('bom-price').value) || 0;

  const data = {
    item_name: document.getElementById('bom-item-name').value,
    quantity: parseInt(document.getElementById('bom-quantity').value),
    price_cents: Math.round(price * 100),
    link: document.getElementById('bom-link').value || null
  };

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/bom`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify(data)
    });

    if (!response.ok) throw new Error('Failed to add item');

    hideAddBomForm();
    await loadBom();
    showToast('Item added to BOM');
  } catch (error) {
    console.error('Error adding BOM item:', error);
    alert('Failed to add item');
  }
});

async function deleteBomItem(itemId) {
  if (!confirm('Delete this item?')) return;

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/bom/${itemId}`, {
      method: 'DELETE',
      credentials: 'include'
    });

    if (!response.ok) throw new Error('Failed to delete');

    await loadBom();
    showToast('Item deleted');
  } catch (error) {
    console.error('Error deleting item:', error);
    alert('Failed to delete item');
  }
}

// Files functions
async function loadFiles() {
  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/files`, {
      credentials: 'include'
    });

    if (!response.ok) return;

    const files = await response.json();
    displayFiles(files);
  } catch (error) {
    console.error('Error loading files:', error);
  }
}

function displayFiles(files) {
  const container = document.getElementById('files-list');

  if (files.length === 0) {
    container.innerHTML = '<p class="text-muted">No files uploaded yet.</p>';
    return;
  }

  container.innerHTML = `
    <div class="list-group">
      ${files.map(file => `
        <div class="list-group-item d-flex justify-content-between align-items-center">
          <div>
            <h6 class="mb-1">${escapeHtml(file.original_filename || file.filename)}</h6>
            <small class="text-muted">${formatFileSize(file.file_size)}</small>
            ${file.description ? `<p class="mb-0 small">${escapeHtml(file.description)}</p>` : ''}
          </div>
          <div>
            <a href="${API_BASE}/projects/${projectId}/files/${file.id}/download" class="btn btn-sm btn-outline-primary me-2">
              <i class="bi bi-download"></i>
            </a>
            <button class="btn btn-sm btn-outline-danger" onclick="deleteFile(${file.id})">
              <i class="bi bi-trash"></i>
            </button>
          </div>
        </div>
      `).join('')}
    </div>
  `;
}

document.getElementById('file-upload-form').addEventListener('submit', async (e) => {
  e.preventDefault();

  const fileInput = document.getElementById('file-upload');
  const file = fileInput.files[0];

  if (!file) return;

  const formData = new FormData();
  formData.append('file', file);
  formData.append('title', document.getElementById('file-title').value);
  formData.append('description', document.getElementById('file-description').value);

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/files`, {
      method: 'POST',
      credentials: 'include',
      body: formData
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Failed to upload');
    }

    document.getElementById('file-upload-form').reset();
    await loadFiles();
    showToast('File uploaded successfully');
  } catch (error) {
    console.error('Error uploading file:', error);
    alert(error.message);
  }
});

async function deleteFile(fileId) {
  if (!confirm('Delete this file?')) return;

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/files/${fileId}`, {
      method: 'DELETE',
      credentials: 'include'
    });

    if (!response.ok) throw new Error('Failed to delete');

    await loadFiles();
    showToast('File deleted');
  } catch (error) {
    console.error('Error deleting file:', error);
    alert('Failed to delete file');
  }
}

// Images functions
async function loadImages() {
  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/images`, {
      credentials: 'include'
    });

    if (!response.ok) return;

    const images = await response.json();
    displayImages(images);
  } catch (error) {
    console.error('Error loading images:', error);
  }
}

function displayImages(images) {
  const container = document.getElementById('images-grid');

  if (images.length === 0) {
    container.innerHTML = '<p class="text-muted">No images uploaded yet.</p>';
    return;
  }

  container.innerHTML = images.map(image => `
    <div class="col-md-4">
      <div class="card">
        <img src="${API_BASE}/projects/${projectId}/images/${image.id}" class="card-img-top" alt="${escapeHtml(image.caption || '')}" style="height: 200px; object-fit: cover;">
        <div class="card-body">
          <p class="card-text small">${escapeHtml(image.caption || 'No caption')}</p>
          <div class="d-flex justify-content-between">
            <button class="btn btn-sm btn-outline-primary" onclick="setPrimaryImage(${image.id})">
              <i class="bi bi-star"></i> Set as Primary
            </button>
            <button class="btn btn-sm btn-outline-danger" onclick="deleteImage(${image.id})">
              <i class="bi bi-trash"></i>
            </button>
          </div>
        </div>
      </div>
    </div>
  `).join('');
}

document.getElementById('image-upload-form').addEventListener('submit', async (e) => {
  e.preventDefault();

  const fileInput = document.getElementById('image-upload');
  const file = fileInput.files[0];

  if (!file) return;

  const formData = new FormData();
  formData.append('file', file);
  formData.append('caption', document.getElementById('image-caption').value);

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/images`, {
      method: 'POST',
      credentials: 'include',
      body: formData
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Failed to upload');
    }

    document.getElementById('image-upload-form').reset();
    await loadImages();
    showToast('Image uploaded successfully');
  } catch (error) {
    console.error('Error uploading image:', error);
    alert(error.message);
  }
});

async function deleteImage(imageId) {
  if (!confirm('Delete this image?')) return;

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/images/${imageId}`, {
      method: 'DELETE',
      credentials: 'include'
    });

    if (!response.ok) throw new Error('Failed to delete');

    await loadImages();
    showToast('Image deleted');
  } catch (error) {
    console.error('Error deleting image:', error);
    alert('Failed to delete image');
  }
}

async function setPrimaryImage(imageId) {
  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/primary-image/${imageId}`, {
      method: 'PUT',
      credentials: 'include'
    });

    if (!response.ok) throw new Error('Failed to set primary image');

    showToast('Primary image updated');
  } catch (error) {
    console.error('Error setting primary image:', error);
    alert('Failed to set primary image');
  }
}

// Links functions
async function loadLinks() {
  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/links`, {
      credentials: 'include'
    });

    if (!response.ok) return;

    const links = await response.json();
    displayLinks(links);
  } catch (error) {
    console.error('Error loading links:', error);
  }
}

function displayLinks(links) {
  const container = document.getElementById('links-list');

  if (links.length === 0) {
    container.innerHTML = '<p class="text-muted">No links added yet.</p>';
    return;
  }

  container.innerHTML = `
    <div class="list-group">
      ${links.map(link => `
        <div class="list-group-item d-flex justify-content-between align-items-center">
          <div>
            <h6 class="mb-1">
              <a href="${escapeHtml(link.url)}" target="_blank">${escapeHtml(link.title)}</a>
            </h6>
            <small class="text-muted">
              <span class="badge bg-secondary">${link.link_type}</span>
            </small>
          </div>
          <button class="btn btn-sm btn-outline-danger" onclick="deleteLink(${link.id})">
            <i class="bi bi-trash"></i>
          </button>
        </div>
      `).join('')}
    </div>
  `;
}

document.getElementById('link-form').addEventListener('submit', async (e) => {
  e.preventDefault();

  const data = {
    title: document.getElementById('link-title').value,
    url: document.getElementById('link-url').value,
    link_type: document.getElementById('link-type').value
  };

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/links`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify(data)
    });

    if (!response.ok) throw new Error('Failed to add link');

    document.getElementById('link-form').reset();
    await loadLinks();
    showToast('Link added successfully');
  } catch (error) {
    console.error('Error adding link:', error);
    alert('Failed to add link');
  }
});

async function deleteLink(linkId) {
  if (!confirm('Delete this link?')) return;

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/links/${linkId}`, {
      method: 'DELETE',
      credentials: 'include'
    });

    if (!response.ok) throw new Error('Failed to delete');

    await loadLinks();
    showToast('Link deleted');
  } catch (error) {
    console.error('Error deleting link:', error);
    alert('Failed to delete link');
  }
}

// Publish button
document.getElementById('publish-btn').addEventListener('click', async () => {
  if (!confirm('Publish this project? It will be visible to everyone.')) return;

  try {
    const response = await fetch(`${API_BASE}/projects/${projectId}/publish`, {
      method: 'PUT',
      credentials: 'include'
    });

    if (!response.ok) throw new Error('Failed to publish');

    projectData = await response.json();
    displayProject();
    showToast('Project published successfully!');
  } catch (error) {
    console.error('Error publishing:', error);
    alert('Failed to publish project');
  }
});

// Utility functions
function showError(message) {
  document.getElementById('loading-container').classList.add('d-none');
  document.getElementById('error-container').classList.remove('d-none');
  document.getElementById('error-message').textContent = message;
}

function showToast(message) {
  // Simple alert for now - could be replaced with Bootstrap toast
  const toast = document.createElement('div');
  toast.className = 'alert alert-success position-fixed top-0 end-0 m-3';
  toast.style.zIndex = '9999';
  toast.textContent = message;
  document.body.appendChild(toast);
  setTimeout(() => toast.remove(), 3000);
}

function escapeHtml(text) {
  if (!text) return '';
  const div = document.createElement('div');
  div.textContent = text;
  return div.innerHTML;
}

function formatFileSize(bytes) {
  if (bytes < 1024) return bytes + ' B';
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
  return (bytes / (1024 * 1024)).toFixed(1) + ' MB';
}

// Load project on page load
loadProject();
</script>

<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.10.0/font/bootstrap-icons.css">
