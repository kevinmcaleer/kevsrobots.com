/**
 * Project Editor — vanilla JS for creating/editing projects.
 * Talks to the Projects API at projects.kevsrobots.com.
 */
(function () {
  const API = 'https://projects.kevsrobots.com';
  const params = new URLSearchParams(window.location.search);
  const projectId = params.get('id');
  let currentProject = null;
  let autoSaveTimer = null;

  // DOM elements
  const editorContainer = document.getElementById('editor-container');
  const authRequired = document.getElementById('auth-required');
  const titleInput = document.getElementById('project-title');
  const descInput = document.getElementById('project-description');
  const contentEditor = document.getElementById('content-editor');
  const contentPreview = document.getElementById('content-preview');
  const difficultySelect = document.getElementById('project-difficulty');
  const timeInput = document.getElementById('project-time');
  const repoInput = document.getElementById('project-repo');
  const tagsContainer = document.getElementById('tags-container');
  const tagInput = document.getElementById('tag-input');
  const statusBadge = document.getElementById('status-badge');
  const saveIndicator = document.getElementById('save-indicator');

  // --- Auth ---
  function getAuthHeaders() {
    // Chatter sets cookies on .kevsrobots.com, they're sent automatically.
    // For API calls we also check localStorage as fallback.
    const token = document.cookie.split(';').find(c => c.trim().startsWith('access_token='));
    if (token) return {};  // Cookie will be sent automatically
    return {};
  }

  async function checkAuth() {
    try {
      const resp = await fetch(API + '/api/projects/my/list', { credentials: 'include' });
      if (resp.status === 401) {
        authRequired.classList.remove('d-none');
        return false;
      }
      editorContainer.classList.remove('d-none');
      return true;
    } catch (e) {
      authRequired.classList.remove('d-none');
      return false;
    }
  }

  // --- Load existing project ---
  async function loadProject() {
    if (!projectId) return;
    try {
      const resp = await fetch(API + '/api/projects/' + projectId, { credentials: 'include' });
      if (!resp.ok) return;
      currentProject = await resp.json();
      titleInput.value = currentProject.title || '';
      descInput.value = currentProject.short_description || '';
      contentEditor.value = currentProject.content_md || '';
      difficultySelect.value = currentProject.difficulty || '';
      timeInput.value = currentProject.estimated_minutes || '';
      repoInput.value = currentProject.code_repo_url || '';
      updateStatusBadge(currentProject.status);
      renderTags(currentProject.tags || []);
      loadBOM();
      loadLinks();
      loadFiles();
      loadImages();
      loadJournal();
    } catch (e) {
      console.error('Failed to load project:', e);
    }
  }

  // --- Save ---
  function showSaveStatus(status, msg) {
    saveIndicator.className = 'save-' + status;
    saveIndicator.textContent = msg;
    if (status === 'saved') setTimeout(() => { saveIndicator.textContent = ''; }, 2000);
  }

  async function saveProject() {
    const data = {
      title: titleInput.value || 'Untitled Project',
      short_description: descInput.value || null,
      content_md: contentEditor.value || null,
      difficulty: difficultySelect.value || null,
      estimated_minutes: timeInput.value ? parseInt(timeInput.value) : null,
      code_repo_url: repoInput.value || null,
      tags: getCurrentTags(),
    };

    showSaveStatus('saving', 'Saving...');
    try {
      let resp;
      if (currentProject) {
        resp = await fetch(API + '/api/projects/' + currentProject.id, {
          method: 'PUT', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(data),
        });
      } else {
        resp = await fetch(API + '/api/projects', {
          method: 'POST', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(data),
        });
      }
      if (!resp.ok) throw new Error(await resp.text());
      currentProject = await resp.json();
      if (!projectId) {
        history.replaceState(null, '', '?id=' + currentProject.id);
      }
      updateStatusBadge(currentProject.status);
      showSaveStatus('saved', 'Saved');
    } catch (e) {
      console.error('Save failed:', e);
      showSaveStatus('error', 'Save failed');
    }
  }

  function scheduleAutoSave() {
    if (autoSaveTimer) clearTimeout(autoSaveTimer);
    autoSaveTimer = setTimeout(saveProject, 3000);
  }

  // --- Publish / Delete ---
  document.getElementById('save-btn').addEventListener('click', saveProject);

  document.getElementById('publish-btn').addEventListener('click', async () => {
    if (!currentProject) await saveProject();
    if (!currentProject) return;
    try {
      const resp = await fetch(API + '/api/projects/' + currentProject.id, {
        method: 'PUT', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ status: 'published' }),
      });
      if (resp.ok) {
        currentProject = await resp.json();
        updateStatusBadge('published');
        showSaveStatus('saved', 'Published!');
      }
    } catch (e) { console.error(e); }
  });

  document.getElementById('unpublish-btn').addEventListener('click', async () => {
    if (!currentProject) return;
    try {
      const resp = await fetch(API + '/api/projects/' + currentProject.id, {
        method: 'PUT', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ status: 'draft' }),
      });
      if (resp.ok) {
        currentProject = await resp.json();
        updateStatusBadge('draft');
        showSaveStatus('saved', 'Reverted to draft');
      }
    } catch (e) { console.error(e); }
  });

  document.getElementById('delete-btn').addEventListener('click', async () => {
    if (!currentProject) return;
    if (!confirm('Delete this project? This cannot be undone.')) return;
    try {
      await fetch(API + '/api/projects/' + currentProject.id, {
        method: 'DELETE', credentials: 'include',
      });
      window.location.href = '/projects/hub';
    } catch (e) { console.error(e); }
  });

  function updateStatusBadge(status) {
    statusBadge.textContent = status === 'published' ? 'Published' : 'Draft';
    statusBadge.className = 'badge ' + (status === 'published' ? 'bg-success' : 'bg-secondary');
    const publishBtn = document.getElementById('publish-btn');
    const unpublishBtn = document.getElementById('unpublish-btn');
    const viewLiveBtn = document.getElementById('view-live-btn');
    if (status === 'published') {
      publishBtn.style.display = 'none';
      unpublishBtn.style.display = '';
      if (currentProject) {
        viewLiveBtn.style.display = '';
        viewLiveBtn.href = '/projects/view.html?id=' + currentProject.id;
      }
    } else {
      publishBtn.style.display = '';
      unpublishBtn.style.display = 'none';
      viewLiveBtn.style.display = 'none';
    }
  }

  // --- Tags ---
  function getCurrentTags() {
    return Array.from(tagsContainer.querySelectorAll('.tag-badge'))
      .map(el => el.dataset.tag);
  }

  function renderTags(tags) {
    tagsContainer.innerHTML = '';
    tags.forEach(addTagBadge);
  }

  function addTagBadge(tag) {
    const el = document.createElement('span');
    el.className = 'tag-badge';
    el.dataset.tag = tag;
    el.innerHTML = tag + ' <span class="remove-tag">&times;</span>';
    el.querySelector('.remove-tag').addEventListener('click', () => {
      el.remove();
      scheduleAutoSave();
    });
    tagsContainer.appendChild(el);
  }

  document.getElementById('add-tag-btn').addEventListener('click', () => {
    const tag = tagInput.value.trim().toLowerCase();
    if (tag && !getCurrentTags().includes(tag)) {
      addTagBadge(tag);
      tagInput.value = '';
      scheduleAutoSave();
    }
  });

  tagInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      document.getElementById('add-tag-btn').click();
    }
  });

  // --- Markdown Editor ---
  document.getElementById('btn-write').addEventListener('click', () => {
    contentEditor.classList.remove('d-none');
    contentPreview.classList.add('d-none');
    document.getElementById('btn-write').classList.add('active');
    document.getElementById('btn-preview').classList.remove('active');
  });

  document.getElementById('btn-preview').addEventListener('click', () => {
    contentEditor.classList.add('d-none');
    contentPreview.classList.remove('d-none');
    contentPreview.innerHTML = marked.parse(contentEditor.value || '*No content yet*');
    document.getElementById('btn-preview').classList.add('active');
    document.getElementById('btn-write').classList.remove('active');
  });

  // Toolbar actions
  document.querySelectorAll('.editor-tool').forEach(btn => {
    btn.addEventListener('click', () => {
      const action = btn.dataset.action;
      const ta = contentEditor;
      const start = ta.selectionStart;
      const end = ta.selectionEnd;
      const sel = ta.value.substring(start, end);
      let insert = '';

      switch (action) {
        case 'bold': insert = '**' + (sel || 'bold text') + '**'; break;
        case 'italic': insert = '*' + (sel || 'italic text') + '*'; break;
        case 'heading': insert = '\n## ' + (sel || 'Heading') + '\n'; break;
        case 'code': insert = '\n```python\n' + (sel || '# code here') + '\n```\n'; break;
        case 'link': insert = '[' + (sel || 'link text') + '](https://)'; break;
        case 'image': insert = '![' + (sel || 'alt text') + '](image-url)'; break;
        case 'list': insert = '\n- ' + (sel || 'item') + '\n'; break;
      }

      ta.value = ta.value.substring(0, start) + insert + ta.value.substring(end);
      ta.focus();
      ta.selectionStart = ta.selectionEnd = start + insert.length;
      scheduleAutoSave();
    });
  });

  // Keyboard shortcuts
  contentEditor.addEventListener('keydown', (e) => {
    if ((e.ctrlKey || e.metaKey) && e.key === 'b') {
      e.preventDefault();
      document.querySelector('[data-action="bold"]').click();
    }
    if ((e.ctrlKey || e.metaKey) && e.key === 'i') {
      e.preventDefault();
      document.querySelector('[data-action="italic"]').click();
    }
    if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
      e.preventDefault();
      document.querySelector('[data-action="link"]').click();
    }
  });

  // --- File Upload ---
  const fileDropzone = document.getElementById('file-dropzone');
  const fileInput = document.getElementById('file-input');

  ['dragenter', 'dragover'].forEach(evt => {
    fileDropzone.addEventListener(evt, e => { e.preventDefault(); fileDropzone.classList.add('dragover'); });
  });
  ['dragleave', 'drop'].forEach(evt => {
    fileDropzone.addEventListener(evt, e => { e.preventDefault(); fileDropzone.classList.remove('dragover'); });
  });

  fileDropzone.addEventListener('drop', e => uploadFiles(e.dataTransfer.files));
  fileInput.addEventListener('change', e => uploadFiles(e.target.files));

  async function uploadFiles(fileList) {
    if (!currentProject) {
      await saveProject();
      if (!currentProject) return;
    }
    for (const file of fileList) {
      const form = new FormData();
      form.append('file', file);
      try {
        const resp = await fetch(API + '/api/projects/' + currentProject.id + '/files', {
          method: 'POST', credentials: 'include', body: form,
        });
        if (!resp.ok) {
          const err = await resp.json();
          alert(err.detail || 'Upload failed');
          continue;
        }
      } catch (e) { console.error(e); }
    }
    loadFiles();
  }

  async function loadFiles() {
    if (!currentProject) return;
    const resp = await fetch(API + '/api/projects/' + currentProject.id + '/files', { credentials: 'include' });
    const files = await resp.json();
    const list = document.getElementById('file-list');
    list.innerHTML = files.map(f => `
      <div class="file-item">
        <div class="file-info">
          <i class="fas fa-file text-muted"></i>
          <a href="${API}/api/projects/${currentProject.id}/files/${f.id}/download">${f.filename}</a>
          <span class="file-size">${(f.file_size / 1024).toFixed(1)} KB</span>
        </div>
        <button class="btn btn-sm btn-outline-danger" onclick="deleteFile(${f.id})"><i class="fas fa-trash"></i></button>
      </div>
    `).join('');
  }

  window.deleteFile = async function(fileId) {
    await fetch(API + '/api/projects/' + currentProject.id + '/files/' + fileId, {
      method: 'DELETE', credentials: 'include',
    });
    loadFiles();
  };

  // --- Image Upload ---
  const imageInput = document.getElementById('image-input');
  const imageDropzone = document.getElementById('image-dropzone');

  ['dragenter', 'dragover'].forEach(evt => {
    imageDropzone.addEventListener(evt, e => { e.preventDefault(); imageDropzone.classList.add('dragover'); });
  });
  ['dragleave', 'drop'].forEach(evt => {
    imageDropzone.addEventListener(evt, e => { e.preventDefault(); imageDropzone.classList.remove('dragover'); });
  });

  imageDropzone.addEventListener('drop', e => uploadImages(e.dataTransfer.files));
  imageInput.addEventListener('change', e => uploadImages(e.target.files));

  async function uploadImages(fileList) {
    if (!currentProject) {
      await saveProject();
      if (!currentProject) return;
    }
    for (const file of fileList) {
      const form = new FormData();
      form.append('file', file);
      await fetch(API + '/api/projects/' + currentProject.id + '/images', {
        method: 'POST', credentials: 'include', body: form,
      });
    }
    loadImages();
  }

  async function loadImages() {
    if (!currentProject) return;
    const resp = await fetch(API + '/api/projects/' + currentProject.id + '/images', { credentials: 'include' });
    const images = await resp.json();
    const gallery = document.getElementById('image-gallery');
    gallery.innerHTML = images.map(img => `
      <div class="col-4">
        <div class="image-thumb">
          <img src="${API}/api/projects/${currentProject.id}/images/${img.id}/view" alt="${img.filename}">
          <button class="delete-overlay" onclick="deleteImage(${img.id})"><i class="fas fa-trash"></i></button>
        </div>
      </div>
    `).join('');
  }

  window.deleteImage = async function(imageId) {
    await fetch(API + '/api/projects/' + currentProject.id + '/images/' + imageId, {
      method: 'DELETE', credentials: 'include',
    });
    loadImages();
  };

  // --- BOM ---
  document.getElementById('add-bom-btn').addEventListener('click', async () => {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    await fetch(API + '/api/projects/' + currentProject.id + '/bom', {
      method: 'POST', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name: '', quantity: 1, unit: 'qty', unit_cost: 0 }),
    });
    loadBOM();
  });

  async function loadBOM() {
    if (!currentProject) return;
    const resp = await fetch(API + '/api/projects/' + currentProject.id + '/bom', { credentials: 'include' });
    const items = await resp.json();
    const tbody = document.getElementById('bom-body');
    tbody.innerHTML = items.map(item => `
      <tr data-bom-id="${item.id}">
        <td><input value="${item.name}" data-field="name" onchange="updateBOM(${item.id}, this)"></td>
        <td><input type="number" value="${item.quantity}" data-field="quantity" style="width:50px" onchange="updateBOM(${item.id}, this)"></td>
        <td><input value="${item.unit}" data-field="unit" style="width:50px" onchange="updateBOM(${item.id}, this)"></td>
        <td><input type="number" step="0.01" value="${item.unit_cost || 0}" data-field="unit_cost" style="width:70px" onchange="updateBOM(${item.id}, this)"></td>
        <td><input value="${item.supplier_url || ''}" data-field="supplier_url" onchange="updateBOM(${item.id}, this)"></td>
        <td><button class="btn btn-sm text-danger" onclick="deleteBOM(${item.id})"><i class="fas fa-times"></i></button></td>
      </tr>
    `).join('');
    updateBOMTotal(items);
  }

  function updateBOMTotal(items) {
    const total = items.reduce((sum, i) => sum + (i.quantity * (i.unit_cost || 0)), 0);
    document.getElementById('bom-total').textContent = '£' + total.toFixed(2);
  }

  window.updateBOM = async function(itemId, input) {
    const row = input.closest('tr');
    const data = {};
    row.querySelectorAll('input').forEach(inp => {
      const field = inp.dataset.field;
      let val = inp.value;
      if (field === 'quantity') val = parseInt(val) || 1;
      else if (field === 'unit_cost') val = parseFloat(val) || 0;
      data[field] = val;
    });
    await fetch(API + '/api/projects/' + currentProject.id + '/bom/' + itemId, {
      method: 'PUT', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data),
    });
    loadBOM();
  };

  window.deleteBOM = async function(itemId) {
    await fetch(API + '/api/projects/' + currentProject.id + '/bom/' + itemId, {
      method: 'DELETE', credentials: 'include',
    });
    loadBOM();
  };

  // --- Links ---
  document.getElementById('add-link-btn').addEventListener('click', async () => {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    const title = prompt('Link title:');
    if (!title) return;
    const url = prompt('URL:');
    if (!url) return;
    const type = prompt('Type (article/video/tutorial/documentation/other):', 'article') || 'article';
    await fetch(API + '/api/projects/' + currentProject.id + '/links', {
      method: 'POST', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ title, url, link_type: type }),
    });
    loadLinks();
  });

  async function loadLinks() {
    if (!currentProject) return;
    const resp = await fetch(API + '/api/projects/' + currentProject.id + '/links', { credentials: 'include' });
    const links = await resp.json();
    const typeIcons = { article: 'fa-newspaper', video: 'fa-play-circle', tutorial: 'fa-graduation-cap', documentation: 'fa-book', other: 'fa-external-link-alt' };
    document.getElementById('links-list').innerHTML = links.map(l => `
      <div class="link-item">
        <div>
          <span class="link-type-icon"><i class="fas ${typeIcons[l.link_type] || typeIcons.other}"></i></span>
          <a href="${l.url}" target="_blank">${l.title}</a>
        </div>
        <button class="btn btn-sm text-danger" onclick="deleteLink(${l.id})"><i class="fas fa-times"></i></button>
      </div>
    `).join('') || '<p class="text-muted small">No links yet</p>';
  }

  window.deleteLink = async function(linkId) {
    await fetch(API + '/api/projects/' + currentProject.id + '/links/' + linkId, {
      method: 'DELETE', credentials: 'include',
    });
    loadLinks();
  };

  // --- Journal ---
  document.getElementById('add-journal-btn').addEventListener('click', async () => {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    const title = prompt('Journal entry title:');
    if (!title) return;
    await fetch(API + '/api/projects/' + currentProject.id + '/journal', {
      method: 'POST', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ title, status: 'in_progress' }),
    });
    loadJournal();
  });

  async function loadJournal() {
    if (!currentProject) return;
    const resp = await fetch(API + '/api/projects/' + currentProject.id + '/journal', { credentials: 'include' });
    const entries = await resp.json();
    const statusLabels = { planning: 'Planning', in_progress: 'In Progress', completed: 'Completed' };
    document.getElementById('journal-list').innerHTML = entries.map(e => `
      <div class="journal-entry">
        <div class="d-flex justify-content-between align-items-center">
          <strong class="small">${e.title}</strong>
          <span class="journal-status ${e.status}">${statusLabels[e.status]}</span>
        </div>
        <small class="text-muted">${new Date(e.created_at).toLocaleDateString()}</small>
      </div>
    `).join('') || '<p class="text-muted small">No journal entries yet</p>';
  }

  // --- Auto-save on input ---
  [titleInput, descInput, contentEditor, difficultySelect, timeInput, repoInput].forEach(el => {
    el.addEventListener('input', scheduleAutoSave);
    el.addEventListener('change', scheduleAutoSave);
  });

  // --- Init ---
  async function init() {
    const authed = await checkAuth();
    if (!authed) return;
    if (projectId) await loadProject();
    editorContainer.classList.remove('d-none');
  }

  init();
})();
