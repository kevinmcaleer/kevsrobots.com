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
  const IS_LOCAL = location.hostname === 'localhost'
    || location.hostname === '0.0.0.0'
    || location.hostname === 'local.kevsrobots.com';

  function apiFetch(url, opts) {
    return ProjectAuth.apiFetch(url, opts || {});
  }

  async function checkAuth() {
    try {
      const resp = await apiFetch(API + '/api/projects/my/list');
      if (resp.status === 401) {
        if (IS_LOCAL) {
          showDevLogin();
        } else {
          authRequired.classList.remove('d-none');
        }
        return false;
      }
      editorContainer.classList.remove('d-none');
      return true;
    } catch (e) {
      if (IS_LOCAL) {
        showDevLogin();
      } else {
        authRequired.classList.remove('d-none');
      }
      return false;
    }
  }

  function showDevLogin() {
    authRequired.innerHTML = `
      <div class="card p-3">
        <h5>Local Dev Login</h5>
        <p class="small text-muted">Secure cookies don't work over HTTP. Generate a token on the server:<br>
          <code>docker exec chatter-chatter-1 python -c "from app.utils import create_access_token; print(create_access_token({'sub': 'kev'}))"</code><br>
          Paste the token below.</p>
        <div class="input-group mb-2">
          <input type="text" id="dev-token-input" class="form-control form-control-sm" placeholder="Paste JWT token...">
          <button class="btn btn-sm btn-primary" onclick="(function(){ var t=document.getElementById('dev-token-input').value.trim(); if(t){localStorage.setItem('dev_jwt_token',t);location.reload();} })()">Set Token</button>
        </div>
        <small class="text-muted">Token is saved in localStorage and persists across reloads.</small>
      </div>
    `;
    authRequired.classList.remove('d-none');
  }

  // --- Load existing project ---
  async function loadProject() {
    if (!projectId) return;
    try {
      const resp = await apiFetch(API + '/api/projects/' + projectId, { credentials: 'include' });
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
        resp = await apiFetch(API + '/api/projects/' + currentProject.id, {
          method: 'PUT', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(data),
        });
      } else {
        resp = await apiFetch(API + '/api/projects', {
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

  // --- Save / Status / Delete ---
  document.getElementById('save-btn').addEventListener('click', saveProject);

  const statusSelect = document.getElementById('project-status');
  if (statusSelect) statusSelect.addEventListener('change', async () => {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    try {
      const resp = await apiFetch(API + '/api/projects/' + currentProject.id, {
        method: 'PUT', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ status: statusSelect.value }),
      });
      if (resp.ok) {
        currentProject = await resp.json();
        updateStatusBadge(currentProject.status);
        showSaveStatus('saved', 'Status updated');
      }
    } catch (e) { console.error(e); }
  });

  const deleteBtn = document.getElementById('delete-btn');
  if (deleteBtn) deleteBtn.addEventListener('click', async () => {
    if (!currentProject) return;
    if (!confirm('Delete this project? This cannot be undone.')) return;
    try {
      await apiFetch(API + '/api/projects/' + currentProject.id, {
        method: 'DELETE', credentials: 'include',
      });
      window.location.href = '/projects/hub';
    } catch (e) { console.error(e); }
  });

  const statusLabels = { wip: 'Work in Progress', completed: 'Completed', archived: 'Archived' };
  const statusColors = { wip: 'bg-info', completed: 'bg-success', archived: 'bg-secondary' };

  function updateStatusBadge(status) {
    statusBadge.textContent = statusLabels[status] || status;
    statusBadge.className = 'badge ' + (statusColors[status] || 'bg-secondary');
    if (statusSelect) statusSelect.value = status;
    const viewLiveBtn = document.getElementById('view-live-btn');
    const viewToggle = document.getElementById('view-toggle-btn');
    if (currentProject) {
      if (viewLiveBtn) {
        viewLiveBtn.style.display = '';
        viewLiveBtn.href = '/projects/view.html?id=' + currentProject.id;
      }
      if (viewToggle) {
        viewToggle.classList.remove('d-none');
        viewToggle.href = '/projects/view.html?id=' + currentProject.id;
      }
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

  document.getElementById('btn-preview').addEventListener('click', async () => {
    contentEditor.classList.add('d-none');
    contentPreview.classList.remove('d-none');
    contentPreview.innerHTML = marked.parse(contentEditor.value || '*No content yet*');
    document.getElementById('btn-preview').classList.add('active');
    document.getElementById('btn-write').classList.remove('active');

    // Render mermaid diagrams in preview if mermaid is available
    try {
      const mermaidMod = await import('https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.esm.min.mjs');
      const mermaid = mermaidMod.default;
      mermaid.initialize({ startOnLoad: false });
      contentPreview.querySelectorAll('pre code.language-mermaid').forEach((block) => {
        const pre = block.parentElement;
        const div = document.createElement('div');
        div.className = 'mermaid';
        div.textContent = block.textContent;
        pre.replaceWith(div);
      });
      await mermaid.run();
    } catch (e) {
      /* mermaid not available — diagrams show as code blocks, which is fine */
    }
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
        case 'diagram': /* handled by dropdown */ return;
        case 'circuit': /* handled by circuit editor launcher */ return;
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
        const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/files', {
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
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/files', { credentials: 'include' });
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
    await apiFetch(API + '/api/projects/' + currentProject.id + '/files/' + fileId, {
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
      await apiFetch(API + '/api/projects/' + currentProject.id + '/images', {
        method: 'POST', credentials: 'include', body: form,
      });
    }
    loadImages();
  }

  let imageOrder = [];

  async function loadImages() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/images', { credentials: 'include' });
    const images = await resp.json();
    imageOrder = images.map(img => img.id);
    const gallery = document.getElementById('image-gallery');
    gallery.innerHTML = images.map((img, idx) => {
      const imgUrl = API + '/api/projects/' + currentProject.id + '/images/' + img.id + '/view';
      return `
      <div class="col-4" data-image-id="${img.id}" draggable="true">
        <div class="image-thumb ${idx === 0 ? 'is-cover' : ''}">
          <img src="${imgUrl}" alt="${img.filename}" data-view-idx="${idx}" class="iv-clickable">
          <button class="delete-overlay" onclick="event.stopPropagation();deleteImage(${img.id})"><i class="fas fa-trash"></i></button>
          ${idx === 0 ? '<span class="cover-badge">★ Cover</span>' : ''}
        </div>
      </div>`;
    }).join('');

    // Set cover_image from first image
    if (images.length > 0) {
      const firstUrl = API + '/api/projects/' + currentProject.id + '/images/' + images[0].id + '/view';
      if (currentProject.cover_image !== firstUrl) {
        currentProject.cover_image = firstUrl;
        apiFetch(API + '/api/projects/' + currentProject.id, {
          method: 'PUT', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ cover_image: firstUrl }),
        });
      }
    }

    // Drag-and-drop reordering
    setupImageDrag(gallery);

    // Image viewer on click (only if not dragging)
    const viewerImages = images.map(img => ({
      url: API + '/api/projects/' + currentProject.id + '/images/' + img.id + '/view',
      alt: img.filename
    }));
    gallery.addEventListener('click', e => {
      const img = e.target.closest('.iv-clickable');
      if (!img) return;
      if (gallery.querySelector('.dragging')) return;
      ImageViewer.open(viewerImages, parseInt(img.dataset.viewIdx));
    });

    // Populate image picker dropdown
    const pickerList = document.getElementById('image-picker-list');
    if (pickerList) {
      if (images.length === 0) {
        pickerList.innerHTML = '<small class="text-muted">No images uploaded yet</small>';
      } else {
        pickerList.innerHTML = images.map(img => {
          const imgUrl = API + '/api/projects/' + currentProject.id + '/images/' + img.id + '/view';
          return `<div class="d-flex align-items-center gap-2 p-1 rounded image-picker-item" style="cursor:pointer" data-img-url="${imgUrl}" data-img-name="${img.filename}">
            <img src="${imgUrl}" style="width:36px;height:36px;object-fit:cover;border-radius:4px">
            <small class="text-truncate" style="max-width:140px">${img.filename}</small>
          </div>`;
        }).join('');
        pickerList.querySelectorAll('.image-picker-item').forEach(item => {
          item.addEventListener('click', () => {
            const url = item.dataset.imgUrl;
            const name = item.dataset.imgName;
            const ta = contentEditor;
            const pos = ta.selectionStart;
            const insert = '![' + name + '](' + url + ')\n';
            ta.value = ta.value.substring(0, pos) + insert + ta.value.substring(ta.selectionEnd);
            ta.focus();
            ta.selectionStart = ta.selectionEnd = pos + insert.length;
            scheduleAutoSave();
            // Close the dropdown
            bootstrap.Dropdown.getOrCreateInstance(item.closest('.dropdown-menu').previousElementSibling).hide();
          });
        });
      }
    }
  }

  function setupImageDrag(gallery) {
    let dragEl = null;
    let dragId = null;
    let ghostEl = null;
    let offsetX = 0, offsetY = 0;

    function getDropTarget(x, y) {
      const items = gallery.querySelectorAll('[data-image-id]');
      for (const item of items) {
        if (item === dragEl) continue;
        const r = item.getBoundingClientRect();
        if (x >= r.left && x <= r.right && y >= r.top && y <= r.bottom) return item;
      }
      return null;
    }

    function onMouseMove(e) {
      if (!ghostEl) return;
      ghostEl.style.left = (e.clientX - offsetX) + 'px';
      ghostEl.style.top = (e.clientY - offsetY) + 'px';

      const target = getDropTarget(e.clientX, e.clientY);
      const items = Array.from(gallery.querySelectorAll('[data-image-id]'));
      items.forEach(item => item.classList.remove('drag-over', 'drag-shift-left', 'drag-shift-right'));

      if (target) {
        target.classList.add('drag-over');
        const fromIdx = items.indexOf(dragEl);
        const toIdx = items.indexOf(target);
        items.forEach((item, i) => {
          if (item === dragEl) return;
          if (fromIdx < toIdx && i > fromIdx && i <= toIdx) item.classList.add('drag-shift-left');
          else if (fromIdx > toIdx && i >= toIdx && i < fromIdx) item.classList.add('drag-shift-right');
        });
      }
    }

    async function onMouseUp(e) {
      document.removeEventListener('mousemove', onMouseMove);
      document.removeEventListener('mouseup', onMouseUp);
      if (ghostEl) { ghostEl.remove(); ghostEl = null; }
      if (!dragEl) return;

      dragEl.classList.remove('dragging');
      gallery.querySelectorAll('.drag-over, .drag-shift-left, .drag-shift-right').forEach(d => {
        d.classList.remove('drag-over', 'drag-shift-left', 'drag-shift-right');
      });

      const target = getDropTarget(e.clientX, e.clientY);
      if (target && dragId) {
        const dropId = target.dataset.imageId;
        if (dropId && dragId !== dropId) {
          const fromIdx = imageOrder.indexOf(parseInt(dragId));
          const toIdx = imageOrder.indexOf(parseInt(dropId));
          if (fromIdx > -1 && toIdx > -1) {
            imageOrder.splice(fromIdx, 1);
            imageOrder.splice(toIdx, 0, parseInt(dragId));
            for (let i = 0; i < imageOrder.length; i++) {
              await apiFetch(API + '/api/projects/' + currentProject.id + '/images/' + imageOrder[i], {
                method: 'PUT', credentials: 'include',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ sort_order: i }),
              });
            }
            showSaveStatus('saved', 'Image order updated');
            loadImages();
          }
        }
      }
      dragEl = null;
      dragId = null;
    }

    gallery.querySelectorAll('[data-image-id]').forEach(el => {
      el.setAttribute('draggable', 'false');
      el.addEventListener('mousedown', e => {
        if (e.target.closest('button')) return;
        e.preventDefault();
        dragEl = el;
        dragId = el.dataset.imageId;

        const rect = el.getBoundingClientRect();
        offsetX = e.clientX - rect.left;
        offsetY = e.clientY - rect.top;

        // Create ghost
        ghostEl = el.cloneNode(true);
        ghostEl.style.position = 'fixed';
        ghostEl.style.left = (e.clientX - offsetX) + 'px';
        ghostEl.style.top = (e.clientY - offsetY) + 'px';
        ghostEl.style.width = rect.width + 'px';
        ghostEl.style.zIndex = '10000';
        ghostEl.style.pointerEvents = 'none';
        ghostEl.style.transform = 'rotate(4deg) scale(1.05)';
        ghostEl.style.opacity = '0.9';
        ghostEl.style.boxShadow = '0 8px 25px rgba(0,0,0,0.3)';
        ghostEl.style.borderRadius = '6px';
        ghostEl.style.transition = 'none';
        document.body.appendChild(ghostEl);

        el.classList.add('dragging');
        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);
      });
    });
  }

  window.deleteImage = async function(imageId) {
    await apiFetch(API + '/api/projects/' + currentProject.id + '/images/' + imageId, {
      method: 'DELETE', credentials: 'include',
    });
    loadImages();
  };

  // --- BOM ---
  document.getElementById('add-bom-btn').addEventListener('click', async () => {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    await apiFetch(API + '/api/projects/' + currentProject.id + '/bom', {
      method: 'POST', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name: '', quantity: 1, unit: 'qty', unit_cost: 0 }),
    });
    loadBOM();
  });

  async function loadBOM() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/bom', { credentials: 'include' });
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
    await apiFetch(API + '/api/projects/' + currentProject.id + '/bom/' + itemId, {
      method: 'PUT', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data),
    });
    loadBOM();
  };

  window.deleteBOM = async function(itemId) {
    await apiFetch(API + '/api/projects/' + currentProject.id + '/bom/' + itemId, {
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
    await apiFetch(API + '/api/projects/' + currentProject.id + '/links', {
      method: 'POST', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ title, url, link_type: type }),
    });
    loadLinks();
  });

  async function loadLinks() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/links', { credentials: 'include' });
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
    await apiFetch(API + '/api/projects/' + currentProject.id + '/links/' + linkId, {
      method: 'DELETE', credentials: 'include',
    });
    loadLinks();
  };

  // --- Journal ---
  const journalInput = document.getElementById('journal-input');

  journalInput.addEventListener('keydown', async (e) => {
    if (e.key !== 'Enter') return;
    e.preventDefault();
    const text = journalInput.value.trim();
    if (!text) return;
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    journalInput.disabled = true;
    await apiFetch(API + '/api/projects/' + currentProject.id + '/journal', {
      method: 'POST', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ title: text, status: 'in_progress' }),
    });
    journalInput.value = '';
    journalInput.disabled = false;
    journalInput.focus();
    loadJournal();
  });

  function formatTimeAgo(dateStr) {
    const diff = Date.now() - new Date(dateStr).getTime();
    const mins = Math.floor(diff / 60000);
    if (mins < 1) return 'just now';
    if (mins < 60) return mins + 'm ago';
    const hrs = Math.floor(mins / 60);
    if (hrs < 24) return hrs + 'h ago';
    const days = Math.floor(hrs / 24);
    if (days < 7) return days + 'd ago';
    return new Date(dateStr).toLocaleDateString();
  }

  async function loadJournal() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/journal', { credentials: 'include' });
    const entries = await resp.json();
    document.getElementById('journal-list').innerHTML = entries.map(e => `
      <div class="journal-entry" data-entry-id="${e.id}">
        <div class="d-flex justify-content-between align-items-start">
          <span class="journal-text">${e.title}</span>
          <span class="journal-actions">
            <button class="edit-journal" onclick="editJournal(${e.id}, this)" title="Edit"><i class="fas fa-pencil-alt"></i></button>
            <button class="delete-journal" onclick="deleteJournal(${e.id})" title="Delete"><i class="fas fa-times"></i></button>
          </span>
        </div>
        <span class="journal-date">${formatTimeAgo(e.created_at)}</span>
      </div>
    `).join('');
  }

  window.deleteJournal = async function(entryId) {
    await apiFetch(API + '/api/projects/' + currentProject.id + '/journal/' + entryId, {
      method: 'DELETE', credentials: 'include',
    });
    loadJournal();
  };

  window.editJournal = function(entryId, btn) {
    const entry = btn.closest('.journal-entry');
    const textEl = entry.querySelector('.journal-text');
    const current = textEl.textContent;
    textEl.innerHTML = '<input type="text" class="journal-edit-input" value="' + current.replace(/"/g, '&quot;') + '">';
    const input = textEl.querySelector('input');
    input.focus();
    input.select();

    async function save() {
      const newText = input.value.trim();
      if (newText && newText !== current) {
        await apiFetch(API + '/api/projects/' + currentProject.id + '/journal/' + entryId, {
          method: 'PUT', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ title: newText, status: 'in_progress' }),
        });
      }
      loadJournal();
    }

    input.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); save(); }
      if (e.key === 'Escape') loadJournal();
    });
    input.addEventListener('blur', save);
  };

  // --- Auto-save on input ---
  [titleInput, descInput, contentEditor, difficultySelect, timeInput, repoInput].forEach(el => {
    el.addEventListener('input', scheduleAutoSave);
    el.addEventListener('change', scheduleAutoSave);
  });

  // --- Diagram toolbar dropdown & Circuit Editor integration ---
  (function setupDiagramTools() {
    const toolbar = document.querySelector('.card-header .editor-tool[data-action="list"]');
    if (!toolbar) return;
    const toolbarRow = toolbar.parentElement;

    // Mermaid diagram templates
    const mermaidTemplates = {
      flowchart: '```mermaid\ngraph TD\n    A[Start] --> B[Step 1]\n    B --> C[Step 2]\n    C --> D[End]\n```\n',
      sequence: '```mermaid\nsequenceDiagram\n    participant A as Device\n    participant B as Server\n    A->>B: Send data\n    B-->>A: Acknowledge\n```\n',
      state: '```mermaid\nstateDiagram-v2\n    [*] --> Idle\n    Idle --> Running: start\n    Running --> Idle: stop\n    Running --> Error: fault\n    Error --> Idle: reset\n```\n',
      gantt: '```mermaid\ngantt\n    title Project Timeline\n    dateFormat  YYYY-MM-DD\n    section Build\n    Design           :a1, 2025-01-01, 7d\n    Assembly         :a2, after a1, 5d\n    section Test\n    Testing          :a3, after a2, 3d\n```\n',
    };

    // Insert text into the markdown editor
    function insertAtCursor(text) {
      const ta = contentEditor;
      const start = ta.selectionStart;
      const end = ta.selectionEnd;
      ta.value = ta.value.substring(0, start) + text + ta.value.substring(end);
      ta.focus();
      ta.selectionStart = ta.selectionEnd = start + text.length;
      scheduleAutoSave();
    }

    // Build the diagram dropdown wrapper
    const wrapper = document.createElement('span');
    wrapper.className = 'diagram-dropdown';
    wrapper.style.position = 'relative';
    wrapper.style.display = 'inline-block';

    // Diagram button
    const diagramBtn = document.createElement('button');
    diagramBtn.className = 'btn btn-sm btn-outline-secondary me-1';
    diagramBtn.title = 'Insert diagram';
    diagramBtn.innerHTML = '<i class="fas fa-project-diagram"></i>';
    wrapper.appendChild(diagramBtn);

    // Dropdown menu
    const menu = document.createElement('div');
    menu.className = 'diagram-dropdown-menu';
    menu.innerHTML = `
      <button data-tpl="flowchart"><i class="fas fa-sitemap"></i> Flowchart</button>
      <button data-tpl="sequence"><i class="fas fa-exchange-alt"></i> Sequence Diagram</button>
      <button data-tpl="state"><i class="fas fa-random"></i> State Diagram</button>
      <button data-tpl="gantt"><i class="fas fa-tasks"></i> Gantt Chart</button>
      <div class="dropdown-divider"></div>
      <button data-tpl="circuit"><i class="fas fa-microchip"></i> Circuit Diagram</button>
    `;
    wrapper.appendChild(menu);

    // Toggle dropdown on click
    diagramBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      menu.classList.toggle('show');
    });

    // Close dropdown when clicking elsewhere
    document.addEventListener('click', () => menu.classList.remove('show'));

    // Handle template selection
    menu.querySelectorAll('button').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        menu.classList.remove('show');
        const tpl = btn.dataset.tpl;

        if (tpl === 'circuit') {
          // Open circuit editor
          openCircuitEditor();
          return;
        }

        if (mermaidTemplates[tpl]) {
          insertAtCursor('\n' + mermaidTemplates[tpl] + '\n');
        }
      });
    });

    // Insert after the list button
    toolbar.after(wrapper);

    // ---- Circuit Editor integration ----
    function openCircuitEditor() {
      if (typeof window.CircuitEditor === 'undefined') {
        alert('Circuit editor is still loading. Please try again in a moment.');
        return;
      }

      const editor = new window.CircuitEditor();
      editor.open({
        onInsert: function (svgString) {
          // Insert SVG as an HTML block in the markdown
          const svgBlock = '\n\n<div class="circuit-diagram">\n' + svgString + '\n</div>\n\n';
          insertAtCursor(svgBlock);
        },
      });
    }
  })();

  // --- Init ---
  async function init() {
    const authed = await checkAuth();
    if (!authed) return;
    if (projectId) await loadProject();
    editorContainer.classList.remove('d-none');
  }

  init();
})();
