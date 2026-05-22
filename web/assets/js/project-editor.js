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
  // Content-mode toggle (Markdown vs Instruction Builder).
  const contentModeBtns = document.querySelectorAll('.content-mode-btn');
  const contentModePaneMarkdown = document.getElementById('content-mode-pane-markdown');
  const contentModePaneBuilder = document.getElementById('content-mode-pane-builder');
  const editorInstructionsSection = document.getElementById('editor-instructions-section');
  const openInstructionBuilderLink = document.getElementById('open-instruction-builder');

  // --- Content-mode toggle (Markdown vs Instruction Builder) ---
  // Reflects the saved choice into the UI, swapping which content
  // surface is visible. Triggered on load (from the project payload)
  // and on user click (PUTs content_mode back to the server).
  function applyContentMode(mode) {
    var m = (mode === 'builder') ? 'builder' : 'markdown';
    contentModeBtns.forEach(function (btn) {
      var isActive = btn.dataset.contentMode === m;
      btn.classList.toggle('is-active', isActive);
      btn.setAttribute('aria-selected', isActive ? 'true' : 'false');
    });
    if (contentModePaneMarkdown) {
      contentModePaneMarkdown.classList.toggle('d-none', m !== 'markdown');
    }
    if (contentModePaneBuilder) {
      contentModePaneBuilder.classList.toggle('d-none', m !== 'builder');
    }
    // The legacy in-page "Build Instructions" lite-stepper (issue #178
    // phase 0) belongs to the markdown surface — hide it in builder
    // mode so the user has one clear path. The full builder
    // supersedes the lite-stepper.
    if (editorInstructionsSection) {
      editorInstructionsSection.classList.toggle('d-none', m !== 'markdown');
    }
    // Builder mode collapses the editor's TOC / sidebar columns so
    // the preview reads at a comfortable width. The CSS hook is
    // ``body.is-builder-mode``; project-editor.css handles the
    // column-width overrides.
    document.body.classList.toggle('is-builder-mode', m === 'builder');
    // Render the read-only preview of the project's builder steps so
    // the author sees exactly what visitors will see. Authoring
    // happens in the focused workspace — the link is at the top of
    // the pane.
    if (m === 'builder') renderBuilderPreview();
  }

  function renderBuilderPreview() {
    if (!currentProject || !currentProject.id) return;
    if (openInstructionBuilderLink) {
      openInstructionBuilderLink.href =
        '/projects/instructions/edit.html?id=' + encodeURIComponent(currentProject.id);
    }
    var surface = document.getElementById('builder-preview-surface');
    if (!surface) return;
    if (window.BuilderRenderer && typeof window.BuilderRenderer.render === 'function') {
      window.BuilderRenderer.render(surface, currentProject.id);
    } else {
      surface.innerHTML =
        '<div class="alert alert-warning small">' +
        '  Preview library failed to load — refresh the page to retry.' +
        '</div>';
    }
  }

  async function persistContentMode(mode) {
    if (!currentProject) return;
    try {
      var resp = await apiFetch(API + '/api/projects/' + currentProject.id, {
        method: 'PUT', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ content_mode: mode }),
      });
      if (resp.ok) {
        currentProject = await resp.json();
      }
    } catch (e) {
      console.error('Failed to persist content_mode:', e);
    }
  }

  contentModeBtns.forEach(function (btn) {
    btn.addEventListener('click', async function () {
      var mode = btn.dataset.contentMode;
      applyContentMode(mode);
      if (!currentProject) {
        // No row yet — force a save so the project gets an id; the
        // active content_mode rides along via getActiveContentMode().
        await saveProject();
        if (currentProject && openInstructionBuilderLink) {
          openInstructionBuilderLink.href =
            '/projects/instructions/edit.html?id=' + encodeURIComponent(currentProject.id);
        }
        // Newly-saved project — kick off the preview now that we
        // have an id (applyContentMode ran before saveProject so it
        // bailed early with no id).
        if (mode === 'builder') renderBuilderPreview();
      } else {
        persistContentMode(mode);
      }
    });
  });

  // The "Open in focused workspace" link is target=_self by default
  // (no target attr) so the user comes back via the in-builder Back
  // link. When they return, browsers either replay this page from
  // bfcache (pageshow with persisted=true) or do a fresh load.
  // Either way we want to re-render the preview so the latest
  // authored steps are reflected.
  window.addEventListener('pageshow', function (e) {
    if (e.persisted && document.body.classList.contains('is-builder-mode')) {
      renderBuilderPreview();
    }
  });

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
        <p class="small text-muted">Secure cookies don't work over HTTP. Generate a token on the server.</p>
        <p class="small text-muted mb-1"><strong>Long-lived token (recommended — 30 days, survives Jekyll rebuilds):</strong><br>
          <code>docker exec chatter-chatter-1 python -c "from datetime import timedelta; from app.utils import create_access_token; print(create_access_token({'sub': 'kev'}, expires_delta=timedelta(days=30)))"</code>
        </p>
        <p class="small text-muted mb-1"><strong>Short-lived (default expiry):</strong><br>
          <code>docker exec chatter-chatter-1 python -c "from app.utils import create_access_token; print(create_access_token({'sub': 'kev'}))"</code>
        </p>
        <p class="small text-muted">Paste the token below.</p>
        <div class="input-group mb-2">
          <input type="text" id="dev-token-input" class="form-control form-control-sm" placeholder="Paste JWT token...">
          <button class="btn btn-sm btn-primary" onclick="(function(){ var t=document.getElementById('dev-token-input').value.trim(); if(t){localStorage.setItem('dev_jwt_token',t);location.reload();} })()">Set Token</button>
        </div>
        <small class="text-muted">Token is saved in localStorage and persists across reloads (and Jekyll hot-rebuilds).</small>
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
      setEditorValue(currentProject.content_md);
      difficultySelect.value = currentProject.difficulty || '';
      timeInput.value = currentProject.estimated_minutes || '';
      repoInput.value = currentProject.code_repo_url || '';
      updateStatusBadge(currentProject.status);
      renderTags(currentProject.tags || []);
      // Surface the saved content-mode choice and point the builder
      // deep-link at this project.
      applyContentMode(currentProject.content_mode || 'markdown');
      if (openInstructionBuilderLink) {
        openInstructionBuilderLink.href =
          '/projects/instructions/edit.html?id=' + encodeURIComponent(currentProject.id);
      }
      loadBOM();
      loadLinks();
      loadVideos();
      loadFiles();
      loadImages();
      loadJournal();
      // Issue #178 Phase 0: bring up the instruction-builder section.
      // Lives in its own module (project-instructions.js) to avoid
      // piling more behaviour onto this file ahead of Phase 1's
      // Fabric.js canvas work. isOwner is implicit here — the editor
      // page only opens for the logged-in user, and the backend
      // rejects non-owner writes with 403 (mirrors the other sections
      // above, which don't track ownership client-side either).
      if (window.ProjectInstructions && typeof window.ProjectInstructions.init === 'function') {
        window.ProjectInstructions.init(currentProject.id, true);
      }
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

  function getActiveContentMode() {
    var active = document.querySelector('.content-mode-btn.is-active');
    return active && active.dataset.contentMode === 'builder' ? 'builder' : 'markdown';
  }

  async function saveProject() {
    const data = {
      title: titleInput.value || 'Untitled Project',
      short_description: descInput.value || null,
      content_md: getEditorValue() || null,
      content_mode: getActiveContentMode(),
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
      // Issue #190: a title save can change the slug, so refresh the
      // "View live" / view-toggle links from the (possibly new) slug.
      // updateStatusBadge already reads currentProject.slug to rebuild
      // both hrefs, so a single call here keeps the displayed public
      // URL in sync silently — no page reload required. The editor's
      // own URL stays at /projects/editor.html?id=<id>, so we do NOT
      // history.replaceState anything slug-shaped here.
      updateStatusBadge(currentProject.status);
      showSaveStatus('saved', 'Saved');
      // Issue #106: surface any badges awarded by this save (typically
      // only on the initial create, but the API may award one on a PUT
      // too if e.g. setting status -> completed crosses a threshold).
      if (window.BadgeToast && currentProject) {
        BadgeToast.fromResponse(currentProject);
      }
    } catch (e) {
      console.error('Save failed:', e);
      showSaveStatus('error', 'Save failed');
    }
  }

  function scheduleAutoSave() {
    if (autoSaveTimer) clearTimeout(autoSaveTimer);
    autoSaveTimer = setTimeout(saveProject, 3000);
  }

  // --- Save / Status / Delete --- (autosave only; no manual save button)

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
      // Issue #152: link to the canonical /projects/<owner>/<slug> URL
      // when the loaded project has a slug; fall back to ?id= for any
      // legacy row whose slug hasn't been backfilled yet.
      var liveUrl = (currentProject.slug && currentProject.author_username)
        ? '/projects/' + encodeURIComponent(currentProject.author_username) + '/' + encodeURIComponent(currentProject.slug)
        : '/projects/view.html?id=' + currentProject.id;
      if (viewLiveBtn) {
        viewLiveBtn.style.display = '';
        viewLiveBtn.href = liveUrl;
      }
      if (viewToggle) {
        viewToggle.classList.remove('d-none');
        viewToggle.href = liveUrl;
      }
    }
  }

  // --- Tags ---
  // Keyword -> tag map for auto-detection. Keys are matched case-insensitively
  // as whole words/phrases against the markdown content (with fenced/inline
  // code blocks and URLs stripped). Extend this map as new maker topics emerge.
  const AUTO_TAG_KEYWORDS = {
    'arduino': 'arduino',
    'raspberry pi': 'raspberry-pi',
    'raspi': 'raspberry-pi',
    'rpi': 'raspberry-pi',
    '3d print': '3d-printing',
    '3d-print': '3d-printing',
    '3d printing': '3d-printing',
    '3d printed': '3d-printing',
    'laser cut': 'laser-cutting',
    'laser-cut': 'laser-cutting',
    'laser cutting': 'laser-cutting',
    'laser cutter': 'laser-cutting',
    'micropython': 'micropython',
    'circuitpython': 'circuitpython',
    'esp32': 'esp32',
    'esp8266': 'esp8266',
    'pico': 'raspberry-pi-pico',
    'rp2040': 'raspberry-pi-pico',
    'rp2350': 'raspberry-pi-pico',
    'cnc': 'cnc',
    'servo': 'servo',
    'stepper': 'stepper-motor',
    'stepper motor': 'stepper-motor',
    'motor': 'motors',
    'python': 'python',
    'c++': 'cpp',
    'cpp': 'cpp',
    'robot': 'robotics',
    'robotics': 'robotics',
    'pcb': 'electronics',
    'kicad': 'electronics',
    'circuit': 'electronics',
    'openscad': 'cad',
    'fusion 360': 'cad',
    'freecad': 'cad',
    'oled': 'display',
    'ssd1306': 'display',
    'lcd': 'display',
    'bluetooth': 'bluetooth',
    'ble': 'bluetooth',
    'wifi': 'wifi',
    'wi-fi': 'wifi',
    'i2c': 'i2c',
    'spi': 'spi',
    'uart': 'uart',
    'sensor': 'sensors',
    'lidar': 'lidar',
    'camera': 'camera',
    'led': 'leds',
    'neopixel': 'neopixel',
    'machine learning': 'machine-learning',
    'tensorflow': 'machine-learning',
    'opencv': 'computer-vision',
    'computer vision': 'computer-vision',
    'mqtt': 'mqtt',
    'home assistant': 'home-assistant',
    'docker': 'docker',
    'linux': 'linux',
    'gpio': 'gpio',
  };

  function escapeRegex(str) {
    return str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  }

  // Pre-compile keyword regexes once. We use lookarounds rather than \b so
  // multi-word phrases and terms with non-word chars (c++, wi-fi, 3d print)
  // still get whole-word semantics. "scarduino" must not match "arduino".
  const AUTO_TAG_PATTERNS = Object.entries(AUTO_TAG_KEYWORDS).map(([keyword, tag]) => ({
    tag,
    regex: new RegExp('(?:^|[^A-Za-z0-9])' + escapeRegex(keyword) + '(?=$|[^A-Za-z0-9])', 'i'),
  }));

  function stripCodeAndUrls(markdown) {
    if (!markdown) return '';
    return markdown
      // Fenced code blocks (```...``` or ~~~...~~~)
      .replace(/```[\s\S]*?```/g, ' ')
      .replace(/~~~[\s\S]*?~~~/g, ' ')
      // Inline code
      .replace(/`[^`\n]*`/g, ' ')
      // Markdown links [text](url) - keep text, drop URL
      .replace(/\[([^\]]*)\]\(([^)]*)\)/g, '$1 ')
      // Bare URLs
      .replace(/https?:\/\/\S+/g, ' ')
      .replace(/www\.\S+/g, ' ');
  }

  function detectTagsFromMarkdown(markdown) {
    const cleaned = stripCodeAndUrls(markdown);
    const found = new Set();
    for (const { tag, regex } of AUTO_TAG_PATTERNS) {
      if (regex.test(cleaned)) found.add(tag);
    }
    return found;
  }

  function getCurrentTags() {
    return Array.from(tagsContainer.querySelectorAll('.tag-badge'))
      .map(el => el.dataset.tag);
  }

  function getManualTags() {
    return Array.from(tagsContainer.querySelectorAll('.tag-badge'))
      .filter(el => el.dataset.auto !== 'true')
      .map(el => el.dataset.tag);
  }

  function getAutoTagElements() {
    return Array.from(tagsContainer.querySelectorAll('.tag-badge[data-auto="true"]'));
  }

  function renderTags(tags) {
    tagsContainer.innerHTML = '';
    // Existing project tags loaded from storage are treated as manual — we
    // don't know which ones were originally auto-detected.
    tags.forEach(tag => addTagBadge(tag, false));
  }

  function addTagBadge(tag, isAuto = false) {
    const el = document.createElement('span');
    el.className = 'tag-badge' + (isAuto ? ' tag-badge-auto' : '');
    el.dataset.tag = tag;
    if (isAuto) el.dataset.auto = 'true';
    const autoBadge = isAuto ? ' <span class="auto-indicator" title="Auto-detected from content">auto</span>' : '';
    el.innerHTML = tag + autoBadge + ' <span class="remove-tag">&times;</span>';
    el.querySelector('.remove-tag').addEventListener('click', () => {
      el.remove();
      scheduleAutoSave();
    });
    tagsContainer.appendChild(el);
    return el;
  }

  // Reconcile auto-detected tags with current content:
  //  - add new auto-tags that don't already exist (manual or auto)
  //  - remove auto-tags whose keyword is no longer present
  //  - never touch manual tags
  function applyAutoDetectedTags(markdown) {
    const detected = detectTagsFromMarkdown(markdown);
    const manual = new Set(getManualTags());
    let changed = false;

    // Remove stale auto-tags
    for (const el of getAutoTagElements()) {
      if (!detected.has(el.dataset.tag)) {
        el.remove();
        changed = true;
      }
    }

    // Add new auto-tags (skip ones already added manually or already auto)
    const currentAuto = new Set(getAutoTagElements().map(el => el.dataset.tag));
    for (const tag of detected) {
      if (manual.has(tag) || currentAuto.has(tag)) continue;
      addTagBadge(tag, true);
      changed = true;
    }

    if (changed) scheduleAutoSave();
  }

  // Debounced trigger fired from the EasyMDE change handler.
  let autoTagTimer = null;
  function scheduleAutoTagDetection() {
    if (autoTagTimer) clearTimeout(autoTagTimer);
    autoTagTimer = setTimeout(() => {
      autoTagTimer = null;
      if (easyMDE) applyAutoDetectedTags(easyMDE.value());
    }, 1000);
  }

  document.getElementById('add-tag-btn').addEventListener('click', () => {
    const tag = tagInput.value.trim().toLowerCase();
    if (!tag) return;
    // If the same tag was previously auto-added, "promote" it to manual so
    // it isn't removed when the keyword disappears from the content.
    const existing = tagsContainer.querySelector(`.tag-badge[data-tag="${CSS.escape(tag)}"]`);
    if (existing) {
      if (existing.dataset.auto === 'true') {
        existing.remove();
        addTagBadge(tag, false);
        tagInput.value = '';
        scheduleAutoSave();
      } else {
        tagInput.value = '';
      }
      return;
    }
    addTagBadge(tag, false);
    tagInput.value = '';
    scheduleAutoSave();
  });

  tagInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      document.getElementById('add-tag-btn').click();
    }
  });

  // --- EasyMDE Markdown Editor ---
  let easyMDE = null;

  function initEditor() {
    if (easyMDE) return;
    easyMDE = new EasyMDE({
      element: contentEditor,
      spellChecker: false,
      autofocus: false,
      placeholder: 'Write your project content in Markdown...\n\n# Background\nDescribe what this project is about...\n\n## How to Build\n- Step 1: ...',
      status: false,
      minHeight: '350px',
      renderingConfig: {
        codeSyntaxHighlighting: true,
      },
      toolbar: [
        'bold', 'italic', 'heading', '|',
        'code', 'quote', 'unordered-list', 'ordered-list', '|',
        'link',
        {
          name: 'insert-image',
          action: function(editor) { showImagePicker(editor); },
          className: 'fas fa-image no-disable',
          title: 'Insert uploaded image',
        },
        'table', '|',
        {
          name: 'diagram',
          action: function(editor) { showDiagramMenu(editor); },
          className: 'fas fa-project-diagram no-disable',
          title: 'Insert diagram',
        },
        '|',
        {
          name: 'preview',
          action: function(editor) {
            // Close side-by-side first
            var container = editor.codemirror.getWrapperElement().closest('.EasyMDEContainer');
            var sbsPreview = container.querySelector('.editor-preview-side-custom');
            if (sbsPreview) {
              sbsPreview.remove();
              container.classList.remove('sided--no-fullscreen');
              editor.codemirror.refresh();
            }
            EasyMDE.togglePreview(editor);
            // Render mermaid in the preview pane + style any markdown tables
            setTimeout(async () => {
              var previewEl = container.querySelector('.editor-preview-active');
              if (!previewEl) return;
              previewEl.querySelectorAll('table').forEach(function(t) {
                t.classList.add('table', 'table-single', 'table-narrow');
              });
              var blocks = previewEl.querySelectorAll('pre code.language-mermaid');
              if (blocks.length === 0) return;
              try {
                var mod = await import('https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.esm.min.mjs');
                var mermaid = mod.default;
                mermaid.initialize({ startOnLoad: false });
                blocks.forEach((block, i) => {
                  var div = document.createElement('div');
                  div.className = 'mermaid';
                  div.id = 'mermaid-preview-' + Date.now() + '-' + i;
                  div.textContent = block.textContent;
                  block.parentElement.replaceWith(div);
                });
                await mermaid.run({ querySelector: '.editor-preview-active .mermaid' });
              } catch (e) { /* mermaid not available */ }
            }, 50);
          },
          className: 'fa fa-eye no-disable',
          title: 'Toggle preview',
        },
        {
          name: 'side-by-side',
          action: function(editor) {
            var cm = editor.codemirror;
            var wrap = cm.getWrapperElement();
            var container = wrap.closest('.EasyMDEContainer');
            var existingPreview = container.querySelector('.editor-preview-side-custom');

            // Close regular preview if it's open
            var regularPreview = container.querySelector('.editor-preview-active');
            if (regularPreview) {
              EasyMDE.togglePreview(editor);
            }

            if (existingPreview) {
              existingPreview.remove();
              container.classList.remove('sided--no-fullscreen');
              requestAnimationFrame(() => cm.refresh());
              return;
            }

            // Create our own preview pane
            var preview = document.createElement('div');
            preview.className = 'editor-preview-side-custom';
            container.appendChild(preview);
            container.classList.add('sided--no-fullscreen');

            let mermaidLib = null;
            async function renderMermaid() {
              const blocks = preview.querySelectorAll('pre code.language-mermaid');
              if (blocks.length === 0) return;
              try {
                if (!mermaidLib) {
                  const mod = await import('https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.esm.min.mjs');
                  mermaidLib = mod.default;
                  mermaidLib.initialize({ startOnLoad: false });
                }
                blocks.forEach((block, i) => {
                  const div = document.createElement('div');
                  div.className = 'mermaid';
                  div.id = 'mermaid-' + Date.now() + '-' + i;
                  div.textContent = block.textContent;
                  block.parentElement.replaceWith(div);
                });
                await mermaidLib.run({ querySelector: '.editor-preview-side-custom .mermaid' });
              } catch (e) { /* mermaid not available */ }
            }

            let renderTimer = null;
            function updatePreview() {
              preview.innerHTML = editor.markdown(editor.value());
              preview.querySelectorAll('table').forEach(t => {
                t.classList.add('table', 'table-single', 'table-narrow');
              });
              clearTimeout(renderTimer);
              renderTimer = setTimeout(renderMermaid, 200);
            }
            updatePreview();
            cm.on('change', updatePreview);

            // Build a map of source-line → top-level block index
            function getBlockStarts(markdown) {
              const lines = markdown.split('\n');
              const starts = [];
              let inBlock = false;
              let inCode = false;
              for (let i = 0; i < lines.length; i++) {
                const line = lines[i];
                if (line.startsWith('```')) {
                  if (!inCode) {
                    if (!inBlock) starts.push(i);
                    inCode = true;
                    inBlock = true;
                  } else {
                    inCode = false;
                  }
                  continue;
                }
                if (inCode) continue;
                if (line.trim() === '') {
                  inBlock = false;
                  continue;
                }
                if (!inBlock) {
                  starts.push(i);
                  inBlock = true;
                }
              }
              return starts;
            }

            // Sync preview to editor's top visible line by mapping blocks
            var syncing = false;
            function syncScroll() {
              if (syncing) return;
              const blockStarts = getBlockStarts(editor.value());
              if (blockStarts.length === 0) return;

              const info = cm.getScrollInfo();
              const topLine = cm.lineAtHeight(info.top, 'local');

              // Find which block the top line belongs to
              let blockIdx = 0;
              for (let i = 0; i < blockStarts.length; i++) {
                if (blockStarts[i] <= topLine) blockIdx = i;
                else break;
              }

              const previewBlocks = Array.from(preview.children);
              const target = previewBlocks[blockIdx];
              if (!target) return;

              // How far through the current block is the editor?
              const blockStartLine = blockStarts[blockIdx];
              const blockEndLine = blockIdx < blockStarts.length - 1
                ? blockStarts[blockIdx + 1] - 1
                : cm.lineCount() - 1;
              const blockSize = Math.max(1, blockEndLine - blockStartLine + 1);
              const blockProgress = Math.min(1, Math.max(0, (topLine - blockStartLine) / blockSize));

              const targetRect = target.getBoundingClientRect();
              const previewRect = preview.getBoundingClientRect();
              const targetTop = targetRect.top - previewRect.top + preview.scrollTop;
              const offset = targetTop + (targetRect.height * blockProgress);

              syncing = true;
              preview.scrollTop = offset;
              setTimeout(() => { syncing = false; }, 50);
            }
            cm.on('scroll', syncScroll);
            cm.on('cursorActivity', syncScroll);

            // Refresh CodeMirror so click coords match the new layout
            requestAnimationFrame(() => cm.refresh());
          },
          className: 'fas fa-columns no-disable',
          title: 'Side-by-side preview',
        },
        'fullscreen', '|',
        'guide',
      ],
    });

    easyMDE.codemirror.on('change', () => {
      scheduleAutoSave();
      scheduleAutoTagDetection();
    });
    setupFloatingToolbar(easyMDE);
    setupImageAutocomplete(easyMDE);
    // Expose for external integrations (e.g. global drop zone — issue #117)
    window.easyMDE = easyMDE;
  }

  // --- Floating toolbar on text selection ---
  function setupFloatingToolbar(editor) {
    const cm = editor.codemirror;
    let toolbar = document.createElement('div');
    toolbar.className = 'mini-toolbar';
    toolbar.style.display = 'none';
    toolbar.innerHTML = `
      <button data-fmt="bold" title="Bold"><i class="fas fa-bold"></i></button>
      <button data-fmt="italic" title="Italic"><i class="fas fa-italic"></i></button>
      <button data-fmt="heading" title="Heading"><i class="fas fa-heading"></i></button>
      <button data-fmt="quote" title="Quote"><i class="fas fa-quote-right"></i></button>
      <button data-fmt="ul" title="Bullet list"><i class="fas fa-list-ul"></i></button>
      <button data-fmt="ol" title="Numbered list"><i class="fas fa-list-ol"></i></button>
      <button data-fmt="code" title="Code"><i class="fas fa-code"></i></button>
      <button data-fmt="link" title="Link"><i class="fas fa-link"></i></button>
    `;
    document.body.appendChild(toolbar);

    function toggleWrap(marker) {
      const sel = cm.getSelection();
      const from = cm.getCursor('from');
      const to = cm.getCursor('to');
      const mlen = marker.length;
      const m = marker.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
      const re = new RegExp('^' + m + '([\\s\\S]*)' + m + '$');
      const match = sel.match(re);
      if (match) {
        // Markers were inside the selection — strip them, re-select inner content
        cm.replaceSelection(match[1]);
        const newTo = cm.getCursor();
        cm.setSelection(from, newTo);
        return;
      }
      const lineFrom = cm.getLine(from.line);
      const lineTo = cm.getLine(to.line);
      const before = lineFrom.substring(Math.max(0, from.ch - mlen), from.ch);
      const after = lineTo.substring(to.ch, to.ch + mlen);
      if (before === marker && after === marker) {
        // Markers were just outside the selection — strip them, keep selection on inner content
        cm.replaceRange('', { line: to.line, ch: to.ch }, { line: to.line, ch: to.ch + mlen });
        cm.replaceRange('', { line: from.line, ch: from.ch - mlen }, { line: from.line, ch: from.ch });
        const newFrom = { line: from.line, ch: from.ch - mlen };
        const newTo = from.line === to.line
          ? { line: to.line, ch: to.ch - mlen }
          : { line: to.line, ch: to.ch };
        cm.setSelection(newFrom, newTo);
        return;
      }
      // Wrap selection with markers and reselect the inner (unwrapped) content
      cm.replaceSelection(marker + sel + marker);
      const newFrom = { line: from.line, ch: from.ch + mlen };
      const newTo = from.line === to.line
        ? { line: to.line, ch: to.ch + mlen }
        : { line: to.line, ch: to.ch };
      cm.setSelection(newFrom, newTo);
    }

    function toggleLinePrefix(prefixRegex, prefixToAdd, lineMatchTest) {
      const from = cm.getCursor('from');
      const to = cm.getCursor('to');
      const lines = [];
      for (let l = from.line; l <= to.line; l++) {
        lines.push(cm.getLine(l));
      }
      const allHavePrefix = lines.every(line => lineMatchTest(line));
      const newLines = lines.map(line => {
        if (allHavePrefix) {
          return line.replace(prefixRegex, '');
        } else {
          return prefixToAdd + line.replace(prefixRegex, '');
        }
      });
      cm.replaceRange(newLines.join('\n'),
        { line: from.line, ch: 0 },
        { line: to.line, ch: cm.getLine(to.line).length }
      );
      // Reselect the full affected line range so the toolbar stays visible
      cm.setSelection(
        { line: from.line, ch: 0 },
        { line: to.line, ch: cm.getLine(to.line).length }
      );
    }

    function cycleHeading() {
      const from = cm.getCursor('from');
      const line = cm.getLine(from.line);
      const match = line.match(/^(#{1,6})\s/);
      let newLine;
      if (!match) {
        newLine = '# ' + line;
      } else if (match[1].length < 6) {
        newLine = '#'.repeat(match[1].length + 1) + ' ' + line.substring(match[0].length);
      } else {
        // h6 → remove heading
        newLine = line.substring(match[0].length);
      }
      cm.replaceRange(newLine, { line: from.line, ch: 0 }, { line: from.line, ch: line.length });
      // Reselect the heading line so the toolbar stays visible
      cm.setSelection(
        { line: from.line, ch: 0 },
        { line: from.line, ch: cm.getLine(from.line).length }
      );
    }

    function applyFormat(fmt) {
      const sel = cm.getSelection();
      switch (fmt) {
        case 'bold': toggleWrap('**'); break;
        case 'italic': toggleWrap('*'); break;
        case 'code': toggleWrap('`'); break;
        case 'heading': cycleHeading(); break;
        case 'quote':
          toggleLinePrefix(/^> /, '> ', l => /^> /.test(l));
          break;
        case 'ul':
          toggleLinePrefix(/^(- |\* |\d+\. )/, '- ', l => /^(- |\* )/.test(l));
          break;
        case 'ol':
          toggleLinePrefix(/^(- |\* |\d+\. )/, '1. ', l => /^\d+\. /.test(l));
          break;
        case 'link': {
          const linkFrom = cm.getCursor('from');
          const linkText = sel || 'text';
          cm.replaceSelection('[' + linkText + '](https://)');
          // Reselect the link text inside [...] so the user can tweak it
          cm.setSelection(
            { line: linkFrom.line, ch: linkFrom.ch + 1 },
            { line: linkFrom.line, ch: linkFrom.ch + 1 + linkText.length }
          );
          break;
        }
      }
      cm.focus();
    }

    toolbar.querySelectorAll('button').forEach(btn => {
      btn.addEventListener('mousedown', e => {
        e.preventDefault();
        applyFormat(btn.dataset.fmt);
        positionToolbar();
      });
    });

    function positionToolbar() {
      if (!cm.somethingSelected()) {
        toolbar.style.display = 'none';
        return;
      }
      const from = cm.getCursor('from');
      const coords = cm.cursorCoords(from, 'window');
      toolbar.style.display = 'flex';
      const tw = toolbar.offsetWidth;
      let left = coords.left;
      if (left + tw > window.innerWidth - 10) left = window.innerWidth - tw - 10;
      toolbar.style.left = Math.max(10, left) + 'px';
      toolbar.style.top = (coords.top - toolbar.offsetHeight - 8) + 'px';
    }

    cm.on('cursorActivity', positionToolbar);
    cm.on('blur', () => {
      setTimeout(() => {
        if (!toolbar.matches(':hover')) toolbar.style.display = 'none';
      }, 150);
    });
    window.addEventListener('scroll', positionToolbar, true);
  }

  // --- Image autocomplete on `![` ---
  function setupImageAutocomplete(editor) {
    const cm = editor.codemirror;
    let popup = null;
    let images = [];
    let filteredImages = [];
    let activeIdx = 0;
    let startCursor = null;

    function fetchImages() {
      if (!currentProject) { images = []; return; }
      apiFetch(API + '/api/projects/' + currentProject.id + '/images', { credentials: 'include' })
        .then(r => r.json())
        .then(imgs => {
          images = imgs.map(img => ({
            url: API + '/api/projects/' + currentProject.id + '/images/' + img.id + '/view',
            name: img.filename,
          }));
        }).catch(() => {});
    }
    fetchImages();
    // Refresh image list periodically
    setInterval(fetchImages, 10000);

    function showPopup(coords) {
      if (!popup) {
        popup = document.createElement('div');
        popup.className = 'image-autocomplete-popup';
        document.body.appendChild(popup);
      }
      popup.style.display = 'block';
      popup.style.left = coords.left + 'px';
      popup.style.top = (coords.bottom + 4) + 'px';
      renderPopup();
    }

    function hidePopup() {
      if (popup) popup.style.display = 'none';
      startCursor = null;
    }

    function renderPopup() {
      if (!popup) return;
      if (filteredImages.length === 0) {
        popup.innerHTML = '<div class="p-2 text-muted small">No matching images</div>';
        return;
      }
      popup.innerHTML = filteredImages.map((img, i) => `
        <div class="image-ac-item ${i === activeIdx ? 'active' : ''}" data-idx="${i}">
          <img src="${img.url}" style="width:30px;height:30px;object-fit:cover;border-radius:3px;margin-right:6px">
          <small>${img.name}</small>
        </div>
      `).join('');
      popup.querySelectorAll('.image-ac-item').forEach(item => {
        item.addEventListener('mousedown', e => {
          e.preventDefault();
          selectImage(parseInt(item.dataset.idx));
        });
      });
    }

    function selectImage(idx) {
      if (!startCursor) return;
      const img = filteredImages[idx];
      if (!img) return;
      const cursor = cm.getCursor();
      cm.replaceRange('![' + img.name + '](' + img.url + ')', startCursor, cursor);
      hidePopup();
      cm.focus();
      scheduleAutoSave();
    }

    cm.on('inputRead', (cm, change) => {
      const cursor = cm.getCursor();
      const lineText = cm.getLine(cursor.line);
      const before = lineText.substring(0, cursor.ch);
      const match = before.match(/!\[([^\]]*)$/);
      if (match) {
        if (!startCursor) {
          startCursor = { line: cursor.line, ch: cursor.ch - match[0].length };
          fetchImages();
        }
        const query = match[1].toLowerCase();
        filteredImages = images.filter(im => im.name.toLowerCase().includes(query));
        activeIdx = 0;
        const coords = cm.cursorCoords(cursor, 'window');
        showPopup(coords);
      } else {
        hidePopup();
      }
    });

    cm.on('keydown', (cm, e) => {
      if (!popup || popup.style.display === 'none') return;
      if (e.key === 'ArrowDown') {
        e.preventDefault();
        activeIdx = (activeIdx + 1) % filteredImages.length;
        renderPopup();
      } else if (e.key === 'ArrowUp') {
        e.preventDefault();
        activeIdx = (activeIdx - 1 + filteredImages.length) % filteredImages.length;
        renderPopup();
      } else if (e.key === 'Enter' || e.key === 'Tab') {
        if (filteredImages.length > 0) {
          e.preventDefault();
          selectImage(activeIdx);
        }
      } else if (e.key === 'Escape') {
        hidePopup();
      }
    });

    cm.on('cursorActivity', () => {
      if (!startCursor) return;
      const cursor = cm.getCursor();
      if (cursor.line !== startCursor.line || cursor.ch < startCursor.ch) {
        hidePopup();
      }
    });
  }

  function getEditorValue() {
    return easyMDE ? easyMDE.value() : contentEditor.value;
  }

  function setEditorValue(val) {
    if (easyMDE) easyMDE.value(val || '');
    else contentEditor.value = val || '';
  }

  // Image picker popover
  function showImagePicker(editor) {
    const existing = document.querySelector('.image-picker-popover');
    if (existing) { existing.remove(); return; }

    const pickerList = document.getElementById('image-picker-list');
    if (!pickerList || !pickerList.innerHTML.trim()) {
      const cm = editor.codemirror;
      cm.replaceSelection('![alt text](image-url)');
      cm.focus();
      return;
    }

    const btn = document.querySelector('.fa-image').closest('button');
    const rect = btn.getBoundingClientRect();
    const pop = document.createElement('div');
    pop.className = 'image-picker-popover';
    pop.style.cssText = 'position:fixed;z-index:10000;background:white;border:1px solid #ddd;border-radius:8px;padding:8px;box-shadow:0 4px 12px rgba(0,0,0,0.15);max-height:250px;overflow-y:auto;min-width:220px;left:' + rect.left + 'px;top:' + (rect.bottom + 4) + 'px;';
    pop.innerHTML = pickerList.innerHTML;
    document.body.appendChild(pop);

    pop.querySelectorAll('.image-picker-item').forEach(item => {
      item.addEventListener('click', () => {
        const url = item.dataset.imgUrl;
        const name = item.dataset.imgName;
        const cm = editor.codemirror;
        cm.replaceSelection('![' + name + '](' + url + ')\n');
        cm.focus();
        pop.remove();
        scheduleAutoSave();
      });
    });

    setTimeout(() => {
      document.addEventListener('click', function closePicker(e) {
        if (!pop.contains(e.target) && !btn.contains(e.target)) {
          pop.remove();
          document.removeEventListener('click', closePicker);
        }
      });
    }, 10);
  }

  // Diagram menu
  function showDiagramMenu(editor) {
    const existing = document.querySelector('.diagram-menu-popover');
    if (existing) { existing.remove(); return; }

    const btn = document.querySelector('.fa-project-diagram').closest('button');
    const rect = btn.getBoundingClientRect();
    const pop = document.createElement('div');
    pop.className = 'diagram-menu-popover';
    pop.style.cssText = 'position:fixed;z-index:10000;background:white;border:1px solid #ddd;border-radius:8px;padding:4px;box-shadow:0 4px 12px rgba(0,0,0,0.15);left:' + rect.left + 'px;top:' + (rect.bottom + 4) + 'px;';

    const items = [
      { label: 'Flowchart', md: '\n```mermaid\ngraph TD\n    A[Start] --> B[Step 1]\n    B --> C[Step 2]\n    C --> D[End]\n```\n' },
      { label: 'Sequence Diagram', md: '\n```mermaid\nsequenceDiagram\n    participant A\n    participant B\n    A->>B: Hello\n    B-->>A: Hi back\n```\n' },
      { label: 'State Diagram', md: '\n```mermaid\nstateDiagram-v2\n    [*] --> Idle\n    Idle --> Running\n    Running --> Idle\n    Running --> [*]\n```\n' },
    ];

    pop.innerHTML = items.map(i => '<div class="image-picker-item p-2" style="cursor:pointer"><small>' + i.label + '</small></div>').join('');
    document.body.appendChild(pop);

    pop.querySelectorAll('.image-picker-item').forEach((item, idx) => {
      item.addEventListener('click', () => {
        editor.codemirror.replaceSelection(items[idx].md);
        editor.codemirror.focus();
        pop.remove();
        scheduleAutoSave();
      });
    });

    setTimeout(() => {
      document.addEventListener('click', function closeMenu(e) {
        if (!pop.contains(e.target) && !btn.contains(e.target)) {
          pop.remove();
          document.removeEventListener('click', closeMenu);
        }
      });
    }, 10);
  }

  // --- File Upload ---
  // The sidebar's old #file-dropzone div was replaced with a click-to-pick
  // button (issue: unify the editor's two drop targets into a single
  // page-wide overlay). All drag-and-drop is now handled by the global
  // overlay further down — see setupGlobalDropzone().
  const fileInput = document.getElementById('file-input');
  fileInput.addEventListener('change', e => uploadFiles(e.target.files));

  const FILE_KINDS = {
    py: 'Python Script', cpp: 'C++ Source', h: 'Header File', ino: 'Arduino Sketch',
    md: 'Markdown', txt: 'Text File', pdf: 'PDF Document', csv: 'Spreadsheet',
    stl: '3D Model (STL)', obj: '3D Model (OBJ)', gcode: 'G-Code', '3mf': '3D Model (3MF)',
    step: 'CAD Model (STEP)', stp: 'CAD Model (STEP)', dxf: 'CAD Drawing (DXF)',
    dwg: 'CAD Drawing (DWG)', scad: 'OpenSCAD', f3d: 'Fusion 360', fcstd: 'FreeCAD',
    kicad_pcb: 'KiCad PCB', kicad_sch: 'KiCad Schematic', brd: 'PCB Board', sch: 'Schematic',
    json: 'JSON Data', xml: 'XML Data', yaml: 'YAML Config', yml: 'YAML Config', toml: 'TOML Config',
    zip: 'ZIP Archive', tar: 'TAR Archive', gz: 'GZIP Archive', '7z': '7-Zip Archive', rar: 'RAR Archive',
    svg: 'SVG Image',
  };

  const FILE_ICONS = {
    py: 'fa-python fab', cpp: 'fa-file-code fas', h: 'fa-file-code fas', ino: 'fa-microchip fas',
    pdf: 'fa-file-pdf fas', md: 'fa-file-alt fas', txt: 'fa-file-alt fas', csv: 'fa-file-csv fas',
    stl: 'fa-cube fas', obj: 'fa-cube fas', gcode: 'fa-cube fas', '3mf': 'fa-cube fas',
    step: 'fa-drafting-compass fas', stp: 'fa-drafting-compass fas', dxf: 'fa-drafting-compass fas',
    dwg: 'fa-drafting-compass fas', scad: 'fa-drafting-compass fas', f3d: 'fa-drafting-compass fas',
    fcstd: 'fa-drafting-compass fas',
    kicad_pcb: 'fa-microchip fas', kicad_sch: 'fa-microchip fas', brd: 'fa-microchip fas', sch: 'fa-microchip fas',
    json: 'fa-file-code fas', xml: 'fa-file-code fas', yaml: 'fa-file-code fas', yml: 'fa-file-code fas',
    zip: 'fa-file-archive fas', tar: 'fa-file-archive fas', gz: 'fa-file-archive fas',
    svg: 'fa-image fas',
  };

  function fileKind(filename) {
    const ext = (filename || '').split('.').pop().toLowerCase();
    return FILE_KINDS[ext] || ext.toUpperCase() + ' File';
  }

  function fileIcon(filename) {
    const ext = (filename || '').split('.').pop().toLowerCase();
    return FILE_ICONS[ext] || 'fa-file fas';
  }

  async function uploadFiles(fileList) {
    if (!currentProject) {
      await saveProject();
      if (!currentProject) return;
    }
    const errors = [];
    for (const file of fileList) {
      const form = new FormData();
      form.append('file', file);
      try {
        const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/files', {
          method: 'POST', credentials: 'include', body: form,
        });
        if (!resp.ok) {
          const err = await resp.json();
          errors.push(file.name + ': ' + (err.detail || 'failed'));
        }
      } catch (e) { errors.push(file.name + ': network error'); }
    }
    if (errors.length > 0) {
      alert('Some files could not be uploaded:\n\n' + errors.join('\n'));
    }
    loadFiles();
  }

  async function loadFiles() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/files', { credentials: 'include' });
    const files = await resp.json();
    const list = document.getElementById('file-list');
    if (files.length === 0) { list.innerHTML = ''; return; }
    // Issue #187: render a Description column with an inline-editable
    // cell. Click the muted placeholder ("Add a description…") or the
    // existing text to flip into an input; Enter / blur saves; Escape
    // cancels. Long descriptions wrap in the column via the
    // .file-description CSS rule (max-width + word-break).
    list.innerHTML = `<table class="table table-single table-narrow table-hover mb-0">
      <thead>
        <tr>
          <th style="width:20px"></th>
          <th>File</th>
          <th>Description</th>
          <th class="text-muted small">Kind</th>
          <th class="text-muted small text-end" style="width:70px">Size</th>
          <th style="width:30px"></th>
        </tr>
      </thead>
      <tbody>${files.map(f => `
        <tr data-file-id="${f.id}">
          <td class="text-muted" style="width:20px"><i class="${fileIcon(f.filename)}"></i></td>
          <td><a href="${API}/api/projects/${currentProject.id}/files/${f.id}/download">${escapeHtmlAttr(f.filename)}</a></td>
          <td class="file-description" data-file-id="${f.id}">${renderFileDescription(f.description)}</td>
          <td class="text-muted small">${fileKind(f.filename)}</td>
          <td class="text-muted small text-end" style="width:70px">${(f.file_size / 1024).toFixed(1)} KB</td>
          <td style="width:30px"><button class="btn btn-sm text-danger p-0" onclick="deleteFile(${f.id})"><i class="fas fa-trash fa-xs"></i></button></td>
        </tr>
      `).join('')}</tbody></table>`;
  }

  // Render the description cell's "view" state — a span the user clicks
  // to edit. Empty values get a muted placeholder so the affordance is
  // discoverable; non-empty values render the description verbatim.
  function renderFileDescription(desc) {
    const hasText = desc != null && String(desc).length > 0;
    if (hasText) {
      return `<span class="kr-file-desc" tabindex="0" role="button"
        title="Click to edit description">${escapeHtmlAttr(desc)}</span>`;
    }
    return `<span class="kr-file-desc text-muted fst-italic" tabindex="0"
      role="button" title="Click to add a description"
      data-empty="1">Add a description…</span>`;
  }

  // Track in-flight PUTs per row so a fast typer doesn't fire overlapping
  // requests — serialise per-row by chaining onto the previous promise.
  // Same idea as the BOM / links autosave debouncer.
  const _fileDescSavePending = {};

  async function saveFileDescription(fileId, newValue) {
    if (!currentProject) return null;
    const prev = _fileDescSavePending[fileId] || Promise.resolve();
    const next = prev.then(async () => {
      const resp = await apiFetch(
        API + '/api/projects/' + currentProject.id + '/files/' + fileId,
        {
          method: 'PUT',
          credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ description: newValue }),
        }
      );
      if (!resp.ok) throw new Error('save failed: ' + resp.status);
      return resp.json();
    });
    _fileDescSavePending[fileId] = next;
    return next;
  }

  // Inline-edit handler: wired via event delegation on #file-list so it
  // survives loadFiles() re-renders without re-binding listeners.
  function setupFileDescriptionEditing() {
    const list = document.getElementById('file-list');
    if (!list || list.dataset.descBound === '1') return;
    list.dataset.descBound = '1';

    list.addEventListener('click', e => {
      const span = e.target.closest('.kr-file-desc');
      if (!span) return;
      // Already editing? Don't re-enter.
      if (span.dataset.editing === '1') return;
      beginFileDescriptionEdit(span);
    });
    list.addEventListener('keydown', e => {
      const span = e.target.closest('.kr-file-desc');
      if (!span || span.dataset.editing === '1') return;
      // Enter / Space activates the editor for keyboard users.
      if (e.key === 'Enter' || e.key === ' ') {
        e.preventDefault();
        beginFileDescriptionEdit(span);
      }
    });
  }

  function beginFileDescriptionEdit(span) {
    const cell = span.closest('.file-description');
    if (!cell) return;
    const fileId = parseInt(cell.dataset.fileId, 10);
    if (!Number.isFinite(fileId)) return;
    const original = span.dataset.empty === '1' ? '' : (span.textContent || '');
    span.dataset.editing = '1';

    const input = document.createElement('input');
    input.type = 'text';
    input.className = 'form-control form-control-sm kr-file-desc-input';
    input.value = original;
    input.maxLength = 2000;
    input.placeholder = 'Describe this file…';
    cell.innerHTML = '';
    cell.appendChild(input);
    input.focus();
    input.select();

    let finished = false;
    const finish = async (commit) => {
      if (finished) return;
      finished = true;
      if (!commit) {
        cell.innerHTML = renderFileDescription(original || null);
        return;
      }
      const next = input.value;
      if (next === original) {
        cell.innerHTML = renderFileDescription(original || null);
        return;
      }
      // Optimistic re-render with the new value; if the save fails the
      // reload at the end of the catch reconciles with the server.
      cell.innerHTML = renderFileDescription(next || null);
      try {
        await saveFileDescription(fileId, next);
      } catch (_) {
        loadFiles();
      }
    };

    input.addEventListener('keydown', e => {
      if (e.key === 'Enter') { e.preventDefault(); finish(true); }
      else if (e.key === 'Escape') { e.preventDefault(); finish(false); }
    });
    input.addEventListener('blur', () => finish(true));
  }

  // Bind the delegated handler once at module init — the file-list
  // element exists from page load even before any files have been
  // uploaded.
  setupFileDescriptionEditing();

  window.deleteFile = async function(fileId) {
    await apiFetch(API + '/api/projects/' + currentProject.id + '/files/' + fileId, {
      method: 'DELETE', credentials: 'include',
    });
    loadFiles();
  };

  // --- Image Upload ---
  // Old #image-dropzone removed (replaced with a click-to-pick button).
  // All drag-and-drop is handled by the global overlay in
  // setupGlobalDropzone() further down.
  const imageInput = document.getElementById('image-input');
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

  function escapeHtmlAttr(s) {
    return String(s == null ? '' : s)
      .replace(/&/g, '&amp;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;');
  }

  // Issue #149: short curated list of currencies the editor offers.
  // Empty value -> NULL server-side ("Other / unknown" — render as a
  // raw number on the public view). The Pydantic pattern only accepts
  // three uppercase letters so anything else is rejected with 422.
  const BOM_CURRENCIES = [
    { code: '',    label: '—' },
    { code: 'GBP', label: 'GBP £' },
    { code: 'USD', label: 'USD $' },
    { code: 'EUR', label: 'EUR €' },
    { code: 'JPY', label: 'JPY ¥' },
    { code: 'AUD', label: 'AUD A$' },
    { code: 'CAD', label: 'CAD C$' },
  ];
  function bomCurrencyOptions(selected) {
    const sel = (selected || '').toUpperCase();
    return BOM_CURRENCIES.map(c =>
      `<option value="${escapeHtmlAttr(c.code)}"${c.code === sel ? ' selected' : ''}>${escapeHtmlAttr(c.label)}</option>`
    ).join('');
  }

  // Supplier-pricing feature: cache part detail responses so a BOM
  // table with N rows linked to the same part doesn't re-fetch the
  // part N times. Cleared on every BOM reload so price edits land.
  let partSuppliersCache = {};

  async function fetchPartSuppliers(slug) {
    if (!slug) return [];
    if (partSuppliersCache[slug]) return partSuppliersCache[slug];
    try {
      const resp = await fetch(API + '/api/parts/' + encodeURIComponent(slug));
      if (!resp.ok) return [];
      const part = await resp.json();
      const suppliers = Array.isArray(part.suppliers) ? part.suppliers : [];
      partSuppliersCache[slug] = suppliers;
      return suppliers;
    } catch (_) { return []; }
  }

  function supplierSelectOptions(suppliers, selectedId) {
    // The "— manual price —" option unlinks the row; selecting it
    // re-enables the manual unit_cost + currency inputs.
    const opts = ['<option value="">— manual price —</option>'];
    suppliers.forEach(s => {
      const priceHint = (s.unit_cost != null)
        ? ` (${(s.currency_code || '').trim() ? s.currency_code + ' ' : ''}${Number(s.unit_cost).toFixed(2)})`
        : '';
      const sel = String(s.id) === String(selectedId) ? ' selected' : '';
      opts.push(
        `<option value="${escapeHtmlAttr(s.id)}"${sel}>${escapeHtmlAttr((s.name || s.url) + priceHint)}</option>`
      );
    });
    return opts.join('');
  }

  async function loadBOM() {
    if (!currentProject) return;
    partSuppliersCache = {};  // force-refresh prices on every load
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/bom', { credentials: 'include' });
    const items = await resp.json();
    const tbody = document.getElementById('bom-body');
    tbody.innerHTML = items.map(item => {
      const linked = item.part_id && item.part_slug;
      const partPill = linked
        ? `<a class="bom-part-pill" href="/parts/view.html?slug=${encodeURIComponent(item.part_slug)}" title="Linked to parts catalog" target="_blank"><i class="fas fa-link"></i> /parts/${escapeHtmlAttr(item.part_slug)}</a>`
        : (item.part_id ? `<span class="bom-part-pill" title="Linked to parts catalog"><i class="fas fa-link"></i> part #${item.part_id}</span>` : '');
      // Supplier-pricing feature: when the row links to a supplier
      // that has a non-NULL price, ``price_source`` is ``"supplier"``
      // and the row's own unit_cost / currency inputs are disabled —
      // the supplier is the source of truth. Selecting "— manual
      // price —" re-enables them.
      const supplierLocked = item.price_source === 'supplier';
      const costDisabled = supplierLocked ? ' disabled' : '';
      const currencyDisabled = supplierLocked ? ' disabled' : '';
      // The cost cell shows the effective price (so the user sees what
      // the project page will render) but the underlying field
      // remains unit_cost — when the supplier is unlinked we still
      // round-trip the row's own value.
      const displayCost = supplierLocked
        ? (item.effective_unit_cost != null ? item.effective_unit_cost : 0)
        : (item.unit_cost != null ? item.unit_cost : 0);
      const displayCurrency = supplierLocked
        ? (item.effective_currency_code || '')
        : (item.currency_code || '');
      const supplierAutoBadge = supplierLocked
        ? `<small class="text-success ms-1"><i class="fas fa-link"></i> Auto</small>`
        : '';
      // The supplier cell switches shape based on whether the row is
      // linked to a part. Linked → <select> populated from the part's
      // suppliers (async-filled below). Unlinked → free-form URL
      // input (the legacy behaviour).
      const supplierCell = linked
        ? `<select class="form-select form-select-sm bom-supplier-select" data-field="supplier_id" data-current-supplier-id="${escapeHtmlAttr(item.supplier_id || '')}" onchange="updateBOM(${item.id}, this)" title="Pick a supplier from the parts catalog">
             <option value="">— loading suppliers… —</option>
           </select>
           <small class="bom-supplier-hint">price comes from chosen supplier</small>`
        : `<input value="${escapeHtmlAttr(item.supplier_url || '')}" data-field="supplier_url" onchange="updateBOM(${item.id}, this)">`;
      return `
      <tr data-bom-id="${item.id}" data-part-id="${item.part_id || ''}" data-part-slug="${escapeHtmlAttr(item.part_slug || '')}" data-supplier-id="${escapeHtmlAttr(item.supplier_id || '')}" data-price-source="${escapeHtmlAttr(item.price_source || 'row')}">
        <td class="bom-part-cell" style="position:relative">
          <input class="bom-part-input" value="${escapeHtmlAttr(item.name)}" data-field="name" autocomplete="off" placeholder="Type to search parts catalog..." onchange="updateBOM(${item.id}, this)">
          ${partPill}
        </td>
        <td><input type="number" value="${item.quantity}" data-field="quantity" style="width:50px" onchange="updateBOM(${item.id}, this)"></td>
        <td><input value="${escapeHtmlAttr(item.unit)}" data-field="unit" style="width:50px" onchange="updateBOM(${item.id}, this)"></td>
        <td><input type="number" step="0.01" value="${displayCost}" data-field="unit_cost" style="width:70px"${costDisabled} onchange="updateBOM(${item.id}, this)">${supplierAutoBadge}</td>
        <td>
          <select class="form-select form-select-sm" data-field="currency_code" style="width:90px"${currencyDisabled} onchange="updateBOM(${item.id}, this)" title="Currency for this unit cost">
            ${bomCurrencyOptions(displayCurrency)}
          </select>
        </td>
        <td>${supplierCell}</td>
        <td><button class="btn btn-sm text-danger" onclick="deleteBOM(${item.id})"><i class="fas fa-times"></i></button></td>
      </tr>`;
    }).join('');
    updateBOMTotal(items);
    // Wire up the parts autosuggest on every Part-name input
    tbody.querySelectorAll('.bom-part-input').forEach(setupPartsAutosuggest);
    // Async-fill the supplier <select>s for linked rows. Promises are
    // independent per row and deduped via partSuppliersCache.
    tbody.querySelectorAll('tr[data-part-slug]').forEach(async (row) => {
      const slug = row.dataset.partSlug;
      if (!slug) return;
      const sel = row.querySelector('.bom-supplier-select');
      if (!sel) return;
      const suppliers = await fetchPartSuppliers(slug);
      const currentId = sel.dataset.currentSupplierId || '';
      sel.innerHTML = supplierSelectOptions(suppliers, currentId);
    });
  }

  function updateBOMTotal(items) {
    const total = items.reduce((sum, i) => sum + (i.quantity * (i.unit_cost || 0)), 0);
    document.getElementById('bom-total').textContent = '£' + total.toFixed(2);
  }

  window.updateBOM = async function(itemId, input) {
    const row = input.closest('tr');
    const data = {};
    // Issue #149: read both <input> and <select> fields so the currency
    // dropdown is included in the PUT body. Supplier-pricing feature:
    // also pick up the supplier_id <select> when the row is linked.
    row.querySelectorAll('input, select').forEach(inp => {
      const field = inp.dataset.field;
      if (!field) return;
      let val = inp.value;
      if (field === 'quantity') val = parseInt(val) || 1;
      else if (field === 'unit_cost') val = parseFloat(val) || 0;
      else if (field === 'currency_code') {
        const code = String(val || '').trim().toUpperCase();
        // Empty -> null so the server-side pattern doesn't fire on "".
        val = code || null;
      }
      else if (field === 'supplier_id') {
        // Empty string means "— manual price —"; send null so the
        // backend clears the FK and the row falls back to its own
        // unit_cost + currency_code.
        const num = parseInt(val);
        val = Number.isFinite(num) ? num : null;
      }
      data[field] = val;
    });
    const partId = row.dataset.partId;
    if (partId) data.part_id = parseInt(partId);
    let resp;
    try {
      resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/bom/' + itemId, {
        method: 'PUT', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data),
      });
    } catch (e) { loadBOM(); return; }
    // If an older backend rejects the new part_id / currency_code /
    // supplier_id field, retry without those so the user's edit
    // still saves. Treat 4xx as "field rejected"; 5xx is a real
    // server error and we just let it fall through.
    const newFields = partId || 'currency_code' in data || 'supplier_id' in data;
    if (!resp.ok && resp.status >= 400 && resp.status < 500 && newFields) {
      const retryData = Object.assign({}, data);
      delete retryData.part_id;
      delete retryData.currency_code;
      delete retryData.supplier_id;
      try {
        await apiFetch(API + '/api/projects/' + currentProject.id + '/bom/' + itemId, {
          method: 'PUT', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(retryData),
        });
      } catch (_) {}
    }
    loadBOM();
  };

  window.deleteBOM = async function(itemId) {
    await apiFetch(API + '/api/projects/' + currentProject.id + '/bom/' + itemId, {
      method: 'DELETE', credentials: 'include',
    });
    loadBOM();
  };

  // --- Parts catalog autosuggest ---
  // Single shared dropdown reused across all Part-name inputs in the BOM table.
  let partsPopup = null;
  let partsActiveInput = null;
  let partsActiveIdx = 0;
  let partsResults = [];
  let partsDebounceTimer = null;
  let partsAbort = null;

  function ensurePartsPopup() {
    if (partsPopup) return partsPopup;
    partsPopup = document.createElement('div');
    partsPopup.className = 'parts-autocomplete';
    partsPopup.style.display = 'none';
    document.body.appendChild(partsPopup);
    // Re-position on scroll so it tracks the input
    window.addEventListener('scroll', () => {
      if (partsPopup.style.display !== 'none' && partsActiveInput) positionPartsPopup(partsActiveInput);
    }, true);
    return partsPopup;
  }

  function positionPartsPopup(input) {
    const rect = input.getBoundingClientRect();
    partsPopup.style.left = rect.left + 'px';
    partsPopup.style.top = (rect.bottom + 2) + 'px';
    partsPopup.style.minWidth = Math.max(280, rect.width) + 'px';
  }

  function hidePartsPopup() {
    if (partsPopup) partsPopup.style.display = 'none';
    partsActiveInput = null;
    partsResults = [];
    partsActiveIdx = 0;
  }

  function renderPartsPopup(query) {
    const popup = ensurePartsPopup();
    const addRowHtml = `
      <div class="parts-autocomplete-item parts-autocomplete-add" data-action="add">
        <span><i class="fas fa-plus me-1"></i> Add as new part&hellip;${query ? ' &ldquo;' + escapeHtmlAttr(query) + '&rdquo;' : ''}</span>
      </div>`;
    if (!partsResults || partsResults.length === 0) {
      popup.innerHTML = `<div class="parts-autocomplete-empty">No matches yet. Keep typing or add a new part below.</div>${addRowHtml}`;
    } else {
      popup.innerHTML = partsResults.map((p, i) => `
        <div class="parts-autocomplete-item ${i === partsActiveIdx ? 'active' : ''}" data-idx="${i}">
          <div class="parts-ac-main">
            <div class="parts-ac-name">${escapeHtmlAttr(p.name)}</div>
            <div class="parts-ac-meta">
              ${p.sku ? `<span class="parts-ac-sku">${escapeHtmlAttr(p.sku)}</span>` : ''}
              ${typeof p.usage_count === 'number' ? `<span class="parts-ac-usage" title="Used in ${p.usage_count} projects">${p.usage_count} use${p.usage_count === 1 ? '' : 's'}</span>` : ''}
              ${p.status === 'verified' ? `<span class="parts-ac-verified"><i class="fas fa-check-circle"></i>verified</span>` : ''}
            </div>
          </div>
        </div>`).join('') + addRowHtml;
    }
    popup.querySelectorAll('.parts-autocomplete-item').forEach(el => {
      el.addEventListener('mousedown', e => {
        e.preventDefault();
        if (el.dataset.action === 'add') {
          openAddPartModal(query, partsActiveInput);
        } else {
          selectPartsResult(parseInt(el.dataset.idx, 10));
        }
      });
    });
  }

  async function fetchPartsSearch(query) {
    if (partsAbort) partsAbort.abort();
    partsAbort = (typeof AbortController !== 'undefined') ? new AbortController() : null;
    try {
      const resp = await apiFetch(
        API + '/api/parts?q=' + encodeURIComponent(query) + '&limit=10',
        partsAbort ? { signal: partsAbort.signal } : {}
      );
      if (!resp.ok) {
        // Backend may not be live yet — fail soft, just show "add new"
        partsResults = [];
        renderPartsPopup(query);
        return;
      }
      partsResults = await resp.json();
      if (!Array.isArray(partsResults)) partsResults = [];
      partsActiveIdx = 0;
      renderPartsPopup(query);
    } catch (e) {
      // network/abort errors are expected — keep popup usable
      if (e && e.name === 'AbortError') return;
      partsResults = [];
      renderPartsPopup(query);
    }
  }

  function selectPartsResult(idx) {
    if (!partsActiveInput) return;
    const part = partsResults[idx];
    if (!part) return;
    const row = partsActiveInput.closest('tr');
    if (!row) return;
    const itemId = row.dataset.bomId;
    row.dataset.partId = part.id;
    row.dataset.partSlug = part.slug || '';
    // Write the chosen part's display name back into the input
    partsActiveInput.value = part.name || '';
    // Auto-fill supplier URL if the part has one
    const supplierInput = row.querySelector('input[data-field="supplier_url"]');
    if (supplierInput && part.primary_supplier_url) supplierInput.value = part.primary_supplier_url;
    hidePartsPopup();
    // Persist the link via updateBOM (which now picks up dataset.partId)
    if (itemId && supplierInput) window.updateBOM(parseInt(itemId, 10), supplierInput);
    else if (itemId) window.updateBOM(parseInt(itemId, 10), partsActiveInput);
  }

  function setupPartsAutosuggest(input) {
    input.addEventListener('input', () => {
      const q = input.value.trim();
      partsActiveInput = input;
      // Clear any previously linked part_id since the user is typing freely
      const row = input.closest('tr');
      if (row) {
        row.dataset.partId = '';
        row.dataset.partSlug = '';
        const pill = row.querySelector('.bom-part-pill');
        if (pill) pill.remove();
      }
      if (q.length < 2) {
        hidePartsPopup();
        return;
      }
      ensurePartsPopup();
      positionPartsPopup(input);
      partsPopup.style.display = 'block';
      if (partsDebounceTimer) clearTimeout(partsDebounceTimer);
      partsDebounceTimer = setTimeout(() => fetchPartsSearch(q), 250);
    });

    input.addEventListener('focus', () => {
      if (input.value.trim().length >= 2) {
        partsActiveInput = input;
        ensurePartsPopup();
        positionPartsPopup(input);
        partsPopup.style.display = 'block';
        renderPartsPopup(input.value.trim());
        fetchPartsSearch(input.value.trim());
      }
    });

    input.addEventListener('blur', () => {
      // Delay so a click on the popup can register first
      setTimeout(() => {
        if (partsPopup && !partsPopup.matches(':hover')) hidePartsPopup();
      }, 200);
    });

    input.addEventListener('keydown', (e) => {
      if (!partsPopup || partsPopup.style.display === 'none') return;
      const total = partsResults.length + 1; // +1 for the "add new" row
      if (e.key === 'ArrowDown') {
        e.preventDefault();
        partsActiveIdx = (partsActiveIdx + 1) % total;
        renderPartsPopup(input.value.trim());
      } else if (e.key === 'ArrowUp') {
        e.preventDefault();
        partsActiveIdx = (partsActiveIdx - 1 + total) % total;
        renderPartsPopup(input.value.trim());
      } else if (e.key === 'Enter') {
        if (partsActiveIdx === partsResults.length) {
          e.preventDefault();
          openAddPartModal(input.value.trim(), input);
        } else if (partsResults[partsActiveIdx]) {
          e.preventDefault();
          selectPartsResult(partsActiveIdx);
        }
      } else if (e.key === 'Escape') {
        hidePartsPopup();
      }
    });
  }

  // --- "Add new part" modal ---
  function ensureAddPartModal() {
    let modal = document.getElementById('add-part-modal');
    if (modal) return modal;
    modal = document.createElement('div');
    modal.id = 'add-part-modal';
    modal.className = 'modal fade';
    modal.tabIndex = -1;
    modal.setAttribute('aria-hidden', 'true');
    modal.innerHTML = `
      <div class="modal-dialog modal-dialog-centered">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title"><i class="fas fa-plus-circle me-2 text-primary"></i>Add new part</h5>
            <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
          </div>
          <div class="modal-body">
            <div id="add-part-error" class="alert alert-warning d-none small mb-2"></div>
            <div class="mb-2">
              <label class="form-label small text-muted">Part name <span class="text-danger">*</span></label>
              <input type="text" id="add-part-name" class="form-control form-control-sm" placeholder="e.g. SG90 micro servo" maxlength="200">
            </div>
            <div class="row g-2 mb-2">
              <div class="col-6">
                <label class="form-label small text-muted">SKU</label>
                <input type="text" id="add-part-sku" class="form-control form-control-sm" placeholder="optional">
              </div>
              <div class="col-6">
                <label class="form-label small text-muted">MPN</label>
                <input type="text" id="add-part-mpn" class="form-control form-control-sm" placeholder="optional">
              </div>
            </div>
            <div class="mb-2">
              <label class="form-label small text-muted">Supplier URL</label>
              <input type="url" id="add-part-supplier" class="form-control form-control-sm" placeholder="https://...">
            </div>
            <div class="mb-2">
              <label class="form-label small text-muted">Tags (comma-separated)</label>
              <input type="text" id="add-part-tags" class="form-control form-control-sm" placeholder="e.g. servo, motor">
            </div>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-sm btn-outline-secondary" data-bs-dismiss="modal">Cancel</button>
            <button type="button" class="btn btn-sm btn-primary" id="add-part-save">Create part</button>
          </div>
        </div>
      </div>`;
    document.body.appendChild(modal);
    document.getElementById('add-part-save').addEventListener('click', submitAddPart);
    return modal;
  }

  let addPartTargetInput = null;
  function openAddPartModal(prefillName, targetInput) {
    ensureAddPartModal();
    addPartTargetInput = targetInput || null;
    document.getElementById('add-part-name').value = prefillName || '';
    document.getElementById('add-part-sku').value = '';
    document.getElementById('add-part-mpn').value = '';
    document.getElementById('add-part-supplier').value = '';
    document.getElementById('add-part-tags').value = '';
    const errEl = document.getElementById('add-part-error');
    errEl.classList.add('d-none');
    errEl.textContent = '';
    hidePartsPopup();
    const modalEl = document.getElementById('add-part-modal');
    if (window.bootstrap && window.bootstrap.Modal) {
      const m = window.bootstrap.Modal.getOrCreateInstance(modalEl);
      m.show();
      setTimeout(() => document.getElementById('add-part-name').focus(), 200);
    } else {
      // Fallback if Bootstrap JS not loaded for some reason
      modalEl.classList.add('show');
      modalEl.style.display = 'block';
    }
  }

  function closeAddPartModal() {
    const modalEl = document.getElementById('add-part-modal');
    if (window.bootstrap && window.bootstrap.Modal) {
      const m = window.bootstrap.Modal.getInstance(modalEl);
      if (m) m.hide();
    } else if (modalEl) {
      modalEl.classList.remove('show');
      modalEl.style.display = 'none';
    }
  }

  async function submitAddPart() {
    const name = document.getElementById('add-part-name').value.trim();
    if (!name) {
      const errEl = document.getElementById('add-part-error');
      errEl.textContent = 'A name is required.';
      errEl.classList.remove('d-none');
      return;
    }
    const sku = document.getElementById('add-part-sku').value.trim() || null;
    const mpn = document.getElementById('add-part-mpn').value.trim() || null;
    const supplier = document.getElementById('add-part-supplier').value.trim() || null;
    const tagsRaw = document.getElementById('add-part-tags').value.trim();
    const tags = tagsRaw ? tagsRaw.split(',').map(t => t.trim().toLowerCase()).filter(Boolean) : [];
    const body = { name };
    if (sku) body.sku = sku;
    if (mpn) body.mpn = mpn;
    if (supplier) body.supplier_url = supplier;
    if (tags.length) body.tags = tags;

    const saveBtn = document.getElementById('add-part-save');
    saveBtn.disabled = true;
    saveBtn.innerHTML = '<i class="fas fa-spinner fa-spin me-1"></i> Saving...';
    try {
      const resp = await apiFetch(API + '/api/parts', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      if (resp.status === 403) {
        showAccountAgeAlert(await resp.json().catch(() => ({})));
        closeAddPartModal();
        return;
      }
      if (!resp.ok) {
        let detail = 'Could not create part.';
        try { const err = await resp.json(); if (err && err.detail) detail = err.detail; } catch (_) {}
        const errEl = document.getElementById('add-part-error');
        errEl.textContent = detail;
        errEl.classList.remove('d-none');
        return;
      }
      const part = await resp.json();
      closeAddPartModal();
      // If we were called from a BOM row, link the new part to that row
      if (addPartTargetInput) {
        const row = addPartTargetInput.closest('tr');
        if (row) {
          const itemId = row.dataset.bomId;
          row.dataset.partId = part.id;
          row.dataset.partSlug = part.slug || '';
          addPartTargetInput.value = part.name || name;
          const supplierInput = row.querySelector('input[data-field="supplier_url"]');
          if (supplierInput && part.primary_supplier_url) supplierInput.value = part.primary_supplier_url;
          if (itemId) window.updateBOM(parseInt(itemId, 10), addPartTargetInput);
        }
      }
    } catch (e) {
      const errEl = document.getElementById('add-part-error');
      errEl.textContent = 'Network error. Please try again.';
      errEl.classList.remove('d-none');
    } finally {
      saveBtn.disabled = false;
      saveBtn.innerHTML = 'Create part';
    }
  }

  // --- Account-age (14-day) friendly error ---
  function showAccountAgeAlert(payload) {
    // Backend may include a date string (e.g. eligible_at) — if so, show it; otherwise generic.
    const eligibleAt = (payload && (payload.eligible_at || payload.eligibleAt || payload.detail?.eligible_at));
    let dateStr = '';
    if (eligibleAt) {
      try { dateStr = new Date(eligibleAt).toLocaleDateString(); } catch (_) {}
    }
    const msg = dateStr
      ? `You need to have had an account for at least 14 days before editing parts. Try again on ${dateStr}.`
      : `You need to have had an account for at least 14 days before editing parts. Please try again later.`;
    let toast = document.getElementById('parts-age-alert');
    if (!toast) {
      toast = document.createElement('div');
      toast.id = 'parts-age-alert';
      toast.className = 'alert alert-warning alert-dismissible position-fixed shadow';
      toast.style.cssText = 'top: 80px; right: 20px; z-index: 11000; max-width: 360px;';
      document.body.appendChild(toast);
    }
    toast.innerHTML = `<i class="fas fa-clock me-1"></i> ${msg}
      <button type="button" class="btn-close" aria-label="Close"></button>`;
    toast.querySelector('.btn-close').addEventListener('click', () => toast.remove());
    setTimeout(() => { if (toast.parentNode) toast.remove(); }, 8000);
  }
  window.showAccountAgeAlert = showAccountAgeAlert;

  // --- Links (issue #171: BOM-style inline editable table) ---
  //
  // The old flow used three prompt() dialogs in sequence to add a link —
  // cancelling at any step lost the entered data, and there was no way
  // to edit a saved link without deleting and re-creating it. The new
  // table mirrors the BOM editor: "Add Link" POSTs an empty row, then
  // each <input>/<select> onchange PUTs to the new partial-update
  // endpoint added in this same change.
  const LINK_TYPES = [
    { value: 'article',       label: 'Article' },
    { value: 'video',         label: 'Video' },
    { value: 'tutorial',      label: 'Tutorial' },
    { value: 'documentation', label: 'Docs' },
    { value: 'other',         label: 'Other' },
  ];
  function linkTypeOptions(selected) {
    return LINK_TYPES.map(t =>
      `<option value="${t.value}"${t.value === selected ? ' selected' : ''}>${t.label}</option>`
    ).join('');
  }

  document.getElementById('add-link-btn').addEventListener('click', async () => {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    // Empty placeholder row — the user fills in the cells inline. We
    // need a non-empty url to satisfy the LinkCreate schema (it's
    // required), so seed it with a blank-ish placeholder the user
    // will overwrite immediately.
    await apiFetch(API + '/api/projects/' + currentProject.id + '/links', {
      method: 'POST', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ title: '', url: 'https://', link_type: 'article' }),
    });
    loadLinks();
  });

  async function loadLinks() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/links', { credentials: 'include' });
    const links = await resp.json();
    const tbody = document.getElementById('links-body');
    const empty = document.getElementById('links-empty');
    if (!tbody) return;
    if (links.length === 0) {
      tbody.innerHTML = '';
      if (empty) empty.classList.remove('d-none');
      return;
    }
    if (empty) empty.classList.add('d-none');
    tbody.innerHTML = links.map(l => `
      <tr data-link-id="${l.id}">
        <td><input class="form-control form-control-sm" data-field="title" value="${escapeHtmlAttr(l.title)}" maxlength="200" placeholder="Link title" onchange="updateLink(${l.id}, this)"></td>
        <td><input class="form-control form-control-sm" data-field="url" type="url" value="${escapeHtmlAttr(l.url)}" placeholder="https://..." onchange="updateLink(${l.id}, this)"></td>
        <td>
          <select class="form-select form-select-sm" data-field="link_type" onchange="updateLink(${l.id}, this)">
            ${linkTypeOptions(l.link_type)}
          </select>
        </td>
        <td><button class="btn btn-sm text-danger" onclick="deleteLink(${l.id})" title="Delete link"><i class="fas fa-times"></i></button></td>
      </tr>
    `).join('');
  }

  // Track in-flight PUTs per row so a fast-typer doesn't fire ten
  // overlapping requests — we serialise per-row by chaining onto the
  // previous promise. Same idea as the BOM autosave debouncer.
  const _linkSavePending = {};
  window.updateLink = async function(linkId, input) {
    const field = input.dataset.field;
    if (!field) return;
    const payload = { [field]: input.value };
    const prev = _linkSavePending[linkId] || Promise.resolve();
    const next = prev.then(async () => {
      try {
        await apiFetch(API + '/api/projects/' + currentProject.id + '/links/' + linkId, {
          method: 'PUT', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload),
        });
      } catch (_) {
        // Network blip or 4xx — fall back to reloading so the UI
        // reflects what's actually in the DB rather than showing
        // stale local edits.
        loadLinks();
      }
    });
    _linkSavePending[linkId] = next;
  };

  window.deleteLink = async function(linkId) {
    await apiFetch(API + '/api/projects/' + currentProject.id + '/links/' + linkId, {
      method: 'DELETE', credentials: 'include',
    });
    loadLinks();
  };

  // --- Videos (issue #171) ---
  //
  // First-class YouTube video list. Pasting a URL extracts the 11-char
  // id client-side so the thumbnail flips immediately; the backend
  // re-validates server-side and 422s on anything malformed.
  //
  // Same shape as the backend extractor in projects_api/youtube.py —
  // keep the two in sync. Case is preserved (YouTube ids are
  // case-sensitive).
  const YT_ID_RE = /^[A-Za-z0-9_-]{11}$/;
  function extractYouTubeId(s) {
    if (!s) return null;
    const trimmed = String(s).trim();
    if (!trimmed) return null;
    if (YT_ID_RE.test(trimmed)) return trimmed;
    let url;
    try { url = new URL(trimmed.includes('://') ? trimmed : 'https://' + trimmed); }
    catch (_) { return null; }
    let host = url.hostname.toLowerCase();
    if (host.startsWith('www.')) host = host.slice(4);
    let candidate = null;
    if (host === 'youtu.be') {
      candidate = url.pathname.replace(/^\//, '').split('/', 1)[0];
    } else if (host === 'youtube.com' || host === 'm.youtube.com' || host === 'music.youtube.com') {
      const p = url.pathname || '';
      if (p === '/watch' || p.startsWith('/watch/')) {
        candidate = url.searchParams.get('v');
      } else if (p.startsWith('/embed/')) {
        candidate = p.slice('/embed/'.length).split('/', 1)[0];
      } else if (p.startsWith('/shorts/')) {
        candidate = p.slice('/shorts/'.length).split('/', 1)[0];
      } else if (p.startsWith('/v/')) {
        candidate = p.slice('/v/'.length).split('/', 1)[0];
      }
    }
    return (candidate && YT_ID_RE.test(candidate)) ? candidate : null;
  }

  document.getElementById('add-video-btn').addEventListener('click', async () => {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    const input = prompt('Paste a YouTube URL (or 11-char video ID):');
    if (input === null) return;
    const trimmed = String(input || '').trim();
    if (!trimmed) return;
    const id = extractYouTubeId(trimmed);
    if (!id) {
      alert('That doesn\'t look like a YouTube URL. Try one of:\n\nhttps://www.youtube.com/watch?v=…\nhttps://youtu.be/…\nhttps://www.youtube.com/embed/…\nhttps://www.youtube.com/shorts/…\n\nOr paste the 11-character video ID directly.');
      return;
    }
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/videos', {
      method: 'POST', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url_or_id: trimmed }),
    });
    if (!resp.ok) {
      let detail = 'Failed to add video.';
      try { const body = await resp.json(); detail = body.detail || detail; } catch (_) {}
      alert(detail);
      return;
    }
    loadVideos();
  });

  async function loadVideos() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/videos', { credentials: 'include' });
    const videos = await resp.json();
    const tbody = document.getElementById('videos-body');
    const empty = document.getElementById('videos-empty');
    if (!tbody) return;
    if (videos.length === 0) {
      tbody.innerHTML = '';
      if (empty) empty.classList.remove('d-none');
      return;
    }
    if (empty) empty.classList.add('d-none');
    tbody.innerHTML = videos.map(v => `
      <tr data-video-id="${v.id}">
        <td>
          <a href="https://www.youtube.com/watch?v=${encodeURIComponent(v.youtube_id)}" target="_blank" rel="noopener" title="${escapeHtmlAttr(v.youtube_id)}">
            <img src="https://img.youtube.com/vi/${encodeURIComponent(v.youtube_id)}/default.jpg" alt="" loading="lazy" style="width:72px;height:auto;border-radius:3px">
          </a>
        </td>
        <td><code class="small">${escapeHtmlAttr(v.youtube_id)}</code></td>
        <td><input class="form-control form-control-sm" data-field="title" value="${escapeHtmlAttr(v.title || '')}" maxlength="200" placeholder="Optional title" onchange="updateVideo(${v.id}, this)"></td>
        <td>
          <div class="btn-group btn-group-sm" role="group" aria-label="Reorder">
            <button class="btn btn-outline-secondary" onclick="moveVideo(${v.id}, -1)" title="Move up"><i class="fas fa-arrow-up"></i></button>
            <button class="btn btn-outline-secondary" onclick="moveVideo(${v.id}, 1)" title="Move down"><i class="fas fa-arrow-down"></i></button>
          </div>
        </td>
        <td><button class="btn btn-sm text-danger" onclick="deleteVideo(${v.id})" title="Delete video"><i class="fas fa-times"></i></button></td>
      </tr>
    `).join('');
    // Stash the loaded list so moveVideo() can compute neighbour
    // sort_orders without a re-fetch.
    window._videosCache = videos;
  }

  const _videoSavePending = {};
  window.updateVideo = async function(videoId, input) {
    const field = input.dataset.field;
    if (!field) return;
    const payload = { [field]: input.value };
    const prev = _videoSavePending[videoId] || Promise.resolve();
    const next = prev.then(async () => {
      try {
        await apiFetch(API + '/api/projects/' + currentProject.id + '/videos/' + videoId, {
          method: 'PUT', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload),
        });
      } catch (_) { loadVideos(); }
    });
    _videoSavePending[videoId] = next;
  };

  window.moveVideo = async function(videoId, delta) {
    const list = (window._videosCache || []);
    const idx = list.findIndex(v => v.id === videoId);
    if (idx < 0) return;
    const target = idx + delta;
    if (target < 0 || target >= list.length) return;
    // Swap sort_order with the neighbour. Two sequential PUTs are fine
    // here — the list is tiny in practice and the backend has no
    // uniqueness constraint on sort_order so a transient duplicate is
    // harmless.
    const mine = list[idx];
    const neighbour = list[target];
    await apiFetch(API + '/api/projects/' + currentProject.id + '/videos/' + mine.id, {
      method: 'PUT', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ sort_order: neighbour.sort_order }),
    });
    await apiFetch(API + '/api/projects/' + currentProject.id + '/videos/' + neighbour.id, {
      method: 'PUT', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ sort_order: mine.sort_order }),
    });
    loadVideos();
  };

  window.deleteVideo = async function(videoId) {
    if (!confirm('Remove this video from the project?')) return;
    await apiFetch(API + '/api/projects/' + currentProject.id + '/videos/' + videoId, {
      method: 'DELETE', credentials: 'include',
    });
    loadVideos();
  };

  // --- Journal ---
  const journalInput = document.getElementById('journal-input');

  async function postJournalEntry(text) {
    const url = API + '/api/projects/' + currentProject.id + '/journal';
    const body = JSON.stringify({ title: text, status: 'in_progress' });
    const doFetch = () => apiFetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body,
    });
    try {
      return await doFetch();
    } catch (err) {
      // Single retry on transient network failures (Failed to fetch, etc.)
      console.warn('Journal POST failed, retrying once:', err);
      await new Promise(r => setTimeout(r, 250));
      return await doFetch();
    }
  }

  journalInput.addEventListener('keydown', async (e) => {
    if (e.key !== 'Enter') return;
    e.preventDefault();
    const text = journalInput.value.trim();
    if (!text) return;
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    journalInput.disabled = true;
    try {
      const resp = await postJournalEntry(text);
      if (!resp.ok) {
        const bodyText = await resp.text().catch(() => '');
        console.error('Journal add failed:', resp.status, bodyText);
        alert('Could not save journal entry (HTTP ' + resp.status + '). ' + (bodyText || ''));
        return;
      }
      journalInput.value = '';
      loadJournal();
    } catch (err) {
      console.error('Journal add error:', err);
      alert('Could not save journal entry: ' + err.message
        + '\n\nCheck the browser console + Network tab for details.');
    } finally {
      journalInput.disabled = false;
      journalInput.focus();
    }
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
  [titleInput, descInput, difficultySelect, timeInput, repoInput].forEach(el => {
    if (el) {
      el.addEventListener('input', scheduleAutoSave);
      el.addEventListener('change', scheduleAutoSave);
    }
  });

  // --- Global drop zone (issue #117) ---
  // Smart routing: images -> uploadImages(), README.md -> editor content,
  // license files -> upload + add link, everything else -> uploadFiles().
  const ALLOWED_FILE_EXTS = new Set([
    'py', 'cpp', 'h', 'ino', 'md', 'txt', 'pdf',
    'stl', 'obj', 'gcode', '3mf', 'step', 'stp', 'dxf', 'dwg', 'svg',
    'json', 'xml', 'yaml', 'yml', 'csv', 'toml',
    'zip', 'tar', 'gz', '7z', 'rar',
    'scad', 'kicad_pcb', 'kicad_sch', 'brd', 'sch',
    'f3d', 'fcstd',
  ]);
  const IMAGE_EXTS = new Set(['png', 'jpg', 'jpeg', 'gif', 'webp', 'svg']);

  function getExt(name) {
    const i = (name || '').lastIndexOf('.');
    return i === -1 ? '' : name.substring(i + 1).toLowerCase();
  }

  function isImageFile(file) {
    if (file.type && file.type.startsWith('image/')) return true;
    return IMAGE_EXTS.has(getExt(file.name));
  }

  function isReadmeFile(file) {
    const n = (file.name || '').toLowerCase();
    return n === 'readme.md' || n === 'index.md';
  }

  function isLicenseFile(file) {
    const n = (file.name || '').toLowerCase();
    return n === 'license' || n === 'license.md' || n === 'license.txt'
      || n === 'licence' || n === 'licence.md' || n === 'licence.txt';
  }

  function isAcceptedFile(file) {
    if (isImageFile(file)) return true;
    const ext = getExt(file.name);
    if (ALLOWED_FILE_EXTS.has(ext)) return true;
    // LICENSE without extension
    const bare = (file.name || '').toLowerCase();
    if (bare === 'license' || bare === 'licence') return true;
    return false;
  }

  // Drop progress toast
  const dropProgressEl = document.getElementById('drop-progress');
  const dropProgressBody = dropProgressEl && dropProgressEl.querySelector('.drop-progress__body');
  const dropProgressTitle = dropProgressEl && dropProgressEl.querySelector('.drop-progress__title');
  const dropProgressClose = document.getElementById('drop-progress-close');
  if (dropProgressClose) dropProgressClose.addEventListener('click', () => dropProgressEl.classList.add('d-none'));

  function showProgress() {
    if (!dropProgressEl) return;
    dropProgressBody.innerHTML = '';
    dropProgressTitle.innerHTML = '<i class="fas fa-spinner fa-spin me-1"></i> Processing files...';
    dropProgressEl.classList.remove('d-none');
  }
  function addProgressLine(name, status) {
    if (!dropProgressBody) return null;
    const row = document.createElement('div');
    row.className = 'drop-progress__row';
    row.innerHTML = '<span class="drop-progress__name"></span><span class="drop-progress__status"></span>';
    row.querySelector('.drop-progress__name').textContent = name;
    row.querySelector('.drop-progress__status').innerHTML = status || '<i class="fas fa-spinner fa-spin"></i>';
    dropProgressBody.appendChild(row);
    return row;
  }
  function updateProgressLine(row, html) {
    if (!row) return;
    row.querySelector('.drop-progress__status').innerHTML = html;
  }
  function finishProgress(summary) {
    if (!dropProgressEl) return;
    dropProgressTitle.innerHTML = '<i class="fas fa-check-circle text-success me-1"></i> ' + summary;
    setTimeout(() => { dropProgressEl.classList.add('d-none'); }, 4000);
  }

  // Upload a single file via the files API and return the response JSON.
  async function uploadSingleFile(file) {
    const form = new FormData();
    form.append('file', file);
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/files', {
      method: 'POST', credentials: 'include', body: form,
    });
    if (!resp.ok) {
      let detail = 'upload failed';
      try { const j = await resp.json(); detail = j.detail || detail; } catch (e) {}
      throw new Error(detail);
    }
    return resp.json();
  }

  // Upload a single image via the images API.
  async function uploadSingleImage(file) {
    const form = new FormData();
    form.append('file', file);
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/images', {
      method: 'POST', credentials: 'include', body: form,
    });
    if (!resp.ok) {
      let detail = 'upload failed';
      try { const j = await resp.json(); detail = j.detail || detail; } catch (e) {}
      throw new Error(detail);
    }
    return resp.json();
  }

  async function readFileAsText(file) {
    return new Promise((resolve, reject) => {
      const r = new FileReader();
      r.onload = () => resolve(r.result);
      r.onerror = () => reject(r.error);
      r.readAsText(file);
    });
  }

  async function handleReadmeDrop(file, row) {
    const text = await readFileAsText(file);
    const current = getEditorValue() || '';
    if (current.trim().length > 50) {
      if (!confirm('Replace existing editor content with ' + file.name + '? Current content is ' + current.length + ' characters.')) {
        updateProgressLine(row, '<span class="text-muted">skipped</span>');
        return false;
      }
    }
    setEditorValue(text);
    updateProgressLine(row, '<span class="text-success"><i class="fas fa-check"></i> loaded into editor</span>');
    return true;
  }

  async function handleLicenseDrop(file, row) {
    // Upload as a regular file, then add a link pointing to its download URL.
    const uploaded = await uploadSingleFile(file);
    try {
      const downloadUrl = API + '/api/projects/' + currentProject.id + '/files/' + uploaded.id + '/download';
      await apiFetch(API + '/api/projects/' + currentProject.id + '/links', {
        method: 'POST', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ title: 'License', url: downloadUrl, link_type: 'documentation' }),
      });
    } catch (e) { /* link add failed — file still uploaded */ }
    updateProgressLine(row, '<span class="text-success"><i class="fas fa-check"></i> uploaded &amp; linked</span>');
  }

  async function processDroppedFiles(fileList) {
    const files = Array.from(fileList || []);
    if (files.length === 0) return;

    // Ensure project exists before uploading.
    if (!currentProject) {
      await saveProject();
      if (!currentProject) {
        alert('Please add a title and save before dropping files.');
        return;
      }
    }

    showProgress();

    // Filter unsupported files up-front with a warning row.
    const accepted = [];
    for (const f of files) {
      if (!isAcceptedFile(f)) {
        const r = addProgressLine(f.name, '<span class="text-warning"><i class="fas fa-exclamation-triangle"></i> unsupported type</span>');
        continue;
      }
      accepted.push(f);
    }

    let imageCount = 0;
    let fileCount = 0;
    let readmeCount = 0;
    let licenseCount = 0;
    let errorCount = 0;
    let needsImageReload = false;
    let needsFileReload = false;
    let needsLinksReload = false;

    for (const file of accepted) {
      const row = addProgressLine(file.name, '<i class="fas fa-spinner fa-spin"></i>');
      try {
        if (isReadmeFile(file)) {
          const ok = await handleReadmeDrop(file, row);
          if (ok) readmeCount++;
        } else if (isLicenseFile(file)) {
          await handleLicenseDrop(file, row);
          licenseCount++;
          needsFileReload = true;
          needsLinksReload = true;
        } else if (isImageFile(file)) {
          await uploadSingleImage(file);
          updateProgressLine(row, '<span class="text-success"><i class="fas fa-check"></i> added to gallery</span>');
          imageCount++;
          needsImageReload = true;
        } else {
          await uploadSingleFile(file);
          updateProgressLine(row, '<span class="text-success"><i class="fas fa-check"></i> uploaded</span>');
          fileCount++;
          needsFileReload = true;
        }
      } catch (e) {
        updateProgressLine(row, '<span class="text-danger"><i class="fas fa-times"></i> ' + (e.message || 'failed') + '</span>');
        errorCount++;
      }
    }

    if (needsImageReload) loadImages();
    if (needsFileReload) loadFiles();
    if (needsLinksReload) loadLinks();

    scheduleAutoSave();

    const parts = [];
    if (imageCount) parts.push(imageCount + ' image' + (imageCount === 1 ? '' : 's'));
    if (fileCount) parts.push(fileCount + ' file' + (fileCount === 1 ? '' : 's'));
    if (readmeCount) parts.push('README');
    if (licenseCount) parts.push('license');
    let summary = parts.length ? 'Added ' + parts.join(', ') : 'Done';
    if (errorCount) summary += ' (' + errorCount + ' failed)';
    finishProgress(summary);
  }

  // Set up global drag overlay using a counter pattern.
  (function setupGlobalDropzone() {
    const overlay = document.getElementById('global-dropzone');
    if (!overlay) return;
    let dragDepth = 0;

    function hasFiles(e) {
      if (!e.dataTransfer) return false;
      const types = e.dataTransfer.types;
      if (!types) return false;
      for (let i = 0; i < types.length; i++) {
        if (types[i] === 'Files' || types[i] === 'application/x-moz-file') return true;
      }
      return false;
    }

    document.addEventListener('dragenter', (e) => {
      if (!hasFiles(e)) return;
      dragDepth++;
      overlay.classList.remove('d-none');
      overlay.classList.add('is-active');
    });

    document.addEventListener('dragover', (e) => {
      if (!hasFiles(e)) return;
      e.preventDefault();
      if (e.dataTransfer) e.dataTransfer.dropEffect = 'copy';
    });

    document.addEventListener('dragleave', (e) => {
      if (!hasFiles(e)) return;
      dragDepth = Math.max(0, dragDepth - 1);
      if (dragDepth === 0) {
        overlay.classList.add('d-none');
        overlay.classList.remove('is-active');
      }
    });

    document.addEventListener('drop', (e) => {
      if (!hasFiles(e)) return;
      e.preventDefault();
      dragDepth = 0;
      overlay.classList.add('d-none');
      overlay.classList.remove('is-active');
      // No exceptions — the entire editor page is one drop target, and
      // processDroppedFiles() routes by type: images → gallery,
      // README/index.md → markdown editor, LICENSE → linked file,
      // anything else in ALLOWED_FILE_EXTS → Files & Models.
      const files = e.dataTransfer && e.dataTransfer.files;
      if (files && files.length) processDroppedFiles(files);
    });

    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && !overlay.classList.contains('d-none')) {
        dragDepth = 0;
        overlay.classList.add('d-none');
        overlay.classList.remove('is-active');
      }
    });
  })();

  // --- Completeness meter (issue #137) ---
  // Reads in-memory editor state only. No API calls. Updates on a
  // requestAnimationFrame cadence — completely separate from the 3s autosave.
  const COMPLETENESS_ITEMS = [
    { key: 'title',  points: 10, label: 'Add a title',                         target: 'project-title',
      check: () => titleInput && titleInput.value.trim().length >= 3 },
    { key: 'desc',   points: 10, label: 'Write a short description',           target: 'project-description',
      check: () => descInput && descInput.value.trim().length >= 10 },
    { key: 'cover',  points: 15, label: 'Upload a cover image',                target: 'image-input',
      check: () => document.querySelectorAll('#image-gallery [data-image-id]').length > 0 },
    { key: 'body',   points: 20, label: 'Write 200+ chars of build steps',     target: 'content-editor',
      check: () => (easyMDE ? easyMDE.value().trim().length : (contentEditor ? contentEditor.value.trim().length : 0)) >= 200 },
    { key: 'diff',   points: 5,  label: 'Set a difficulty level',              target: 'project-difficulty',
      check: () => difficultySelect && difficultySelect.value !== '' },
    { key: 'time',   points: 5,  label: 'Estimate build time',                 target: 'project-time',
      check: () => !!(timeInput && timeInput.value) && Number(timeInput.value) > 0 },
    { key: 'tags',   points: 5,  label: 'Add at least one tag',                target: 'tag-input',
      check: () => tagsContainer && tagsContainer.querySelectorAll('.tag-badge').length > 0 },
    { key: 'bom',    points: 10, label: 'Add at least one BOM item',           target: 'add-bom-btn',
      check: () => document.querySelectorAll('#bom-body tr').length > 0 },
    { key: 'repo',   points: 10, label: 'Link a code repository',              target: 'project-repo',
      check: () => repoInput && repoInput.value.trim().length > 0 },
    // Issue #171: a YouTube video counts toward "extra content" too —
    // it's just as much a doc artefact as a file or a journal entry.
    { key: 'extra',  points: 10, label: 'Add a file, journal entry, or video', target: 'journal-input',
      check: () => document.querySelectorAll('#file-list tbody tr').length > 0
                || document.querySelectorAll('#journal-list .journal-entry').length > 0
                || document.querySelectorAll('#videos-body tr').length > 0 },
  ];

  let completenessFrame = null;
  function recomputeCompleteness() {
    if (completenessFrame) cancelAnimationFrame(completenessFrame);
    completenessFrame = requestAnimationFrame(() => {
      completenessFrame = null;
      _doRecomputeCompleteness();
    });
  }

  function _bandClass(pct) {
    if (pct >= 90) return 'completeness-bar-gold';
    if (pct >= 70) return 'completeness-bar-green';
    if (pct >= 40) return 'completeness-bar-amber';
    return 'completeness-bar-red';
  }

  function _doRecomputeCompleteness() {
    const cardEl = document.getElementById('completeness-card');
    const scoreEl = document.getElementById('completeness-score');
    const pointsEl = document.getElementById('completeness-points');
    const barEl = document.getElementById('completeness-bar');
    const nextEl = document.getElementById('completeness-next');
    const nextLink = document.getElementById('completeness-next-link');
    const listEl = document.getElementById('completeness-checklist');
    const celebrateEl = document.getElementById('completeness-celebration');
    const completeBadge = document.getElementById('completeness-complete-badge');
    const cardBody = cardEl && cardEl.querySelector('.card-body');
    if (!scoreEl || !barEl || !listEl) return;

    let earned = 0;
    const results = COMPLETENESS_ITEMS.map(item => {
      let met = false;
      try { met = !!item.check(); } catch (e) { met = false; }
      if (met) earned += item.points;
      return { item, met };
    });
    const pct = Math.round(earned);

    scoreEl.textContent = pct + '%';
    if (pointsEl) pointsEl.textContent = earned + ' / 100';
    barEl.style.width = pct + '%';
    barEl.setAttribute('aria-valuenow', String(pct));
    barEl.className = 'progress-bar ' + _bandClass(pct);

    // At 100%, collapse the card body to just the header. The header gets
    // a small "Complete" pill so the user can still see status at a glance
    // without the whole checklist taking up sidebar space.
    if (pct >= 100) {
      if (cardBody) cardBody.classList.add('d-none');
      if (completeBadge) completeBadge.classList.remove('d-none');
      return;
    }
    if (cardBody) cardBody.classList.remove('d-none');
    if (completeBadge) completeBadge.classList.add('d-none');
    if (celebrateEl) celebrateEl.classList.add('d-none');
    listEl.classList.remove('d-none');

    // Next suggestion: highest-point unmet item — still ordered by point
    // value internally (biggest wins surfaced first) but no "+N" suffix
    // shown in the UI, per design feedback that the numbers cluttered.
    const unmet = results.filter(r => !r.met).sort((a, b) => b.item.points - a.item.points);
    if (unmet.length && nextEl && nextLink) {
      nextLink.textContent = unmet[0].item.label;
      nextLink.dataset.target = unmet[0].item.target;
      nextEl.classList.remove('d-none');
    } else if (nextEl) {
      nextEl.classList.add('d-none');
    }

    listEl.innerHTML = results.map(({ item, met }) => {
      if (met) {
        return '<li class="completeness-item completeness-item-met text-muted py-1">'
             + '<i class="fas fa-circle-check me-2"></i>'
             + '<span>' + item.label + '</span>'
             + '</li>';
      }
      return '<li class="completeness-item completeness-item-unmet py-1">'
           + '<i class="far fa-circle me-2 text-primary"></i>'
           + '<a href="#" class="completeness-link text-decoration-none" data-target="' + item.target + '">' + item.label + '</a>'
           + '</li>';
    }).join('');
  }

  function focusCompletenessTarget(id) {
    const el = document.getElementById(id);
    if (!el) return;
    try {
      el.scrollIntoView({ behavior: 'smooth', block: 'center' });
    } catch (e) {
      el.scrollIntoView();
    }
    // Special-case the markdown editor: focus CodeMirror, not the hidden textarea.
    if (id === 'content-editor' && easyMDE && easyMDE.codemirror) {
      setTimeout(() => easyMDE.codemirror.focus(), 250);
      return;
    }
    setTimeout(() => { try { el.focus(); } catch (_) {} }, 250);
  }

  document.addEventListener('click', (e) => {
    const link = e.target.closest('.completeness-link, #completeness-next-link');
    if (!link) return;
    const target = link.dataset.target;
    if (!target) return;
    e.preventDefault();
    focusCompletenessTarget(target);
  });

  // Fan-in: same set as autosave, but on a separate (fast) cadence.
  [titleInput, descInput, difficultySelect, timeInput, repoInput].forEach(el => {
    if (el) {
      el.addEventListener('input', recomputeCompleteness);
      el.addEventListener('change', recomputeCompleteness);
    }
  });

  // EasyMDE body changes
  function _wireEasyMDEForCompleteness() {
    if (easyMDE && easyMDE.codemirror) {
      easyMDE.codemirror.on('change', recomputeCompleteness);
      return true;
    }
    return false;
  }
  if (!_wireEasyMDEForCompleteness()) {
    const _wireTimer = setInterval(() => {
      if (_wireEasyMDEForCompleteness()) clearInterval(_wireTimer);
    }, 200);
  }

  // Watch DOM sections we render imperatively (tags badges, BOM rows, gallery,
  // file list, journal list). MutationObserver keeps the meter in sync without
  // touching every render function call site.
  function _observe(id) {
    const el = document.getElementById(id);
    if (!el) return;
    new MutationObserver(recomputeCompleteness).observe(el, { childList: true, subtree: true });
  }
  _observe('tags-container');
  _observe('bom-body');
  _observe('image-gallery');
  _observe('file-list');
  _observe('journal-list');
  // Issue #171: re-score the completeness meter when a video lands.
  _observe('videos-body');

  // Expose so loadProject() / init can trigger explicitly.
  window._recomputeCompleteness = recomputeCompleteness;

  // Initialise Bootstrap popover for the info button (if Bootstrap is present).
  (function initCompletenessPopover() {
    const btn = document.getElementById('completeness-info');
    if (!btn || typeof bootstrap === 'undefined' || !bootstrap.Popover) return;
    new bootstrap.Popover(btn);
  })();

  // First paint so the meter shows 0% (new projects) or actual score (after load).
  recomputeCompleteness();

  // --- Init ---
  async function init() {
    const authed = await checkAuth();
    if (!authed) return;
    initEditor();
    if (projectId) await loadProject();
    editorContainer.classList.remove('d-none');
    recomputeCompleteness();
  }

  init();
})();
