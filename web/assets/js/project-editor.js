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
      setEditorValue(currentProject.content_md);
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
      content_md: getEditorValue() || null,
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
            // Render mermaid in the preview pane
            setTimeout(async () => {
              var previewEl = container.querySelector('.editor-preview-active');
              if (!previewEl) return;
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
      const m = marker.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
      const re = new RegExp('^' + m + '([\\s\\S]*)' + m + '$');
      const match = sel.match(re);
      if (match) {
        cm.replaceSelection(match[1]);
      } else {
        // Check if selection is bordered by marker outside
        const from = cm.getCursor('from');
        const to = cm.getCursor('to');
        const lineFrom = cm.getLine(from.line);
        const lineTo = cm.getLine(to.line);
        const before = lineFrom.substring(Math.max(0, from.ch - marker.length), from.ch);
        const after = lineTo.substring(to.ch, to.ch + marker.length);
        if (before === marker && after === marker) {
          cm.replaceRange('', { line: to.line, ch: to.ch }, { line: to.line, ch: to.ch + marker.length });
          cm.replaceRange('', { line: from.line, ch: from.ch - marker.length }, { line: from.line, ch: from.ch });
        } else {
          cm.replaceSelection(marker + sel + marker);
        }
      }
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
        case 'link':
          cm.replaceSelection('[' + (sel || 'text') + '](https://)');
          break;
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
    list.innerHTML = `<table class="table table-sm table-hover mb-0"><tbody>${files.map(f => `
      <tr>
        <td class="text-muted" style="width:20px"><i class="${fileIcon(f.filename)}"></i></td>
        <td><a href="${API}/api/projects/${currentProject.id}/files/${f.id}/download">${f.filename}</a></td>
        <td class="text-muted small">${fileKind(f.filename)}</td>
        <td class="text-muted small text-end" style="width:70px">${(f.file_size / 1024).toFixed(1)} KB</td>
        <td style="width:30px"><button class="btn btn-sm text-danger p-0" onclick="deleteFile(${f.id})"><i class="fas fa-trash fa-xs"></i></button></td>
      </tr>
    `).join('')}</tbody></table>`;
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

  imageDropzone.addEventListener('drop', e => {
    // Split mixed drops: images go to the gallery, non-images get routed by
    // the smart processor (files / README / license) so .dxf etc. don't 400.
    const all = Array.from(e.dataTransfer.files || []);
    const images = all.filter(isImageFile);
    const others = all.filter(f => !isImageFile(f));
    if (images.length) uploadImages(images);
    if (others.length) processDroppedFiles(others);
  });
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
      // Don't intercept drops on the sidebar dropzones — they handle their own files.
      const target = e.target;
      if (target && (target.closest('#file-dropzone') || target.closest('#image-dropzone'))) {
        return;
      }
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

  // --- Init ---
  async function init() {
    const authed = await checkAuth();
    if (!authed) return;
    initEditor();
    if (projectId) await loadProject();
    editorContainer.classList.remove('d-none');
  }

  init();
})();
