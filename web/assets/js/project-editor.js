/**
 * Project Editor — vanilla JS for creating/editing projects.
 * Talks to the Projects API at projects.kevsrobots.com.
 */
(function () {
  const API = 'https://projects.kevsrobots.com';
  // Grammar, Look Up and Synonyms (stacks/language-tools). Spelling works without it.
  const LANGUAGE_API = 'https://language.kevsrobots.com';
  const params = new URLSearchParams(window.location.search);
  const projectId = params.get('id');
  let currentProject = null;
  let autoSaveTimer = null;

  // DOM elements
  const editorContainer = document.getElementById('editor-container');
  const authRequired = document.getElementById('auth-required');
  const titleInput = document.getElementById('project-title');
  const titleWarningEl = document.getElementById('title-needs-attention');
  const descInput = document.getElementById('project-description');

  // Issue: early-adopter projects all defaulted to "Untitled Project".
  // Show the warning banner + flag the input whenever the title is empty
  // or still the literal default. Called on load + every input change so
  // the nudge disappears the moment they start typing a real name.
  function updateTitleWarning() {
    if (!titleInput || !titleWarningEl) return;
    const v = (titleInput.value || '').trim();
    const needsWork = !v || v.toLowerCase() === 'untitled project';
    titleWarningEl.classList.toggle('d-none', !needsWork);
    titleInput.classList.toggle('border-warning', needsWork);
    titleInput.classList.toggle('border-2', needsWork);
    if (needsWork) {
      titleInput.setAttribute('aria-invalid', 'true');
    } else {
      titleInput.removeAttribute('aria-invalid');
    }
  }
  if (titleInput) {
    titleInput.addEventListener('input', updateTitleWarning);
  }

  const contentEditor = document.getElementById('content-editor');
  const contentPreview = document.getElementById('content-preview');
  const difficultySelect = document.getElementById('project-difficulty');
  const timeInput = document.getElementById('project-time');
  const tagsContainer = document.getElementById('tags-container');
  const tagInput = document.getElementById('tag-input');
  const publishSwitch = document.getElementById('publish-switch');
  const publishSwitchLabel = document.getElementById('publish-switch-label');
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
      updateTitleWarning();
      descInput.value = currentProject.short_description || '';
      setEditorValue(withRepoSection(currentProject.content_md, currentProject.code_repo_url));
      difficultySelect.value = currentProject.difficulty || '';
      timeInput.value = currentProject.estimated_minutes || '';
      updateStatusBadge(currentProject.status);
      renderTags(currentProject.tags || []);
      loadVideos();
      loadFiles();
      loadImages();
    } catch (e) {
      console.error('Failed to load project:', e);
    }
  }

  // --- Code repository section ---
  // The repository link is a `/` section of the document ("Code
  // repository", project-sections.js). The project's code_repo_url is
  // kept in step with the first link in that section, so the view page's
  // View Code button and the README export still have it.
  const REPO_SECTION = 'code-repository';
  const URL_RE = /https?:\/\/[^\s<>()\[\]]+/;

  // The section's first link, '' when the section has none yet, or null
  // when there's no section.
  function repoUrlFromMarkdown(markdown) {
    let found = null, inSection = false, level = 0, fence = null;
    for (const line of String(markdown || '').split('\n')) {
      const f = line.match(/^ {0,3}(`{3,}|~{3,})/);
      if (f) {
        if (fence === null) fence = f[1];
        else if (f[1][0] === fence[0] && f[1].length >= fence.length) fence = null;
        continue;
      }
      if (fence !== null) continue;
      const h = line.match(/^ {0,3}(#{1,6})\s+(.+?)\s*#*\s*$/);
      if (h) {
        const section = window.ProjectSections && ProjectSections.match(h[2]);
        if (inSection && h[1].length <= level) inSection = false;
        if (section && section.id === REPO_SECTION) { inSection = true; level = h[1].length; found = ''; }
        continue;
      }
      if (inSection) {
        const m = line.match(URL_RE);
        if (m) return m[0].replace(/[.,;:!?'"]+$/, '');
      }
    }
    return found;
  }

  // A project saved before the section existed has only code_repo_url:
  // give it the section, so the link stays editable. It's saved with the
  // next edit.
  function withRepoSection(markdown, url) {
    const md = markdown || '';
    if (!url || repoUrlFromMarkdown(md) !== null) return md;
    const heading = ProjectSections.get(REPO_SECTION).heading;
    return md.replace(/\s*$/, '') + (md.trim() ? '\n\n' : '') + '# ' + heading + '\n\n' + url + '\n';
  }

  // --- Save ---
  function showSaveStatus(status, msg) {
    saveIndicator.className = 'save-' + status;
    saveIndicator.textContent = msg;
    if (status === 'saved') setTimeout(() => { saveIndicator.textContent = ''; }, 2000);
  }

  async function saveProject() {
    const content = getEditorValue();
    const data = {
      title: titleInput.value || 'Untitled Project',
      short_description: descInput.value || null,
      content_md: content || null,
      difficulty: difficultySelect.value || null,
      estimated_minutes: timeInput.value ? parseInt(timeInput.value) : null,
      code_repo_url: repoUrlFromMarkdown(content) || null,
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
        // Issue #260: a new project is a private draft until published.
        resp = await apiFetch(API + '/api/projects', {
          method: 'POST', credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(Object.assign({ status: 'draft' }, data)),
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
      // The server has this text now, so the editor's local draft of it
      // can go (unless more was typed while the save was in flight).
      if (mde && mde.getDoc() === (data.content_md || '')) mde.clearDraft();
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

  // The top bar's switch is Draft / Published (issue #260). Drafts are
  // private to the author; switching on publishes as ``completed``. (A
  // legacy ``wip`` row from an API client also counts as published.)
  // The Archived checkbox beside it is the third state; publishing again,
  // or unticking it, un-archives (back to a draft).
  async function setStatus(status) {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    try {
      const resp = await apiFetch(API + '/api/projects/' + currentProject.id, {
        method: 'PUT', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ status: status }),
      });
      if (resp.ok) {
        currentProject = await resp.json();
        updateStatusBadge(currentProject.status);
        showSaveStatus('saved', 'Status updated');
        if (window.BadgeToast) BadgeToast.fromResponse(currentProject);
      } else if (resp.status === 422 && status === 'draft') {
        // A projects API from before issue #260 doesn't know "draft" yet.
        showSaveStatus('error', "Couldn't set Draft: the projects API needs updating to support drafts");
        updateStatusBadge(currentProject.status);
      } else {
        throw new Error('HTTP ' + resp.status);
      }
    } catch (e) {
      console.error(e);
      showSaveStatus('error', 'Status not saved');
      updateStatusBadge(currentProject ? currentProject.status : 'draft');
    }
  }

  if (publishSwitch) publishSwitch.addEventListener('change', () => {
    setStatus(publishSwitch.checked ? 'completed' : 'draft');
  });

  const archiveCheck = document.getElementById('archive-check');
  if (archiveCheck) archiveCheck.addEventListener('change', () => {
    setStatus(archiveCheck.checked ? 'archived' : 'draft');
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

  // The switch only ever reads Draft or Published; Archived is shown by
  // its own checkbox.
  function updateStatusBadge(status) {
    const published = status === 'wip' || status === 'completed';
    if (publishSwitch) publishSwitch.checked = published;
    if (publishSwitchLabel) publishSwitchLabel.textContent = published ? 'Published' : 'Draft';
    if (archiveCheck) archiveCheck.checked = status === 'archived';
    const viewToggle = document.getElementById('view-toggle-btn');
    if (currentProject) {
      // Issue #152: link to the canonical /projects/<owner>/<slug> URL
      // when the loaded project has a slug; fall back to ?id= for any
      // legacy row whose slug hasn't been backfilled yet.
      var liveUrl = (currentProject.slug && currentProject.author_username)
        ? '/projects/' + encodeURIComponent(currentProject.author_username) + '/' + encodeURIComponent(currentProject.slug)
        : '/projects/view.html?id=' + currentProject.id;
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

  // Debounced trigger fired from the editor's change handler.
  let autoTagTimer = null;
  function scheduleAutoTagDetection() {
    if (autoTagTimer) clearTimeout(autoTagTimer);
    autoTagTimer = setTimeout(() => {
      autoTagTimer = null;
      if (mde) applyAutoDetectedTags(mde.getDoc());
    }, 1000);
  }

  // Tags are typed straight into the row under the description: Enter or
  // a comma adds one, and Backspace in the empty box removes the last.
  function addTagFromInput() {
    const tag = tagInput.value.trim().replace(/,+$/, '').trim().toLowerCase();
    if (!tag) { tagInput.value = ''; return; }
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
  }

  tagInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' || e.key === ',') {
      e.preventDefault();
      addTagFromInput();
    } else if (e.key === 'Backspace' && !tagInput.value) {
      const last = tagsContainer.querySelector('.tag-badge:last-child');
      if (last) { last.remove(); scheduleAutoSave(); }
    }
  });
  tagInput.addEventListener('blur', addTagFromInput);

  // --- Markdown editor ---
  // markdown-studio's editor bundle (assets/js/markdown-editor/, issue #255),
  // in place of EasyMDE. The live view, selection toolbar, find & replace,
  // spelling, image picker, drafts and status bar come with it; this wires
  // it to the Projects API and draws ```mermaid blocks as diagrams. There's
  // no separate Preview: the live view is the preview, with Raw for source.
  const MAX_IMAGE_BYTES = 10 * 1024 * 1024; // the API's max_image_size
  const MERMAID_URL = 'https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.esm.min.mjs';
  const DIAGRAM_TEMPLATES = [
    { label: 'Flowchart', md: '\n```mermaid\ngraph TD\n    A[Start] --> B[Step 1]\n    B --> C[Step 2]\n    C --> D[End]\n```\n' },
    { label: 'Sequence Diagram', md: '\n```mermaid\nsequenceDiagram\n    participant A\n    participant B\n    A->>B: Hello\n    B-->>A: Hi back\n```\n' },
    { label: 'State Diagram', md: '\n```mermaid\nstateDiagram-v2\n    [*] --> Idle\n    Idle --> Running\n    Running --> Idle\n    Running --> [*]\n```\n' },
  ];
  let mde = null;
  let mermaidLib = null;

  function imageViewUrl(img) {
    return API + '/api/projects/' + currentProject.id + '/images/' + img.id + '/view';
  }

  async function fetchProjectImages() {
    if (!currentProject) return [];
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/images', { credentials: 'include' });
    return resp.ok ? resp.json() : [];
  }

  // Pasted, dropped and picked images go into the project's gallery.
  async function uploadEditorImage(file) {
    if (!currentProject) {
      await saveProject();
      if (!currentProject) throw new Error('Add a title and save before adding images');
    }
    const img = await uploadSingleImage(file);
    loadImages();
    return imageViewUrl(img);
  }

  // A ```mermaid block in the live view: its diagram, until the cursor goes in.
  let mermaidCount = 0;
  async function renderMermaid(source, element) {
    if (!mermaidLib) {
      mermaidLib = (await import(MERMAID_URL)).default;
      mermaidLib.initialize({ startOnLoad: false });
    }
    const { svg } = await mermaidLib.render('project-mermaid-' + (++mermaidCount), source);
    element.innerHTML = svg;
  }

  function initEditor() {
    if (mde) return;
    // The default toolbar, with Raw in place of Preview and Side by side.
    const toolbar = MarkdownEditor.DEFAULT_TOOLBAR
      .filter(item => item !== 'preview' && item !== 'side-by-side');
    toolbar.splice(toolbar.indexOf('fullscreen'), 0, 'raw');
    toolbar.splice(toolbar.indexOf('table') + 1, 0, {
      id: 'diagram',
      label: 'Insert diagram',
      icon: 'M2 2.5h5v4H2zM9 9.5h5v4H9zM4.5 6.5v5H9',
      run: (editor, button) => showDiagramMenu(editor, button),
    });

    mde = MarkdownEditor.mount(contentEditor, {
      placeholder: 'Write your project content in Markdown...\n\n# Background\nDescribe what this project is about...\n\n## How to Build\n- Step 1: ...',
      minHeight: 350,
      theme: 'light',
      toolbar: toolbar,
      draft: { key: 'projects.' + (projectId || 'new') },
      uploadImage: uploadEditorImage,
      // Several images dropped on the text at once go to the gallery only.
      addImages: files => processDroppedFiles(files),
      imageLibrary: async () => (await fetchProjectImages()).map(img => ({
        url: imageViewUrl(img), alt: img.caption || img.filename,
      })),
      images: { maxBytes: MAX_IMAGE_BYTES, libraryName: 'the gallery' },
      preview: false,
      codeRenderers: { mermaid: renderMermaid },
      slashCommands: ProjectSlashCommands.create(slashContext()),
      proofing: { languageApiBase: LANGUAGE_API, languageApi: languageRequest },
    });

    mde.on('change', () => {
      scheduleAutoSave();
      scheduleAutoTagDetection();
    });
    setupImageAutocomplete(mde);
    // For the TOC in editor.html and other page scripts.
    window.projectMarkdownEditor = mde;
    document.dispatchEvent(new CustomEvent('project-markdown-editor:ready', { detail: mde }));
  }

  // The editor's language requests, through apiFetch so the dev token goes too.
  async function languageRequest(path, options) {
    const body = options && options.body;
    const resp = await apiFetch(LANGUAGE_API + path, body === undefined
      ? { headers: { Accept: 'application/json' } }
      : { method: 'POST', headers: { Accept: 'application/json', 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    if (!resp.ok) throw Object.assign(new Error(String(resp.status)), { status: resp.status });
    return resp.json();
  }

  // What the Project `/` commands (issue #257) read from and upload to.
  function fileDownloadUrl(f) {
    return API + '/api/projects/' + currentProject.id + '/files/' + f.id + '/download';
  }

  function slashContext() {
    const getJson = async path => {
      if (!currentProject) return [];
      const resp = await apiFetch(API + '/api/projects/' + currentProject.id + path, { credentials: 'include' });
      return resp.ok ? resp.json() : [];
    };
    return {
      listFiles: async () => (await getJson('/files')).map(f => ({
        name: f.filename, description: f.description, url: fileDownloadUrl(f),
      })),
      modelFiles: () => currentProject ? projectFiles.map(f => ({ name: f.filename, url: fileDownloadUrl(f) })) : [],
      uploadFile: () => new Promise(resolve => {
        const input = document.createElement('input');
        input.type = 'file';
        input.addEventListener('cancel', () => resolve(null));
        input.addEventListener('change', async () => {
          const file = input.files && input.files[0];
          if (!file) { resolve(null); return; }
          try {
            if (!currentProject) await saveProject();
            if (!currentProject) throw new Error('Add a title and save before adding files');
            const uploaded = await uploadSingleFile(file);
            loadFiles();
            resolve({ name: uploaded.filename || file.name, url: fileDownloadUrl(uploaded) });
          } catch (e) {
            alert(file.name + ' could not be uploaded: ' + e.message);
            resolve(null);
          }
        });
        input.click();
      }),
      listVideos: async () => (await getJson('/videos')).map(v => ({ id: v.youtube_id, title: v.title })),
      listParts: async () => (await getJson('/bom')).filter(item => item.name).map(item => ({
        name: item.name,
        url: item.part_slug ? '/parts/view.html?slug=' + encodeURIComponent(item.part_slug) : null,
        detail: item.quantity + ' ' + (item.unit === 'qty' ? '\u00d7' : item.unit) + (item.part_slug ? ' \u00b7 in the parts catalog' : ''),
      })),
      diagrams: DIAGRAM_TEMPLATES,
    };
  }

  // --- Image autocomplete on `![` ---
  function setupImageAutocomplete(editor) {
    const view = editor.view;
    let popup = null;
    let images = [];
    let filteredImages = [];
    let activeIdx = 0;
    let start = null;

    function fetchImages() {
      fetchProjectImages().then(imgs => {
        images = imgs.map(img => ({ url: imageViewUrl(img), name: img.filename }));
        if (start !== null) update();
      }).catch(() => {});
    }
    fetchImages();

    function isOpen() {
      return popup && popup.style.display !== 'none';
    }

    function hidePopup() {
      if (popup) popup.style.display = 'none';
      start = null;
    }

    function renderPopup() {
      if (filteredImages.length === 0) {
        popup.innerHTML = '<div class="p-2 text-muted small">No matching images</div>';
        return;
      }
      popup.innerHTML = filteredImages.map((img, i) => `
        <div class="image-ac-item ${i === activeIdx ? 'active' : ''}" data-idx="${i}">
          <img src="${escapeHtmlAttr(img.url)}" style="width:30px;height:30px;object-fit:cover;border-radius:3px;margin-right:6px">
          <small>${escapeHtmlAttr(img.name)}</small>
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
      const img = filteredImages[idx];
      if (start === null || !img) return;
      const insert = '![' + img.name + '](' + img.url + ')';
      view.dispatch({
        changes: { from: start, to: view.state.selection.main.head, insert: insert },
        selection: { anchor: start + insert.length },
      });
      hidePopup();
      view.focus();
    }

    // Opens, filters or closes the list from the text before the cursor.
    function update() {
      const sel = view.state.selection.main;
      const line = view.state.doc.lineAt(sel.head);
      const match = sel.empty && line.text.slice(0, sel.head - line.from).match(/!\[([^\]]*)$/);
      if (!match) { hidePopup(); return; }
      if (start === null) {
        start = sel.head - match[0].length;
        fetchImages();
      }
      const query = match[1].toLowerCase();
      filteredImages = images.filter(im => im.name.toLowerCase().includes(query));
      activeIdx = 0;
      if (!popup) {
        popup = document.createElement('div');
        popup.className = 'image-autocomplete-popup';
        document.body.appendChild(popup);
      }
      const coords = view.coordsAtPos(sel.head);
      if (!coords) { hidePopup(); return; }
      popup.style.display = 'block';
      popup.style.left = coords.left + 'px';
      popup.style.top = (coords.bottom + 4) + 'px';
      renderPopup();
    }

    editor.on('change', () => {
      if (view.hasFocus) update();
      else hidePopup();
    });
    editor.on('blur', hidePopup);

    // Capturing on the editor's outer element runs before CodeMirror's keys.
    view.dom.addEventListener('keydown', e => {
      if (!isOpen()) return;
      if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
        if (filteredImages.length === 0) return;
        e.preventDefault();
        e.stopPropagation();
        const step = e.key === 'ArrowDown' ? 1 : -1;
        activeIdx = (activeIdx + step + filteredImages.length) % filteredImages.length;
        renderPopup();
      } else if ((e.key === 'Enter' || e.key === 'Tab') && filteredImages.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        selectImage(activeIdx);
      } else if (e.key === 'Escape') {
        e.stopPropagation();
        hidePopup();
      } else if (e.key === 'ArrowLeft' || e.key === 'ArrowRight' || e.key === 'Home' || e.key === 'End') {
        hidePopup();
      }
    }, true);
    view.dom.addEventListener('mousedown', hidePopup);
  }

  function getEditorValue() {
    return mde ? mde.getDoc() : contentEditor.value;
  }

  function setEditorValue(val) {
    if (mde) mde.setDoc(val || '');
    else contentEditor.value = val || '';
  }

  // Diagram menu, under the toolbar's diagram button.
  function showDiagramMenu(editor, btn) {
    const existing = document.querySelector('.diagram-menu-popover');
    if (existing) { existing.remove(); return; }

    const rect = btn.getBoundingClientRect();
    const pop = document.createElement('div');
    pop.className = 'diagram-menu-popover';
    pop.style.cssText = 'position:fixed;z-index:10000;background:white;border:1px solid #ddd;border-radius:8px;padding:4px;box-shadow:0 4px 12px rgba(0,0,0,0.15);left:' + rect.left + 'px;top:' + (rect.bottom + 4) + 'px;';
    pop.innerHTML = DIAGRAM_TEMPLATES.map(i => '<div class="image-picker-item p-2" style="cursor:pointer"><small>' + i.label + '</small></div>').join('');
    document.body.appendChild(pop);

    pop.querySelectorAll('.image-picker-item').forEach((item, idx) => {
      item.addEventListener('click', () => {
        editor.replaceSelection(DIAGRAM_TEMPLATES[idx].md);
        editor.focus();
        pop.remove();
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
    // The backend routes image uploads to the Images gallery (returning an
    // image-shaped body with `sort_order` instead of `file_type`), so refresh
    // that list too when any upload landed there.
    let movedToImages = false;
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
        } else {
          const saved = await resp.json().catch(() => null);
          if (saved && saved.file_type === undefined) movedToImages = true;
        }
      } catch (e) { errors.push(file.name + ': network error'); }
    }
    if (errors.length > 0) {
      alert('Some files could not be uploaded:\n\n' + errors.join('\n'));
    }
    loadFiles();
    if (movedToImages) loadImages();
  }

  let projectFiles = []; // the last list loadFiles() fetched, for `/3d printed parts`
  async function loadFiles() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/files', { credentials: 'include' });
    const files = await resp.json();
    projectFiles = files;
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
  let galleryImages = [];

  async function loadImages() {
    if (!currentProject) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/images', { credentials: 'include' });
    const images = await resp.json();
    galleryImages = images;
    imageOrder = images.map(img => img.id);
    const gallery = document.getElementById('image-gallery');
    // Issue #258: each thumbnail has a star to make it the cover, an alt
    // text field (the default alt when /image inserts it) and can be
    // moved with the left/right arrow keys as well as by dragging.
    gallery.innerHTML = images.map((img, idx) => {
      const isCover = idx === 0;
      const name = escHtml(img.filename);
      return `
      <div class="col-6 col-sm-4 col-md-3 col-xl-2" data-image-id="${img.id}" draggable="true">
        <div class="image-thumb ${isCover ? 'is-cover' : ''}" tabindex="0"
             aria-label="${name}${isCover ? ' (cover)' : ''}. Left and right arrow keys move it.">
          <img src="${imageViewUrl(img)}" alt="${escHtml(img.caption || img.filename)}" data-view-idx="${idx}" class="iv-clickable">
          <button type="button" class="cover-star ${isCover ? 'is-cover' : ''}" data-cover-id="${img.id}"
                  title="${isCover ? 'This is the cover' : 'Make this the cover'}"
                  aria-label="${isCover ? 'Cover image' : 'Make ' + name + ' the cover'}" aria-pressed="${isCover}">
            <i class="${isCover ? 'fas' : 'far'} fa-star"></i>
          </button>
          <button type="button" class="delete-overlay" data-delete-id="${img.id}" aria-label="Delete ${name}"><i class="fas fa-trash"></i></button>
          ${isCover ? '<span class="cover-badge">★ Cover</span>' : ''}
        </div>
        <input type="text" class="form-control form-control-sm image-alt mt-1" data-alt-id="${img.id}"
               value="${escHtml(img.caption || '')}" placeholder="Alt text" maxlength="300"
               aria-label="Alt text for ${name}">
      </div>`;
    }).join('');

    // Set cover_image from first image
    if (images.length > 0) {
      const firstUrl = imageViewUrl(images[0]);
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
  }

  function escHtml(t) {
    return String(t == null ? '' : t)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  // PUT each image's position from imageOrder, then redraw. The cover is
  // whichever image is first, so this is also how a cover is chosen.
  async function saveImageOrder(message) {
    for (let i = 0; i < imageOrder.length; i++) {
      await apiFetch(API + '/api/projects/' + currentProject.id + '/images/' + imageOrder[i], {
        method: 'PUT', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ sort_order: i }),
      });
    }
    showSaveStatus('saved', message || 'Image order updated');
    await loadImages();
  }

  async function moveImage(id, toIdx) {
    const fromIdx = imageOrder.indexOf(id);
    if (fromIdx < 0 || toIdx < 0 || toIdx >= imageOrder.length || toIdx === fromIdx) return false;
    imageOrder.splice(fromIdx, 1);
    imageOrder.splice(toIdx, 0, id);
    await saveImageOrder(toIdx === 0 ? 'Cover updated' : 'Image order updated');
    return true;
  }

  // One set of listeners on the gallery, however often it's redrawn.
  (function wireGallery() {
    const gallery = document.getElementById('image-gallery');
    if (!gallery) return;
    gallery.addEventListener('click', e => {
      const star = e.target.closest('.cover-star');
      if (star) {
        e.stopPropagation();
        moveImage(parseInt(star.dataset.coverId, 10), 0);
        return;
      }
      const del = e.target.closest('.delete-overlay');
      if (del) {
        e.stopPropagation();
        window.deleteImage(parseInt(del.dataset.deleteId, 10));
        return;
      }
      const img = e.target.closest('.iv-clickable');
      if (!img || gallery.querySelector('.dragging')) return;
      ImageViewer.open(galleryImages.map(i => ({ url: imageViewUrl(i), alt: i.caption || i.filename })),
        parseInt(img.dataset.viewIdx, 10));
    });
    gallery.addEventListener('keydown', async e => {
      const thumb = e.target.closest('.image-thumb');
      if (!thumb || (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight')) return;
      e.preventDefault();
      const id = parseInt(thumb.closest('[data-image-id]').dataset.imageId, 10);
      const to = imageOrder.indexOf(id) + (e.key === 'ArrowLeft' ? -1 : 1);
      if (await moveImage(id, to)) {
        const moved = gallery.querySelector('[data-image-id="' + id + '"] .image-thumb');
        if (moved) moved.focus();
      }
    });
    gallery.addEventListener('change', async e => {
      const input = e.target.closest('.image-alt');
      if (!input) return;
      const id = parseInt(input.dataset.altId, 10);
      const caption = input.value.trim();
      const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/images/' + id, {
        method: 'PUT', credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ caption: caption || null }),
      });
      if (!resp.ok) { showSaveStatus('error', 'Alt text not saved'); return; }
      const img = galleryImages.find(i => i.id === id);
      if (img) img.caption = caption || null;
      showSaveStatus('saved', 'Alt text saved');
    });
  })();

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
          await moveImage(parseInt(dragId, 10), imageOrder.indexOf(parseInt(dropId, 10)));
        }
      }
      dragEl = null;
      dragId = null;
    }

    gallery.querySelectorAll('[data-image-id]').forEach(el => {
      el.setAttribute('draggable', 'false');
      el.addEventListener('mousedown', e => {
        if (e.target.closest('button, input')) return;
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
    // Issue #258: warn before deleting an image the text still shows.
    const inText = getEditorValue().includes('/images/' + imageId + '/view');
    if (inText && !confirm('This image is used in your project text. Delete it anyway? The text will show a broken image until you remove it.')) return;
    await apiFetch(API + '/api/projects/' + currentProject.id + '/images/' + imageId, {
      method: 'DELETE', credentials: 'include',
    });
    loadImages();
  };

  function escapeHtmlAttr(s) {
    return String(s == null ? '' : s)
      .replace(/&/g, '&amp;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;');
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

  // --- Videos (issue #171) ---
  //
  // First-class YouTube video list. Pasting a URL extracts the 11-char
  // id client-side so the thumbnail flips immediately; the backend
  // re-validates server-side and 422s on anything malformed.
  //
  // The id extractor is shared with the `/video` command and the view
  // page, in project-embeds.js.
  const extractYouTubeId = ProjectEmbeds.youTubeId;

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

  // The Build Journal was retired from the editor (it belongs in the
  // page content now). Existing entries stay stored and the API is
  // unchanged; the editor just no longer reads or writes them.

  // --- Auto-save on input ---
  [titleInput, descInput, difficultySelect, timeInput].forEach(el => {
    if (el) {
      el.addEventListener('input', scheduleAutoSave);
      el.addEventListener('change', scheduleAutoSave);
    }
  });

  // --- Global drop zone (issue #117) ---
  // Smart routing: images -> uploadImages(), README.md -> editor content,
  // license files and everything else -> uploadFiles().
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
    // Related Links is retired from the editor, so a licence is just a file.
    await uploadSingleFile(file);
    updateProgressLine(row, '<span class="text-success"><i class="fas fa-check"></i> added to Files &amp; Models</span>');
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

    // Only images, as far as the browser lets on before the drop.
    function onlyImages(e) {
      const items = Array.from((e.dataTransfer && e.dataTransfer.items) || []);
      return items.length > 0 && items.every(item => item.kind === 'file' && (!item.type || item.type.startsWith('image/')));
    }

    document.addEventListener('dragover', (e) => {
      if (!hasFiles(e)) return;
      e.preventDefault();
      if (e.dataTransfer) e.dataTransfer.dropEffect = 'copy';
      // Over the text, images are the editor's: it shows where they'll land,
      // so the overlay steps aside.
      const overText = mde && mde.view && mde.view.dom.contains(e.target) && onlyImages(e);
      overlay.classList.toggle('d-none', !!overText);
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
      // Whether the editor took the drop, before this handler takes it too.
      const editorTook = e.defaultPrevented;
      e.preventDefault();
      dragDepth = 0;
      overlay.classList.add('d-none');
      overlay.classList.remove('is-active');
      // The entire editor page is one drop target, and
      // processDroppedFiles() routes by type: images → gallery,
      // README/index.md → markdown editor, LICENSE → linked file,
      // anything else in ALLOWED_FILE_EXTS → Files & Models.
      // The one exception: images dropped on the text belong to the editor
      // bundle, which places them where they land and files them in the
      // gallery through the uploadImage hook.
      const files = Array.from((e.dataTransfer && e.dataTransfer.files) || []);
      const onText = mde && mde.element && mde.element.contains(e.target);
      // The editor took the images (it prevents the default when it does);
      // anything dropped with them still gets routed here.
      if (onText && editorTook) {
        const rest = files.filter(f => !(f.type && f.type.startsWith('image/')));
        if (rest.length) processDroppedFiles(rest);
        return;
      }
      if (onText && files.length && files.every(isImageFile)) return;
      if (files.length) processDroppedFiles(files);
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
    { key: 'cover',  points: 15, label: 'Upload a cover image',                target: 'editor-images-section',
      check: () => document.querySelectorAll('#image-gallery [data-image-id]').length > 0 },
    { key: 'body',   points: 20, label: 'Write 200+ chars of build steps',
      target: 'content-editor',
      check: () => getEditorValue().trim().length >= 200 },
    { key: 'diff',   points: 5,  label: 'Set a difficulty level',              target: 'project-difficulty',
      check: () => difficultySelect && difficultySelect.value !== '' },
    { key: 'time',   points: 5,  label: 'Estimate build time',                 target: 'project-time',
      check: () => !!(timeInput && timeInput.value) && Number(timeInput.value) > 0 },
    { key: 'tags',   points: 5,  label: 'Add at least one tag',                target: 'tag-input',
      check: () => tagsContainer && tagsContainer.querySelectorAll('.tag-badge').length > 0 },
    // The Bill of Materials is a table in the document now: a part row
    // under the Bill of Materials heading counts, as /part reads it.
    { key: 'bom',    points: 10, label: 'Add a part to the Bill of Materials', target: 'content-editor',
      check: () => ProjectSlashCommands.bomTableParts(getEditorValue()).length > 0 },
    { key: 'repo',   points: 10, label: 'Add a Code repository section (type /)', target: 'content-editor',
      check: () => !!repoUrlFromMarkdown(getEditorValue()) },
    // Issue #171: a YouTube video counts toward "extra content" too —
    // it's just as much a doc artefact as a file.
    { key: 'extra',  points: 10, label: 'Add a file or video',                 target: 'editor-files-section',
      check: () => document.querySelectorAll('#file-list tbody tr').length > 0
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
    const miniEl = document.getElementById('completeness-mini');
    if (miniEl) miniEl.textContent = pct + '%';
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
      if (miniEl) miniEl.classList.add('d-none');
      return;
    }
    if (miniEl) miniEl.classList.remove('d-none');
    if (cardBody) cardBody.classList.remove('d-none');
    if (completeBadge) completeBadge.classList.add('d-none');
    if (celebrateEl) celebrateEl.classList.add('d-none');
    listEl.classList.remove('d-none');

    // Resolve target. Items can declare ``target`` as a string OR as
    // a function — the function form lets an item swap its target
    // based on live state (e.g. content_mode = builder vs markdown).
    function resolveTarget(item) {
      var t = item.target;
      return (typeof t === 'function') ? t() : t;
    }

    // Next suggestion: highest-point unmet item — still ordered by point
    // value internally (biggest wins surfaced first) but no "+N" suffix
    // shown in the UI, per design feedback that the numbers cluttered.
    const unmet = results.filter(r => !r.met).sort((a, b) => b.item.points - a.item.points);
    if (unmet.length && nextEl && nextLink) {
      nextLink.textContent = unmet[0].item.label;
      nextLink.dataset.target = resolveTarget(unmet[0].item);
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
           + '<a href="#" class="completeness-link text-decoration-none" data-target="' + resolveTarget(item) + '">' + item.label + '</a>'
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
    // Special-case the markdown editor: focus it, not the hidden textarea.
    if (id === 'content-editor' && mde) {
      try {
        mde.element.scrollIntoView({ behavior: 'smooth', block: 'center' });
      } catch (e) {
        mde.element.scrollIntoView();
      }
      setTimeout(() => mde.focus(), 250);
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
  [titleInput, descInput, difficultySelect, timeInput].forEach(el => {
    if (el) {
      el.addEventListener('input', recomputeCompleteness);
      el.addEventListener('change', recomputeCompleteness);
    }
  });

  // Markdown editor body changes (it mounts once auth has been checked)
  function _wireEditorForCompleteness() {
    if (mde) {
      mde.on('change', recomputeCompleteness);
      return true;
    }
    return false;
  }
  if (!_wireEditorForCompleteness()) {
    const _wireTimer = setInterval(() => {
      if (_wireEditorForCompleteness()) clearInterval(_wireTimer);
    }, 200);
  }

  // Watch DOM sections we render imperatively (tags badges, gallery,
  // file list, videos). MutationObserver keeps the meter in sync without
  // touching every render function call site.
  function _observe(id) {
    const el = document.getElementById(id);
    if (!el) return;
    new MutationObserver(recomputeCompleteness).observe(el, { childList: true, subtree: true });
  }
  _observe('tags-container');
  _observe('image-gallery');
  _observe('file-list');
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

  // Collapse / expand the overlay. Remembered per browser; small screens
  // start collapsed so the overlay doesn't cover the document.
  (function initCompletenessOverlay() {
    const card = document.getElementById('completeness-card');
    const toggle = document.getElementById('completeness-toggle');
    const chevron = document.getElementById('completeness-collapse');
    if (!card || !toggle) return;
    const KEY = 'projectEditor.completenessCollapsed';
    let collapsed = window.matchMedia('(max-width: 991.98px)').matches;
    try {
      const saved = localStorage.getItem(KEY);
      if (saved !== null) collapsed = saved === '1';
    } catch (e) {}
    function apply() {
      card.classList.toggle('is-collapsed', collapsed);
      toggle.setAttribute('aria-expanded', String(!collapsed));
      if (chevron) {
        chevron.setAttribute('aria-label', (collapsed ? 'Expand' : 'Collapse') + ' project completeness');
        chevron.innerHTML = '<i class="fas fa-chevron-' + (collapsed ? 'up' : 'down') + '"></i>';
      }
    }
    function flip() {
      collapsed = !collapsed;
      try { localStorage.setItem(KEY, collapsed ? '1' : '0'); } catch (e) {}
      apply();
    }
    toggle.addEventListener('click', flip);
    if (chevron) chevron.addEventListener('click', flip);
    apply();
  })();

  // First paint so the meter shows 0% (new projects) or actual score (after load).
  recomputeCompleteness();

  // --- Init ---
  async function init() {
    const authed = await checkAuth();
    if (!authed) return;
    // Load first so the editor opens on the saved text, and only offers
    // its local draft back when that differs.
    if (projectId) await loadProject();
    initEditor();
    editorContainer.classList.remove('d-none');
    const completenessCard = document.getElementById('completeness-card');
    if (completenessCard) completenessCard.classList.remove('d-none');
    recomputeCompleteness();
  }

  init();
})();
