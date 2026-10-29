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
      projectVideos = await fetchVideos();
      setEditorValue(withVideosSection(
        withRepoSection(currentProject.content_md, currentProject.code_repo_url), projectVideos));
      difficultySelect.value = currentProject.difficulty || '';
      timeInput.value = currentProject.estimated_minutes || '';
      updateStatusBadge(currentProject.status);
      renderTags(currentProject.tags || []);
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
      if (projectVideos === null && !projectId) projectVideos = [];
      // Issue #190: a title save can change the slug, so refresh the
      // "View live" / view-toggle links from the (possibly new) slug.
      // updateStatusBadge already reads currentProject.slug to rebuild
      // both hrefs, so a single call here keeps the displayed public
      // URL in sync silently — no page reload required. The editor's
      // own URL stays at /projects/editor.html?id=<id>, so we do NOT
      // history.replaceState anything slug-shaped here.
      updateStatusBadge(currentProject.status);
      showSaveStatus('saved', 'Saved');
      syncVideos(content);
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

  // The top bar's switch is Draft / Published (issue #260); switching on
  // publishes as ``completed``. ``wip`` also reads as Draft: a projects
  // API from before #260 has no ``draft`` status, so Draft is saved as
  // ``wip`` there (still public on that API). The Archived checkbox beside
  // the switch is the third state; publishing, or unticking it, un-archives
  // (back to a draft).
  let draftStatus = 'draft'; // becomes 'wip' once the API turns 'draft' down

  async function putStatus(status) {
    return apiFetch(API + '/api/projects/' + currentProject.id, {
      method: 'PUT', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ status: status }),
    });
  }

  async function setStatus(status) {
    if (!currentProject) { await saveProject(); if (!currentProject) return; }
    try {
      if (status === 'draft') status = draftStatus;
      let resp = await putStatus(status);
      if (resp.status === 422 && status === 'draft') {
        // An API from before issue #260 doesn't know "draft": use "wip".
        draftStatus = status = 'wip';
        resp = await putStatus(status);
      }
      if (resp.ok) {
        currentProject = await resp.json();
        updateStatusBadge(currentProject.status);
        showSaveStatus('saved', currentProject.status === 'wip'
          ? 'Saved as Draft (still listed in the hub until the projects API is updated)'
          : 'Status updated');
        if (window.BadgeToast) BadgeToast.fromResponse(currentProject);
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
    const published = status === 'completed';
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
    const added = addTagBadge(tag, false);
    added.scrollIntoView({ block: 'nearest', inline: 'nearest' });
    tagInput.value = '';
    scheduleAutoSave();
  }

  // The tag chips scroll sideways on one line; let a mouse wheel scroll them.
  tagsContainer.addEventListener('wheel', (e) => {
    if (!e.deltaY || Math.abs(e.deltaX) > Math.abs(e.deltaY)) return;
    if (tagsContainer.scrollWidth <= tagsContainer.clientWidth) return;
    e.preventDefault();
    tagsContainer.scrollLeft += e.deltaY;
  }, { passive: false });

  // Mark which ends of the chip strip have tags scrolled out of view.
  function updateTagFades() {
    const { scrollLeft, scrollWidth, clientWidth } = tagsContainer;
    tagsContainer.classList.toggle('more-before', scrollLeft > 1);
    tagsContainer.classList.toggle('more-after', scrollLeft + clientWidth < scrollWidth - 1);
  }
  tagsContainer.addEventListener('scroll', updateTagFades, { passive: true });
  new MutationObserver(updateTagFades).observe(tagsContainer, { childList: true });
  if (window.ResizeObserver) new ResizeObserver(updateTagFades).observe(tagsContainer);

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
  // it to the Projects API and draws ```mermaid blocks as diagrams (and edits
  // them in Mermaid Factory's split code + diagram editor). There's
  // no separate Preview: the live view is the preview, with Raw for source.
  const MAX_IMAGE_BYTES = 10 * 1024 * 1024; // the API's max_image_size
  // Mermaid Factory's mermaid-tools (scripts/update-mermaid-tools.sh).
  const MERMAID_TOOLS_URL = '/assets/js/mermaid-tools/mermaid-tools.js';
  const DIAGRAM_TEMPLATES = [
    { label: 'Flowchart', md: '\n```mermaid\ngraph TD\n    A[Start] --> B[Step 1]\n    B --> C[Step 2]\n    C --> D[End]\n```\n' },
    { label: 'Sequence Diagram', md: '\n```mermaid\nsequenceDiagram\n    participant A\n    participant B\n    A->>B: Hello\n    B-->>A: Hi back\n```\n' },
    { label: 'State Diagram', md: '\n```mermaid\nstateDiagram-v2\n    [*] --> Idle\n    Idle --> Running\n    Running --> Idle\n    Running --> [*]\n```\n' },
  ];
  let mde = null;
  let mermaidTools = null;

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

  function loadMermaidTools() {
    if (!mermaidTools) mermaidTools = import(MERMAID_TOOLS_URL);
    return mermaidTools;
  }

  // A ```mermaid block in the live view: its diagram, until the cursor goes in.
  async function renderMermaid(source, element) {
    const tools = await loadMermaidTools();
    const result = await tools.renderMermaid(source);
    if (result.error) throw new Error(result.error);
    element.innerHTML = result.svg;
  }

  // ...and with the cursor in it: the code beside its live diagram, where
  // clicking a flowchart shape edits its text and shape or adds a shape joined to it.
  function editMermaid(context) {
    let editor = null;
    let gone = false;
    let source = context.source;
    loadMermaidTools().then(tools => {
      if (gone) return;
      editor = tools.mountEditor(context.element, {
        source: source,
        onChange: context.onChange,
        onExit: context.onExit,
        autofocus: context.focus,
      });
    }, err => {
      context.element.textContent = 'Couldn\'t load the diagram editor: ' + err.message;
    });
    return {
      setSource: next => { source = next; if (editor) editor.setSource(next); },
      focus: () => editor && editor.focus(),
      destroy: () => { gone = true; if (editor) editor.destroy(); },
    };
  }

  // The editor's toolbar sticks to the top of the viewport as the page
  // scrolls; stop it below the site's sticky top bar instead of under it.
  // The bar wraps onto more lines on a narrow screen, so follow its height.
  function pinToolbarBelowTopBar() {
    const bar = document.querySelector('.navbar.sticky-top');
    if (!bar) return;
    const root = document.documentElement;
    const update = () => root.style.setProperty('--kr-top-bar-height', bar.offsetHeight + 'px');
    update();
    if (window.ResizeObserver) new ResizeObserver(update).observe(bar);
  }

  function initEditor() {
    if (mde) return;
    // The default toolbar, with Line numbers and Raw in place of Preview and Side by side.
    const toolbar = MarkdownEditor.DEFAULT_TOOLBAR
      .filter(item => item !== 'preview' && item !== 'side-by-side');
    toolbar.splice(toolbar.indexOf('fullscreen'), 0, 'line-numbers', 'raw');
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
      lineNumbers: true,
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
      codeEditors: { mermaid: editMermaid },
      slashCommands: ProjectSlashCommands.create(slashContext()),
      proofing: { languageApiBase: LANGUAGE_API, languageApi: languageRequest },
    });

    mde.on('change', () => {
      scheduleAutoSave();
      scheduleAutoTagDetection();
      scheduleBomTotals();
    });
    setupImageAutocomplete(mde);
    pinToolbarBelowTopBar();
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
            const uploaded = await uploadProjectFile(file);
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
  fileInput.addEventListener('change', e => {
    // Picked files go through the same sorter as a drop.
    processDroppedFiles(Array.from(e.target.files || []));
    fileInput.value = '';
  });

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
    // .file-description CSS rule (word-break).
    // Models first (issue #261), each with its thumbnail (issue #262).
    const ordered = files.filter(f => f.file_type === 'model').concat(files.filter(f => f.file_type !== 'model'));
    // Full width; the Description column takes the spare room.
    list.innerHTML = `<table class="table table-single table-hover mb-0 files-table">
      <thead>
        <tr>
          <th style="width:48px"></th>
          <th>File</th>
          <th class="w-100">Description</th>
          <th class="text-muted small">Kind</th>
          <th class="text-muted small text-end">Size</th>
          <th style="width:30px"></th>
        </tr>
      </thead>
      <tbody>${ordered.map(f => `
        <tr data-file-id="${f.id}">
          <td class="text-muted text-center" style="width:48px">${fileThumb(f)}</td>
          <td><a href="${fileDownloadUrl(f)}">${escapeHtmlAttr(f.filename)}</a></td>
          <td class="file-description">
            <div class="kr-file-field" data-file-id="${f.id}" data-field="description">${renderFileField('description', f.description)}</div>
            ${f.file_type === 'model' ? `<div class="kr-file-field small" data-file-id="${f.id}" data-field="print_settings">${renderFileField('print_settings', f.print_settings)}</div>` : ''}
          </td>
          <td class="text-muted small">${fileKind(f.filename)}</td>
          <td class="text-muted small text-end">${(f.file_size / 1024).toFixed(1)} KB</td>
          <td style="width:30px"><button class="btn btn-sm text-danger p-0" onclick="deleteFile(${f.id})"><i class="fas fa-trash fa-xs"></i></button></td>
        </tr>
      `).join('')}</tbody></table>`;
  }

  function fileThumbUrl(f) {
    return API + '/api/projects/' + currentProject.id + '/files/' + f.id + '/thumbnail?v=' + f.thumbnail_version;
  }

  // A model's rendered thumbnail, or its icon until it has one.
  function fileThumb(f) {
    if (f.thumbnail_version) {
      return `<img src="${fileThumbUrl(f)}" alt="" class="kr-file-thumb" width="40" height="40" loading="lazy">`;
    }
    return `<i class="${fileIcon(f.filename)}"></i>`;
  }

  // Render an editable file field's "view" state — a span the user
  // clicks to edit. Empty values get a muted placeholder so the affordance
  // is discoverable. Models also carry print settings (issue #261).
  const FILE_FIELDS = {
    description: { add: 'Add a description…', edit: 'Describe this file…' },
    print_settings: { add: 'Add print settings…', edit: 'Layer height, infill, supports…', label: 'Print settings: ' },
  };

  function renderFileField(field, value) {
    const meta = FILE_FIELDS[field];
    const hasText = value != null && String(value).length > 0;
    if (hasText) {
      return `<span class="kr-file-desc" tabindex="0" role="button"
        title="Click to edit">${meta.label ? `<span class="text-muted">${meta.label}</span>` : ''}${escapeHtmlAttr(value)}</span>`;
    }
    return `<span class="kr-file-desc text-muted fst-italic" tabindex="0"
      role="button" title="Click to add"
      data-empty="1">${meta.add}</span>`;
  }

  // Track in-flight PUTs per row so a fast typer doesn't fire overlapping
  // requests — serialise per-row by chaining onto the previous promise.
  const _fileDescSavePending = {};

  async function saveFileField(fileId, field, newValue) {
    if (!currentProject) return null;
    const prev = _fileDescSavePending[fileId] || Promise.resolve();
    const next = prev.then(async () => {
      const resp = await apiFetch(
        API + '/api/projects/' + currentProject.id + '/files/' + fileId,
        {
          method: 'PUT',
          credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ [field]: newValue }),
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
    const cell = span.closest('.kr-file-field');
    if (!cell) return;
    const fileId = parseInt(cell.dataset.fileId, 10);
    if (!Number.isFinite(fileId)) return;
    const field = cell.dataset.field;
    const file = projectFiles.find(f => f.id === fileId);
    const original = (file && file[field]) || '';
    span.dataset.editing = '1';

    const input = document.createElement('input');
    input.type = 'text';
    input.className = 'form-control form-control-sm kr-file-desc-input';
    input.value = original;
    input.maxLength = 2000;
    input.placeholder = FILE_FIELDS[field].edit;
    cell.innerHTML = '';
    cell.appendChild(input);
    input.focus();
    input.select();

    let finished = false;
    const finish = async (commit) => {
      if (finished) return;
      finished = true;
      const next = input.value;
      if (!commit || next === original) {
        cell.innerHTML = renderFileField(field, original || null);
        return;
      }
      // Optimistic re-render with the new value; if the save fails the
      // reload at the end of the catch reconciles with the server.
      cell.innerHTML = renderFileField(field, next || null);
      if (file) file[field] = next;
      try {
        await saveFileField(fileId, field, next);
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

  // --- Bill of Materials totals ---
  //
  // When the Bill of Materials table has Qty and Price columns, the editor
  // keeps a Total column (Qty × Price per row, added if missing) and a
  // "**Total**" row under the last part with the sum. They're written
  // into the Markdown, so the project page and any export show them too.
  const BOM_QTY = /^(qty|quantity|count)$/;
  const BOM_PRICE = /^(price|unit price|price each|each|cost|unit cost)$/;
  const BOM_TOTAL = /^(total|subtotal|line total|row total)$/;

  function parseAmount(cell) {
    const m = String(cell || '').replace(/[*_`]/g, '').replace(/,/g, '').match(/-?\d+(?:\.\d+)?/);
    return m ? parseFloat(m[0]) : null;
  }

  // "£", "$", "€"... in front of the number, or after it ("12 EUR").
  function currencyOf(cell) {
    const text = String(cell || '').replace(/[*_`]/g, '').trim();
    const m = text.match(/^([^\d\s.-]+)\s*-?\d/) || text.match(/\d\s*([^\d\s.,]+)$/);
    return m ? { symbol: m[1], before: /^[^\d\s.-]/.test(text) } : null;
  }

  function money(n, currency) {
    const amount = n.toFixed(2);
    if (!currency) return amount;
    return currency.before ? currency.symbol + amount : amount + ' ' + currency.symbol;
  }

  // The lines to replace: [{ line, text }] (line counted from 0), plus
  // where a missing total row goes. Empty when there's nothing to change.
  function bomTotalsChanges(markdown) {
    const table = ProjectSections.sectionTable(markdown, 'bill-of-materials');
    if (!table) return [];
    const header = table.header;
    const qtyCol = header.findIndex(h => BOM_QTY.test(h));
    const priceCol = header.findIndex(h => BOM_PRICE.test(h));
    if (qtyCol < 0 || priceCol < 0) return [];
    const lines = markdown.split('\n');
    const rowText = cells => '| ' + cells.join(' | ') + ' |';
    const raw = line => lines[line].trim().replace(/^\|/, '').replace(/(?<!\\)\|$/, '').split(/(?<!\\)\|/).map(c => c.trim());
    const changes = [];

    let totalCol = header.findIndex(h => BOM_TOTAL.test(h));
    const adding = totalCol < 0;
    if (adding) {
      totalCol = priceCol + 1;
      const head = raw(table.headerLine);
      head.splice(totalCol, 0, 'Total');
      changes.push({ line: table.headerLine, text: rowText(head) });
      const sep = raw(table.headerLine + 1);
      sep.splice(totalCol, 0, '---');
      changes.push({ line: table.headerLine + 1, text: rowText(sep) });
    }

    let currency = null;
    let sum = 0;
    let priced = 0;
    const totalRows = [];
    table.rows.forEach((row, r) => {
      const line = table.lines[r];
      if (ProjectSlashCommands.isTotalRow(row)) { totalRows.push(line); return; }
      const cells = raw(line);
      while (cells.length < header.length) cells.push('');
      if (adding) cells.splice(totalCol, 0, '');
      const qty = parseAmount(cells[qtyCol]);
      const price = parseAmount(cells[priceCol]);
      currency = currency || currencyOf(cells[priceCol]);
      let total = '';
      if (qty !== null && price !== null) {
        sum += qty * price;
        priced++;
        total = money(qty * price, currencyOf(cells[priceCol]) || currency);
      }
      if (adding || (cells[totalCol] || '') !== total) {
        cells[totalCol] = total;
        changes.push({ line, text: rowText(cells) });
      }
    });

    // The grand total sits under the last part; any other total row goes.
    const width = header.length + (adding ? 1 : 0);
    const lastLine = table.lines[table.lines.length - 1];
    const totalCells = new Array(width).fill('');
    totalCells[0] = '**Total**';
    totalCells[totalCol] = '**' + money(sum, currency) + '**';
    const wantText = priced ? rowText(totalCells) : null;
    totalRows.forEach(line => {
      if (line === lastLine && wantText) {
        if (lines[line].trim() !== wantText) changes.push({ line, text: wantText });
      } else {
        changes.push({ line, text: null }); // remove
      }
    });
    if (wantText && totalRows[totalRows.length - 1] !== lastLine) {
      changes.push({ line: lastLine, append: wantText });
    }
    return changes;
  }

  function applyBomTotals() {
    if (!mde || !mde.view) return;
    const view = mde.view;
    const doc = view.state.doc;
    const changes = bomTotalsChanges(doc.toString());
    if (!changes.length) return;
    const specs = [];
    changes.forEach(c => {
      const line = doc.line(c.line + 1);
      if (c.append) specs.push({ from: line.to, insert: '\n' + c.append });
      else if (c.text === null) specs.push({ from: line.from, to: Math.min(line.to + 1, doc.length) });
      else specs.push({ from: line.from, to: line.to, insert: c.text });
    });
    view.dispatch({ changes: specs, userEvent: 'input.bomtotals' });
  }

  let bomTotalsTimer = null;
  function scheduleBomTotals() {
    clearTimeout(bomTotalsTimer);
    bomTotalsTimer = setTimeout(applyBomTotals, 800);
  }

  // --- Videos section (issue #171) ---
  //
  // Videos are an optional "Videos" section of the document: a table
  // with the YouTube link in its first column and an optional title.
  // The project's video list (the API, which the project page's player
  // and the cards read) follows that table after each save. A project
  // saved before the section existed gets one made from its videos when
  // it's opened, the way code_repo_url became a section.
  const extractYouTubeId = ProjectEmbeds.youTubeId;
  let projectVideos = null; // the API's list as last seen; null until it loads

  // [{ youtube_id, title }] in table order, each video once.
  function videosFromMarkdown(markdown) {
    const table = ProjectSections.sectionTable(markdown, 'videos');
    if (!table) return [];
    let videoCol = table.header.findIndex(h => /^(video|videos|url|link|youtube)$/.test(h));
    if (videoCol < 0) videoCol = 0;
    const titleCol = table.header.findIndex(h => /^(title|name|caption|description)$/.test(h));
    const seen = new Set();
    const out = [];
    table.rows.forEach(row => {
      const cell = row[videoCol] || '';
      const link = cell.match(/\]\(([^)\s]+)/);
      const id = extractYouTubeId(link ? link[1] : cell.replace(/^<|>$/g, ''));
      if (!id || seen.has(id)) return;
      seen.add(id);
      const title = titleCol >= 0 ? (row[titleCol] || '').replace(/\\\|/g, '|').trim() : '';
      out.push({ youtube_id: id, title: title || null });
    });
    return out;
  }

  function withVideosSection(markdown, videos) {
    const md = markdown || '';
    if (!videos || !videos.length || ProjectSections.present(md).has('videos')) return md;
    const cell = t => String(t || '').replace(/\|/g, '\\|').replace(/\n/g, ' ');
    const rows = videos.map(v => '| ' + ProjectEmbeds.videoMarkdown(v.youtube_id) + ' | ' + cell(v.title) + ' |').join('\n');
    return md.replace(/\s*$/, '') + (md.trim() ? '\n\n' : '') +
      '# ' + ProjectSections.get('videos').heading + '\n\n| Video | Title |\n| --- | --- |\n' + rows + '\n';
  }

  async function fetchVideos() {
    try {
      const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/videos', { credentials: 'include' });
      if (!resp.ok) return null;
      const list = await resp.json();
      return Array.isArray(list) ? list.sort((a, b) => a.sort_order - b.sort_order) : null;
    } catch (_) { return null; }
  }

  // Brings the API's list in line with the table. One run at a time; a
  // save during a run queues one more.
  let videoSync = Promise.resolve();
  function syncVideos(markdown) {
    videoSync = videoSync.then(() => syncVideosNow(markdown)).catch(e => console.error('Video sync failed:', e));
    return videoSync;
  }

  async function syncVideosNow(markdown) {
    // Not before the list has loaded: an empty table then would delete them.
    if (!currentProject || projectVideos === null) return;
    const want = videosFromMarkdown(markdown);
    const have = projectVideos;
    const call = (path, method, body) => apiFetch(API + '/api/projects/' + currentProject.id + '/videos' + path, {
      method, credentials: 'include',
      headers: body ? { 'Content-Type': 'application/json' } : undefined,
      body: body ? JSON.stringify(body) : undefined,
    });
    const wanted = new Set(want.map(v => v.youtube_id));
    let changed = false;
    for (const v of have) {
      if (!wanted.has(v.youtube_id)) { await call('/' + v.id, 'DELETE'); changed = true; }
    }
    for (let i = 0; i < want.length; i++) {
      const w = want[i];
      let row = have.find(v => v.youtube_id === w.youtube_id);
      if (!row) {
        const resp = await call('', 'POST', { url_or_id: w.youtube_id, title: w.title });
        if (!resp.ok) continue;
        row = await resp.json();
        changed = true;
      }
      if ((row.title || null) !== w.title || row.sort_order !== i) {
        await call('/' + row.id, 'PUT', { title: w.title || '', sort_order: i });
        changed = true;
      }
    }
    if (changed) projectVideos = (await fetchVideos()) || projectVideos;
  }

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
  // Smart routing (issue #261): images -> the gallery, README.md -> editor
  // content, 3D printable models -> Files & Models with a rendered
  // thumbnail, everything else -> Files. A dropped folder is walked.
  const ALLOWED_FILE_EXTS = new Set([
    'py', 'cpp', 'h', 'ino', 'md', 'txt', 'pdf',
    'stl', 'obj', 'gcode', '3mf', 'step', 'stp', 'dxf', 'dwg', 'svg',
    'json', 'xml', 'yaml', 'yml', 'csv', 'toml',
    'zip', 'tar', 'gz', '7z', 'rar',
    'scad', 'kicad_pcb', 'kicad_sch', 'brd', 'sch',
    'f3d', 'fcstd',
  ]);
  const IMAGE_EXTS = new Set(['png', 'jpg', 'jpeg', 'gif', 'webp', 'svg', 'heic', 'heif']);
  const HEIC_EXTS = new Set(['heic', 'heif']);
  // What the server files each upload as (storage.file_category).
  const MODEL_EXTS = new Set(['stl', '3mf', 'obj', 'step', 'stp', 'gcode']);

  function getExt(name) {
    const i = (name || '').lastIndexOf('.');
    return i === -1 ? '' : name.substring(i + 1).toLowerCase();
  }

  function isImageFile(file) {
    if (file.type && file.type.startsWith('image/')) return true;
    return IMAGE_EXTS.has(getExt(file.name));
  }

  // Each file found inside a dropped folder, by how deep it sat: only a
  // README at the top of the folder is the project's write-up.
  const dropDepth = new WeakMap();

  function isReadmeFile(file) {
    const n = (file.name || '').toLowerCase();
    return (n === 'readme.md' || n === 'index.md') && (dropDepth.get(file) || 0) <= 1;
  }

  function isModelFile(file) {
    return MODEL_EXTS.has(getExt(file.name));
  }

  // Folder entries the browser offers with a drop. Read them during the
  // drop event: the DataTransfer is emptied once the handler returns.
  function droppedEntries(dataTransfer) {
    const items = Array.from((dataTransfer && dataTransfer.items) || []);
    return items
      .filter(item => item.kind === 'file')
      .map(item => (item.webkitGetAsEntry ? item.webkitGetAsEntry() : null));
  }

  // Not worth filing: hidden files, macOS zip debris and OS thumbnails.
  function isJunkEntry(name) {
    return name.startsWith('.') || name === '__MACOSX' || name === 'Thumbs.db' || name === 'desktop.ini';
  }

  const MAX_FOLDER_FILES = 200;

  // Every file under the dropped entries, depth-first. A directory reader
  // hands back entries in batches until it returns an empty one.
  async function walkEntries(entries) {
    const out = [];
    async function visit(entry, depth) {
      if (!entry || isJunkEntry(entry.name) || out.length >= MAX_FOLDER_FILES) return;
      if (entry.isFile) {
        const file = await new Promise((resolve, reject) => entry.file(resolve, reject));
        dropDepth.set(file, depth);
        out.push(file);
        return;
      }
      if (!entry.isDirectory) return;
      const reader = entry.createReader();
      for (;;) {
        const batch = await new Promise((resolve, reject) => reader.readEntries(resolve, reject));
        if (!batch.length) break;
        for (const child of batch) await visit(child, depth + 1);
      }
    }
    for (const entry of entries) await visit(entry, 0);
    return out;
  }

  // HEIC photos (iPhone) become JPEGs before they go to the gallery, which
  // browsers can't show HEIC in. The converter loads on first use.
  const HEIC2ANY_URL = 'https://cdn.jsdelivr.net/npm/heic2any@0.0.4/dist/heic2any.min.js';
  let heicLoading = null;
  async function heicToJpeg(file) {
    if (!window.heic2any) {
      heicLoading = heicLoading || new Promise((resolve, reject) => {
        const script = document.createElement('script');
        script.src = HEIC2ANY_URL;
        script.onload = resolve;
        script.onerror = () => { heicLoading = null; reject(new Error('could not load the HEIC converter')); };
        document.head.appendChild(script);
      });
      await heicLoading;
    }
    const out = await window.heic2any({ blob: file, toType: 'image/jpeg', quality: 0.9 });
    const blob = Array.isArray(out) ? out[0] : out;
    return new File([blob], file.name.replace(/\.(heic|heif)$/i, '.jpg'), { type: 'image/jpeg' });
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

  // Upload a file and, for a model the browser can draw, its thumbnail
  // (issue #262). A thumbnail that fails leaves the model filed without
  // one; the admin backfill can render it later.
  async function uploadProjectFile(file) {
    const saved = await uploadSingleFile(file);
    if (saved.file_type === 'model' && window.ModelThumbnails && ModelThumbnails.canRender(file.name)) {
      try {
        const url = API + '/api/projects/' + currentProject.id + '/files/' + saved.id + '/thumbnail';
        return await ModelThumbnails.upload(apiFetch, url, await file.arrayBuffer());
      } catch (e) {
        console.warn('No thumbnail for ' + file.name, e);
      }
    }
    return saved;
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
    let modelCount = 0;
    let fileCount = 0;
    let readmeCount = 0;
    let errorCount = 0;
    let needsImageReload = false;
    let needsFileReload = false;
    const skippedCount = files.length - accepted.length;

    for (const file of accepted) {
      const row = addProgressLine(file.name, '<i class="fas fa-spinner fa-spin"></i>');
      try {
        if (isReadmeFile(file)) {
          const ok = await handleReadmeDrop(file, row);
          if (ok) readmeCount++;
        } else if (isImageFile(file)) {
          const image = HEIC_EXTS.has(getExt(file.name)) ? await heicToJpeg(file) : file;
          await uploadSingleImage(image);
          updateProgressLine(row, '<span class="text-success"><i class="fas fa-check"></i> added to gallery</span>');
          imageCount++;
          needsImageReload = true;
        } else if (isModelFile(file)) {
          await uploadProjectFile(file);
          updateProgressLine(row, '<span class="text-success"><i class="fas fa-cube"></i> added to Files &amp; Models</span>');
          modelCount++;
          needsFileReload = true;
        } else {
          // Licences included: Related Links is retired, so a licence is just a file.
          await uploadSingleFile(file);
          updateProgressLine(row, '<span class="text-success"><i class="fas fa-check"></i> added to Files</span>');
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

    // "12 images, 3 models, 4 files added"
    const plural = (n, word) => n + ' ' + word + (n === 1 ? '' : 's');
    const parts = [];
    if (imageCount) parts.push(plural(imageCount, 'image'));
    if (modelCount) parts.push(plural(modelCount, 'model'));
    if (fileCount) parts.push(plural(fileCount, 'file'));
    let summary = parts.length ? parts.join(', ') + ' added' : '';
    if (readmeCount) summary += (summary ? ', ' : '') + 'README loaded';
    if (!summary) summary = 'Nothing added';
    if (errorCount) summary += ' (' + errorCount + ' failed)';
    if (skippedCount) summary += ' (' + skippedCount + ' unsupported)';
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
      // README/index.md → markdown editor, models → Files & Models,
      // anything else in ALLOWED_FILE_EXTS → Files.
      // The one exception: images dropped on the text belong to the editor
      // bundle, which places them where they land and files them in the
      // gallery through the uploadImage hook.
      const files = Array.from((e.dataTransfer && e.dataTransfer.files) || []);
      const entries = droppedEntries(e.dataTransfer);
      const onText = mde && mde.element && mde.element.contains(e.target);
      const tookImage = (f) => onText && editorTook && f.type && f.type.startsWith('image/');
      // A folder (issue #261): walk it, leaving out any loose images the
      // editor already took.
      if (entries.some(entry => entry && entry.isDirectory)) {
        const rest = entries.filter((entry, i) => entry && !(files[i] && tookImage(files[i])));
        walkEntries(rest).then(found => {
          if (found.length >= MAX_FOLDER_FILES) {
            alert('Only the first ' + MAX_FOLDER_FILES + ' files in that folder will be added.');
          }
          processDroppedFiles(found);
        }).catch(err => alert('Could not read that folder: ' + (err.message || err)));
        return;
      }
      // The editor took the images (it prevents the default when it does);
      // anything dropped with them still gets routed here.
      if (onText && editorTook) {
        const rest = files.filter(f => !tookImage(f));
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
    // Read from the document on every change, so deleting the section
    // (or emptying it) takes the points away again.
    // Any text under the heading counts, so one step from /assembly does.
    { key: 'body',   points: 20, label: 'Add Assembly instructions with at least one step (type /)',
      target: 'content-editor',
      check: () => /\w/.test(ProjectSections.sectionText(getEditorValue(), 'assembly-instructions') || '') },
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
                || videosFromMarkdown(getEditorValue()).length > 0 },
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
    scheduleBomTotals();
    editorContainer.classList.remove('d-none');
    const completenessCard = document.getElementById('completeness-card');
    if (completenessCard) completenessCard.classList.remove('d-none');
    recomputeCompleteness();
  }

  init();
})();
