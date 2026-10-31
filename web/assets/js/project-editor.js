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
      <div class="kr-dev-login">
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
      loadLegacySchematic();
    } catch (e) {
      console.error('Failed to load project:', e);
    }
  }

  // A schematic from the old schematic editor (issue #273): offer it until
  // the owner redraws it in the electronics workspace.
  async function loadLegacySchematic() {
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/schematic', { credentials: 'include' }).catch(() => null);
    if (!resp || !resp.ok) return;
    const schematic = await resp.json();
    if (!schematic.schematic_data) return;
    // Redrawn in the electronics workspace: the wiring replaces it.
    const wiring = await apiFetch(API + '/api/projects/' + currentProject.id + '/wiring', { credentials: 'include' }).catch(() => null);
    if (wiring && wiring.ok) return;
    const link = document.getElementById('legacy-schematic-link');
    link.href = '/projects/schematic/edit.html?id=' + encodeURIComponent(currentProject.id);
    document.getElementById('legacy-schematic').classList.remove('d-none');
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
    // The first gallery picture is the cover (see coverUrl).
    const cover = coverUrl(content);
    if (cover) data.cover_image = cover;

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
      syncBom(content);
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
  // Where a diagram block's "Open in Mermaid Factory" link goes.
  const MERMAID_FACTORY_URL = 'https://mermaid.kevspowertools.com';
  // The starters if mermaid-tools won't load; normally they come from its gallery.
  const DIAGRAM_TEMPLATES = [
    { label: 'Flowchart', md: '\n```mermaid\ngraph TD\n    A[Start] --> B[Step 1]\n    B --> C[Step 2]\n    C --> D[End]\n```\n' },
    { label: 'Sequence Diagram', md: '\n```mermaid\nsequenceDiagram\n    participant A\n    participant B\n    A->>B: Hello\n    B-->>A: Hi back\n```\n' },
    { label: 'State Diagram', md: '\n```mermaid\nstateDiagram-v2\n    [*] --> Idle\n    Idle --> Running\n    Running --> Idle\n    Running --> [*]\n```\n' },
  ];
  let mde = null;
  let mermaidTools = null;
  let diagramStarters = null;

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

  // Mermaid Factory's gallery of starters (flowchart first, then sequence,
  // class, state, ER, journey, Gantt, pie…) as [{ label, detail, md }].
  function loadDiagramStarters() {
    if (!diagramStarters) {
      diagramStarters = loadMermaidTools().then(tools => tools.templates.map(t => ({
        label: t.name,
        detail: t.description,
        md: '\n```mermaid\n' + t.source.trim() + '\n```\n',
      })), () => DIAGRAM_TEMPLATES);
    }
    return diagramStarters;
  }

  // A ```mermaid block in the live view: its diagram, until the cursor goes in.
  async function renderMermaid(source, element) {
    const tools = await loadMermaidTools();
    const result = await tools.renderMermaid(source, window.ProjectMermaidTheme);
    if (result.error) throw new Error(result.error);
    element.innerHTML = result.svg;
  }

  function projectTitle() {
    return (titleInput && titleInput.value.trim()) || undefined;
  }

  // "Open in Mermaid Factory" opens the diagram there in a new tab, where
  // "Save to your project" puts the edited code back in its block here.
  function openInMermaidFactory(tools, code) {
    let current = code;
    tools.openInFactory(code, {
      base: MERMAID_FACTORY_URL,
      title: projectTitle(),
      label: 'your project',
      onSave: next => {
        if (!mde || !replaceMermaidBlock(current, next)) {
          return 'The diagram was changed or removed in the project since you opened it, so it wasn\'t replaced. Copy the code across instead.';
        }
        current = next;
        return true;
      },
    });
  }

  // Swaps the code of the first ```mermaid block whose code is `oldCode` for `newCode`.
  function replaceMermaidBlock(oldCode, newCode) {
    const doc = mde.view.state.doc;
    const want = oldCode.replace(/\s+$/, '');
    for (let n = 1; n <= doc.lines; n++) {
      const fence = /^ {0,3}(`{3,}|~{3,})\s*mermaid\b/i.exec(doc.line(n).text);
      if (!fence) continue;
      const close = new RegExp('^ {0,3}' + fence[1][0] + '{' + fence[1].length + ',}\\s*$');
      let end = n + 1;
      while (end <= doc.lines && !close.test(doc.line(end).text)) end++;
      if (end > doc.lines) return false; // an unclosed fence runs to the end
      const empty = end === n + 1;
      const from = empty ? doc.line(end).from : doc.line(n + 1).from;
      const to = empty ? from : doc.line(end - 1).to;
      if (doc.sliceString(from, to).replace(/\s+$/, '') === want) {
        const insert = newCode.replace(/\s+$/, '') + (empty ? '\n' : '');
        mde.view.dispatch({ changes: { from, to, insert }, userEvent: 'input' });
        return true;
      }
      n = end;
    }
    return false;
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
        ...window.ProjectMermaidTheme,
        source: source,
        onChange: context.onChange,
        onExit: context.onExit,
        autofocus: context.focus,
        factoryUrl: MERMAID_FACTORY_URL,
        title: projectTitle(),
        onOpenInFactory: code => openInMermaidFactory(tools, code),
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
    // The default toolbar, with Raw (which has line numbers) in place of Preview and Side by side.
    const toolbar = MarkdownEditor.DEFAULT_TOOLBAR
      .filter(item => item !== 'preview' && item !== 'side-by-side');
    toolbar.splice(toolbar.indexOf('fullscreen'), 0, 'raw');
    toolbar.splice(toolbar.indexOf('table') + 1, 0, {
      id: 'diagram',
      label: 'Insert Mermaid diagram',
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
      // Several images dropped on the text at once go to the end of the gallery.
      addImages: files => processDroppedFiles(files),
      imageLibrary: async () => (await fetchProjectImages()).map(img => ({
        url: imageViewUrl(img), alt: img.caption || img.filename,
      })),
      deleteLibraryImage: deleteLibraryImage,
      images: { maxBytes: MAX_IMAGE_BYTES, libraryName: 'the gallery' },
      preview: false,
      codeRenderers: { mermaid: renderMermaid },
      codeEditors: { mermaid: editMermaid },
      // Mermaid Factory's highlighter for ```mermaid fences in Raw.
      codeLanguages: { mermaid: () => loadMermaidTools().then(tools => tools.mermaidStreamParser) },
      // A `::: videos` block, and an older project's Videos table, as the
      // view page's cards until they're clicked.
      videoRenderer: videos => (videos.length ? ProjectEmbeds.videoCards(videos) : null),
      tableRenderer: ({ rows, heading }) => {
        const section = heading && ProjectSections.match(heading);
        if (!section || section.id !== 'videos') return null;
        const items = ProjectEmbeds.videoTableItems(rows);
        return items.length ? ProjectEmbeds.videoCards(items) : null;
      },
      // A tip on Qty, Price and the automatic totals above the Bill of Materials table.
      // Above the pinout table: a note with Refresh when it's out of date.
      tableNote: ({ rows, heading }) => bomTableTip(rows, heading) || pinoutTableNote(rows),
      // Up and down buttons on the Bill of Materials' Qty cells.
      tableStepper: bomQtyStepper,
      // The pinout table's marker line (pinout-sync.js) stays out of sight.
      hiddenComments: comment => PinoutSync.isMarker(comment),
      slashCommands: ProjectSlashCommands.create(slashContext()),
      // Project topics in the ? Markdown guide, after the editor's own.
      guide: projectGuideTopics(),
      proofing: { languageApiBase: LANGUAGE_API, languageApi: languageRequest },
      // Canonical sections (Bill of Materials, Wiring…) are # headings, so a
      // project has several H1s on purpose: no "More than one H1" hint.
      lintSkip: ['multiple-h1'],
    });

    mde.on('change', () => {
      scheduleAutoSave();
      scheduleAutoTagDetection();
      scheduleBomTotals();
      scheduleBomLinks();
      scheduleGalleryAddBar();
      schedulePinout();
    });
    schedulePinout();
    // Hints set before the editor was mounted (a closed completeness panel).
    if (editorHints.size) mde.setHints(Array.from(editorHints.values()));
    setupImageAutocomplete(mde);
    setupPartLinks(mde);
    setupImageEditButtons(mde);
    pinToolbarBelowTopBar();
    // For the TOC in editor.html and other page scripts.
    window.projectMarkdownEditor = mde;
    document.dispatchEvent(new CustomEvent('project-markdown-editor:ready', { detail: mde }));
  }

  // The ? guide's project topics: the sections a project page knows, and
  // what the Bill of Materials, Videos and diagram blocks do with them.
  function projectGuideTopics() {
    const heading = id => '## ' + ProjectSections.get(id).heading;
    return [
      {
        title: 'Project sections',
        rows: [
          ['/', 'Type / on a new line to add a section, a part, videos, a file or a diagram'],
          [heading('bill-of-materials'), 'The parts list, as a table'],
          [heading('assembly-instructions'), 'How to build it, step by step'],
          [heading('wiring'), 'The wiring diagram, drawn in the wiring workspace'],
          [heading('code-repository'), 'A link to your code repository'],
          [heading('3d-printed-parts'), 'Your STL files'],
          [heading('videos'), 'Your YouTube videos, as a group of cards: / then Videos, then add, delete or move link lines'],
          [heading('gallery'), 'Your pictures as a gallery: / then Gallery and pick a layout. Add pictures in its bar; hover a picture to move or remove it, or drag it'],
        ],
        note: 'These headings fill the page navigation and the completeness check.',
      },
      {
        title: 'Bill of Materials',
        rows: [
          ['| Part | Qty | Price |', 'Add Qty and Price columns: a Total column and a Total row are filled in for you'],
          ['4.99', 'Write prices with a dot'],
          ['[Pico](/parts/view.html?slug=pico)', 'A link to a catalog part shows its picture and details (type / then Part)'],
        ],
      },
      {
        title: 'Pictures and the gallery',
        rows: [
          ['::: gallery filmstrip', 'The gallery: its first picture is the project\u2019s cover'],
          ['Add pictures', 'In the gallery\u2019s bar: pick from your uploads or upload more (several at once)'],
          ['Markdown', 'Shows the gallery\u2019s picture lines; Show gallery goes back'],
          ['Drop pictures on the page', 'They\u2019re uploaded and added to the end of the gallery (a Gallery section is made if there isn\u2019t one)'],
        ],
        note: 'Delete an upload from the image picker (toolbar picture button): hover it and click the bin.',
      },
      {
        title: 'Project videos',
        rows: [
          ['/ then Videos', 'Adds a ::: videos block: paste YouTube links, or pick this project\u2019s videos'],
          ['| Video | Title |', 'An older Videos table still shows as cards'],
        ],
        note: 'On the project page the cards play in place. The videos are listed with your project too.',
      },
      {
        title: 'Diagrams',
        rows: [
          ['```mermaid', 'A Mermaid diagram: code and drawing side by side while you edit it, just the drawing otherwise'],
          ['Open in Mermaid Factory', 'Edit it by clicking the shapes, then Save to bring it back'],
        ],
        note: 'The diagram button on the toolbar, or / then Mermaid diagram, starts one from 28 templates.',
      },
    ];
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
      // The gallery's pictures, in its order, for a new `::: gallery` block.
      listImages: async () => (await fetchProjectImages()).map(img => ({
        url: imageViewUrl(img), name: img.filename, caption: img.caption || '',
      })),
      listParts: async () => (await getJson('/bom')).filter(item => item.name).map(item => ({
        name: item.name,
        url: item.part_slug ? '/parts/view.html?slug=' + encodeURIComponent(item.part_slug) : null,
        detail: item.quantity + ' ' + (item.unit === 'qty' ? '\u00d7' : item.unit) + (item.part_slug ? ' \u00b7 in the parts catalog' : ''),
      })),
      diagrams: loadDiagramStarters,
      openWiring: () => openWiring(),
      diagramTemplates: DiagramEditor.TEMPLATES,
      newDiagram: (view, template) => newDiagram(view, template),
      listDiagrams: async () => (await getJson('/diagrams')),
      diagramMarkdown: d => diagramMarkdown(d),
      searchParts: searchCatalogParts,
      createPart: createCatalogPart,
      browseCatalog: async (view, q) => {
        const parts = await PartsCatalog.open({
          api: API, query: q, title: 'Add parts to the Bill of Materials', addLabel: 'Add to Bill of Materials',
          origin: view.coordsAtPos(view.state.selection.main.head) || view.dom,
        });
        if (parts.length) addBomRows(view, parts);
        else view.focus();
      },
    };
  }

  // --- Wiring (issue #268) ---
  //
  // Snakie's electronics workspace (project-wiring.js), opened from the
  // Wiring `/` command or the "Edit wiring" button on the diagram. The
  // diagram is an image under the Wiring heading.
  function openWiring() {
    const view = mde.view;
    return ProjectWiring.open({
      projectId: async () => {
        if (!currentProject) await saveProject();
        if (!currentProject) { alert('Add a title and save before wiring the project.'); return null; }
        return currentProject.id;
      },
      markdown: () => view.state.doc.toString(),
      bomSlugs: () => {
        const parsed = BomSync.parse(view.state.doc.toString());
        return parsed ? [...new Set(parsed.rows.map(r => r.part_slug).filter(Boolean))] : [];
      },
      addBomRows: parts => addBomRows(view, parts),
      removeBomRows: slugs => removeBomRows(view, slugs),
      placeImage: url => placeWiringImage(view, url),
      insertPinout: block => putPinout(block),
      hasPinout: () => !!PinoutSync.find(view.state.doc.toString()),
      wiringSaved: block => {
        pinoutFresh = { key: wiringKey(view.state.doc.toString()), block };
        return checkPinout();
      },
      apiFetch: (url, opts) => apiFetch(url, Object.assign({ credentials: 'include' }, opts || {})),
    });
  }

  // The wiring diagram in the text: its link refreshed when it's there,
  // else put under the Wiring heading (added at the end when missing).
  function placeWiringImage(view, url) {
    const markdown = view.state.doc.toString();
    const found = ProjectWiring.findImage(markdown);
    if (found) {
      const from = found.index + found.alt.length + 4; // past "![alt]("
      view.dispatch({ changes: { from, to: from + found.url.length, insert: url }, userEvent: 'input' });
      return;
    }
    const image = '![Wiring diagram](' + url + ')';
    const heading = ProjectSections.headings(markdown).find(h => ProjectSections.match(h.text) === ProjectSections.get('wiring'));
    if (heading) {
      const line = view.state.doc.line(heading.line + 1);
      const last = line.number === view.state.doc.lines;
      view.dispatch({ changes: { from: line.to, insert: '\n\n' + image + (last ? '\n' : '') }, userEvent: 'input' });
    } else {
      const tail = !markdown ? '' : markdown.endsWith('\n\n') ? '' : markdown.endsWith('\n') ? '\n' : '\n\n';
      view.dispatch({ changes: { from: markdown.length, insert: tail + '# ' + ProjectSections.get('wiring').heading + '\n\n' + image + '\n' }, userEvent: 'input' });
    }
  }

  // Markdown put after the wiring diagram (the pinout table), else at the
  // end, with a blank line on each side: a table straight after it would
  // otherwise run on as part of the same table.
  function insertAfterWiring(view, md) {
    const markdown = view.state.doc.toString();
    const found = ProjectWiring.findImage(markdown);
    const at = found ? view.state.doc.lineAt(found.index).to : markdown.length;
    const rest = markdown.slice(at);
    const end = !rest ? '\n' : rest.startsWith('\n\n') ? '' : rest.startsWith('\n') ? '\n' : '\n\n';
    view.dispatch({ changes: { from: at, insert: '\n\n' + md.trim() + end }, userEvent: 'input' });
  }

  // --- Pinout table ---
  //
  // "Insert pinout table" in the wiring workspace writes the table with a
  // hidden marker (pinout-sync.js) recording what it was made from. The
  // editor makes the table again from the saved wiring (out of sight, see
  // ProjectWiring.pinoutFor) and, when it differs, puts a note above the
  // table and a hint in the hints panel, both with Refresh. Refreshing a
  // table the writer has edited asks first, since their changes are lost.
  let pinoutFresh = null; // { key, block }: the table the wiring makes, for the image link `key`
  let pinoutState = null; // { stale, edited } for the table in the text, null when unknown
  let pinoutLoading = null;
  let pinoutTimer = null;

  // The wiring image's link (its ?v= changes on each save), '' without one.
  function wiringKey(markdown) {
    const img = ProjectWiring.findImage(markdown);
    return img ? img.url : '';
  }

  // The table the saved wiring makes now ('' when nothing's wired, null
  // without wiring), worked out once per save of the wiring.
  async function freshPinout() {
    const key = wiringKey(mde.view.state.doc.toString());
    if (pinoutFresh && pinoutFresh.key === key) return pinoutFresh.block;
    if (!currentProject) return null;
    if (!pinoutLoading) {
      pinoutLoading = ProjectWiring.pinoutFor({
        projectId: currentProject.id,
        bomSlugs: () => {
          const parsed = BomSync.parse(mde.view.state.doc.toString());
          return parsed ? [...new Set(parsed.rows.map(r => r.part_slug).filter(Boolean))] : [];
        },
        apiFetch: (url, opts) => apiFetch(url, Object.assign({ credentials: 'include' }, opts || {})),
      }).then(block => { pinoutFresh = { key, block }; return block; })
        .finally(() => { pinoutLoading = null; });
    }
    return pinoutLoading;
  }

  // The editor's box: mount() puts it after the #content-editor textarea, so
  // their parent holds both.
  function editorBox() {
    return contentEditor.parentElement;
  }
  editorBox().addEventListener('click', e => {
    if (!e.target.closest('.pinout-refresh')) return;
    e.preventDefault();
    e.stopPropagation();
    refreshPinout();
  });

  function schedulePinout() {
    clearTimeout(pinoutTimer);
    pinoutTimer = setTimeout(() => {
      if (!mde || !PinoutSync.find(mde.view.state.doc.toString())) { checkPinout(); return; }
      if (pinoutFresh && pinoutFresh.key === wiringKey(mde.view.state.doc.toString())) { checkPinout(); return; }
      freshPinout().then(checkPinout).catch(e => console.warn('Pinout check failed:', e));
    }, 1500);
  }

  // Compares the table in the text with the last table the wiring made,
  // and shows the result. Returns { stale, edited } or null.
  function checkPinout() {
    const markdown = mde ? mde.view.state.doc.toString() : '';
    const found = PinoutSync.find(markdown);
    const fresh = pinoutFresh && pinoutFresh.key === wiringKey(markdown) ? pinoutFresh.block : null;
    pinoutState = found && fresh ? PinoutSync.compare(found, fresh) : null;
    const stale = !!(pinoutState && pinoutState.stale);
    setEditorHint('pinout', stale ? {
      title: 'The pinout table is out of date',
      detail: 'The wiring diagram has changed since the pinout table was made.' +
        (pinoutState.edited === false ? '' : ' Refreshing it replaces any changes you made to the table.'),
      actions: [
        { label: 'Refresh the table', run: () => refreshPinout() },
        { label: 'Show me', run: () => {
          const note = editorBox().querySelector('.pinout-table-note:not([hidden])');
          if (note) note.scrollIntoView({ block: 'center', behavior: 'smooth' });
        } },
      ],
    } : null);
    editorBox().querySelectorAll('.pinout-table-note').forEach(fillPinoutNote);
    return pinoutState;
  }

  // The note above the pinout table: shown only while it's out of date.
  // Every table that could be one gets a (hidden) note, because the editor
  // only redraws its tables when the text changes and the check finishes
  // later: checkPinout() fills in the notes already on screen.
  const PINOUT_HEADERS = [PinoutSync.MCU_HEADER.join('|'), 'from|to|net'];
  function pinoutTableNote(rows) {
    const header = (rows[0] || []).map(c => c.replace(/[*_`]/g, '').trim().toLowerCase()).join('|');
    if (!PINOUT_HEADERS.includes(header)) return null;
    const note = document.createElement('div');
    note.className = 'kr-notice pinout-table-note';
    note.setAttribute('role', 'note');
    note.dataset.header = header;
    fillPinoutNote(note);
    return note;
  }

  function fillPinoutNote(note) {
    const found = mde ? PinoutSync.find(mde.view.state.doc.toString()) : null;
    const mine = found && found.header.map(c => c.replace(/[*_`]/g, '').trim().toLowerCase()).join('|') === note.dataset.header;
    const stale = !!(mine && pinoutState && pinoutState.stale);
    note.hidden = !stale;
    if (!stale) { note.innerHTML = ''; return; }
    note.innerHTML =
      '<i class="fas fa-triangle-exclamation" aria-hidden="true"></i>' +
      '<span>This pinout table doesn\u2019t match the wiring diagram any more.' +
      (pinoutState.edited === false ? '' : ' Refreshing it replaces any changes you made to the table.') + '</span>' +
      '<button type="button" class="btn btn-sm btn-outline-secondary pinout-refresh"><i class="fas fa-rotate me-1"></i>Refresh</button>';
  }

  // Puts `block` in the text as the pinout table: replacing the one there
  // (asking first when the writer's changes would be lost) or after the
  // wiring image. Returns false when the writer said no.
  function putPinout(block) {
    const view = mde.view;
    const markdown = view.state.doc.toString();
    const found = PinoutSync.find(markdown);
    const text = PinoutSync.withMarker(block);
    if (!found) {
      insertAfterWiring(view, text);
    } else {
      const same = PinoutSync.normalize(found.block) === PinoutSync.normalize(block);
      const edited = found.hash ? PinoutSync.hash(found.block) !== found.hash : true;
      if (!same && edited && !window.confirm(found.hash
        ? 'You\u2019ve edited the pinout table since it was made. Refreshing it from the wiring diagram replaces your changes.\n\nRefresh the table anyway?'
        : 'Refreshing the pinout table from the wiring diagram replaces the table as it is now, including any changes you made to it.\n\nRefresh the table anyway?')) return false;
      view.dispatch({ changes: { from: found.from, to: found.to, insert: text }, userEvent: 'input' });
    }
    pinoutFresh = { key: wiringKey(view.state.doc.toString()), block };
    checkPinout();
    return true;
  }

  async function refreshPinout() {
    let block;
    try { block = await freshPinout(); } catch (e) { alert('Could not read the wiring: ' + (e.message || e)); return; }
    if (block == null) { alert('This project has no saved wiring to make the table from.'); return; }
    if (!block) { alert('Nothing is wired yet, so there is no pinout table to make.'); return; }
    putPinout(block);
  }

  // The Bill of Materials rows that link any of `slugs`, taken out.
  function removeBomRows(view, slugs) {
    const table = ProjectSections.sectionTable(view.state.doc.toString(), 'bill-of-materials');
    if (!table) return;
    const want = new Set(slugs);
    const changes = [];
    table.rows.forEach((cells, r) => {
      const linked = cells.some(cell => {
        const m = cell.match(/\]\(([^)\s]+)/);
        return m && want.has(PartLinks.slugOf(m[1]));
      });
      if (!linked) return;
      const line = view.state.doc.line(table.lines[r] + 1);
      changes.push({ from: line.from, to: Math.min(line.to + 1, view.state.doc.length) });
    });
    if (changes.length) view.dispatch({ changes, userEvent: 'delete' });
  }

  // --- Diagrams (issue #270) ---
  //
  // The drawing tool (diagram-editor.js). `/diagram` creates one from a
  // template and puts its image at the cursor; "Edit diagram" on the
  // image reopens it. A new diagram closed without saving is taken out.
  const DIAGRAM_RE = /!\[([^\]]*)\]\((https:\/\/projects\.kevsrobots\.com\/api\/projects\/\d+\/diagrams\/(\d+)\.svg(?:\?v=\d+)?)\)/g;
  const diagramSrcId = src => { const m = String(src || '').match(/\/api\/projects\/\d+\/diagrams\/(\d+)\.svg/); return m ? +m[1] : null; };

  function diagramUrl(d) {
    return API + '/api/projects/' + d.project_id + '/diagrams/' + d.id + '.svg';
  }
  function diagramMarkdown(d, version) {
    const alt = String(d.title || 'Diagram').replace(/([\[\]])/g, '\\$1');
    return '![' + alt + '](' + diagramUrl(d) + '?v=' + (version || Date.parse(d.updated_at) || 0) + ')';
  }
  // Every image of diagram `id` in the text, as { from, to }.
  function diagramLinks(markdown, id) {
    const out = [];
    for (const m of markdown.matchAll(DIAGRAM_RE)) if (+m[3] === id) out.push({ from: m.index, to: m.index + m[0].length });
    return out;
  }

  async function newDiagram(view, template) {
    if (!currentProject) await saveProject();
    if (!currentProject) { alert('Add a title and save before adding a diagram.'); return; }
    const count = (await (await apiFetch(API + '/api/projects/' + currentProject.id + '/diagrams', { credentials: 'include' })).json().catch(() => [])).length || 0;
    const t = DiagramEditor.TEMPLATES.find(x => x.id === template);
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/diagrams', {
      method: 'POST', credentials: 'include', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ title: (t && t.id !== 'blank' ? t.label : 'Diagram') + (count ? ' ' + (count + 1) : '') }),
    });
    if (!resp.ok) { alert('Could not create the diagram (HTTP ' + resp.status + ').'); return; }
    const d = await resp.json();
    ProjectSlashCommands.insertAtCursor(view, diagramMarkdown(d, 0), true);
    openDiagram(d.id, { template, fresh: true });
  }

  function openDiagram(id, opts) {
    opts = opts || {};
    const view = mde.view;
    return DiagramEditor.open({
      projectId: currentProject.id,
      diagramId: id,
      template: opts.template,
      apiFetch: (url, o) => apiFetch(url, Object.assign({ credentials: 'include' }, o || {})),
      listImages: async () => {
        const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/images', { credentials: 'include' });
        return resp.ok ? (await resp.json()).map(img => ({ url: imageViewUrl(img), title: img.caption || (img.filename || '').replace(/\.[^.]+$/, '') })) : [];
      },
      uploadImage: async (blob, name) => {
        const file = blob instanceof File ? blob : new File([blob], name || 'diagram-image.png', { type: blob.type || 'image/png' });
        const img = await uploadSingleImage(file);
        loadImages();
        return { url: imageViewUrl(img) };
      },
      placeImage: d => {
        const md = view.state.doc.toString();
        const changes = diagramLinks(md, d.id).map(r => ({ ...r, insert: diagramMarkdown(d) }));
        if (changes.length) view.dispatch({ changes, userEvent: 'input' });
        else ProjectSlashCommands.insertAtCursor(view, diagramMarkdown(d), true);
      },
      onClose: async saved => {
        if (!opts.fresh || saved) return;
        // Never saved: take the new diagram back out.
        const md = view.state.doc.toString();
        const changes = diagramLinks(md, id).map(r => {
          const after = md.slice(r.to, r.to + 2);
          return { from: r.from, to: r.to + (after === '\n\n' ? 2 : after[0] === '\n' ? 1 : 0) };
        });
        if (changes.length) view.dispatch({ changes, userEvent: 'delete' });
        await apiFetch(API + '/api/projects/' + currentProject.id + '/diagrams/' + id, { method: 'DELETE', credentials: 'include' }).catch(() => {});
      },
    });
  }

  // "Edit wiring" / "Edit diagram" on those images in the live view. The
  // editor redraws its images at will, so the buttons go back on whenever
  // one appears.
  function setupImageEditButtons(mde) {
    const root = mde.view.dom;
    const button = (wrap, icon, label, run) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'project-wiring-edit';
      btn.innerHTML = '<i class="fas ' + icon + ' me-1"></i>' + label;
      // Keep the click from moving the cursor into the image's Markdown.
      btn.addEventListener('mousedown', e => { e.preventDefault(); e.stopPropagation(); });
      btn.addEventListener('click', e => { e.preventDefault(); e.stopPropagation(); run(); });
      wrap.appendChild(btn);
    };
    const decorate = () => {
      root.querySelectorAll('.cm-md-image-wrap').forEach(wrap => {
        const img = wrap.querySelector('img.cm-md-image');
        if (!img || wrap.querySelector('.project-wiring-edit')) return;
        const src = img.getAttribute('src');
        if (ProjectWiring.isWiringSrc(src)) button(wrap, 'fa-plug', 'Edit wiring', openWiring);
        const id = diagramSrcId(src);
        if (id) button(wrap, 'fa-shapes', 'Edit diagram', () => currentProject && openDiagram(id));
      });
    };
    new MutationObserver(decorate).observe(root, { childList: true, subtree: true });
    decorate();
  }

  // Issue #265: a row in the Bill of Materials table for each part, one
  // of each, after its last part. Parts the table already links are left
  // out. A project without the table gets one, under its Bill of
  // Materials heading when it has that.
  function addBomRows(view, parts) {
    const markdown = view.state.doc.toString();
    const parsed = BomSync.parse(markdown);
    const have = new Set(parsed ? parsed.rows.map(r => r.part_slug).filter(Boolean) : []);
    const adding = parts.filter(p => !have.has(p.slug));
    if (!adding.length) { view.focus(); return; }
    const link = p => '[' + p.name.replace(/([\[\]])/g, '\\$1').replace(/\|/g, '\\|') + '](' + PartLinks.href(p.slug) + ')';
    const doc = view.state.doc;
    const table = ProjectSections.sectionTable(markdown, 'bill-of-materials');
    let at;
    let insert;
    if (table) {
      let partCol = table.header.findIndex(h => /^(part|parts|item|items|name|component|components)$/.test(h));
      if (partCol < 0) partCol = 0;
      const qtyCol = table.header.findIndex(h => /^(qty|quantity|count)$/.test(h));
      let after = table.headerLine + 1;
      table.rows.forEach((row, r) => { if (!ProjectSlashCommands.isTotalRow(row)) after = table.lines[r]; });
      const rows = adding.map(p => {
        const cells = table.header.map(() => '');
        cells[partCol] = link(p);
        if (qtyCol >= 0) cells[qtyCol] = '1';
        return '| ' + cells.join(' | ') + ' |';
      });
      at = doc.line(after + 1).to;
      insert = '\n' + rows.join('\n');
    } else {
      const items = adding.map(p => ({ name: p.name, quantity: 1, part_slug: p.slug, unit_cost: null }));
      const section = ProjectSections.get('bill-of-materials');
      const lines = markdown.split('\n');
      const headingLine = lines.findIndex(line => {
        const m = line.match(/^ {0,3}#{1,6}\s+(.*?)\s*#*\s*$/);
        return m && ProjectSections.match(m[1]) === section;
      });
      if (headingLine >= 0) {
        at = doc.line(headingLine + 1).to;
        insert = '\n\n' + BomSync.tableFromRows(items).split('\n').slice(2).join('\n').replace(/\n$/, '');
      } else {
        at = doc.length;
        const tail = markdown.endsWith('\n\n') || !markdown ? '' : markdown.endsWith('\n') ? '\n' : '\n\n';
        insert = tail + BomSync.tableFromRows(items, section.heading);
      }
    }
    view.dispatch({ changes: { from: at, insert }, selection: { anchor: at + insert.length }, scrollIntoView: true, userEvent: 'input' });
    view.focus();
  }

  // --- Part links (issue #263) ---
  //
  // A part named in the text links to its catalog page; part-links.js
  // draws those links as chips with the part's photo and a card on hover.
  // Right-clicking one offers Open, Change and Unlink, and a bar over the
  // editor offers to link the Bill of Materials rows that name a catalog
  // part exactly.
  const partApiUrl = url => (url && url.startsWith('/') ? API + url : url);

  async function searchCatalogParts(q) {
    const resp = await fetch(API + '/api/parts?limit=8&q=' + encodeURIComponent(q || ''));
    if (!resp.ok) return [];
    return (await resp.json()).map(p => ({
      name: p.name,
      slug: p.slug,
      url: PartLinks.href(p.slug),
      thumb: partApiUrl(p.cover_url),
      detail: [p.category, 'used in ' + p.usage_count + (p.usage_count === 1 ? ' project' : ' projects')]
        .filter(Boolean).join(' \u00b7 '),
    }));
  }

  async function createCatalogPart(name) {
    const resp = await apiFetch(API + '/api/parts', {
      method: 'POST',
      credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }),
    });
    const body = await resp.json().catch(() => ({}));
    if (resp.status === 403) {
      showAccountAgeAlert(body.detail);
      return null;
    }
    if (!resp.ok) throw new Error(typeof body.detail === 'string' ? body.detail : 'The part could not be created');
    return { name: body.name, slug: body.slug, url: PartLinks.href(body.slug) };
  }

  // The [text](url) of the link to `slug` nearest where `link` was
  // clicked (x, y) in the text.
  function partLinkRange(view, link, slug, x, y) {
    let pos = view.posAtCoords({ x, y });
    if (pos == null) {
      try { pos = view.posAtDOM(link); } catch (_) { pos = 0; }
    }
    const doc = view.state.doc.toString();
    let best = null;
    for (const m of doc.matchAll(/\[((?:\\.|[^\]\\])*)\]\(([^)\s]+)\)/g)) {
      if (doc[m.index - 1] === '!' || PartLinks.slugOf(m[2]) !== slug) continue;
      const from = m.index;
      const to = from + m[0].length;
      const d = pos < from ? from - pos : pos > to ? pos - to : 0;
      if (!best || d < best.d) best = { from, to, d, text: m[1].replace(/\\([\[\]\\])/g, '$1') };
    }
    return best;
  }

  let partMenu = null;
  function closePartMenu() {
    if (partMenu) partMenu.remove();
    partMenu = null;
  }

  function showPartMenu(view, link, x, y) {
    closePartMenu();
    const range = partLinkRange(view, link.el, link.slug, x, y);
    if (!range) return;
    // The link as it was when the menu opened. The text under it is
    // redrawn as the cursor moves, so `link.el` may be gone by now.
    const source = view.state.sliceDoc(range.from, range.to);
    const replace = insert => {
      if (view.state.sliceDoc(range.from, range.to) !== source) return;
      view.dispatch({ changes: { from: range.from, to: range.to, insert }, selection: { anchor: range.from + insert.length }, userEvent: 'input' });
      view.focus();
    };
    const actions = [
      ['Open part page', () => window.open(PartLinks.href(link.slug), '_blank', 'noopener')],
      ['Change part\u2026', () => {
        view.dispatch({ selection: { anchor: range.to } });
        ProjectSlashCommands.partPicker(view, slashContext(), part => {
          replace(ProjectSlashCommands.partLink(part));
        }, { placeholder: 'Change to\u2026' });
      }],
      ['Unlink', () => replace(range.text)],
    ];
    partMenu = document.createElement('div');
    partMenu.className = 'part-link-menu';
    partMenu.setAttribute('role', 'menu');
    actions.forEach(([label, run]) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.setAttribute('role', 'menuitem');
      b.textContent = label;
      b.addEventListener('click', () => { closePartMenu(); run(); });
      partMenu.appendChild(b);
    });
    partMenu.addEventListener('keydown', e => {
      const items = [...partMenu.querySelectorAll('button')];
      const i = items.indexOf(document.activeElement);
      if (e.key === 'Escape') { e.preventDefault(); closePartMenu(); view.focus(); }
      else if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
        e.preventDefault();
        items[(i + (e.key === 'ArrowDown' ? 1 : -1) + items.length) % items.length].focus();
      }
    });
    document.body.appendChild(partMenu);
    partMenu.style.left = Math.min(x, window.innerWidth - partMenu.offsetWidth - 8) + 'px';
    partMenu.style.top = Math.min(y, window.innerHeight - partMenu.offsetHeight - 8) + 'px';
    partMenu.querySelector('button').focus();
  }

  function setupPartLinks(editor) {
    const view = editor.view;
    PartLinks.attachHover(view.contentDOM);
    PartLinks.thumbsFor(view.state.doc.toString());
    view.contentDOM.addEventListener('contextmenu', e => {
      const link = PartLinks.linkAt(e.target);
      if (!link) return;
      e.preventDefault();
      PartLinks.hideCard();
      showPartMenu(view, link, e.clientX, e.clientY);
    });
    document.addEventListener('mousedown', e => {
      if (partMenu && !partMenu.contains(e.target)) closePartMenu();
    }, true);
    const bar = document.getElementById('bom-link-bar');
    if (bar) {
      bar.addEventListener('click', e => {
        const action = e.target.closest('[data-action]');
        if (!action) return;
        if (action.dataset.action === 'link-all') linkAllBomParts();
        else if (action.dataset.action === 'review') reviewBomPart();
        else if (action.dataset.action === 'insert-bom') insertBomTable();
        else if (action.dataset.action === 'dismiss') { bomLinkBarDismissed = true; bar.hidden = true; updateBomLinkBar(); }
      });
    }
    scheduleBomLinks();
  }

  // The Bill of Materials table's rows whose part isn't a link:
  // [{ name, text, line, start, end }], line counted from 0 and start/end
  // the columns of the cell's text.
  function unlinkedBomRows(markdown) {
    const table = ProjectSections.sectionTable(markdown, 'bill-of-materials');
    if (!table) return [];
    let col = table.header.findIndex(h => /^(part|parts|item|name|component)s?$/.test(h));
    if (col < 0) col = 0;
    const lines = markdown.split('\n');
    const out = [];
    table.lines.forEach((line, r) => {
      if (ProjectSlashCommands.isTotalRow(table.rows[r])) return;
      const text = lines[line];
      const pipes = [...text.matchAll(/(?<!\\)\|/g)].map(m => m.index);
      if (pipes[col] === undefined) return;
      let start = pipes[col] + 1;
      let end = pipes[col + 1] === undefined ? text.length : pipes[col + 1];
      while (start < end && /\s/.test(text[start])) start++;
      while (end > start && /\s/.test(text[end - 1])) end--;
      const cell = text.slice(start, end);
      if (/\]\(/.test(cell)) return; // already a link
      const name = cell.replace(/[*_`]/g, '').replace(/\\\|/g, '|').trim();
      if (name) out.push({ name, text: cell, line, start, end });
    });
    return out;
  }

  const resolvedParts = new Map(); // lower-cased name -> { slug } or null
  let bomLinkBarDismissed = false;

  async function resolvePartNames(names) {
    const missing = [...new Set(names.map(n => n.toLowerCase()))].filter(n => !resolvedParts.has(n));
    if (missing.length) {
      const qs = missing.slice(0, 100).map(n => 'names=' + encodeURIComponent(n)).join('&');
      const resp = await fetch(API + '/api/parts/_resolve?' + qs).catch(() => null);
      if (resp && resp.ok) {
        for (const r of await resp.json()) resolvedParts.set(r.name.toLowerCase(), r.slug ? r : null);
      }
    }
    return names.map(n => resolvedParts.get(n.toLowerCase()) || null);
  }

  async function updateBomLinkBar() {
    if (!mde || !mde.view) return;
    const markdown = mde.view.state.doc.toString();
    PartLinks.thumbsFor(markdown);
    const bar = document.getElementById('bom-link-bar');
    if (!bar) return;
    // A project saved before the table lived in the text (issue #264).
    if (storedBom && storedBom.length && !BomSync.parse(markdown)) {
      const n = storedBom.length;
      setBomHint({
        title: n + ' Bill of Materials ' + (n === 1 ? 'row isn\u2019t' : 'rows aren\u2019t') + ' in the text yet',
        actions: [{ label: 'Insert BOM table', run: insertBomTable }],
      });
      bar.innerHTML =
        '<i class="fas fa-list" aria-hidden="true"></i>' +
        '<span>This project has ' + n + ' Bill of Materials ' + (n === 1 ? 'row' : 'rows') +
        ' that ' + (n === 1 ? 'isn\u2019t' : 'aren\u2019t') + ' in the text yet.</span>' +
        '<button type="button" class="btn btn-sm btn-success" data-action="insert-bom">Insert BOM table</button>';
      bar.hidden = false;
      return;
    }
    const rows = unlinkedBomRows(markdown);
    if (!rows.length) { setBomHint(null); bar.hidden = true; return; }
    const matches = (await resolvePartNames(rows.map(r => r.name))).filter(Boolean).length;
    const n = rows.length;
    const linkLabel = 'Link ' + (matches === n ? (n === 1 ? 'it' : 'all ' + n) : matches + ' exact ' + (matches === 1 ? 'match' : 'matches'));
    // Listed in the hints panel even once the bar is closed, which is
    // how the bar comes back.
    setBomHint({
      title: n + (n === 1 ? ' part in the Bill of Materials isn\u2019t' : ' parts in the Bill of Materials aren\u2019t') + ' linked',
      detail: 'Link them to the parts catalog so their photos and details show.',
      actions: [
        ...(bomLinkBarDismissed ? [{ label: 'Show the bar', run: showBomLinkBar }] : []),
        ...(matches ? [{ label: linkLabel, run: linkAllBomParts }] : []),
        { label: 'Review', run: reviewBomPart },
      ],
    });
    if (bomLinkBarDismissed) { bar.hidden = true; return; }
    bar.innerHTML =
      '<i class="fas fa-link" aria-hidden="true"></i>' +
      '<span>' + n + (n === 1 ? ' part in the Bill of Materials isn\u2019t' : ' parts in the Bill of Materials aren\u2019t') +
      ' linked to the parts catalog.</span>' +
      (matches ? '<button type="button" class="btn btn-sm btn-success" data-action="link-all">' + linkLabel + '</button>' : '') +
      '<button type="button" class="btn btn-sm btn-outline-secondary" data-action="review">Review</button>' +
      '<button type="button" class="btn-close ms-auto" data-action="dismiss" aria-label="Hide"></button>';
    bar.hidden = false;
  }

  // The Bill of Materials hint in the editor's hints panel (the status
  // bar's "N hints"), or none.
  function setBomHint(hint) {
    setEditorHint('bom-links', hint);
  }

  // setHints replaces the whole list, so each of the page's hints is kept
  // here by id and the list is sent again whenever one changes.
  const editorHints = new Map();
  function setEditorHint(id, hint) {
    if (hint) editorHints.set(id, Object.assign({ id: id }, hint));
    else editorHints.delete(id);
    if (!mde || typeof mde.setHints !== 'function') return;
    mde.setHints(Array.from(editorHints.values()));
  }

  function showBomLinkBar() {
    bomLinkBarDismissed = false;
    updateBomLinkBar().then(() => {
      const bar = document.getElementById('bom-link-bar');
      if (bar && !bar.hidden) bar.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    });
  }

  let bomLinksTimer = null;
  let galleryAddTimer = null;
  function scheduleGalleryAddBar() {
    clearTimeout(galleryAddTimer);
    galleryAddTimer = setTimeout(updateGalleryAddBar, 1000);
  }

  function scheduleBomLinks() {
    clearTimeout(bomLinksTimer);
    bomLinksTimer = setTimeout(updateBomLinkBar, 1000);
  }

  // --- Bill of Materials tip ---
  //
  // Above the Bill of Materials table in the editor: what the Qty and Price
  // columns do (BomSync.totalsChanges keeps the Total column and row). It
  // can be closed (remembered on this device); the hints panel brings it
  // back. Closing hides it with a class rather than redrawing, because the
  // editor only redraws its tables when the text changes.
  const BOM_TIP_CLOSED_KEY = 'projectEditor.bomTipClosed';
  let bomTipClosed = false;
  try { bomTipClosed = localStorage.getItem(BOM_TIP_CLOSED_KEY) === '1'; } catch (e) {}

  function bomTableTip(rows, heading) {
    const section = heading && ProjectSections.match(heading);
    if (!section || section.id !== 'bill-of-materials') return null;
    const header = (rows[0] || []).map(h => h.replace(/[*_`]/g, '').trim().toLowerCase());
    const priced = header.some(h => /^(qty|quantity|count)$/.test(h)) &&
      header.some(h => /^(price|unit price|price each|each|cost|unit cost)$/.test(h));
    const tip = document.createElement('div');
    tip.className = 'kr-notice bom-table-tip';
    tip.setAttribute('role', 'note');
    tip.innerHTML =
      '<i class="fas fa-calculator" aria-hidden="true"></i>' +
      '<span>' + (priced
        ? 'Each row\u2019s <strong>Total</strong> is Qty \u00d7 Price, and the <strong>Total</strong> row adds them up. The editor fills both in for you. Write prices with a dot, like 4.99.'
        : 'Add a <strong>Qty</strong> column and a <strong>Price</strong> column, and the editor adds a <strong>Total</strong> column and a <strong>Total</strong> row for you.') +
      '</span>' +
      '<button type="button" class="btn-close" aria-label="Hide this tip"></button>';
    tip.querySelector('.btn-close').addEventListener('click', () => setBomTipClosed(true));
    return tip;
  }

  // The Bill of Materials' Qty (Quantity, Count) cells get up and down
  // buttons in the editor, and Alt+Up / Alt+Down. Each step writes the plain
  // number into the Markdown, so the totals are worked out again as when
  // it's typed. Not the Total row, whose Qty cell stays empty.
  function bomQtyStepper({ header, row, heading }) {
    const section = heading && ProjectSections.match(heading);
    if (!section || section.id !== 'bill-of-materials') return null;
    if (!/^(qty|quantity|count)$/.test(header.replace(/[*_`]/g, '').trim().toLowerCase())) return null;
    if (ProjectSlashCommands.isTotalRow(row)) return null;
    return { min: 0 };
  }

  function setBomTipClosed(closed) {
    bomTipClosed = closed;
    try {
      if (closed) localStorage.setItem(BOM_TIP_CLOSED_KEY, '1');
      else localStorage.removeItem(BOM_TIP_CLOSED_KEY);
    } catch (e) {}
    contentEditor.parentElement.classList.toggle('bom-tip-closed', closed);
    setEditorHint('bom-tip', closed ? {
      title: 'Bill of Materials totals',
      detail: 'Add Qty and Price columns to the Bill of Materials table, and a Total column and row are added for you.',
      actions: [{ label: 'Show the tip', run: () => {
        setBomTipClosed(false);
        const tip = contentEditor.parentElement.querySelector('.bom-table-tip');
        if (tip) tip.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
      } }],
    } : null);
  }
  if (bomTipClosed) setBomTipClosed(true);

  // --- Bill of Materials sync (issue #264) ---
  //
  // The stored BOM rows (cost estimates, the parts catalog's "used in
  // projects") follow the Bill of Materials table after each save. Without
  // a table nothing changes, so a project saved before the table lived in
  // the text keeps its rows, and the bar above offers to write them out.
  let storedBom = null; // the API's rows as last seen; null until loaded
  let lastBomSync = null; // the rows last sent
  let bomSync = Promise.resolve();

  async function loadStoredBom() {
    if (!currentProject) { storedBom = []; return; }
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/bom', { credentials: 'include' });
    storedBom = resp.ok ? await resp.json() : null;
    scheduleBomLinks();
  }

  function syncBom(markdown) {
    bomSync = bomSync.then(() => syncBomNow(markdown)).catch(e => console.error('BOM sync failed:', e));
    return bomSync;
  }

  async function syncBomNow(markdown) {
    if (!currentProject) return;
    const parsed = BomSync.parse(markdown);
    if (!parsed) return;
    const body = JSON.stringify({ rows: parsed.rows });
    if (body === lastBomSync) return;
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/bom/sync', {
      method: 'PUT', credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body,
    });
    if (!resp.ok) throw new Error(await resp.text());
    lastBomSync = body;
    storedBom = await resp.json();
  }

  function insertBomTable() {
    if (!storedBom || !storedBom.length) return;
    const view = mde.view;
    const doc = view.state.doc;
    const heading = ProjectSections.get('bill-of-materials').heading;
    const lead = !doc.length ? '' : doc.toString().endsWith('\n\n') ? '' : doc.toString().endsWith('\n') ? '\n' : '\n\n';
    const insert = lead + BomSync.tableFromRows(storedBom, heading);
    view.dispatch({ changes: { from: doc.length, insert }, selection: { anchor: doc.length + insert.length }, scrollIntoView: true, userEvent: 'input' });
    view.focus();
    updateBomLinkBar();
  }

  // Links each unlinked row that names a catalog part exactly, keeping
  // the row's own wording as the link's text.
  async function linkAllBomParts() {
    const view = mde.view;
    const rows = unlinkedBomRows(view.state.doc.toString());
    const found = await resolvePartNames(rows.map(r => r.name));
    const changes = [];
    rows.forEach((r, i) => {
      if (!found[i]) return;
      const line = view.state.doc.line(r.line + 1);
      changes.push({
        from: line.from + r.start,
        to: line.from + r.end,
        insert: '[' + r.text.replace(/([\[\]])/g, '\\$1') + '](' + PartLinks.href(found[i].slug) + ')',
      });
    });
    if (changes.length) view.dispatch({ changes, userEvent: 'input' });
    updateBomLinkBar();
  }

  // Picks the catalog part for the first unlinked row, starting from its name.
  function reviewBomPart() {
    const view = mde.view;
    const row = unlinkedBomRows(view.state.doc.toString())[0];
    if (!row) return;
    const line = view.state.doc.line(row.line + 1);
    const from = line.from + row.start;
    view.dispatch({ selection: { anchor: from, head: line.from + row.end }, scrollIntoView: true });
    view.focus();
    ProjectSlashCommands.partPicker(view, slashContext(), part => {
      if (!part.url) return;
      const cur = unlinkedBomRows(view.state.doc.toString()).find(r => r.line === row.line);
      if (!cur) return;
      const l = view.state.doc.line(cur.line + 1);
      view.dispatch({
        changes: { from: l.from + cur.start, to: l.from + cur.end, insert: '[' + cur.text.replace(/([\[\]])/g, '\\$1') + '](' + part.url + ')' },
        userEvent: 'input',
      });
      view.focus();
      updateBomLinkBar();
    }, { placeholder: 'Link \u201c' + row.name + '\u201d to\u2026', catalogOnly: true, query: row.name });
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
    if (mde) schedulePinout();
  }

  // Diagram menu, under the toolbar's diagram button.
  async function showDiagramMenu(editor, btn) {
    const existing = document.querySelector('.diagram-menu-popover');
    if (existing) { existing.remove(); return; }
    const starters = await loadDiagramStarters();

    const rect = btn.getBoundingClientRect();
    const pop = document.createElement('div');
    pop.className = 'diagram-menu-popover';
    pop.style.cssText = 'position:fixed;z-index:10000;background:white;border:1px solid #ddd;border-radius:8px;padding:4px;box-shadow:0 4px 12px rgba(0,0,0,0.15);max-height:60vh;overflow-y:auto;left:' + rect.left + 'px;top:' + (rect.bottom + 4) + 'px;';
    pop.innerHTML = starters.map(() => '<div class="image-picker-item p-2" style="cursor:pointer"><small></small></div>').join('');
    document.body.appendChild(pop);

    pop.querySelectorAll('.image-picker-item').forEach((item, idx) => {
      item.querySelector('small').textContent = starters[idx].label;
      if (starters[idx].detail) item.title = starters[idx].detail;
      item.addEventListener('click', () => {
        editor.replaceSelection(starters[idx].md);
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

  // --- Pictures ---
  // Uploaded pictures are the project's image library: the image picker
  // shows them (and can delete them), and the page's `::: gallery` block
  // shows the ones chosen, in its order. The old Images section under the
  // document is retired. The cover is the first gallery picture, else the
  // first picture in the text, else the first upload.
  let projectImages = [];

  async function loadImages() {
    if (!currentProject) return;
    try { projectImages = await fetchProjectImages(); } catch (e) { return; }
    updateGalleryAddBar();
    if (window._recomputeCompleteness) window._recomputeCompleteness();
  }

  function escHtml(t) {
    return String(t == null ? '' : t)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  const GALLERY_OPEN = /^[ \t]{0,3}:::[ \t]*gallery\b.*$/i;
  const GALLERY_CLOSE = /^[ \t]{0,3}:::[ \t]*$/;
  const PICTURE = /!\[(?:\\.|[^\]\\])*\]\(\s*<?([^\s)>]+)>?/;

  // The first closed `::: gallery` block: { open, close } line numbers from 0.
  function firstGallery(lines) {
    let fence = null;
    for (let i = 0; i < lines.length; i++) {
      const marker = /^[ \t]{0,3}(`{3,}|~{3,})/.exec(lines[i]);
      if (fence) { if (marker && marker[1][0] === fence[0]) fence = null; continue; }
      if (marker) { fence = marker[1]; continue; }
      if (!GALLERY_OPEN.test(lines[i])) continue;
      for (let j = i + 1; j < lines.length; j++) {
        if (GALLERY_CLOSE.test(lines[j])) return { open: i, close: j };
        if (GALLERY_OPEN.test(lines[j])) break;
      }
    }
    return null;
  }

  // The cover: the first gallery picture, else the first picture in the
  // text, else the first upload; null when there's none.
  function coverUrl(markdown) {
    const lines = String(markdown || '').split('\n');
    const gallery = firstGallery(lines);
    const inGallery = gallery && PICTURE.exec(lines.slice(gallery.open + 1, gallery.close).join('\n'));
    if (inGallery) return inGallery[1];
    const anywhere = PICTURE.exec(markdown || '');
    if (anywhere) return anywhere[1];
    return projectImages.length && currentProject ? imageViewUrl(projectImages[0]) : null;
  }

  // Uploads the text doesn't show: pictures from the old Images section, or
  // ones added since and taken out again.
  function unplacedImages(markdown) {
    return projectImages.filter(img => !markdown.includes('/images/' + img.id + '/view'));
  }

  // Adds pictures ({ url, name, caption }) to the end of the page's first
  // gallery, or a new Gallery section at the end when it has none.
  function addPicturesToGallery(pictures) {
    if (!pictures.length || !mde || !mde.view) return;
    const view = mde.view;
    const doc = view.state.doc;
    const lines = doc.toString().split('\n');
    const gallery = firstGallery(lines);
    const block = ProjectSlashCommands.galleryMarkdown('filmstrip', pictures);
    const pictureLines = block.split('\n').slice(1, -1).join('\n');
    if (gallery) {
      const close = doc.line(gallery.close + 1);
      view.dispatch({ changes: { from: close.from, insert: pictureLines + '\n' }, userEvent: 'input.gallery' });
      return;
    }
    const markdown = doc.toString();
    const heading = ProjectSections.get('gallery').heading;
    const hasHeading = markdown.split('\n').some(line => {
      const m = /^#{1,6}\s+(.*?)\s*#*\s*$/.exec(line);
      const section = m && ProjectSections.match(m[1]);
      return section && section.id === 'gallery';
    });
    const tail = markdown.trim() ? (markdown.endsWith('\n\n') ? '' : markdown.endsWith('\n') ? '\n' : '\n\n') : '';
    const insert = tail + (hasHeading ? '' : '# ' + heading + '\n\n') + block + '\n';
    view.dispatch({ changes: { from: doc.length, insert: insert }, userEvent: 'input.gallery' });
  }

  // The notice over the document when uploads aren't on the page, with a
  // button that puts them in the gallery. "Not now" hides it until another
  // picture is left out.
  function galleryAddDismissKey() {
    return 'projects.' + (currentProject ? currentProject.id : 'new') + '.gallery-add-dismissed';
  }
  function updateGalleryAddBar() {
    const bar = document.getElementById('gallery-add-bar');
    if (!bar || !mde || !mde.view) return;
    const unplaced = unplacedImages(mde.view.state.doc.toString());
    let dismissed = [];
    try { dismissed = JSON.parse(localStorage.getItem(galleryAddDismissKey()) || '[]'); } catch (e) {}
    if (!unplaced.length || unplaced.every(img => dismissed.includes(img.id))) { bar.hidden = true; return; }
    const n = unplaced.length;
    bar.innerHTML =
      '<i class="fas fa-images" aria-hidden="true"></i>' +
      '<span>' + n + ' uploaded ' + (n === 1 ? 'picture isn\u2019t' : 'pictures aren\u2019t') +
      ' on your page. The Images section is now the gallery in your text.</span>' +
      '<button type="button" class="btn btn-sm btn-success" data-action="add-to-gallery">Add ' + (n === 1 ? 'it' : 'them') + ' to the gallery</button>' +
      '<button type="button" class="btn btn-sm btn-link" data-action="dismiss">Not now</button>';
    bar.hidden = false;
  }
  (function wireGalleryAddBar() {
    const bar = document.getElementById('gallery-add-bar');
    if (!bar) return;
    bar.addEventListener('click', e => {
      const action = e.target.closest('[data-action]');
      if (!action || !mde || !mde.view) return;
      const unplaced = unplacedImages(mde.view.state.doc.toString());
      if (action.dataset.action === 'add-to-gallery') {
        addPicturesToGallery(unplaced.map(img => ({ url: imageViewUrl(img), name: img.filename, caption: img.caption || '' })));
      } else if (action.dataset.action === 'dismiss') {
        try { localStorage.setItem(galleryAddDismissKey(), JSON.stringify(unplaced.map(img => img.id))); } catch (e) {}
      }
      updateGalleryAddBar();
    });
  })();

  // The image picker's delete button (it asks first).
  async function deleteLibraryImage(image) {
    const m = /\/images\/(\d+)\/view/.exec(image.url || '');
    if (!m || !currentProject) throw new Error('Not one of this project\u2019s pictures');
    const resp = await apiFetch(API + '/api/projects/' + currentProject.id + '/images/' + m[1], {
      method: 'DELETE', credentials: 'include',
    });
    if (!resp.ok) throw new Error('Couldn\u2019t delete the picture');
    loadImages();
  }

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
  // keeps a Total column and a "**Total**" row in the Markdown
  // (BomSync.totalsChanges works out which lines change).
  function applyBomTotals() {
    if (!mde || !mde.view) return;
    const view = mde.view;
    const doc = view.state.doc;
    const changes = BomSync.totalsChanges(doc.toString());
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
  // Videos live in the document: `::: videos` blocks of YouTube links
  // (ProjectEmbeds.findVideoBlocks), or an older project's "Videos"
  // section table with the YouTube link in its first column and an
  // optional title. The project's video list (the API, which the project
  // page's player and the cards read) follows them after each save. A
  // project saved before the section existed gets one made from its
  // videos when it's opened, the way code_repo_url became a section.
  const extractYouTubeId = ProjectEmbeds.youTubeId;
  let projectVideos = null; // the API's list as last seen; null until it loads

  // [{ youtube_id, title }] in document order (the Videos table first,
  // then the blocks), each video once.
  function videosFromMarkdown(markdown) {
    const seen = new Set();
    const out = [];
    const add = (id, title) => {
      if (!id || seen.has(id)) return;
      seen.add(id);
      out.push({ youtube_id: id, title: title || null });
    };
    const table = ProjectSections.sectionTable(markdown, 'videos');
    if (table) {
      let videoCol = table.header.findIndex(h => /^(video|videos|url|link|youtube)$/.test(h));
      if (videoCol < 0) videoCol = 0;
      const titleCol = table.header.findIndex(h => /^(title|name|caption|description)$/.test(h));
      table.rows.forEach(row => {
        const cell = row[videoCol] || '';
        const link = cell.match(/\]\(([^)\s]+)/);
        const id = extractYouTubeId(link ? link[1] : cell.replace(/^<|>$/g, ''));
        add(id, titleCol >= 0 ? (row[titleCol] || '').replace(/\\\|/g, '|').trim() : '');
      });
    }
    ProjectEmbeds.findVideoBlocks(markdown).forEach(block => block.videos.forEach(v => add(v.id, v.title)));
    return out;
  }

  function withVideosSection(markdown, videos) {
    const md = markdown || '';
    if (!videos || !videos.length || ProjectSections.present(md).has('videos') || ProjectEmbeds.findVideoBlocks(md).length) return md;
    const block = ProjectEmbeds.videoBlock(videos.map(v => ({ id: v.youtube_id, title: v.title })));
    return md.replace(/\s*$/, '') + (md.trim() ? '\n\n' : '') +
      '# ' + ProjectSections.get('videos').heading + '\n\n' + block + '\n';
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
    const uploadedImages = [];
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
          const img = await uploadSingleImage(image);
          uploadedImages.push({ url: imageViewUrl(img), name: img.filename || file.name, caption: img.caption || '' });
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

    // Into the page's gallery, so they show (a new Gallery section if it has none).
    addPicturesToGallery(uploadedImages);
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
    // The cover is the gallery's first picture (or the text's, or the first upload).
    { key: 'cover',  points: 15, label: 'Add a cover picture (the first in your gallery)', target: 'content-editor',
      check: () => !!coverUrl(getEditorValue()) },
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
  let completenessPct = 0;
  let wasComplete = false;
  // Set by initCompletenessOverlay: collapses the panel without saving that.
  let collapseCompleteness = null;
  // Closed with the panel's ×: listed in the editor's hints instead,
  // which is how it comes back. Remembered per browser.
  const COMPLETENESS_CLOSED_KEY = 'projectEditor.completenessClosed';
  let completenessClosed = false;
  try { completenessClosed = localStorage.getItem(COMPLETENESS_CLOSED_KEY) === '1'; } catch (e) {}
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
    const scoreEl = document.getElementById('completeness-score');
    const pointsEl = document.getElementById('completeness-points');
    const barEl = document.getElementById('completeness-bar');
    const nextEl = document.getElementById('completeness-next');
    const nextLink = document.getElementById('completeness-next-link');
    const listEl = document.getElementById('completeness-checklist');
    const celebrateEl = document.getElementById('completeness-celebration');
    const completeBadge = document.getElementById('completeness-complete-badge');
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
    completenessPct = pct;
    if (completenessClosed) setCompletenessHint();

    // At 100% the header gets a "Complete" pill in place of the score, and
    // the panel collapses to just its header when it gets there. Expanding
    // it still shows the checklist, all ticked, under the celebration.
    const complete = pct >= 100;
    if (miniEl) miniEl.classList.toggle('d-none', complete);
    if (completeBadge) completeBadge.classList.toggle('d-none', !complete);
    if (celebrateEl) celebrateEl.classList.toggle('d-none', !complete);
    if (complete && !wasComplete && typeof collapseCompleteness === 'function') collapseCompleteness();
    wasComplete = complete;
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
           + '<i class="far fa-circle me-2"></i>'
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
    collapseCompleteness = () => { collapsed = true; apply(); };
    apply();
    const closeBtn = document.getElementById('completeness-close');
    if (closeBtn) closeBtn.addEventListener('click', closeCompleteness);
  })();

  function setCompletenessHint() {
    setEditorHint('completeness', completenessClosed ? {
      title: 'Project completeness: ' + completenessPct + '%',
      detail: completenessPct >= 100 ? 'Everything\u2019s filled in.' : 'See what\u2019s left to fill in.',
      actions: [{ label: 'Show project completeness', run: showCompleteness }],
    } : null);
  }

  // The editor's "N hints" button in its status bar, where the panel goes
  // when it's closed and comes back from.
  function hintsButton() {
    const btn = document.querySelector('.mde .writing-lint .writing-toggle');
    if (!btn) return null;
    const r = btn.getBoundingClientRect();
    return (r.width && r.bottom > 0 && r.top < window.innerHeight) ? r : null;
  }

  // Moves and shrinks the panel onto the hints button (or back from it).
  function flyCompleteness(card, toHints) {
    const target = hintsButton();
    const reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (!target || reduce || typeof card.animate !== 'function') return Promise.resolve();
    const from = card.getBoundingClientRect();
    const dx = (target.left + target.width / 2) - (from.left + from.width / 2);
    const dy = (target.top + target.height / 2) - (from.top + from.height / 2);
    const scale = Math.max(target.width / from.width, 0.05);
    const away = { transform: 'translate(' + dx + 'px,' + dy + 'px) scale(' + scale + ')', opacity: 0.1 };
    const home = { transform: 'none', opacity: 1 };
    const anim = card.animate(toHints ? [home, away] : [away, home], {
      duration: 420,
      easing: toHints ? 'cubic-bezier(.5,0,.75,0)' : 'cubic-bezier(.25,1,.5,1)',
    });
    return anim.finished.catch(() => {});
  }

  function pulseHintsButton() {
    const btn = document.querySelector('.mde .writing-lint .writing-toggle');
    if (!btn) return;
    btn.classList.remove('completeness-landed');
    void btn.offsetWidth;
    btn.classList.add('completeness-landed');
    setTimeout(() => btn.classList.remove('completeness-landed'), 700);
  }

  async function closeCompleteness() {
    const card = document.getElementById('completeness-card');
    if (!card || completenessClosed) return;
    completenessClosed = true;
    try { localStorage.setItem(COMPLETENESS_CLOSED_KEY, '1'); } catch (e) {}
    card.style.transformOrigin = 'center';
    await flyCompleteness(card, true);
    card.classList.add('d-none');
    setCompletenessHint();
    pulseHintsButton();
  }

  function showCompleteness() {
    const card = document.getElementById('completeness-card');
    if (!card) return;
    completenessClosed = false;
    try { localStorage.removeItem(COMPLETENESS_CLOSED_KEY); } catch (e) {}
    setCompletenessHint();
    card.classList.remove('d-none');
    flyCompleteness(card, false);
  }

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
    loadStoredBom().catch(() => {});
    editorContainer.classList.remove('d-none');
    const completenessCard = document.getElementById('completeness-card');
    if (completenessCard && !completenessClosed) completenessCard.classList.remove('d-none');
    recomputeCompleteness();
  }

  init();
})();
