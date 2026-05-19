/**
 * Build-instructions section (issue #178, Phase 0).
 *
 * Deliberately a per-section module rather than another block inside the
 * 2765-line project-editor.js — Phase 1 will add Fabric.js canvas code
 * which MUST live somewhere modular and not pile on top of the editor
 * monolith. This file establishes that pattern.
 *
 * Public surface:
 *   window.ProjectInstructions.init(projectId, isOwner)
 *
 * Backend contract: see stacks/projects-api/projects_api/routers/instructions.py.
 * Idempotent POST on the instruction means we can lazily create it on
 * first input — no need to "create first" up-front.
 */
(function () {
  'use strict';

  var API = 'https://projects.kevsrobots.com';

  // Use ProjectAuth.apiFetch when present (sends cookies + dev token) and
  // run 403 responses through TermsGate.handleResponse so a logged-in
  // user who hasn't accepted the T&Cs sees the modal rather than a
  // silent failure. Falls back to plain fetch for unauthenticated reads.
  function apiFetch(url, opts) {
    opts = opts || {};
    if (window.ProjectAuth && typeof window.ProjectAuth.apiFetch === 'function') {
      return window.ProjectAuth.apiFetch(url, opts);
    }
    opts.credentials = opts.credentials || 'include';
    return fetch(url, opts);
  }

  async function apiFetchWithTermsRetry(url, opts) {
    var resp = await apiFetch(url, opts);
    if (resp.status === 403 && window.TermsGate && typeof window.TermsGate.handleResponse === 'function') {
      try {
        var retry = await window.TermsGate.handleResponse(resp);
        if (retry) return apiFetch(url, opts);
      } catch (_) { /* swallow — original 403 will surface to caller */ }
    }
    return resp;
  }

  // --- Tiny helpers -------------------------------------------------------

  function escapeHtml(s) {
    if (s === null || s === undefined) return '';
    var d = document.createElement('div');
    d.textContent = String(s);
    return d.innerHTML;
  }

  // Per-input debounce so blur saves don't queue multiple PUTs on the
  // same field. Key is a string identifier; value is the active timer.
  var _debounces = {};
  function debounce(key, fn, ms) {
    if (_debounces[key]) clearTimeout(_debounces[key]);
    _debounces[key] = setTimeout(function () {
      delete _debounces[key];
      fn();
    }, ms);
  }

  // Flash a "Saved" indicator next to an input. Matches the visual
  // language of project-editor.js's showSaveStatus(...) without
  // coupling to its (closure-scoped) implementation.
  function flashSaved(indicatorEl) {
    if (!indicatorEl) return;
    indicatorEl.classList.remove('d-none');
    if (indicatorEl._fadeTimer) clearTimeout(indicatorEl._fadeTimer);
    indicatorEl._fadeTimer = setTimeout(function () {
      indicatorEl.classList.add('d-none');
    }, 1500);
  }

  // --- Module state -------------------------------------------------------

  var state = {
    projectId: null,
    isOwner: false,
    instructionId: null,  // null until the instruction row exists server-side
    steps: [],            // last-fetched ordered list (cache for move-up/down)
  };

  // --- DOM handles --------------------------------------------------------

  var dom = {};
  function bindDom() {
    dom.section = document.getElementById('editor-instructions-section');
    dom.title = document.getElementById('instruction-title');
    dom.description = document.getElementById('instruction-description');
    dom.metaSaved = document.getElementById('instruction-meta-saved');
    dom.stepsList = document.getElementById('instruction-steps-list');
    dom.addBtn = document.getElementById('add-instruction-step');
  }

  // --- Backend calls ------------------------------------------------------

  async function fetchInstruction() {
    var resp = await apiFetch(API + '/api/projects/' + state.projectId + '/instruction');
    if (resp.status === 404) return null;
    if (!resp.ok) throw new Error('Failed to load instruction: HTTP ' + resp.status);
    return resp.json();
  }

  async function ensureInstruction() {
    // Idempotent: backend returns the existing row with 200 if one already
    // exists for this project. Safe to call repeatedly under double-click.
    if (state.instructionId) return state.instructionId;
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          title: dom.title.value || null,
          description: dom.description.value || null,
        }),
      }
    );
    if (!resp.ok) throw new Error('Failed to create instruction: HTTP ' + resp.status);
    var body = await resp.json();
    state.instructionId = body.id;
    return body.id;
  }

  async function putInstructionFields(fields) {
    await ensureInstruction();
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction',
      {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(fields),
      }
    );
    if (!resp.ok) throw new Error('Failed to save instruction: HTTP ' + resp.status);
    return resp.json();
  }

  async function fetchSteps() {
    var resp = await apiFetch(
      API + '/api/projects/' + state.projectId + '/instruction/steps'
    );
    if (!resp.ok) throw new Error('Failed to list steps: HTTP ' + resp.status);
    return resp.json();
  }

  async function postStep() {
    await ensureInstruction();
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction/steps',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ title: '', description: '' }),
      }
    );
    if (!resp.ok) throw new Error('Failed to add step: HTTP ' + resp.status);
    return resp.json();
  }

  async function putStep(stepId, fields) {
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction/steps/' + stepId,
      {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(fields),
      }
    );
    if (!resp.ok) throw new Error('Failed to update step: HTTP ' + resp.status);
    return resp.json();
  }

  async function deleteStep(stepId) {
    var resp = await apiFetchWithTermsRetry(
      API + '/api/projects/' + state.projectId + '/instruction/steps/' + stepId,
      { method: 'DELETE' }
    );
    if (!resp.ok && resp.status !== 204) {
      throw new Error('Failed to delete step: HTTP ' + resp.status);
    }
  }

  // --- Rendering ----------------------------------------------------------

  function renderSteps(steps) {
    state.steps = steps.slice();
    if (!dom.stepsList) return;
    if (steps.length === 0) {
      dom.stepsList.innerHTML =
        '<p class="text-muted small fst-italic mb-0">' +
        'No steps yet &mdash; click <strong>Add step</strong> below to start outlining your build.' +
        '</p>';
      return;
    }
    var disabledAttr = state.isOwner ? '' : ' disabled';
    var hideActions = state.isOwner ? '' : ' style="display:none"';
    dom.stepsList.innerHTML = steps.map(function (s, idx) {
      var isFirst = idx === 0;
      var isLast = idx === steps.length - 1;
      return (
        '<div class="step-card" data-step-id="' + s.id + '">' +
          '<div class="d-flex align-items-start gap-2">' +
            '<span class="step-number-badge">' + s.step_number + '</span>' +
            '<div class="flex-grow-1">' +
              '<input class="form-control form-control-sm mb-1 step-title" ' +
                     'placeholder="Step title" maxlength="200" ' +
                     'value="' + escapeHtml(s.title || '') + '"' + disabledAttr + '>' +
              '<textarea class="form-control form-control-sm step-description" ' +
                        'rows="2" placeholder="Describe what happens in this step…"' +
                        disabledAttr + '>' + escapeHtml(s.description || '') + '</textarea>' +
              '<div class="step-saved text-success small mt-1 d-none">' +
                '<i class="fas fa-check me-1"></i>Saved</div>' +
            '</div>' +
            '<div class="step-actions"' + hideActions + '>' +
              '<button class="btn btn-sm btn-outline-secondary step-up" ' +
                      'title="Move up"' +
                      (isFirst ? ' disabled' : '') + '>' +
                '<i class="fas fa-arrow-up"></i></button>' +
              '<button class="btn btn-sm btn-outline-secondary step-down" ' +
                      'title="Move down"' +
                      (isLast ? ' disabled' : '') + '>' +
                '<i class="fas fa-arrow-down"></i></button>' +
              '<button class="btn btn-sm btn-outline-danger step-delete" ' +
                      'title="Delete step">' +
                '<i class="fas fa-trash"></i></button>' +
            '</div>' +
          '</div>' +
        '</div>'
      );
    }).join('');

    // Wire per-step handlers. Event delegation would also work, but
    // direct binding keeps the closure scope clean and avoids leaking
    // step ids into shared dataset attributes.
    Array.prototype.forEach.call(
      dom.stepsList.querySelectorAll('.step-card'),
      function (card) {
        var stepId = parseInt(card.dataset.stepId, 10);
        var titleInput = card.querySelector('.step-title');
        var descInput = card.querySelector('.step-description');
        var savedInd = card.querySelector('.step-saved');

        if (titleInput) {
          titleInput.addEventListener('blur', function () {
            debounce('step-title-' + stepId, function () {
              putStep(stepId, { title: titleInput.value || null })
                .then(function () { flashSaved(savedInd); })
                .catch(function () { /* let the user see stale state — they'll retry */ });
            }, 500);
          });
        }
        if (descInput) {
          descInput.addEventListener('blur', function () {
            debounce('step-desc-' + stepId, function () {
              putStep(stepId, { description: descInput.value || null })
                .then(function () { flashSaved(savedInd); })
                .catch(function () { });
            }, 500);
          });
        }

        if (!state.isOwner) return;

        var upBtn = card.querySelector('.step-up');
        var downBtn = card.querySelector('.step-down');
        var delBtn = card.querySelector('.step-delete');

        if (upBtn) upBtn.addEventListener('click', function () {
          var idx = state.steps.findIndex(function (s) { return s.id === stepId; });
          if (idx <= 0) return;
          var newPos = state.steps[idx - 1].step_number;
          putStep(stepId, { step_number: newPos })
            .then(reloadSteps)
            .catch(function () { });
        });
        if (downBtn) downBtn.addEventListener('click', function () {
          var idx = state.steps.findIndex(function (s) { return s.id === stepId; });
          if (idx < 0 || idx >= state.steps.length - 1) return;
          var newPos = state.steps[idx + 1].step_number;
          putStep(stepId, { step_number: newPos })
            .then(reloadSteps)
            .catch(function () { });
        });
        if (delBtn) delBtn.addEventListener('click', function () {
          if (!window.confirm('Delete this step? This cannot be undone.')) return;
          deleteStep(stepId).then(reloadSteps).catch(function () { });
        });
      }
    );
  }

  async function reloadSteps() {
    try {
      var steps = await fetchSteps();
      renderSteps(steps);
    } catch (e) {
      // Network blip — render the empty-state hint rather than a stack
      // trace. The user can refresh to retry.
      renderSteps([]);
    }
  }

  // --- Init ---------------------------------------------------------------

  function wireMetaInputs() {
    if (!dom.title || !dom.description) return;

    function saveMeta(field, value) {
      putInstructionFields({ [field]: value || null })
        .then(function () { flashSaved(dom.metaSaved); })
        .catch(function () { });
    }

    dom.title.addEventListener('blur', function () {
      debounce('instruction-title', function () {
        saveMeta('title', dom.title.value);
      }, 500);
    });
    dom.description.addEventListener('blur', function () {
      debounce('instruction-description', function () {
        saveMeta('description', dom.description.value);
      }, 500);
    });
  }

  function wireAddButton() {
    if (!dom.addBtn) return;
    dom.addBtn.addEventListener('click', function () {
      dom.addBtn.disabled = true;
      postStep()
        .then(reloadSteps)
        .catch(function () { })
        .then(function () { dom.addBtn.disabled = false; });
    });
  }

  function applyOwnerGating() {
    if (state.isOwner) return;
    if (dom.title) dom.title.disabled = true;
    if (dom.description) dom.description.disabled = true;
    if (dom.addBtn) dom.addBtn.classList.add('d-none');
  }

  // Insert a prominent "Open Instruction Builder" CTA at the top of the
  // instructions card. Routes to the standalone Fabric.js page (Phase 1).
  // Rendered here (not in editor.html) so it stays adjacent to its
  // owning section and only shows up once the section module has
  // initialised — keeps the editor template lean.
  function renderOpenBuilderButton() {
    if (!dom.section) return;
    if (document.getElementById('open-instruction-builder')) return; // idempotent
    if (!state.isOwner) return; // viewers shouldn't see an editor entry-point
    var cardBody = dom.section.querySelector('.card-body');
    if (!cardBody) return;
    var btn = document.createElement('a');
    btn.id = 'open-instruction-builder';
    btn.className = 'btn btn-primary mb-3';
    btn.href = '/projects/instructions/edit.html?id=' + encodeURIComponent(state.projectId);
    btn.innerHTML = '<i class="fas fa-paint-brush me-2"></i>Open Instruction Builder';
    // Place right under the lead paragraph (above the title input) so
    // the user can dive straight into the visual editor without
    // scrolling past the outline UI.
    cardBody.insertBefore(btn, cardBody.firstChild);
  }

  async function init(projectId, isOwner) {
    state.projectId = projectId;
    state.isOwner = isOwner !== false;  // default to owner if unspecified
    bindDom();
    if (!dom.section) return;  // page didn't render the section — nothing to do
    applyOwnerGating();
    renderOpenBuilderButton();
    wireMetaInputs();
    wireAddButton();

    try {
      var instruction = await fetchInstruction();
      if (instruction) {
        state.instructionId = instruction.id;
        dom.title.value = instruction.title || '';
        dom.description.value = instruction.description || '';
        renderSteps(instruction.steps || []);
      } else {
        renderSteps([]);
      }
    } catch (e) {
      renderSteps([]);
    }
  }

  window.ProjectInstructions = { init: init };
})();
