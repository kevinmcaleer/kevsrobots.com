/**
 * Builder Renderer — renders an Instruction Builder project's steps
 * into a container element. Shared between the public viewer
 * (/projects/view.html) and the editor's Builder-mode preview pane
 * (/projects/editor.html). Both surfaces show the same rendered
 * output so what the author sees in the editor matches what
 * visitors see on the public page.
 *
 * Dependencies (assumed loaded on the host page):
 *   - window.marked            — markdown parser (CDN)
 *   - window.ProjectAuth       — wraps fetch with auth headers
 *   - window.PROJECTS_API_BASE — base URL of the projects API,
 *                                falls back to ''/auto.
 *
 * Usage:
 *   BuilderRenderer.render(containerEl, projectId).then(...);
 */
(function () {
  'use strict';

  // Both editor + viewer share the same Cloudflare-fronted API host.
  // Host pages can override by setting window.PROJECTS_API_BASE before
  // this script runs (useful for dev / staging).
  function apiBase() {
    return window.PROJECTS_API_BASE || 'https://projects.kevsrobots.com';
  }

  function esc(t) {
    var d = document.createElement('div');
    d.textContent = (t == null) ? '' : String(t);
    return d.innerHTML;
  }

  function renderVideoEmbed(url) {
    if (!url) return '<em class="text-muted">No video yet.</em>';
    var yt = String(url).match(
      /(?:youtube\.com\/watch\?v=|youtu\.be\/|youtube\.com\/embed\/)([\w-]{11})/);
    if (yt) {
      return (
        '<div class="ratio ratio-16x9 mb-2">' +
        '  <iframe src="https://www.youtube.com/embed/' + esc(yt[1]) + '"' +
        '          title="YouTube video" frameborder="0"' +
        '          allow="accelerometer; autoplay; clipboard-write; encrypted-media;' +
                       ' gyroscope; picture-in-picture" allowfullscreen></iframe>' +
        '</div>'
      );
    }
    return (
      '<video controls class="w-100 mb-2" style="max-height:480px">' +
      '  <source src="' + esc(url) + '">' +
      '</video>'
    );
  }

  function renderStepBody(step, stepIdx) {
    var type = step.step_type || 'photo';
    if (type === 'text') {
      var body = step.body || '';
      return '<div class="builder-step-text">' +
        (body && window.marked ? window.marked.parse(body)
                               : (body ? esc(body)
                                       : '<em class="text-muted">No text yet.</em>')) +
        '</div>';
    }
    if (type === 'video') {
      return renderVideoEmbed(step.video_url);
    }
    if (type === 'schematic') {
      // Placeholder with a hook for the second-pass renderer to
      // attach an iframe-captured PNG once the schematic editor
      // posts a snapshot back.
      return (
        '<div class="builder-step-schematic-preview" data-step-idx="' + stepIdx + '">' +
        '  <div class="builder-step-loading text-muted small">' +
        '    <i class="fas fa-spinner fa-spin me-1"></i> Rendering schematic&hellip;' +
        '  </div>' +
        '</div>'
      );
    }
    // photo / blank — second-pass: load Fabric.js once, replay the
    // saved canvas_json into an off-screen canvas, and inject the
    // PNG via toDataURL. The placeholder below is what the user
    // sees during that flow; the renderer replaces it with an <img>.
    if (!step.canvas_json) {
      return (
        '<div class="builder-step-canvas-empty alert alert-light border text-center small text-muted">' +
        '  <i class="fas fa-image me-1"></i> Empty canvas' +
        '</div>'
      );
    }
    return (
      '<div class="builder-step-canvas-preview" data-step-idx="' + stepIdx + '">' +
      '  <div class="builder-step-loading text-muted small">' +
      '    <i class="fas fa-spinner fa-spin me-1"></i> Rendering canvas&hellip;' +
      '  </div>' +
      '</div>'
    );
  }

  function renderInto(containerEl, instr) {
    if (!instr || !Array.isArray(instr.steps) || instr.steps.length === 0) {
      containerEl.innerHTML =
        '<div class="alert alert-light border small">' +
        '  <i class="fas fa-info-circle me-1 text-muted"></i> ' +
        '  This project hasn&rsquo;t added any build steps yet.' +
        '</div>';
      return;
    }
    var steps = instr.steps.slice().sort(function (a, b) {
      return (a.step_number || 0) - (b.step_number || 0);
    });
    var parts = [];
    if (instr.title || instr.description) {
      parts.push('<header class="builder-view-header mb-4">');
      if (instr.title) {
        parts.push('<h2 class="h4 mb-1">' + esc(instr.title) + '</h2>');
      }
      if (instr.description) {
        parts.push('<p class="text-muted mb-0">' + esc(instr.description) + '</p>');
      }
      parts.push('</header>');
    }
    steps.forEach(function (s, idx) {
      var n = (s.step_number != null) ? s.step_number : (idx + 1);
      // Predictable anchor ids — the host's TOC builder relies on
      // them so each step gets a deep-linkable entry like #step-3.
      var anchorId = 'step-' + n;
      parts.push('<section class="builder-step mb-4" id="' + anchorId + '" ' +
        'data-step-num="' + n + '" ' +
        'data-step-title="' + esc(s.title || '') + '">');
      parts.push(
        '<header class="builder-step-header d-flex align-items-center mb-2">' +
        '  <span class="builder-step-num">' + n + '</span>' +
        '  <h3 class="h5 mb-0 builder-step-title" id="step-' + n + '-heading">' +
              esc(s.title || ('Step ' + n)) +
        '  </h3>' +
        '</header>'
      );
      if (s.description) {
        parts.push('<p class="builder-step-desc text-muted">' +
          esc(s.description) + '</p>');
      }
      parts.push('<div class="builder-step-body">');
      parts.push(renderStepBody(s, idx));
      parts.push('</div>');
      parts.push('</section>');
    });
    containerEl.innerHTML = parts.join('');
    // Second pass: rasterise canvas + schematic steps into real
    // images. Both kicked off in parallel; placeholders update
    // independently as each one finishes.
    rasteriseCanvasSteps(containerEl, steps).catch(function () {});
    var projectId = instr.project_id || instr.projectId || (instr.project && instr.project.id);
    if (projectId) {
      rasteriseSchematicSteps(containerEl, steps, projectId).catch(function () {});
    }
  }

  // ---------------------------------------------------------------
  // Canvas (photo / blank) rasterisation — lazy-load Fabric.js once,
  // replay each step's canvas_json into an off-screen canvas, then
  // swap the placeholder for an <img>. Aspect ratio is the same
  // 1200×900 the instruction builder authors with.
  // ---------------------------------------------------------------
  var CANVAS_W = 1200, CANVAS_H = 900;
  var _fabricPromise = null;

  function loadFabric() {
    if (window.fabric && window.fabric.StaticCanvas) {
      return Promise.resolve(window.fabric);
    }
    if (_fabricPromise) return _fabricPromise;
    _fabricPromise = new Promise(function (resolve, reject) {
      var s = document.createElement('script');
      s.src = 'https://cdn.jsdelivr.net/npm/fabric@6.4.0/dist/index.min.js';
      s.async = true;
      s.onload = function () {
        if (window.fabric) resolve(window.fabric);
        else reject(new Error('Fabric did not expose window.fabric'));
      };
      s.onerror = function () { reject(new Error('Failed to load Fabric.js')); };
      document.head.appendChild(s);
    });
    return _fabricPromise;
  }

  async function rasteriseCanvasSteps(rootEl, steps) {
    var slots = rootEl.querySelectorAll('.builder-step-canvas-preview');
    if (!slots.length) return;
    var fabric;
    try { fabric = await loadFabric(); }
    catch (_) {
      slots.forEach(function (slot) {
        slot.innerHTML =
          '<div class="alert alert-light border text-center small text-muted">' +
          '<i class="fas fa-image me-1"></i> Canvas preview unavailable.' +
          '</div>';
      });
      return;
    }
    // One reusable off-screen StaticCanvas — no event wiring, no
    // selection, no DOM presence. We re-use the same canvas across
    // steps to avoid allocating N×1200×900 buffers.
    var off = document.createElement('canvas');
    off.width = CANVAS_W; off.height = CANVAS_H;
    var fcanvas = new fabric.StaticCanvas(off, {
      width: CANVAS_W, height: CANVAS_H,
      backgroundColor: '#ffffff',
      renderOnAddRemove: false,
    });
    for (var i = 0; i < slots.length; i++) {
      var slot = slots[i];
      var idx = parseInt(slot.dataset.stepIdx, 10);
      var step = steps[idx];
      if (!step || !step.canvas_json) continue;
      var parsed = null;
      try { parsed = JSON.parse(step.canvas_json); } catch (_) {}
      if (!parsed) {
        slot.innerHTML =
          '<div class="alert alert-light border text-center small text-muted">' +
          '<i class="fas fa-image me-1"></i> Canvas data unreadable.' +
          '</div>';
        continue;
      }
      try {
        fcanvas.clear();
        // Fabric v6: loadFromJSON returns a Promise.
        // eslint-disable-next-line no-await-in-loop
        await Promise.resolve(fcanvas.loadFromJSON(parsed));
        fcanvas.requestRenderAll();
        var dataUrl = fcanvas.toDataURL({
          format: 'png',
          multiplier: 1,
          enableRetinaScaling: false,
        });
        slot.innerHTML =
          '<img class="builder-step-canvas-img" alt="" src="' + dataUrl + '">';
      } catch (e) {
        slot.innerHTML =
          '<div class="alert alert-light border text-center small text-muted">' +
          '<i class="fas fa-image me-1"></i> Canvas preview failed to render.' +
          '</div>';
      }
    }
    try { fcanvas.dispose(); } catch (_) {}
  }

  // ---------------------------------------------------------------
  // Schematic step rasterisation — mount the schematic editor in a
  // hidden iframe with ?embedded=1, request a png-snapshot via the
  // existing postMessage bridge, then drop the PNG into every
  // schematic-type step. All schematic steps in a project share the
  // same underlying schematic (one schematic per project), so a
  // single capture covers them all.
  // ---------------------------------------------------------------
  function rasteriseSchematicSteps(rootEl, steps, projectId) {
    var slots = rootEl.querySelectorAll('.builder-step-schematic-preview');
    if (!slots.length) return Promise.resolve();
    return new Promise(function (resolve) {
      var iframe = document.createElement('iframe');
      iframe.style.cssText =
        'position:fixed; left:-99999px; top:-99999px; ' +
        'width:1200px; height:900px; border:0; visibility:hidden;';
      iframe.src = '/projects/schematic/edit.html?id=' +
        encodeURIComponent(projectId) + '&embedded=1';
      var done = false;
      function finish(dataUrl) {
        if (done) return;
        done = true;
        window.removeEventListener('message', onMsg);
        if (dataUrl) {
          slots.forEach(function (slot) {
            slot.innerHTML =
              '<img class="builder-step-schematic-img" alt="" src="' + dataUrl + '">';
          });
        } else {
          slots.forEach(function (slot) {
            slot.innerHTML =
              '<div class="alert alert-light border d-flex align-items-center small text-muted">' +
              '<i class="fas fa-diagram-project me-2"></i>' +
              'Schematic preview unavailable.' +
              '</div>';
          });
        }
        try { iframe.remove(); } catch (_) {}
        resolve();
      }
      function onMsg(e) {
        var msg = e && e.data;
        if (!msg || !msg.kr_se_event) return;
        if (msg.kr_se_event === 'ready') {
          try {
            iframe.contentWindow.postMessage({
              kr_se_action: 'png-snapshot',
              requestId: 'builder-renderer-' + Date.now(),
              multiplier: 2,
            }, '*');
          } catch (_) { finish(null); }
        } else if (msg.kr_se_event === 'png-snapshot') {
          finish(msg.dataUrl || null);
        }
      }
      window.addEventListener('message', onMsg);
      // Hard fallback: if the editor never replies (no schematic
      // saved, JS error, etc.), give up after 12s and show the
      // unavailable state so the placeholder doesn't spin forever.
      setTimeout(function () { finish(null); }, 12000);
      document.body.appendChild(iframe);
    });
  }

  async function render(containerEl, projectId) {
    if (!containerEl) return;
    containerEl.innerHTML =
      '<div class="text-muted small mb-3">' +
      '<i class="fas fa-spinner fa-spin me-1"></i> Loading instructions&hellip;' +
      '</div>';
    var instr = null;
    try {
      var fetcher = (window.ProjectAuth && window.ProjectAuth.apiFetch)
        ? window.ProjectAuth.apiFetch
        : function (u, o) { return fetch(u, o); };
      var resp = await fetcher(
        apiBase() + '/api/projects/' + projectId + '/instruction',
        { credentials: 'include' });
      if (resp.ok) instr = await resp.json();
    } catch (_) {}
    renderInto(containerEl, instr);
  }

  window.BuilderRenderer = { render: render, renderInto: renderInto };
})();
