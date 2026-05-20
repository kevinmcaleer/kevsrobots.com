/**
 * Instruction Viewer — Phase 2a (issue #178).
 *
 * Public, read-only slideshow consumer for the build-instructions feature.
 * Lives on web/projects/view.html and is initialised once the project
 * record has loaded. Pulls the public GET /api/projects/{pid}/instruction
 * endpoint, pre-renders each step's canvas_json to a PNG dataURL via an
 * off-screen Fabric.js canvas, then renders the slideshow as <img>
 * swapping for fast navigation.
 *
 * Out of scope for Phase 2a: PDF export (2b), image series ZIP (2c),
 * editing affordances, STL viewer, per-step likes/comments.
 *
 * Public surface:
 *   window.InstructionViewer.init(projectId)
 *
 * Behaviour summary:
 *   * 404 from API or empty step list → section stays hidden, silent bail.
 *   * Fabric.js v6 is lazy-loaded from CDN only after we confirm there is
 *     at least one step to render. Projects without instructions don't
 *     pay the ~280KB cost.
 *   * If Fabric fails to load, we degrade to a plain text step list so
 *     the section still surfaces *something* useful.
 *   * Each step is rendered to a PNG once at init time; the visible
 *     slideshow then just swaps <img> sources — flat in step count.
 */
(function () {
  'use strict';

  // -------- Constants -----------------------------------------------------

  var API = 'https://projects.kevsrobots.com';
  // Match the builder's coordinate space exactly so the pre-render frames
  // the scene the way the author saw it.
  var CANVAS_W = 1200;
  var CANVAS_H = 900;
  var FABRIC_CDN = 'https://cdn.jsdelivr.net/npm/fabric@6.4.0/dist/index.min.js';

  // -------- Module state --------------------------------------------------

  var state = {
    projectId: null,
    steps: [],         // array of { step_number, title, description, image (dataURL or null) }
    activeIdx: 0,
    initialised: false,
  };

  // -------- DOM handles ---------------------------------------------------

  var dom = {};

  function bindDom() {
    dom.section = document.getElementById('instruction-slideshow-section');
    dom.frame = document.getElementById('iv-frame');
    dom.stage = document.getElementById('iv-stage');
    dom.spinner = document.getElementById('iv-spinner');
    dom.badge = document.getElementById('iv-step-badge');
    dom.prev = document.getElementById('iv-prev');
    dom.next = document.getElementById('iv-next');
    dom.fullscreen = document.getElementById('iv-fullscreen');
    dom.dots = document.getElementById('iv-dots');
    dom.title = document.getElementById('iv-step-title');
    dom.description = document.getElementById('iv-step-description');
    dom.fallback = document.getElementById('iv-fallback');
  }

  // -------- Public entry --------------------------------------------------

  function init(projectId) {
    if (state.initialised) return;
    state.initialised = true;
    state.projectId = projectId;
    bindDom();
    if (!dom.section) {
      // view.html didn't include the slideshow markup — nothing to do.
      return;
    }
    loadInstruction().catch(function () {
      // Any unhandled rejection ends with the section staying hidden,
      // which is the desired graceful failure mode.
    });
  }

  // -------- Fetch + bootstrap --------------------------------------------

  function loadInstruction() {
    // Public endpoint — no credentials, no auth header.
    return fetch(API + '/api/projects/' + encodeURIComponent(state.projectId) + '/instruction')
      .then(function (resp) {
        if (resp.status === 404) return null;
        if (!resp.ok) {
          // Any other non-2xx → behave like 404 (section stays hidden).
          return null;
        }
        return resp.json();
      })
      .then(function (instruction) {
        if (!instruction || !Array.isArray(instruction.steps) || instruction.steps.length === 0) {
          // No instruction or no steps — section stays hidden.
          return;
        }
        // Normalise + sort defensively by step_number (the API already
        // orders them, but we don't want a server-side bug to scramble
        // the slideshow).
        var sorted = instruction.steps.slice().sort(function (a, b) {
          return (a.step_number || 0) - (b.step_number || 0);
        });
        state.steps = sorted.map(function (s, i) {
          // B3: carry through the step_type discriminator + per-type
          // fields so the slideshow renderer can swap behaviour. Unknown
          // / missing types default to 'photo' (the legacy behaviour).
          var stepType = s.step_type;
          if (['photo', 'schematic', 'text', 'video', 'blank'].indexOf(stepType) < 0) {
            stepType = 'photo';
          }
          return {
            step_number: s.step_number || (i + 1),
            title: s.title || ('Step ' + (s.step_number || (i + 1))),
            description: s.description || '',
            canvas_json: s.canvas_json || null,
            step_type: stepType,
            body: s.body || '',
            video_url: s.video_url || '',
            image: null,
          };
        });
        revealSection();
        return prerenderAll().then(function () {
          // If pre-render dropped all images and we don't have Fabric,
          // prerenderAll's catch path has already shown the fallback.
          // Otherwise, wire up the slideshow.
          if (dom.section.classList.contains('iv-fallback-mode')) return;
          buildDots();
          wireControls();
          showStep(0);
        });
      });
  }

  function revealSection() {
    dom.section.classList.remove('d-none');
  }

  // -------- Pre-render ----------------------------------------------------

  function loadFabric() {
    if (window.fabric && window.fabric.StaticCanvas) {
      return Promise.resolve(window.fabric);
    }
    return new Promise(function (resolve, reject) {
      var existing = document.querySelector('script[data-iv-fabric]');
      if (existing) {
        existing.addEventListener('load', function () {
          window.fabric ? resolve(window.fabric) : reject(new Error('Fabric global missing'));
        });
        existing.addEventListener('error', function () { reject(new Error('Fabric load failed')); });
        return;
      }
      var s = document.createElement('script');
      s.src = FABRIC_CDN;
      s.async = true;
      s.setAttribute('data-iv-fabric', '1');
      s.onload = function () {
        if (window.fabric && window.fabric.StaticCanvas) {
          resolve(window.fabric);
        } else {
          reject(new Error('Fabric global missing after load'));
        }
      };
      s.onerror = function () { reject(new Error('Fabric load failed')); };
      document.head.appendChild(s);
    });
  }

  // Parse a canvas_json blob defensively. Returns the parsed object or
  // null if the string is empty / unparseable / has no objects worth
  // rendering. Treating "empty {}" as null lets text-only steps fall
  // through to the placeholder branch cleanly.
  function parseCanvasJson(raw) {
    if (!raw) return null;
    var trimmed = String(raw).trim();
    if (!trimmed) return null;
    if (trimmed === '{}' || trimmed === '[]' || trimmed === 'null') return null;
    try {
      var parsed = JSON.parse(trimmed);
      if (!parsed) return null;
      // Fabric scenes typically have an `objects` array; if it's empty
      // and there's no backgroundImage either, treat as a blank canvas
      // and use the placeholder instead.
      var objs = Array.isArray(parsed.objects) ? parsed.objects : [];
      if (objs.length === 0 && !parsed.backgroundImage && !parsed.background) {
        return null;
      }
      return parsed;
    } catch (_) {
      return null;
    }
  }

  // B3: photo + blank steps want a canvas pre-render to PNG. The other
  // types (text / video / schematic) render inline via DOM elements at
  // showStep time, so they don't need Fabric. This helper centralises
  // the decision so prerenderAll + renderStep agree on what gets
  // pre-rendered.
  function stepNeedsCanvasPrerender(step) {
    if (step.step_type === 'text' || step.step_type === 'video' || step.step_type === 'schematic') {
      return false;
    }
    // photo + blank — pre-render iff there's actual canvas content. A
    // blank step with no annotations falls through to the placeholder.
    return parseCanvasJson(step.canvas_json) !== null;
  }

  // Pre-render each step to a PNG dataURL. We lazy-load Fabric here
  // because everything before this point can be done without it.
  function prerenderAll() {
    // Are there any steps that actually need Fabric? If every step is
    // text / video / schematic / placeholder-blank we don't need to
    // fetch the library at all.
    var needsFabric = state.steps.some(stepNeedsCanvasPrerender);

    if (!needsFabric) {
      // All steps are placeholder-only — image stays null on each.
      hideSpinner();
      return Promise.resolve();
    }

    return loadFabric()
      .then(function (fabric) {
        // Create one off-screen canvas and reuse it across steps to keep
        // the pre-render cost down. StaticCanvas accepts an HTMLCanvasElement
        // in v6 (it also accepts a string id, but new HTMLCanvasElement is
        // safer because we don't need to attach it to the DOM).
        var offEl = document.createElement('canvas');
        offEl.width = CANVAS_W;
        offEl.height = CANVAS_H;
        var off = new fabric.StaticCanvas(offEl, {
          width: CANVAS_W,
          height: CANVAS_H,
          backgroundColor: '#ffffff',
          enableRetinaScaling: false,
        });

        // Render one step at a time, sequentially. We use a serial chain
        // because Fabric's loadFromJSON mutates the canvas in place and
        // running concurrent renders against the same canvas would race.
        var chain = Promise.resolve();
        state.steps.forEach(function (step, idx) {
          chain = chain.then(function () { return renderStep(off, step); })
            .then(function (dataUrl) {
              state.steps[idx].image = dataUrl;
            })
            .catch(function () {
              // A single step failing to render shouldn't break the rest.
              state.steps[idx].image = null;
            });
        });
        return chain.then(function () {
          try { off.dispose(); } catch (_) {}
          hideSpinner();
        });
      })
      .catch(function () {
        // Fabric refused to load (offline, CDN blocked, CSP, …). Fall
        // back to a plain text step list so the section still shows
        // useful content rather than a broken loader.
        renderFallback();
      });
  }

  function renderStep(off, step) {
    // B3: non-canvas types skip pre-render entirely — they render via
    // DOM elements at showStep time.
    if (!stepNeedsCanvasPrerender(step)) {
      return Promise.resolve(null);
    }
    var parsed = parseCanvasJson(step.canvas_json);
    if (!parsed) {
      // Text-only step — leave image=null; the slideshow renders the
      // placeholder panel in this case.
      return Promise.resolve(null);
    }
    return new Promise(function (resolve) {
      try {
        off.loadFromJSON(parsed, function () {
          try {
            off.renderAll();
            var url = off.toDataURL({ format: 'png', multiplier: 1 });
            resolve(url);
          } catch (e) {
            resolve(null);
          }
          // Clear the canvas so the next step starts from a clean slate.
          try { off.clear(); off.backgroundColor = '#ffffff'; } catch (_) {}
        });
      } catch (_) {
        resolve(null);
      }
    });
  }

  function hideSpinner() {
    if (dom.spinner) dom.spinner.classList.add('d-none');
  }

  // -------- Fallback (Fabric failed to load) -----------------------------

  function renderFallback() {
    if (!dom.fallback) return;
    dom.section.classList.add('iv-fallback-mode');
    // Hide the slideshow shell entirely — the fallback list replaces it.
    if (dom.frame) dom.frame.classList.add('d-none');
    if (dom.dots) dom.dots.classList.add('d-none');
    var stepMeta = document.getElementById('iv-step-meta');
    if (stepMeta) stepMeta.classList.add('d-none');

    var html = '<ol>';
    state.steps.forEach(function (s) {
      html += '<li><strong>' + escapeHtml(s.title || ('Step ' + s.step_number)) + '</strong>';
      if (s.description) {
        html += '<span class="iv-fallback-desc">' + escapeHtml(s.description) + '</span>';
      }
      html += '</li>';
    });
    html += '</ol>';
    dom.fallback.innerHTML = html;
    dom.fallback.classList.remove('d-none');
  }

  function escapeHtml(s) {
    var d = document.createElement('div');
    d.textContent = String(s == null ? '' : s);
    return d.innerHTML;
  }

  // -------- Slideshow render ---------------------------------------------

  // B3: extract the YouTube video id from any of the common URL shapes;
  // returns null for non-YouTube URLs (which render via <video controls>).
  function extractYouTubeId(url) {
    if (!url) return null;
    var m = String(url).match(/(?:youtube\.com\/watch\?v=|youtu\.be\/|youtube\.com\/embed\/)([\w-]{11})/);
    return m ? m[1] : null;
  }

  // Build the stage content for a given step. The slideshow controls
  // (prev / next / dots / keyboard / swipe / fullscreen) are shared
  // across step types — only the stage child element changes.
  function buildStageContent(step) {
    var stepType = step.step_type || 'photo';

    if (stepType === 'text') {
      var card = document.createElement('div');
      card.className = 'iv-text-card';
      var body = (step.body || '').trim();
      if (body) {
        // Plain-text rendering for B3 — preserve line breaks via
        // white-space: pre-wrap CSS rather than re-implementing
        // markdown server-side. textContent + CSS is enough.
        var pre = document.createElement('div');
        pre.className = 'iv-text-body';
        pre.textContent = body;
        card.appendChild(pre);
      } else {
        // No body — render the step title as the only content so the
        // slideshow shows *something* for this step.
        var t = document.createElement('div');
        t.className = 'iv-text-empty';
        t.textContent = step.title || ('Step ' + step.step_number);
        card.appendChild(t);
      }
      return card;
    }

    if (stepType === 'video') {
      var wrap = document.createElement('div');
      wrap.className = 'iv-video-wrap';
      var url = (step.video_url || '').trim();
      var ytId = extractYouTubeId(url);
      if (ytId) {
        var iframe = document.createElement('iframe');
        iframe.className = 'iv-video-iframe';
        iframe.src = 'https://www.youtube.com/embed/' + encodeURIComponent(ytId);
        iframe.title = step.title || ('Step ' + step.step_number);
        iframe.setAttribute('frameborder', '0');
        iframe.setAttribute(
          'allow',
          'accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture'
        );
        iframe.setAttribute('allowfullscreen', '');
        wrap.appendChild(iframe);
      } else if (url) {
        var video = document.createElement('video');
        video.className = 'iv-video-el';
        video.controls = true;
        video.preload = 'metadata';
        video.src = url;
        wrap.appendChild(video);
      } else {
        var empty = document.createElement('div');
        empty.className = 'iv-placeholder';
        empty.textContent = 'No video set for this step.';
        wrap.appendChild(empty);
      }
      return wrap;
    }

    if (stepType === 'schematic') {
      var sch = document.createElement('div');
      sch.className = 'iv-schematic-card';
      sch.innerHTML =
        '<i class="fas fa-diagram-project iv-schematic-icon"></i>' +
        '<div class="iv-schematic-title">Embedded schematic</div>' +
        '<div class="iv-schematic-desc">' +
          'This step embeds the project schematic — visible once the schematic editor ships.' +
        '</div>';
      return sch;
    }

    // photo / blank — pre-rendered to PNG; if pre-render dropped (no
    // canvas content), fall through to the placeholder panel.
    if (step.image) {
      var img = document.createElement('img');
      img.src = step.image;
      img.alt = step.title || ('Step ' + step.step_number);
      img.loading = 'lazy';
      return img;
    }
    var ph = document.createElement('div');
    ph.className = 'iv-placeholder';
    var phTitle = document.createElement('div');
    phTitle.className = 'iv-placeholder-title';
    phTitle.textContent = step.title || ('Step ' + step.step_number);
    ph.appendChild(phTitle);
    return ph;
  }

  function showStep(idx) {
    if (idx < 0 || idx >= state.steps.length) return;
    state.activeIdx = idx;
    var step = state.steps[idx];
    var total = state.steps.length;

    // Wipe whatever's in the stage and rebuild it for this step. The
    // stage is small (one DOM element) so re-creating is cheaper than
    // tracking which mode was previously rendered.
    while (dom.stage.firstChild) dom.stage.removeChild(dom.stage.firstChild);
    dom.stage.appendChild(buildStageContent(step));

    // Badge + meta + dots.
    if (dom.badge) {
      dom.badge.textContent = 'Step ' + (idx + 1) + ' of ' + total;
    }
    if (dom.title) {
      dom.title.textContent = step.title || ('Step ' + step.step_number);
    }
    if (dom.description) {
      var desc = (step.description || '').trim();
      if (desc) {
        dom.description.textContent = desc;
        dom.description.classList.remove('d-none');
      } else {
        dom.description.textContent = '';
        dom.description.classList.add('d-none');
      }
    }
    if (dom.prev) dom.prev.disabled = (idx === 0);
    if (dom.next) dom.next.disabled = (idx === total - 1);
    updateDotsActive();
  }

  function buildDots() {
    if (!dom.dots) return;
    dom.dots.innerHTML = '';
    state.steps.forEach(function (_, i) {
      var b = document.createElement('button');
      b.type = 'button';
      b.className = 'iv-dot';
      b.setAttribute('aria-label', 'Go to step ' + (i + 1));
      b.dataset.idx = String(i);
      b.addEventListener('click', function () { showStep(i); });
      dom.dots.appendChild(b);
    });
  }

  function updateDotsActive() {
    if (!dom.dots) return;
    Array.prototype.forEach.call(dom.dots.children, function (el, i) {
      if (i === state.activeIdx) el.classList.add('is-active');
      else el.classList.remove('is-active');
    });
  }

  // -------- Controls (buttons, keyboard, swipe, fullscreen) --------------

  function wireControls() {
    if (dom.prev) {
      dom.prev.addEventListener('click', function () { showStep(state.activeIdx - 1); });
    }
    if (dom.next) {
      dom.next.addEventListener('click', function () { showStep(state.activeIdx + 1); });
    }
    if (dom.fullscreen) {
      dom.fullscreen.addEventListener('click', toggleFullscreen);
    }

    // Keyboard arrows — only when the section is in view. We attach to
    // window so the user doesn't have to focus the slideshow first; the
    // visibility check prevents stealing arrow keys when the user is
    // scrolled elsewhere on the page (e.g. reading the description).
    window.addEventListener('keydown', function (e) {
      if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return;
      // Don't hijack arrows while a form field has focus.
      var ae = document.activeElement;
      if (ae && (ae.tagName === 'INPUT' || ae.tagName === 'TEXTAREA' || ae.isContentEditable)) return;
      if (!isSectionInView()) return;
      if (e.key === 'ArrowLeft') showStep(state.activeIdx - 1);
      else showStep(state.activeIdx + 1);
    });

    // Touch swipe — track delta on touchend. We require a horizontal
    // displacement of at least 40px and that |dx| > |dy| so vertical
    // page-scroll gestures don't get hijacked.
    var touchStart = null;
    if (dom.frame) {
      dom.frame.addEventListener('touchstart', function (e) {
        if (e.touches.length !== 1) { touchStart = null; return; }
        var t = e.touches[0];
        touchStart = { x: t.clientX, y: t.clientY, time: Date.now() };
      }, { passive: true });
      dom.frame.addEventListener('touchend', function (e) {
        if (!touchStart) return;
        var t = e.changedTouches[0];
        var dx = t.clientX - touchStart.x;
        var dy = t.clientY - touchStart.y;
        touchStart = null;
        if (Math.abs(dx) < 40) return;
        if (Math.abs(dy) > Math.abs(dx)) return;
        if (dx < 0) showStep(state.activeIdx + 1);
        else showStep(state.activeIdx - 1);
      }, { passive: true });
      dom.frame.addEventListener('touchcancel', function () { touchStart = null; });
    }
  }

  function isSectionInView() {
    if (!dom.section) return false;
    var rect = dom.section.getBoundingClientRect();
    var viewH = window.innerHeight || document.documentElement.clientHeight;
    return rect.bottom > 0 && rect.top < viewH;
  }

  function toggleFullscreen() {
    if (!dom.frame) return;
    var inFs = document.fullscreenElement || document.webkitFullscreenElement;
    if (inFs) {
      var exit = document.exitFullscreen || document.webkitExitFullscreen;
      if (exit) {
        try { exit.call(document); } catch (_) {}
      }
    } else {
      var req = dom.frame.requestFullscreen || dom.frame.webkitRequestFullscreen;
      if (req) {
        try { req.call(dom.frame); } catch (_) {}
      }
    }
  }

  // -------- Export --------------------------------------------------------

  window.InstructionViewer = { init: init };
})();
