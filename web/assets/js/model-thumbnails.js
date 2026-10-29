// 3D model thumbnails and viewer for project files (issue #262).
//
// Thumbnails are rendered in the browser, so the Pi cluster never renders
// anything: after an STL uploads, the editor renders it off-screen with
// Three.js (the copy the site's STL viewer already uses) at a fixed
// isometric angle on a transparent background, and posts a 256x256 PNG to
// /api/projects/{id}/files/{file_id}/thumbnail. The admin backfill page
// does the same for models uploaded before this existed.
//
// The project page opens a model in ModelThumbnails.openViewer(): the same
// renderer, drag to turn, wheel or pinch to zoom.
//
// Only STL renders today. OBJ and 3MF show a cube icon until they do.
(function () {
  'use strict';

  const THREE_URL = '/assets/js/three.min.js';
  const SIZE = 256;
  const COLOUR = 0x3d8b5a;

  let threeLoading = null;
  function loadThree() {
    if (window.THREE) return Promise.resolve(window.THREE);
    if (!threeLoading) {
      threeLoading = new Promise((resolve, reject) => {
        const script = document.createElement('script');
        script.src = THREE_URL;
        script.onload = () => resolve(window.THREE);
        script.onerror = () => { threeLoading = null; reject(new Error('Could not load the 3D renderer')); };
        document.head.appendChild(script);
      });
    }
    return threeLoading;
  }

  function ext(name) {
    const i = (name || '').lastIndexOf('.');
    return i === -1 ? '' : name.slice(i + 1).toLowerCase();
  }

  function canRender(name) {
    return ext(name) === 'stl';
  }

  // Binary STL: an 80-byte header, a triangle count, then 50 bytes a
  // triangle. Anything else is read as ASCII "vertex x y z" lines.
  function parseStl(THREE, buffer) {
    const view = new DataView(buffer);
    let positions;
    const count = buffer.byteLength >= 84 ? view.getUint32(80, true) : 0;
    if (buffer.byteLength >= 84 && 84 + count * 50 === buffer.byteLength) {
      positions = new Float32Array(count * 9);
      for (let t = 0; t < count; t++) {
        const base = 84 + t * 50 + 12; // skip the facet normal
        for (let k = 0; k < 9; k++) positions[t * 9 + k] = view.getFloat32(base + k * 4, true);
      }
    } else {
      const text = new TextDecoder().decode(buffer);
      const re = /vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)/g;
      const out = [];
      let m;
      while ((m = re.exec(text))) out.push(+m[1], +m[2], +m[3]);
      positions = new Float32Array(out);
    }
    if (positions.length < 9) throw new Error('No triangles in this STL');
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.computeVertexNormals();
    geometry.computeBoundingBox();
    const centre = new THREE.Vector3();
    geometry.boundingBox.getCenter(centre);
    geometry.translate(-centre.x, -centre.y, -centre.z);
    geometry.computeBoundingSphere();
    return geometry;
  }

  // A lit scene with the model in it, Z-up (as printers and CAD export it)
  // turned to Three's Y-up, and a camera on the isometric diagonal.
  function buildScene(THREE, geometry, aspect) {
    const scene = new THREE.Scene();
    const mesh = new THREE.Mesh(geometry, new THREE.MeshPhongMaterial({
      color: COLOUR, specular: 0x222222, shininess: 40,
    }));
    mesh.rotation.x = -Math.PI / 2;
    const pivot = new THREE.Group();
    pivot.add(mesh);
    scene.add(pivot);
    scene.add(new THREE.AmbientLight(0xffffff, 0.45));
    const key = new THREE.DirectionalLight(0xffffff, 0.75);
    key.position.set(1, 2, 1.5);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.3);
    fill.position.set(-1.5, -0.5, -1);
    scene.add(fill);

    const fov = 30;
    const camera = new THREE.PerspectiveCamera(fov, aspect, 0.1, 1);
    const radius = geometry.boundingSphere.radius || 1;
    const distance = radius / Math.sin((fov / 2) * Math.PI / 180) * 1.05;
    camera.position.set(1, 0.8, 1).normalize().multiplyScalar(distance);
    camera.near = distance / 100;
    camera.far = distance * 10;
    camera.lookAt(0, 0, 0);
    camera.updateProjectionMatrix();
    return { scene, camera, pivot, distance };
  }

  // A PNG Blob of the model in `buffer` (an ArrayBuffer of STL bytes).
  async function renderPng(buffer, size) {
    const THREE = await loadThree();
    const px = size || SIZE;
    const geometry = parseStl(THREE, buffer);
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = px;
    const renderer = new THREE.WebGLRenderer({
      canvas, alpha: true, antialias: true, preserveDrawingBuffer: true,
    });
    try {
      renderer.setClearColor(0x000000, 0);
      renderer.setSize(px, px, false);
      const { scene, camera } = buildScene(THREE, geometry, 1);
      renderer.render(scene, camera);
      return await new Promise((resolve, reject) => {
        canvas.toBlob(b => (b ? resolve(b) : reject(new Error('Could not render the model'))), 'image/png');
      });
    } finally {
      geometry.dispose();
      renderer.dispose();
      // A backfill renders many models; give each context back.
      if (renderer.forceContextLoss) renderer.forceContextLoss();
    }
  }

  // Render `buffer` and post it as file `fileId`'s thumbnail. `request` is
  // the page's fetch (the editor's carries the dev token).
  async function upload(request, url, buffer) {
    const png = await renderPng(buffer);
    const form = new FormData();
    form.append('file', png, 'thumbnail.png');
    const resp = await request(url, { method: 'POST', credentials: 'include', body: form });
    if (!resp.ok) {
      let detail = 'thumbnail upload failed';
      try { detail = (await resp.json()).detail || detail; } catch (e) {}
      throw new Error(detail);
    }
    return resp.json();
  }

  // --- Viewer ---------------------------------------------------------

  let viewer = null;
  function viewerShell() {
    if (viewer) return viewer;
    const el = document.createElement('div');
    el.className = 'model-viewer d-none';
    el.setAttribute('role', 'dialog');
    el.setAttribute('aria-modal', 'true');
    el.innerHTML =
      '<div class="model-viewer__panel">' +
        '<div class="model-viewer__header">' +
          '<span class="model-viewer__title"></span>' +
          '<a class="model-viewer__download btn btn-sm btn-outline-secondary me-2" download><i class="fas fa-download me-1"></i> Download</a>' +
          '<button type="button" class="btn-close" aria-label="Close"></button>' +
        '</div>' +
        '<div class="model-viewer__stage"><div class="model-viewer__status"></div></div>' +
        '<div class="model-viewer__hint">Drag to turn, scroll to zoom</div>' +
      '</div>';
    document.body.appendChild(el);
    const style = document.createElement('style');
    style.textContent =
      '.model-viewer{position:fixed;inset:0;z-index:2000;background:rgba(0,0,0,.6);display:flex;align-items:center;justify-content:center;padding:16px}' +
      '.model-viewer.d-none{display:none}' +
      '.model-viewer__panel{background:var(--bs-body-bg,#fff);border-radius:12px;width:min(900px,100%);max-height:100%;display:flex;flex-direction:column;overflow:hidden;box-shadow:0 10px 40px rgba(0,0,0,.3)}' +
      '.model-viewer__header{display:flex;align-items:center;padding:10px 14px;border-bottom:1px solid rgba(0,0,0,.1)}' +
      '.model-viewer__title{flex:1;font-weight:600;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}' +
      '.model-viewer__stage{position:relative;height:min(70vh,600px);background:radial-gradient(circle,#f8f9fa,#dee2e6);touch-action:none;cursor:grab;user-select:none}' +
      '.model-viewer__stage canvas{display:block;width:100%;height:100%}' +
      '.model-viewer__status{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:#6c757d}' +
      '.model-viewer__hint{font-size:.8rem;color:#6c757d;text-align:center;padding:6px}';
    document.head.appendChild(style);
    viewer = {
      el,
      title: el.querySelector('.model-viewer__title'),
      download: el.querySelector('.model-viewer__download'),
      stage: el.querySelector('.model-viewer__stage'),
      status: el.querySelector('.model-viewer__status'),
      close: null,
    };
    el.querySelector('.btn-close').addEventListener('click', () => viewer.close && viewer.close());
    el.addEventListener('click', e => { if (e.target === el && viewer.close) viewer.close(); });
    document.addEventListener('keydown', e => {
      if (e.key === 'Escape' && viewer.close) viewer.close();
    });
    return viewer;
  }

  // Show the STL at `url` (fetched with credentials, so a draft's author
  // sees theirs). `downloadUrl` is the counted download link.
  async function openViewer({ url, name, downloadUrl }) {
    const v = viewerShell();
    if (v.close) v.close();
    v.title.textContent = name || '';
    v.download.href = downloadUrl || url;
    v.status.textContent = 'Loading model…';
    v.status.style.display = '';
    v.el.classList.remove('d-none');

    let renderer = null;
    let geometry = null;
    let closed = false;
    const cleanups = [];
    v.close = () => {
      closed = true;
      v.el.classList.add('d-none');
      cleanups.forEach(fn => fn());
      if (geometry) geometry.dispose();
      if (renderer) {
        renderer.dispose();
        if (renderer.forceContextLoss) renderer.forceContextLoss();
        renderer.domElement.remove();
      }
      v.close = null;
    };

    try {
      const THREE = await loadThree();
      const resp = await fetch(url, { credentials: 'include' });
      if (!resp.ok) throw new Error('Could not load the model (' + resp.status + ')');
      const buffer = await resp.arrayBuffer();
      if (closed) return;
      geometry = parseStl(THREE, buffer);
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
      renderer.setPixelRatio(window.devicePixelRatio || 1);
      v.stage.appendChild(renderer.domElement);
      const rect = v.stage.getBoundingClientRect();
      renderer.setSize(rect.width, rect.height, false);
      const { scene, camera, pivot, distance } = buildScene(THREE, geometry, rect.width / rect.height);
      v.status.style.display = 'none';

      let zoom = 1;
      const draw = () => {
        camera.position.normalize().multiplyScalar(distance * zoom);
        renderer.render(scene, camera);
      };
      draw();

      // Drag turns the model; wheel and pinch zoom.
      const pointers = new Map();
      let pinch = 0;
      const onDown = e => {
        v.stage.setPointerCapture(e.pointerId);
        pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      };
      const onMove = e => {
        const last = pointers.get(e.pointerId);
        if (!last) return;
        if (pointers.size === 2) {
          pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
          const [a, b] = Array.from(pointers.values());
          const d = Math.hypot(a.x - b.x, a.y - b.y);
          if (pinch) zoom = Math.min(4, Math.max(0.25, zoom * pinch / d));
          pinch = d;
        } else {
          pivot.rotation.y += (e.clientX - last.x) * 0.01;
          pivot.rotation.x = Math.max(-1.5, Math.min(1.5, pivot.rotation.x + (e.clientY - last.y) * 0.01));
          pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
        }
        draw();
      };
      const onUp = e => { pointers.delete(e.pointerId); pinch = 0; };
      const onWheel = e => {
        e.preventDefault();
        zoom = Math.min(4, Math.max(0.25, zoom * (e.deltaY > 0 ? 1.1 : 0.9)));
        draw();
      };
      const onResize = () => {
        const r = v.stage.getBoundingClientRect();
        renderer.setSize(r.width, r.height, false);
        camera.aspect = r.width / r.height;
        camera.updateProjectionMatrix();
        draw();
      };
      v.stage.addEventListener('pointerdown', onDown);
      v.stage.addEventListener('pointermove', onMove);
      v.stage.addEventListener('pointerup', onUp);
      v.stage.addEventListener('pointercancel', onUp);
      v.stage.addEventListener('wheel', onWheel, { passive: false });
      window.addEventListener('resize', onResize);
      cleanups.push(() => {
        v.stage.removeEventListener('pointerdown', onDown);
        v.stage.removeEventListener('pointermove', onMove);
        v.stage.removeEventListener('pointerup', onUp);
        v.stage.removeEventListener('pointercancel', onUp);
        v.stage.removeEventListener('wheel', onWheel);
        window.removeEventListener('resize', onResize);
      });
    } catch (e) {
      if (!closed) v.status.textContent = e.message || 'Could not show this model';
    }
  }

  window.ModelThumbnails = { canRender, renderPng, upload, openViewer, parseStl, loadThree };
})();
