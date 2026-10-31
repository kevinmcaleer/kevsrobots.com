// "Download files": a project zipped in the browser, used by the project
// page and the editor's Files & Models section. The zip holds the
// write-up as README.md, the models in models/, the other files in
// files/, every uploaded picture (the gallery's included) in images/, the
// stored BOM as a CSV and, where the page can make one, the write-up as a
// PDF. (The old builder instructions.pdf is gone: it drew the retired
// step-by-step instructions, not the write-up.)
//
// ProjectDownload.zip(project, button, { contentMd, pdf }) — contentMd is
// the editor's current text, which may be newer than the saved copy; pdf
// is an async function giving the write-up as a PDF Blob.
(function () {
  'use strict';

  const API = 'https://projects.kevsrobots.com';
  const JSZIP_URL = 'https://cdn.jsdelivr.net/npm/jszip@3/dist/jszip.min.js';

  function request(url) {
    return window.ProjectAuth ? ProjectAuth.apiFetch(url, {}) : fetch(url, { credentials: 'include' });
  }

  let jszipLoading = null;
  function loadJSZip() {
    if (window.JSZip) return Promise.resolve(window.JSZip);
    if (!jszipLoading) {
      jszipLoading = new Promise((resolve, reject) => {
        const script = document.createElement('script');
        script.src = JSZIP_URL;
        script.onload = () => resolve(window.JSZip);
        script.onerror = () => { jszipLoading = null; reject(new Error('Could not load the zip library')); };
        document.head.appendChild(script);
      });
    }
    return jszipLoading;
  }

  function projectSlug(project) {
    return (project.title || '').toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'project';
  }

  // Issue #152: the canonical project URL, so downloads link back to the
  // SEO-friendly path. Falls back to ?id= for legacy projects.
  function publicProjectUrl(project) {
    return (project.author_username && project.slug)
      ? 'https://www.kevsrobots.com/projects/' + encodeURIComponent(project.author_username) + '/' + encodeURIComponent(project.slug)
      : 'https://www.kevsrobots.com/projects/view.html?id=' + project.id;
  }

  async function getJson(path) {
    const resp = await request(API + '/api/projects/' + path);
    return resp.ok ? resp.json() : [];
  }

  async function addBlob(zip, name, url) {
    try {
      const resp = await request(url);
      if (resp.ok) zip.file(name, await resp.blob());
    } catch (e) { /* a file that won't fetch is left out */ }
  }

  async function zipProject(project, btn, options) {
    const opts = options || {};
    const label = btn.innerHTML;
    btn.disabled = true;
    btn.innerHTML = '<i class="fas fa-spinner fa-spin me-1"></i> Packaging…';

    try {
      const JSZip = await loadJSZip();
      const zip = new JSZip();
      const content = opts.contentMd != null ? opts.contentMd : project.content_md;

      let readme = '# ' + project.title + '\n\n';
      if (project.short_description) readme += '*' + project.short_description + '*\n\n';
      if (project.difficulty) readme += '**Difficulty:** ' + project.difficulty + '\n';
      if (project.estimated_minutes) readme += '**Estimated time:** ' + project.estimated_minutes + ' minutes\n';
      if (project.code_repo_url) readme += '**Code:** ' + project.code_repo_url + '\n';
      readme += '\n---\n\n';
      if (content) readme += (window.PartLinks ? PartLinks.absolutize(content) : content) + '\n';
      readme += '\n\n*Downloaded from [KevsRobots](' + publicProjectUrl(project) + ')*\n';
      zip.file('README.md', readme);

      const base = API + '/api/projects/' + project.id;
      // The pictures, the gallery's among them.
      for (const img of await getJson(project.id + '/images')) {
        await addBlob(zip, 'images/' + img.filename, base + '/images/' + img.id + '/view');
      }
      // Models and the other files.
      for (const f of await getJson(project.id + '/files')) {
        const folder = f.file_type === 'model' ? 'models/' : 'files/';
        await addBlob(zip, folder + f.filename, base + '/files/' + f.id + '/download');
      }

      // BOM as CSV (issue #149: with the currency).
      const bom = await getJson(project.id + '/bom');
      if (bom.length > 0) {
        let csv = 'Part,Quantity,Unit,Unit Cost,Currency,Supplier URL\n';
        bom.forEach(item => {
          csv += '"' + (item.name || '').replace(/"/g, '""') + '",' + item.quantity + ',' + item.unit + ',' + (item.unit_cost || 0) + ',"' + (item.currency_code || '') + '","' + (item.supplier_url || '') + '"\n';
        });
        zip.file('bill-of-materials.csv', csv);
      }

      // The write-up as a PDF, made the way Download PDF lays it out (the
      // project page passes it in). A failure skips the PDF, not the zip.
      if (opts.pdf) {
        try {
          btn.innerHTML = '<i class="fas fa-spinner fa-spin me-1"></i> Making the PDF…';
          const pdf = await opts.pdf();
          if (pdf) zip.file(projectSlug(project) + '.pdf', pdf);
        } catch (e) {
          console.warn('[ZIP] PDF skipped:', e);
        }
        btn.innerHTML = '<i class="fas fa-spinner fa-spin me-1"></i> Packaging…';
      }

      const blob = await zip.generateAsync({ type: 'blob' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = projectSlug(project) + '.zip';
      document.body.appendChild(a);
      a.click();
      a.remove();
      setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    } catch (e) {
      console.error('Download failed:', e);
      alert('Failed to package project. Please try again.');
    }

    btn.disabled = false;
    btn.innerHTML = label;
  }

  window.ProjectDownload = { zip: zipProject, projectSlug, publicProjectUrl };
})();
