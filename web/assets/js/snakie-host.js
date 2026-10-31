// Snakie's parts tools on kevsrobots (issues #268, #269).
//
// The Part Editor, the wiring canvas and the read-only part view come from
// Snakie's embeddable bundle (Snakie#1277), vendored in
// /assets/js/snakie-parts/ (see its VERSION). It's 1.4 MB, so pages load it
// only when someone opens one of them: SnakieHost.load().
//
// Parts reach the bundle as Snakie PartDefinitions, built from the API's
// own parts.yml export, so the catalog and Snakie read a part the same way.
(function () {
  const API = 'https://projects.kevsrobots.com';
  const BASE = '/assets/js/snakie-parts/';
  const FONTS = 'https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600&family=Plus+Jakarta+Sans:wght@400;600;700&display=swap';

  let loading = null;

  function addCss(href) {
    if (document.querySelector('link[href="' + href + '"]')) return;
    const link = document.createElement('link');
    link.rel = 'stylesheet';
    link.href = href;
    document.head.appendChild(link);
  }

  // The bundle, once: resolves to window.SnakieParts.
  function load() {
    if (window.SnakieParts) return Promise.resolve(window.SnakieParts);
    if (!loading) {
      addCss(FONTS);
      addCss(BASE + 'snakie-parts.css');
      loading = new Promise((resolve, reject) => {
        const script = document.createElement('script');
        script.src = BASE + 'snakie-parts.js';
        script.onload = () => resolve(window.SnakieParts);
        script.onerror = () => { loading = null; reject(new Error('Could not load the part editor')); };
        document.head.appendChild(script);
      });
    }
    return loading;
  }

  function photoUrl(photo) {
    return photo.is_external ? photo.url : API + photo.url;
  }

  // An image as a data: URL, so the editor can read its pixels (background
  // removal needs that, and a cross-origin URL won't allow it). Falls back
  // to the URL itself when the image can't be fetched.
  async function toDataUrl(url) {
    try {
      const resp = await fetch(url);
      if (!resp.ok) return url;
      const blob = await resp.blob();
      return await new Promise((resolve) => {
        const reader = new FileReader();
        reader.onload = () => resolve(reader.result);
        reader.onerror = () => resolve(url);
        reader.readAsDataURL(blob);
      });
    } catch (_) {
      return url;
    }
  }

  // The photo a part's pins were placed over: the definition's photo_id,
  // else the picture imported from Snakie's library (a definition from
  // Snakie places its pins over that cut-out, not over the catalog's own
  // product shots), else the cover (first) photo.
  function isSnakiePhoto(photo) {
    return / \(Snakie\)$/.test(photo.source || '');
  }

  function pinPhoto(part) {
    const photos = part.photos || [];
    const id = part.definition && part.definition.photo_id;
    return photos.find(p => p.id === id)
      || photos.filter(isSnakiePhoto).sort((a, b) => a.id - b.id)[0]
      || photos[0] || null;
  }

  // A catalog part (GET /api/parts/{slug}) as a Snakie PartDefinition, with
  // `photo` (a PartPhoto, default pinPhoto) as its image. `inline` reads
  // the photo into a data: URL.
  async function partFor(part, opts) {
    opts = opts || {};
    const SP = await load();
    const resp = await fetch(API + '/api/parts/' + encodeURIComponent(part.slug) + '/parts.yml');
    if (!resp.ok) throw new Error('Could not read the part (HTTP ' + resp.status + ')');
    const def = SP.partFromYaml(await resp.text());
    const photo = opts.photo === undefined ? pinPhoto(part) : opts.photo;
    delete def.image;
    delete def.imageData;
    if (photo) {
      const url = photoUrl(photo);
      def.imageData = opts.inline ? await toDataUrl(url) : url;
    }
    return def;
  }

  window.SnakieHost = { API, load, photoUrl, toDataUrl, pinPhoto, partFor };
})();
