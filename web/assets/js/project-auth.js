/**
 * Shared auth helper for Projects Hub pages.
 * Handles dev token from localStorage for local development
 * where secure cookies don't work over HTTP.
 */
var ProjectAuth = (function() {
  function getDevToken() {
    return localStorage.getItem('dev_jwt_token');
  }

  function apiFetch(url, opts) {
    opts = opts || {};
    opts.credentials = 'include';
    var token = getDevToken();
    if (token) {
      opts.headers = opts.headers || {};
      opts.headers['Authorization'] = 'Bearer ' + token;
    }
    return fetch(url, opts);
  }

  function clearToken() {
    localStorage.removeItem('dev_jwt_token');
  }

  // A draft's images and thumbnails are its author's alone (#260). On
  // the site the login cookie goes with every <img> request, but with a
  // dev token there's no cookie and an <img> can't send the header, so
  // the API sees a stranger and the picture breaks. With a dev token,
  // fetch those pictures with it and show them from a blob URL instead.
  // That covers uploaded images and file thumbnails, and the drawings the
  // API renders: the wiring diagram (#268) and diagram exports (#270).
  var PRIVATE_IMAGE_RE = new RegExp('^https://projects\\.kevsrobots\\.com/api/(?:' + [
    '(?:projects|makes)/\\d+/(?:images|files)/\\d+/(?:view|thumbnail)\\b',
    'projects/\\d+/wiring\\.svg(?:[?#]|$)',
    'projects/\\d+/diagrams/\\d+\\.(?:svg|png)(?:[?#]|$)',
  ].join('|') + ')');
  var blobUrls = {}; // api url -> Promise of a blob: url

  function isPrivateImageUrl(url) {
    return PRIVATE_IMAGE_RE.test(url || '');
  }

  function blobUrlFor(url) {
    if (!blobUrls[url]) {
      blobUrls[url] = apiFetch(url).then(function(resp) {
        if (!resp.ok) throw new Error(resp.status);
        return resp.blob();
      }).then(function(blob) {
        return URL.createObjectURL(blob);
      }).catch(function() {
        delete blobUrls[url]; // try again next time it's drawn
        return null;
      });
    }
    return blobUrls[url];
  }

  function showWithToken(img) {
    var url = img.getAttribute('src');
    if (!isPrivateImageUrl(url)) return;
    blobUrlFor(url).then(function(blobUrl) {
      // Skip it if the picture has moved on to another image meanwhile.
      if (blobUrl && img.getAttribute('src') === url) img.src = blobUrl;
    });
  }

  function watchImages(root) {
    root.querySelectorAll('img[src]').forEach(showWithToken);
    new MutationObserver(function(mutations) {
      mutations.forEach(function(m) {
        if (m.type === 'attributes') {
          if (m.target.tagName === 'IMG') showWithToken(m.target);
          return;
        }
        m.addedNodes.forEach(function(node) {
          if (node.nodeType !== 1) return;
          if (node.tagName === 'IMG') showWithToken(node);
          else node.querySelectorAll('img[src]').forEach(showWithToken);
        });
      });
    }).observe(root, { childList: true, subtree: true, attributes: true, attributeFilter: ['src'] });
  }

  if (typeof document !== 'undefined' && getDevToken()) {
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', function() { watchImages(document.body); });
    } else {
      watchImages(document.body);
    }
  }

  return {
    apiFetch: apiFetch, getDevToken: getDevToken, clearToken: clearToken,
    isPrivateImageUrl: isPrivateImageUrl, watchImages: watchImages,
  };
})();

if (typeof module !== 'undefined') module.exports = ProjectAuth;
