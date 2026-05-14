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

  return { apiFetch: apiFetch, getDevToken: getDevToken, clearToken: clearToken };
})();
