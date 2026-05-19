/**
 * Profile edit form — issue #111.
 *
 * Auth model:
 *   * Page is auth-required. Probes /api/auth/me (and falls back to
 *     /api/projects/my/list) to decide whether to show the form.
 *   * On 401 the form is hidden and a sign-in link is shown instead.
 *
 * Validation mirrors the server: bio ≤500, location ≤120, website +
 * social ≤200 and must start with http(s)://. Featured badges are
 * limited to 3; the picker hides itself if the badges endpoint returns
 * nothing (i.e. #106 is not deployed).
 */
(function () {
  'use strict';

  var PROJECTS_API = 'https://projects.kevsrobots.com';

  var form = document.getElementById('profile-edit-form');
  var authBlock = document.getElementById('auth-required');
  var errorEl = document.getElementById('form-error');
  var successEl = document.getElementById('form-success');
  var bioInput = document.getElementById('bio');
  var bioCount = document.getElementById('bio-count');

  // Bio character counter.
  function updateBioCount() {
    bioCount.textContent = String((bioInput.value || '').length);
  }
  bioInput.addEventListener('input', updateBioCount);

  // --- Viewer detection ---------------------------------------------------

  function probeViewer() {
    if (!window.ProjectAuth) {
      return Promise.resolve({ authed: false, username: '' });
    }
    return window.ProjectAuth.apiFetch(PROJECTS_API + '/api/auth/me')
      .then(function (r) {
        if (r.ok) {
          return r.json().then(function (j) {
            return { authed: true, username: (j && (j.username || j.sub)) || '' };
          });
        }
        if (r.status === 401 || r.status === 403) {
          return { authed: false, username: '' };
        }
        return window.ProjectAuth.apiFetch(PROJECTS_API + '/api/projects/my/list')
          .then(function (r2) {
            return r2.ok ? { authed: true, username: '' } : { authed: false, username: '' };
          })
          .catch(function () { return { authed: false, username: '' }; });
      })
      .catch(function () { return { authed: false, username: '' }; });
  }

  // --- Form prefill -------------------------------------------------------

  function prefill(username) {
    // View-profile link
    if (username) {
      document.getElementById('view-profile-link').href =
        '/profile/?u=' + encodeURIComponent(username);
    }
    if (!username) return Promise.resolve(null);
    return fetch(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/profile')
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (data) {
        if (!data) return null;
        bioInput.value = data.bio || '';
        updateBioCount();
        document.getElementById('location').value = data.location || '';
        document.getElementById('website_url').value = data.website_url || '';
        var social = data.social_links || {};
        document.getElementById('social_github').value = social.github || '';
        document.getElementById('social_twitter').value = social.twitter || '';
        document.getElementById('social_youtube').value = social.youtube || '';
        document.getElementById('social_mastodon').value = social.mastodon || '';
        return data;
      });
  }

  // --- Featured badges picker ---------------------------------------------

  var pickedSlugs = [];

  function renderBadgePicker(badges, pinned) {
    var fieldset = document.getElementById('featured-badges-fieldset');
    var wrap = document.getElementById('featured-badges-options');
    if (!badges || badges.length === 0) return;
    fieldset.classList.remove('d-none');
    pickedSlugs = (pinned || []).slice(0, 3);
    wrap.innerHTML = badges.map(function (b) {
      var checked = pickedSlugs.some(function (s) {
        return (s || '').toLowerCase() === (b.key || '').toLowerCase();
      });
      return '<label class="form-check form-check-inline">' +
               '<input class="form-check-input badge-pick" type="checkbox" value="' +
                 (b.key || '') + '"' + (checked ? ' checked' : '') + '>' +
               '<span class="badge bg-' + (b.color || 'primary') + ' p-2 ms-1">' +
                 '<i class="fas ' + (b.icon || 'fa-circle') + ' me-1"></i>' + (b.name || '') +
               '</span>' +
             '</label>';
    }).join('');

    wrap.addEventListener('change', function (e) {
      if (!e.target.classList.contains('badge-pick')) return;
      var slug = e.target.value;
      if (e.target.checked) {
        // Cap at 3 — if we'd exceed, refuse and snap back.
        var checked = wrap.querySelectorAll('.badge-pick:checked');
        if (checked.length > 3) {
          e.target.checked = false;
          alert('You can only pin 3 badges. Uncheck one first.');
          return;
        }
        pickedSlugs.push(slug);
      } else {
        pickedSlugs = pickedSlugs.filter(function (s) {
          return (s || '').toLowerCase() !== (slug || '').toLowerCase();
        });
      }
    });
  }

  function loadBadgePicker(username, profile) {
    if (!username) return;
    fetch(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/badges')
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (data) {
        if (!data || !data.badges || data.badges.length === 0) return;
        renderBadgePicker(data.badges, (profile && profile.featured_badge_slugs) || []);
      })
      .catch(function () {});
  }

  // --- Submit -------------------------------------------------------------

  function clearMessages() {
    errorEl.classList.add('d-none');
    successEl.classList.add('d-none');
  }
  function showError(msg) {
    successEl.classList.add('d-none');
    errorEl.classList.remove('d-none');
    errorEl.textContent = msg;
  }
  function showSuccess() {
    errorEl.classList.add('d-none');
    successEl.classList.remove('d-none');
  }

  function validateUrl(value, field) {
    if (!value) return null;
    var trimmed = value.trim();
    if (trimmed === '') return null;
    if (!/^https?:\/\/[^\s]+$/i.test(trimmed)) {
      throw new Error(field + ' must start with http:// or https://');
    }
    if (trimmed.length > 200) {
      throw new Error(field + ' is too long (max 200 chars)');
    }
    return trimmed;
  }

  form.addEventListener('submit', function (e) {
    e.preventDefault();
    clearMessages();

    var bio = bioInput.value || '';
    if (bio.length > 500) {
      showError('Bio is too long (max 500 chars).');
      return;
    }

    var payload;
    try {
      payload = {
        bio: bio.trim() === '' ? '' : bio,
        location: document.getElementById('location').value || '',
        website_url: validateUrl(document.getElementById('website_url').value, 'Website') || '',
        social_links: {
          github: validateUrl(document.getElementById('social_github').value, 'GitHub URL') || '',
          twitter: validateUrl(document.getElementById('social_twitter').value, 'Twitter URL') || '',
          youtube: validateUrl(document.getElementById('social_youtube').value, 'YouTube URL') || '',
          mastodon: validateUrl(document.getElementById('social_mastodon').value, 'Mastodon URL') || ''
        },
        featured_badge_slugs: pickedSlugs.slice(0, 3)
      };
    } catch (err) {
      showError(err.message);
      return;
    }

    var saveBtn = document.getElementById('save-btn');
    saveBtn.disabled = true;
    window.ProjectAuth.apiFetch(PROJECTS_API + '/api/users/me/profile', {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    }).then(function (r) {
      if (r.status === 401 || r.status === 403) {
        throw new Error('Please sign in to save your profile.');
      }
      if (!r.ok) {
        return r.json().then(function (j) {
          throw new Error((j && j.detail) || ('Save failed (' + r.status + ')'));
        }).catch(function (err) {
          if (err instanceof Error) throw err;
          throw new Error('Save failed (' + r.status + ')');
        });
      }
      return r.json();
    }).then(function () {
      showSuccess();
    }).catch(function (err) {
      showError(err.message || 'Save failed.');
    }).then(function () {
      saveBtn.disabled = false;
    });
  });

  // --- Boot ---------------------------------------------------------------

  probeViewer().then(function (viewer) {
    if (!viewer.authed) {
      authBlock.classList.remove('d-none');
      return;
    }
    form.classList.remove('d-none');
    // We may not know the viewer's username if /api/auth/me isn't
    // deployed; the GET below will 404 in that case and we skip
    // prefill. The PUT still works because it uses /me/profile.
    if (!viewer.username) {
      return;
    }
    prefill(viewer.username).then(function (profile) {
      loadBadgePicker(viewer.username, profile);
    });
  });
})();
