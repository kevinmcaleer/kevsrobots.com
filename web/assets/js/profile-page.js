/**
 * Public user profile page — issue #111.
 *
 * Replaces the original #140-era page with the full profile system:
 *   1. Header: avatar + name + bio + location + website + social links
 *   2. Stats bar: projects, makes, downloads, likes, followers, following
 *   3. Featured badges row (up to 3 — pinned by user)
 *   4. Tabs: Projects | Makes | Activity
 *   5. Follower / following modal lists
 *   6. Comments (from Chatter)
 *
 * The page is read-only public; auth-requiring actions (Follow button)
 * appear only when the viewer is logged in and not viewing their own
 * profile.
 *
 * Defensive design: any API endpoint can be missing/404 — the page
 * gracefully hides that section. This is necessary because #106
 * (badges system) is being built in parallel; if it isn't deployed
 * yet, the badges card hides itself.
 */
(function () {
  'use strict';

  var PROJECTS_API = 'https://projects.kevsrobots.com';
  var CHATTER_API = 'https://chatter.kevsrobots.com';

  // --- Username extraction ------------------------------------------------

  function getUsernameFromUrl() {
    var urlParams = new URLSearchParams(window.location.search);
    var username = urlParams.get('u') || urlParams.get('username');
    if (username) return username;
    // /profile/<username>/ form — the /profile/index.html redirector
    // forwards to ?username= form, but if a static deploy ever serves
    // the bare path we still want to render.
    var parts = window.location.pathname.split('/').filter(Boolean);
    if (parts.length > 1 && (parts[0] === 'profile' || parts[0] === 'profile.html')) {
      try { return decodeURIComponent(parts[1]); } catch (e) { return parts[1]; }
    }
    return null;
  }

  var username = getUsernameFromUrl();
  if (!username) {
    document.getElementById('profile-root').innerHTML =
      '<div class="alert alert-danger">No username provided.</div>';
    return;
  }

  // --- Helpers ------------------------------------------------------------

  function esc(s) {
    var d = document.createElement('div');
    d.textContent = s == null ? '' : String(s);
    return d.innerHTML;
  }

  function profileLink(u) {
    return '/profile/?u=' + encodeURIComponent(u);
  }

  // Probe who the viewer is. Returns { authed, username }.
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
        // Fall back to my-projects which the older deploy supports.
        return window.ProjectAuth.apiFetch(PROJECTS_API + '/api/projects/my/list')
          .then(function (r2) {
            return r2.ok ? { authed: true, username: '' } : { authed: false, username: '' };
          })
          .catch(function () { return { authed: false, username: '' }; });
      })
      .catch(function () {
        return window.ProjectAuth.apiFetch(PROJECTS_API + '/api/projects/my/list')
          .then(function (r2) {
            return r2.ok ? { authed: true, username: '' } : { authed: false, username: '' };
          })
          .catch(function () { return { authed: false, username: '' }; });
      });
  }

  // --- Chatter header (avatar + display name + location) ------------------
  //
  // Chatter is the source of truth for avatar / display name; the
  // /api/users/{username}/profile endpoint owns bio, social links etc.
  // If Chatter returns 404 (e.g. local dev with no Chatter mock) we
  // fall back to a generated initial avatar.

  function loadChatterProfile() {
    return fetch(CHATTER_API + '/profile/' + encodeURIComponent(username), {
      credentials: 'include'
    })
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (profile) {
        var avatarDiv = document.getElementById('profile-avatar');
        var initial = (username && username[0]) || '?';
        var fallbackHtml =
          '<div class="rounded-circle bg-secondary d-inline-flex align-items-center ' +
          'justify-content-center text-white" ' +
          'style="width:150px;height:150px;font-size:3rem;">' +
          esc(initial.toUpperCase()) + '</div>';
        if (profile && profile.profile_picture_url) {
          var pictureUrl = profile.profile_picture_url.indexOf('http') === 0
            ? profile.profile_picture_url
            : CHATTER_API + profile.profile_picture_url;
          // Chatter rotates the picture URL hash per profile fetch; if the
          // image 404s (stale URL, cache mismatch, missing file) fall back
          // to the initial-letter avatar instead of a broken-image icon.
          avatarDiv.innerHTML =
            '<img src="' + esc(pictureUrl) + '" alt="' + esc(username) + '" ' +
            'class="rounded-circle img-fluid" ' +
            'style="max-width:150px;max-height:150px;object-fit:cover;" ' +
            'onerror="this.parentNode.innerHTML=' +
            JSON.stringify(fallbackHtml).replace(/"/g, '&quot;') + ';">';
        } else {
          avatarDiv.innerHTML = fallbackHtml;
        }

        var name = profile
          ? [profile.firstname, profile.lastname].filter(Boolean).join(' ').trim()
          : '';
        document.getElementById('profile-name').textContent = name || username;
        document.getElementById('profile-username').textContent = '@' + username;
        document.title = (name || username) + "'s Profile - Kev's Robots";

        return profile || { username: username, is_own_profile: false };
      });
  }

  // --- Profile body (bio, website, socials, joined_at, featured badges) ---

  var socialIcons = {
    github: 'fa-brands fa-github',
    twitter: 'fa-brands fa-x-twitter',
    youtube: 'fa-brands fa-youtube',
    mastodon: 'fa-brands fa-mastodon'
  };

  function renderSocialLinks(links) {
    var wrap = document.getElementById('profile-socials');
    if (!wrap) return;
    var html = '';
    Object.keys(socialIcons).forEach(function (key) {
      var url = links && links[key];
      if (!url) return;
      html += '<a class="btn btn-sm btn-outline-secondary" href="' + esc(url) +
              '" target="_blank" rel="noopener noreferrer nofollow" title="' + esc(key) + '">' +
              '<i class="' + socialIcons[key] + '"></i></a>';
    });
    wrap.innerHTML = html;
  }

  function loadProfile() {
    return fetch(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/profile')
      .then(function (r) {
        if (r.status === 404) return null;
        return r.ok ? r.json() : null;
      })
      .then(function (profile) {
        if (!profile) return null;
        if (profile.bio) {
          var bioEl = document.getElementById('profile-bio');
          bioEl.style.display = 'block';
          bioEl.textContent = profile.bio;
        }
        if (profile.location) {
          document.getElementById('profile-location').style.display = 'block';
          document.getElementById('location-text').textContent = profile.location;
        }
        if (profile.website_url) {
          var wrap = document.getElementById('profile-website');
          var link = document.getElementById('website-link');
          wrap.style.display = 'block';
          link.href = profile.website_url;
          link.textContent = profile.website_url.replace(/^https?:\/\//, '');
        }
        renderSocialLinks(profile.social_links || {});
        if (profile.joined_at) {
          var d = new Date(profile.joined_at);
          document.getElementById('profile-joined').textContent =
            'Joined ' + d.toLocaleDateString(undefined, { month: 'long', year: 'numeric' });
        }
        // Stats
        var stats = profile.stats || {};
        document.getElementById('stat-projects').textContent = stats.projects || 0;
        document.getElementById('stat-makes').textContent = stats.makes || 0;
        document.getElementById('stat-downloads').textContent = stats.downloads || 0;
        document.getElementById('stat-likes').textContent = stats.likes || 0;
        document.getElementById('stat-followers').textContent = stats.followers || 0;
        document.getElementById('stat-following').textContent = stats.following || 0;
        return profile;
      })
      .catch(function () { return null; });
  }

  // --- Follow button ------------------------------------------------------

  function setupFollowButton(viewer, profile) {
    var btn = document.getElementById('follow-btn');
    if (!btn) return;
    if (!viewer.authed) return;
    if (profile && profile.is_own_profile) return;
    if (viewer.username && viewer.username === username) return;

    btn.classList.remove('d-none');

    function render(following) {
      var label = btn.querySelector('[data-follow-label]');
      var icon = btn.querySelector('i');
      if (following) {
        btn.classList.remove('btn-outline-primary');
        btn.classList.add('btn-primary');
        if (label) label.textContent = 'Following';
        if (icon) icon.className = 'fas fa-user-check me-1';
        btn.setAttribute('data-following', '1');
      } else {
        btn.classList.add('btn-outline-primary');
        btn.classList.remove('btn-primary');
        if (label) label.textContent = 'Follow';
        if (icon) icon.className = 'fas fa-user-plus me-1';
        btn.setAttribute('data-following', '0');
      }
    }

    window.ProjectAuth.apiFetch(
      PROJECTS_API + '/api/users/me/follows/' + encodeURIComponent(username)
    ).then(function (r) {
      return r.ok ? r.json() : null;
    }).then(function (data) {
      render(!!(data && data.following));
    }).catch(function () { render(false); });

    btn.addEventListener('click', function () {
      var following = btn.getAttribute('data-following') === '1';
      var method = following ? 'DELETE' : 'POST';
      btn.disabled = true;
      window.ProjectAuth.apiFetch(
        PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/follow',
        { method: method }
      ).then(function (r) {
        if (!r.ok) throw new Error('follow_failed');
        return r.json();
      }).then(function (data) {
        render(!!data.following);
        var fc = document.getElementById('stat-followers');
        if (fc) {
          var n = parseInt(fc.textContent, 10) || 0;
          n += following ? -1 : 1;
          fc.textContent = Math.max(0, n);
        }
      }).catch(function () {
        alert('Failed to update follow. Please try again.');
      }).then(function () {
        btn.disabled = false;
      });
    });
  }

  // --- Edit profile button (own profile only) -----------------------------

  function setupEditButton(viewer) {
    if (!viewer.authed) return;
    if (viewer.username && viewer.username !== username) return;
    // Show the edit link. If we don't know the viewer's username we still
    // show it — /account itself requires auth and will bounce to /login.
    // All profile editing now lives on /account (issue #151 consolidation);
    // /profile/edit.html is a redirect stub kept only for legacy bookmarks.
    document.getElementById('edit-profile-button').innerHTML =
      '<a href="/account" class="btn btn-outline-secondary btn-sm">' +
      '<i class="fas fa-pencil-alt me-1"></i>Edit profile</a>';
  }

  // --- Projects tab -------------------------------------------------------

  function renderProjectCard(p) {
    // Use the shared thumbnail helper; the .kr-project-card CSS wrapper
    // pins it to 4:3 with object-fit:cover so all cards stay uniform
    // regardless of cover image aspect.
    var thumb = typeof projectThumbnail === 'function'
      ? projectThumbnail(p)
      : (p.cover_image
          ? '<img src="' + esc(p.cover_image) + '" class="card-img-top" loading="lazy" alt="">'
          : '<div class="card-img-top d-flex align-items-center justify-content-center text-muted"><i class="fas fa-cube fa-2x"></i></div>');
    var diff = p.difficulty
      ? '<span class="badge bg-light text-dark me-1">' + esc(p.difficulty) + '</span>'
      : '';
    var status = p.status === 'completed'
      ? '<span class="badge bg-success me-1">Completed</span>' : '';
    var remix = p.is_remix
      ? '<span class="badge bg-light text-muted me-1" title="Remix"><i class="fas fa-code-branch"></i></span>'
      : '';
    // Issue #152: prefer canonical /projects/<owner>/<slug> URL when
    // the API surfaced a slug on the list item.
    var viewHref = (p.slug && p.author_username)
      ? '/projects/' + encodeURIComponent(p.author_username) + '/' + encodeURIComponent(p.slug)
      : '/projects/view.html?id=' + p.id;
    return '' +
      '<div class="col">' +
        '<a href="' + viewHref + '" class="text-decoration-none">' +
          '<div class="card kr-project-card h-100 shadow-sm card-hover">' +
            thumb +
            '<div class="card-body p-3">' +
              '<h6 class="card-title text-dark mb-1">' + esc(p.title) + '</h6>' +
              '<p class="card-text text-muted small mb-2 kr-card-description">' +
                esc(p.short_description || '') +
              '</p>' +
              '<div class="kr-card-tags">' + status + diff + remix + '</div>' +
            '</div>' +
          '</div>' +
        '</a>' +
      '</div>';
  }

  function loadProjects() {
    var url = PROJECTS_API + '/api/projects?author=' + encodeURIComponent(username) + '&limit=100';
    fetch(url).then(function (r) { return r.ok ? r.json() : []; })
      .then(function (projects) {
        document.getElementById('projects-loading').classList.add('d-none');
        var grid = document.getElementById('projects-grid');
        var empty = document.getElementById('projects-empty');
        if (!projects || projects.length === 0) {
          empty.classList.remove('d-none');
          return;
        }
        grid.classList.remove('d-none');
        grid.innerHTML = projects.map(renderProjectCard).join('');
      })
      .catch(function () {
        document.getElementById('projects-loading').classList.add('d-none');
        document.getElementById('projects-empty').classList.remove('d-none');
      });
  }

  // --- Makes tab (lazy: loads on first tab-show) --------------------------

  var makesLoaded = false;
  function loadMakes() {
    if (makesLoaded) return;
    makesLoaded = true;
    var loading = document.getElementById('makes-loading');
    var empty = document.getElementById('makes-empty');
    var grid = document.getElementById('makes-grid');
    loading.classList.remove('d-none');
    fetch(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/makes')
      .then(function (r) { return r.ok ? r.json() : []; })
      .then(function (makes) {
        loading.classList.add('d-none');
        if (!makes || makes.length === 0) {
          empty.classList.remove('d-none');
          return;
        }
        grid.classList.remove('d-none');
        grid.innerHTML = makes.map(function (m) {
          var title = m.project_title || 'a project';
          return '' +
            '<div class="col">' +
              '<a href="/projects/view.html?id=' + m.project_id + '#makes" class="text-decoration-none">' +
                '<div class="card h-100 shadow-sm card-hover">' +
                  '<div class="card-body p-3">' +
                    '<h6 class="card-title text-dark mb-1"><i class="fas fa-hammer me-2 text-muted"></i>' + esc(title) + '</h6>' +
                    (m.notes ? '<p class="card-text text-muted small mb-2">' + esc((m.notes || '').slice(0, 120)) + '</p>' : '') +
                    '<small class="text-muted">' + new Date(m.created_at).toLocaleDateString() + '</small>' +
                  '</div>' +
                '</div>' +
              '</a>' +
            '</div>';
        }).join('');
      })
      .catch(function () {
        loading.classList.add('d-none');
        empty.classList.remove('d-none');
      });
  }

  // --- Activity tab (lazy) ------------------------------------------------

  var activityLoaded = false;
  var ACTIVITY_ICONS = {
    project_published: { icon: 'fa-rocket', color: 'text-success', verb: 'Published' },
    project_updated: { icon: 'fa-cube', color: 'text-primary', verb: 'Updated' },
    make_posted: { icon: 'fa-hammer', color: 'text-dark', verb: 'Posted a make on' },
    comment_posted: { icon: 'fa-comment', color: 'text-info', verb: 'Commented on' },
    badge_earned: { icon: 'fa-medal', color: 'text-warning', verb: 'Earned a badge:' },
    collection_created: { icon: 'fa-folder', color: 'text-secondary', verb: 'Created collection' }
  };

  function timeAgo(iso) {
    var d = new Date(iso);
    var s = Math.round((Date.now() - d.getTime()) / 1000);
    if (s < 60) return s + 's ago';
    var m = Math.round(s / 60);
    if (m < 60) return m + 'm ago';
    var h = Math.round(m / 60);
    if (h < 24) return h + 'h ago';
    var dd = Math.round(h / 24);
    if (dd < 30) return dd + 'd ago';
    return d.toLocaleDateString();
  }

  function loadActivity() {
    if (activityLoaded) return;
    activityLoaded = true;
    var loading = document.getElementById('activity-loading');
    var list = document.getElementById('activity-list');
    var empty = document.getElementById('activity-empty');
    loading.classList.remove('d-none');
    fetch(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/activity?limit=20')
      .then(function (r) { return r.ok ? r.json() : { items: [] }; })
      .then(function (data) {
        loading.classList.add('d-none');
        var items = (data && data.items) || [];
        if (items.length === 0) {
          empty.classList.remove('d-none');
          return;
        }
        list.classList.remove('d-none');
        list.innerHTML = items.map(function (e) {
          var meta = ACTIVITY_ICONS[e.kind] || { icon: 'fa-circle', color: 'text-muted', verb: e.kind };
          var subject = e.subject_url
            ? '<a href="' + esc(e.subject_url) + '">' + esc(e.subject_title || 'an item') + '</a>'
            : esc(e.subject_title || '');
          return '<li class="activity-item d-flex align-items-start mb-2">' +
                   '<i class="fas ' + esc(meta.icon) + ' ' + esc(meta.color) + ' me-2 mt-1"></i>' +
                   '<div class="flex-grow-1">' +
                     '<div>' + esc(meta.verb) + ' ' + subject + '</div>' +
                     '<small class="text-muted">' + timeAgo(e.created_at) + '</small>' +
                   '</div>' +
                 '</li>';
        }).join('');
      })
      .catch(function () {
        loading.classList.add('d-none');
        empty.classList.remove('d-none');
      });
  }

  // Hook tab switches.
  var tabMakes = document.getElementById('tab-makes-btn');
  if (tabMakes) tabMakes.addEventListener('shown.bs.tab', loadMakes);
  var tabActivity = document.getElementById('tab-activity-btn');
  if (tabActivity) tabActivity.addEventListener('shown.bs.tab', loadActivity);

  // --- Featured badges row ------------------------------------------------
  //
  // Reads the user's pinned slugs from /profile and the full badge
  // catalog from /badges, then renders the intersection. If badges API
  // 404s or the slug list is empty, the section stays hidden.

  function loadFeaturedBadges(profile) {
    var pinned = (profile && profile.featured_badge_slugs) || [];
    if (!pinned || pinned.length === 0) return;
    fetch(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/badges')
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (data) {
        if (!data || !data.badges) return;
        var byKey = {};
        data.badges.forEach(function (b) { byKey[b.key.toLowerCase()] = b; });
        var matched = pinned.slice(0, 3)
          .map(function (s) { return byKey[(s || '').toLowerCase()]; })
          .filter(Boolean);
        if (matched.length === 0) return;
        var card = document.getElementById('badges-card');
        var list = document.getElementById('badges-list');
        card.classList.remove('d-none');
        list.innerHTML = matched.map(function (b) {
          var cls = 'bg-' + (b.color || 'primary');
          return '<span class="badge ' + cls + ' p-2 d-inline-flex align-items-center" ' +
                 'title="' + esc(b.description) + '">' +
                 '<i class="fas ' + esc(b.icon) + ' me-1"></i> ' + esc(b.name) +
                 '</span>';
        }).join('');
      })
      .catch(function () {});
  }

  // --- Followers / following modal ---------------------------------------

  function openUserListModal(kind /* 'followers' or 'following' */) {
    var title = document.getElementById('user-list-title');
    var loading = document.getElementById('user-list-loading');
    var list = document.getElementById('user-list-list');
    var empty = document.getElementById('user-list-empty');
    title.textContent = kind === 'followers' ? 'Followers' : 'Following';
    list.classList.add('d-none');
    empty.classList.add('d-none');
    loading.classList.remove('d-none');
    list.innerHTML = '';
    if (window.bootstrap && window.bootstrap.Modal) {
      var modalEl = document.getElementById('user-list-modal');
      var modal = window.bootstrap.Modal.getOrCreateInstance(modalEl);
      modal.show();
    }
    fetch(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/' + kind + '?limit=200')
      .then(function (r) { return r.ok ? r.json() : { users: [] }; })
      .then(function (data) {
        loading.classList.add('d-none');
        var users = (data && data.users) || [];
        if (users.length === 0) {
          empty.classList.remove('d-none');
          return;
        }
        list.classList.remove('d-none');
        list.innerHTML = users.map(function (u) {
          return '<li class="d-flex align-items-center mb-2">' +
                   '<i class="fas fa-user-circle me-2 text-muted"></i>' +
                   '<a href="' + profileLink(u.username) + '" class="text-decoration-none">' +
                     esc(u.username) +
                   '</a>' +
                 '</li>';
        }).join('');
      })
      .catch(function () {
        loading.classList.add('d-none');
        empty.classList.remove('d-none');
      });
  }

  var fBtn = document.getElementById('stat-followers-btn');
  if (fBtn) fBtn.addEventListener('click', function () { openUserListModal('followers'); });
  var gBtn = document.getElementById('stat-following-btn');
  if (gBtn) gBtn.addEventListener('click', function () { openUserListModal('following'); });

  // --- Chatter comments ---------------------------------------------------

  function loadComments() {
    var commentsDiv = document.getElementById('comments-list');
    fetch(CHATTER_API + '/profile/' + encodeURIComponent(username) + '/comments?limit=10')
      .then(function (r) { return r.ok ? r.json() : { comments: [] }; })
      .then(function (data) {
        if (data.comments && data.comments.length > 0) {
          commentsDiv.innerHTML = data.comments.map(function (c) {
            return '<div class="border-bottom pb-3 mb-3">' +
                     '<p class="mb-2">' + esc(c.content) + '</p>' +
                     '<small class="text-muted">' +
                       '<i class="fas fa-link me-1"></i>' +
                       '<a href="https://www.kevsrobots.com/' + esc(c.url) + '" ' +
                          'target="_blank" class="text-decoration-none">' +
                         esc(c.url) +
                       '</a>' +
                       '<span class="ms-3"><i class="fas fa-clock me-1"></i>' +
                         new Date(c.created_at).toLocaleDateString() + '</span>' +
                     '</small>' +
                   '</div>';
          }).join('');
        } else {
          commentsDiv.innerHTML = '<p class="text-muted small mb-0">No comments yet.</p>';
        }
      })
      .catch(function () {
        commentsDiv.innerHTML = '<p class="text-muted small mb-0">Comments unavailable.</p>';
      });
  }

  // --- Boot ---------------------------------------------------------------

  Promise.all([loadChatterProfile(), probeViewer(), loadProfile()]).then(function (results) {
    var chatterProfile = results[0];
    var viewer = results[1];
    var profile = results[2];
    setupFollowButton(viewer, chatterProfile);
    setupEditButton(viewer);
    loadFeaturedBadges(profile);
  });

  loadProjects();
  loadComments();
})();
