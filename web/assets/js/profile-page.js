/**
 * Public user profile page — issue #140.
 *
 * Renders:
 *   1. Chatter profile header (name, avatar, bio, join date)
 *   2. Followers / following counts + Follow/Unfollow button
 *   3. Earned badges (computed by projects-api)
 *   4. The user's published projects (excludes archived / blocked)
 *   5. Recent activity (own project creates/updates + community makes)
 *   6. Recent Chatter comments
 *
 * Auth model: page is read-only public. Follow actions require a JWT
 * cookie or dev token (via ProjectAuth.apiFetch).
 */
(function () {
  'use strict';

  var PROJECTS_API = 'https://projects.kevsrobots.com';
  var CHATTER_API = 'https://chatter.kevsrobots.com';

  // --- Username extraction ------------------------------------------------

  function getUsernameFromUrl() {
    var urlParams = new URLSearchParams(window.location.search);
    var username = urlParams.get('username') || urlParams.get('u');
    if (username) return username;
    var parts = window.location.pathname.split('/').filter(Boolean);
    // /profile/<username> or /profile.html/<username>
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

  // Probe who the viewer is. Prefer /api/auth/me (added by issue #139),
  // fall back to the projects "my list" endpoint that already exists.
  // Returns { authed, username } — username is empty if /api/auth/me
  // isn't deployed yet (we'll still show the button by other means).
  function probeViewer() {
    return window.ProjectAuth.apiFetch(PROJECTS_API + '/api/auth/me')
      .then(function (r) {
        if (r.ok) return r.json().then(function (j) {
          return { authed: true, username: (j && (j.username || j.sub)) || '' };
        });
        if (r.status === 401 || r.status === 403) {
          return { authed: false, username: '' };
        }
        // Fallback: hit my/list — if it 200s the user is logged in but
        // we won't know their name. That's fine for the follow button:
        // we treat "logged in & viewing somebody else" as the trigger.
        return window.ProjectAuth.apiFetch(PROJECTS_API + '/api/projects/my/list')
          .then(function (r2) {
            if (r2.ok) return { authed: true, username: '' };
            return { authed: false, username: '' };
          })
          .catch(function () { return { authed: false, username: '' }; });
      })
      .catch(function () {
        return window.ProjectAuth.apiFetch(PROJECTS_API + '/api/projects/my/list')
          .then(function (r2) {
            if (r2.ok) return { authed: true, username: '' };
            return { authed: false, username: '' };
          })
          .catch(function () { return { authed: false, username: '' }; });
      });
  }

  // --- Chatter profile header --------------------------------------------

  function loadChatterProfile() {
    return fetch(CHATTER_API + '/profile/' + encodeURIComponent(username), {
      credentials: 'include'
    })
      .then(function (r) {
        if (!r.ok) throw new Error('not_found');
        return r.json();
      })
      .then(function (profile) {
        document.title = profile.username + "'s Profile - Kev's Robots";

        var avatarDiv = document.getElementById('profile-avatar');
        if (profile.profile_picture_url) {
          var pictureUrl = profile.profile_picture_url.indexOf('http') === 0
            ? profile.profile_picture_url
            : CHATTER_API + profile.profile_picture_url;
          avatarDiv.innerHTML =
            '<img src="' + esc(pictureUrl) + '" alt="' + esc(profile.username) + '" ' +
            'class="rounded-circle img-fluid" ' +
            'style="max-width:150px;max-height:150px;object-fit:cover;">';
        } else {
          var initial = (profile.username && profile.username[0]) || '?';
          avatarDiv.innerHTML =
            '<div class="rounded-circle bg-secondary d-inline-flex align-items-center ' +
            'justify-content-center text-white" ' +
            'style="width:150px;height:150px;font-size:3rem;">' +
            esc(initial.toUpperCase()) + '</div>';
        }

        var name = [profile.firstname, profile.lastname].filter(Boolean).join(' ').trim();
        document.getElementById('profile-name').textContent = name || profile.username;
        document.getElementById('profile-username').textContent = '@' + profile.username;

        if (profile.is_own_profile) {
          document.getElementById('edit-profile-button').innerHTML =
            '<a href="' + CHATTER_API + '/account" class="btn btn-outline-secondary btn-sm">' +
            '<i class="fas fa-pencil-alt me-1"></i>Edit profile</a>';
        }

        if (profile.location) {
          document.getElementById('profile-location').style.display = 'block';
          document.getElementById('location-text').textContent = profile.location;
        }
        if (profile.bio) {
          var bioEl = document.getElementById('profile-bio');
          bioEl.style.display = 'block';
          bioEl.textContent = profile.bio;
        }
        if (profile.created_at) {
          var joinDate = new Date(profile.created_at);
          document.getElementById('profile-joined').textContent =
            'Joined ' + joinDate.toLocaleDateString(undefined, { month: 'long', year: 'numeric' });
        }
        return profile;
      })
      .catch(function () {
        // If Chatter doesn't have the user, render a minimal header so
        // the projects/badges sections still work — we know the username
        // exists if they've created projects.
        document.getElementById('profile-name').textContent = username;
        document.getElementById('profile-username').textContent = '@' + username;
        var avatarDiv = document.getElementById('profile-avatar');
        var initial = username[0] || '?';
        avatarDiv.innerHTML =
          '<div class="rounded-circle bg-secondary d-inline-flex align-items-center ' +
          'justify-content-center text-white" ' +
          'style="width:150px;height:150px;font-size:3rem;">' +
          esc(initial.toUpperCase()) + '</div>';
        return { username: username, is_own_profile: false };
      });
  }

  // --- Follow / unfollow --------------------------------------------------

  function setupFollowButton(viewer, profile) {
    var btn = document.getElementById('follow-btn');
    if (!btn) return;

    // Hide when anonymous, viewing own profile, or no auth at all.
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

    // Pull current follow state.
    window.ProjectAuth.apiFetch(
      PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/follow'
    ).then(function (r) {
      if (r.ok) return r.json();
      return null;
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
        // Update follower count optimistically.
        var fc = document.getElementById('profile-followers-count');
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

  // --- Counts -------------------------------------------------------------

  function loadFollowCounts() {
    var fol = PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/followers/count';
    var fng = PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/following/count';
    fetch(fol).then(function (r) { return r.ok ? r.json() : null; })
      .then(function (d) {
        if (d) document.getElementById('profile-followers-count').textContent = d.count;
      }).catch(function () {});
    fetch(fng).then(function (r) { return r.ok ? r.json() : null; })
      .then(function (d) {
        if (d) document.getElementById('profile-following-count').textContent = d.count;
      }).catch(function () {});
  }

  // --- Projects -----------------------------------------------------------

  function loadProjects() {
    var url = PROJECTS_API + '/api/projects?author=' + encodeURIComponent(username) + '&limit=100';
    fetch(url).then(function (r) { return r.ok ? r.json() : []; })
      .then(function (projects) {
        document.getElementById('projects-loading').classList.add('d-none');
        var grid = document.getElementById('projects-grid');
        var empty = document.getElementById('projects-empty');
        var countEl = document.getElementById('projects-count');
        if (!projects || projects.length === 0) {
          empty.classList.remove('d-none');
          return;
        }
        countEl.textContent = '(' + projects.length + ')';
        grid.classList.remove('d-none');
        grid.innerHTML = projects.map(function (p) {
          var thumb = p.cover_image
            ? '<img src="' + esc(p.cover_image) + '" class="card-img-top" loading="lazy" alt="" style="height:140px;object-fit:cover;background:#f1f3f5;">'
            : '<div class="card-img-top d-flex align-items-center justify-content-center text-muted" style="height:140px;background:#f1f3f5;"><i class="fas fa-cube fa-2x"></i></div>';
          var diff = p.difficulty
            ? '<span class="badge bg-light text-dark me-1">' + esc(p.difficulty) + '</span>'
            : '';
          var status = p.status === 'completed'
            ? '<span class="badge bg-success me-1">Completed</span>'
            : '';
          var remix = p.is_remix
            ? '<span class="badge bg-light text-muted me-1" title="Remix"><i class="fas fa-code-branch"></i></span>'
            : '';
          return '' +
            '<div class="col">' +
              '<a href="/projects/view.html?id=' + p.id + '" class="text-decoration-none">' +
                '<div class="card h-100 shadow-sm card-hover">' +
                  thumb +
                  '<div class="card-body p-3">' +
                    '<h6 class="card-title text-dark mb-1">' + esc(p.title) + '</h6>' +
                    '<p class="card-text text-muted small mb-2">' +
                      esc((p.short_description || '').slice(0, 100)) +
                      ((p.short_description || '').length > 100 ? '…' : '') +
                    '</p>' +
                    '<div>' + status + diff + remix + '</div>' +
                  '</div>' +
                '</div>' +
              '</a>' +
            '</div>';
        }).join('');
      })
      .catch(function () {
        document.getElementById('projects-loading').classList.add('d-none');
        document.getElementById('projects-empty').classList.remove('d-none');
      });
  }

  // --- Badges -------------------------------------------------------------

  function loadBadges() {
    var url = PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/badges';
    fetch(url).then(function (r) { return r.ok ? r.json() : null; })
      .then(function (data) {
        if (!data || !data.badges || data.badges.length === 0) return;
        var card = document.getElementById('badges-card');
        var list = document.getElementById('badges-list');
        card.classList.remove('d-none');
        list.innerHTML = data.badges.map(function (b) {
          var cls = 'bg-' + (b.color || 'primary');
          return '<span class="badge ' + cls + ' p-2 d-inline-flex align-items-center" ' +
                 'data-bs-toggle="tooltip" title="' + esc(b.description) + '">' +
                 '<i class="fas ' + esc(b.icon) + ' me-1"></i> ' + esc(b.name) +
                 '</span>';
        }).join('');
        // Initialise tooltips when bootstrap JS is present.
        try {
          if (window.bootstrap && window.bootstrap.Tooltip) {
            list.querySelectorAll('[data-bs-toggle="tooltip"]').forEach(function (el) {
              new window.bootstrap.Tooltip(el);
            });
          }
        } catch (e) {}
      }).catch(function () {});
  }

  // --- Recent activity ----------------------------------------------------

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
    // Activity combines two sources: (a) project create/update events
    // from this user's project list, and (b) "I Made This!" makes posted
    // by this user. Both are read-only public endpoints. We merge and
    // sort by timestamp desc.
    var pProjects = fetch(
      PROJECTS_API + '/api/projects?author=' + encodeURIComponent(username) + '&limit=20'
    ).then(function (r) { return r.ok ? r.json() : []; }).catch(function () { return []; });

    var pMakes = fetch(
      PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/makes'
    ).then(function (r) { return r.ok ? r.json() : []; }).catch(function () { return []; });

    Promise.all([pProjects, pMakes]).then(function (results) {
      var projects = results[0] || [];
      var makes = results[1] || [];
      var events = [];

      projects.forEach(function (p) {
        events.push({
          when: p.created_at,
          icon: 'fa-cube',
          color: 'text-primary',
          html: 'Created project <a href="/projects/view.html?id=' + p.id + '">' +
                esc(p.title) + '</a>'
        });
      });
      makes.forEach(function (m) {
        var title = m.project_title || 'a project';
        events.push({
          when: m.created_at,
          icon: 'fa-hammer',
          color: 'text-dark',
          html: 'Posted a Make on <a href="/projects/view.html?id=' + m.project_id + '">' +
                esc(title) + '</a>'
        });
      });

      document.getElementById('activity-loading').classList.add('d-none');
      var list = document.getElementById('activity-list');
      var empty = document.getElementById('activity-empty');
      if (events.length === 0) {
        empty.classList.remove('d-none');
        return;
      }
      events.sort(function (a, b) { return new Date(b.when) - new Date(a.when); });
      list.classList.remove('d-none');
      list.innerHTML = events.slice(0, 15).map(function (e) {
        return '<li class="activity-item d-flex align-items-start mb-2">' +
                 '<i class="fas ' + esc(e.icon) + ' ' + esc(e.color) + ' me-2 mt-1"></i>' +
                 '<div class="flex-grow-1">' +
                   '<div>' + e.html + '</div>' +
                   '<small class="text-muted">' + timeAgo(e.when) + '</small>' +
                 '</div>' +
               '</li>';
      }).join('');
    });
  }

  // --- Comments (Chatter) -------------------------------------------------

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

  Promise.all([loadChatterProfile(), probeViewer()]).then(function (results) {
    var profile = results[0];
    var viewer = results[1];
    setupFollowButton(viewer, profile);
  });

  loadFollowCounts();
  loadBadges();
  loadProjects();
  loadActivity();
  loadComments();
})();
