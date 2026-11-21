/**
 * Public user profile page (profile.html): a maker's portfolio.
 *
 *   Header      avatar, name, @handle, location, Follow / Edit, Share profile
 *   Meta        bio, stats (projects, likes received, followers, following),
 *               website and socials, "Builds with" chips, featured badges
 *   Pinned      up to 3 projects the owner picked (else their 3 most liked)
 *   Tabs        Projects (filter by app, sort newest / most liked), Makes,
 *               Activity (with recent comments), Liked (only if public)
 *
 * Requests on load, all in parallel: the projects-api profile (bio, stats,
 * pinned, builds_with), the project list, Chatter's profile (name, avatar),
 * then one Chatter batch call for every card's likes and comments
 * (kr-project-card.js). Makes, Activity and Liked load when opened.
 *
 * Short link: /@username. 404.html forwards it here as ?u=, and this page
 * puts /@username back in the address bar. ?u=, ?username= and
 * /profile/<username> links keep working.
 *
 * Any endpoint can be missing; its section stays hidden.
 */
(function () {
  'use strict';

  var PROJECTS_API = 'https://projects.kevsrobots.com';
  var CHATTER_API = 'https://chatter.kevsrobots.com';
  var Card = window.KrProjectCard;

  // --- Username -------------------------------------------------------------

  function getUsernameFromUrl() {
    var params = new URLSearchParams(window.location.search);
    var u = params.get('u') || params.get('username');
    if (u) return u;
    var parts = window.location.pathname.split('/').filter(Boolean);
    var first = parts[0] || '';
    try {
      if (first.charAt(0) === '@') return decodeURIComponent(first.slice(1));
      if (parts.length > 1 && (first === 'profile' || first === 'profile.html')) return decodeURIComponent(parts[1]);
    } catch (e) { /* bad escape: fall through */ }
    return null;
  }

  var username = getUsernameFromUrl();
  var root = document.getElementById('profile-root');
  if (!username) {
    root.innerHTML = '<p class="kr-profile-empty">No username in the link. Profile links look like kevsrobots.com/@username.</p>';
    return;
  }
  var shortUrl = window.location.origin + '/@' + encodeURIComponent(username);
  try { window.history.replaceState(null, '', '/@' + encodeURIComponent(username) + window.location.hash); } catch (e) { /* keep the old URL */ }

  var me = Card ? Card.viewer() : null;
  var isOwn = !!me && me === username;

  // --- Helpers ----------------------------------------------------------------

  function $(id) { return document.getElementById(id); }

  function esc(s) {
    return String(s == null ? '' : s)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
  }

  function getJSON(url, opts) {
    return fetch(url, opts).then(function (r) { return r.ok ? r.json() : null; }).catch(function () { return null; });
  }

  function hue(text) {
    var h = 0;
    for (var i = 0; i < text.length; i++) h = (h * 31 + text.charCodeAt(i)) % 360;
    return h;
  }

  function compact(n) {
    n = n || 0;
    if (n < 1000) return String(n);
    return (n / 1000).toFixed(n < 10000 ? 1 : 0).replace(/\.0$/, '') + 'k';
  }

  var APP_NAMES = { snakie: 'Snakie', bettercad: 'BetterCAD' };

  // --- Header ---------------------------------------------------------------

  function paintAvatar(chatter) {
    var el = $('profile-avatar');
    var h = hue(username);
    el.style.setProperty('--krp-h', h);
    $('profile-banner').style.setProperty('--krp-h', h);
    el.textContent = username.charAt(0).toUpperCase();
    var pic = chatter && chatter.profile_picture_url;
    if (!pic) return;
    var img = new Image();
    img.alt = '';
    img.onload = function () { el.textContent = ''; el.appendChild(img); };
    img.src = pic.indexOf('http') === 0 ? pic : CHATTER_API + pic;
  }

  function paintName(chatter) {
    var name = chatter ? [chatter.firstname, chatter.lastname].filter(Boolean).join(' ').trim() : '';
    $('profile-name').textContent = name || username;
    $('profile-username').textContent = '@' + username;
    document.title = (name || username) + ' on kevsrobots';
  }

  var SOCIALS = {
    github: ['fa-brands fa-github', 'GitHub'],
    twitter: ['fa-brands fa-x-twitter', 'X'],
    youtube: ['fa-brands fa-youtube', 'YouTube'],
    mastodon: ['fa-brands fa-mastodon', 'Mastodon']
  };

  function paintProfile(p) {
    if (!p) return;
    if (p.bio) { $('profile-bio').hidden = false; $('profile-bio').textContent = p.bio; }
    if (p.location) { $('profile-location').hidden = false; $('location-text').textContent = p.location; }
    if (p.website_url) {
      var w = $('profile-website');
      w.hidden = false;
      w.href = p.website_url;
      w.textContent = p.website_url.replace(/^https?:\/\//, '').replace(/\/$/, '');
    }
    var links = p.social_links || {};
    $('profile-socials').innerHTML = Object.keys(SOCIALS).filter(function (k) { return links[k]; }).map(function (k) {
      return '<a href="' + esc(links[k]) + '" target="_blank" rel="noopener noreferrer nofollow" title="' + SOCIALS[k][1] + '" aria-label="' + SOCIALS[k][1] + '">' +
        '<i class="' + SOCIALS[k][0] + '" aria-hidden="true"></i></a>';
    }).join('');
    if (p.joined_at) {
      $('profile-joined').textContent = 'Joined ' + new Date(p.joined_at).toLocaleDateString(undefined, { month: 'long', year: 'numeric' });
    }
    var s = p.stats || {};
    $('stat-projects').textContent = compact(s.projects);
    $('stat-followers').textContent = compact(s.followers);
    $('stat-following').textContent = compact(s.following);
    $('tab-projects-n').textContent = s.projects || '';
    $('tab-makes-n').textContent = s.makes || '';

    var builds = p.builds_with || {};
    var apps = Object.keys(builds).filter(function (a) { return APP_NAMES[a]; });
    if (apps.length) {
      var wrap = $('profile-builds');
      wrap.hidden = false;
      apps.forEach(function (a) {
        var b = document.createElement('button');
        b.type = 'button';
        b.className = 'kr-profile-build kr-profile-build-' + a;
        b.setAttribute('data-filter', a);
        b.textContent = APP_NAMES[a] + ' · ' + builds[a] + (builds[a] === 1 ? ' project' : ' projects');
        wrap.appendChild(b);
        document.querySelector('#project-filters [data-filter="' + a + '"]').hidden = false;
      });
    }
  }

  // Follow button: same .krc-follow wiring as the cards (kr-project-card.js).
  function setupFollow() {
    var btn = $('follow-btn');
    if (isOwn) { $('edit-profile-button').hidden = false; return; }
    btn.hidden = false;
    btn.setAttribute('data-krc-follow', username);
    if (Card) {
      Card.hydrate(document.createElement('div'));
      Card.following().then(function (set) {
        if (!set) return;
        var on = set.has(username);
        btn.setAttribute('aria-pressed', on ? 'true' : 'false');
        btn.textContent = on ? 'Following' : 'Follow';
      });
    }
    document.addEventListener('krc:followed', function (e) {
      if (e.detail.username !== username) return;
      var el = $('stat-followers');
      var n = parseInt(el.textContent, 10);
      if (!isNaN(n) && el.textContent.indexOf('k') === -1) el.textContent = Math.max(0, n + (e.detail.following ? 1 : -1));
    });
  }

  function setupShare() {
    var btn = $('share-profile-btn');
    btn.addEventListener('click', function () {
      if (Card) Card.openShare(btn, shortUrl, $('profile-name').textContent + ' on kevsrobots');
    });
  }

  // --- Badges -----------------------------------------------------------------

  function loadBadges(profile) {
    var pinned = (profile && profile.featured_badge_slugs) || [];
    if (!pinned.length) return;
    getJSON(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/badges').then(function (data) {
      if (!data || !data.badges) return;
      var byKey = {};
      data.badges.forEach(function (b) { byKey[(b.key || '').toLowerCase()] = b; });
      var matched = pinned.slice(0, 3).map(function (s) { return byKey[(s || '').toLowerCase()]; }).filter(Boolean);
      if (!matched.length) return;
      var list = $('badges-list');
      list.hidden = false;
      list.innerHTML = matched.map(function (b) {
        return '<span class="kr-profile-badge" title="' + esc(b.description) + '"><i class="fas ' + esc(b.icon) + '" aria-hidden="true"></i>' + esc(b.name) + '</span>';
      }).join('');
    });
  }

  // --- Projects ---------------------------------------------------------------

  var projects = [];
  var filter = 'all';
  var sort = 'newest';
  var likeCounts = {};

  function readLikeCounts(container) {
    container.querySelectorAll('.krc').forEach(function (c) {
      likeCounts[c.getAttribute('data-krc-key')] = parseInt(c.querySelector('.krc-like-n').textContent, 10) || 0;
    });
  }

  function likesOf(p) { return likeCounts[Card.likeKey(p)] || 0; }

  function visibleProjects() {
    var list = projects.filter(function (p) {
      return filter === 'all' || (p.made_with || []).indexOf(filter) !== -1;
    });
    if (sort === 'liked') list = list.slice().sort(function (a, b) { return likesOf(b) - likesOf(a); });
    return list;
  }

  function drawProjects() {
    var grid = $('projects-grid');
    var list = visibleProjects();
    $('projects-empty').hidden = list.length > 0;
    $('projects-empty').textContent = projects.length
      ? 'No projects made with ' + (APP_NAMES[filter] || 'that') + ' yet.'
      : (isOwn ? 'You have no public projects yet. Publish one from My Projects.' : 'No public projects yet.');
    return Card.render(grid, list, { showAuthor: false }).then(function () {
      grid.removeAttribute('aria-busy');
      readLikeCounts(grid);
    });
  }

  function setPressed(group, attr, value) {
    group.querySelectorAll('[' + attr + ']').forEach(function (b) {
      b.setAttribute('aria-pressed', b.getAttribute(attr) === value ? 'true' : 'false');
    });
  }

  function setFilter(value) {
    filter = value;
    setPressed($('project-filters'), 'data-filter', value);
    selectTab('projects');
    drawProjects();
  }

  function setupProjectControls() {
    $('project-filters').addEventListener('click', function (e) {
      var b = e.target.closest('[data-filter]');
      if (b) setFilter(b.getAttribute('data-filter'));
    });
    $('profile-builds').addEventListener('click', function (e) {
      var b = e.target.closest('[data-filter]');
      if (!b) return;
      setFilter(filter === b.getAttribute('data-filter') ? 'all' : b.getAttribute('data-filter'));
      $('tab-projects').scrollIntoView({ behavior: 'smooth', block: 'start' });
    });
    document.querySelector('.kr-profile-sort').addEventListener('click', function (e) {
      var b = e.target.closest('[data-sort]');
      if (!b) return;
      sort = b.getAttribute('data-sort');
      setPressed(document.querySelector('.kr-profile-sort'), 'data-sort', sort);
      drawProjects();
    });
    document.addEventListener('krc:liked', function (e) {
      var card = document.querySelector('#projects-grid .krc[data-krc-key="' + e.detail.key + '"]');
      if (card) readLikeCounts($('projects-grid'));
    });
  }

  function loadProjects() {
    return getJSON(PROJECTS_API + '/api/projects?author=' + encodeURIComponent(username) + '&limit=100', { credentials: 'include' })
      .then(function (list) {
        projects = (list || []).filter(function (p) { return p.status !== 'draft' || isOwn; });
        return drawProjects();
      });
  }

  function totalLikes() {
    var sum = 0;
    projects.forEach(function (p) { sum += likesOf(p); });
    $('stat-likes').textContent = compact(sum);
  }

  // Pinned: the owner's picks, else their three most liked (with any likes).
  function drawPinned(profile) {
    var cards = (profile && profile.pinned) || [];
    if (!cards.length) {
      cards = projects.slice().sort(function (a, b) { return likesOf(b) - likesOf(a); })
        .filter(function (p) { return likesOf(p) > 0; }).slice(0, 3);
      if (projects.length < 4) cards = [];
    }
    if (!cards.length) return;
    $('pinned-section').hidden = false;
    $('pinned-title').textContent = profile && profile.pinned && profile.pinned.length ? 'Pinned' : 'Most liked';
    Card.render($('pinned-grid'), cards, { showAuthor: false, size: 'lg' });
  }

  // --- Tabs -------------------------------------------------------------------

  var loaders = {};
  var tabs = ['projects', 'makes', 'activity', 'liked'];

  function selectTab(name, focus) {
    tabs.forEach(function (t) {
      var on = t === name;
      var btn = $('tab-' + t + '-btn');
      btn.setAttribute('aria-selected', on ? 'true' : 'false');
      btn.tabIndex = on ? 0 : -1;
      $('tab-' + t).hidden = !on;
      if (on && focus) btn.focus();
    });
    if (loaders[name]) { loaders[name](); loaders[name] = null; }
  }

  function setupTabs() {
    var bar = document.querySelector('.kr-profile-tabs');
    bar.addEventListener('click', function (e) {
      var b = e.target.closest('[role="tab"]');
      if (b) selectTab(b.id.replace(/^tab-|-btn$/g, ''));
    });
    bar.addEventListener('keydown', function (e) {
      if (e.key !== 'ArrowRight' && e.key !== 'ArrowLeft') return;
      var shown = tabs.filter(function (t) { return !$('tab-' + t + '-btn').hidden; });
      var cur = shown.indexOf(document.activeElement.id.replace(/^tab-|-btn$/g, ''));
      var next = shown[(cur + (e.key === 'ArrowRight' ? 1 : -1) + shown.length) % shown.length];
      selectTab(next, true);
      e.preventDefault();
    });
  }

  loaders.makes = function () {
    getJSON(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/makes').then(function (makes) {
      makes = makes || [];
      $('makes-empty').hidden = makes.length > 0;
      $('makes-grid').innerHTML = makes.map(function (m) {
        return '<a class="kr-profile-make" href="/projects/view.html?id=' + encodeURIComponent(m.project_id) + '#makes">' +
          '<b>' + esc(m.project_title || 'a project') + '</b>' +
          (m.notes ? '<span>' + esc(String(m.notes).slice(0, 140)) + '</span>' : '') +
          '<small>' + new Date(m.created_at).toLocaleDateString() + '</small></a>';
      }).join('');
    });
  };

  var VERBS = {
    project_published: 'Published',
    project_updated: 'Updated',
    make_posted: 'Posted a make of',
    comment_posted: 'Commented on',
    badge_earned: 'Earned the badge',
    collection_created: 'Created the collection'
  };

  function timeAgo(iso) {
    var s = Math.round((Date.now() - new Date(iso).getTime()) / 1000);
    if (s < 3600) return Math.max(1, Math.round(s / 60)) + ' min ago';
    if (s < 86400) return Math.round(s / 3600) + ' h ago';
    if (s < 30 * 86400) return Math.round(s / 86400) + ' days ago';
    return new Date(iso).toLocaleDateString();
  }

  loaders.activity = function () {
    getJSON(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/activity?limit=20').then(function (data) {
      var items = (data && data.items) || [];
      $('activity-empty').hidden = items.length > 0;
      $('activity-list').innerHTML = items.map(function (e) {
        var subject = e.subject_url
          ? '<a href="' + esc(e.subject_url) + '">' + esc(e.subject_title || 'an item') + '</a>'
          : esc(e.subject_title || '');
        return '<li><span>' + esc(VERBS[e.kind] || e.kind) + ' ' + subject + '</span><small>' + timeAgo(e.created_at) + '</small></li>';
      }).join('');
    });
    getJSON(CHATTER_API + '/profile/' + encodeURIComponent(username) + '/comments?limit=10').then(function (data) {
      var list = (data && data.comments) || [];
      $('comments-list').innerHTML = list.length ? list.map(function (c) {
        return '<div class="kr-profile-comment"><p>' + esc(c.content) + '</p><small><a href="/' + esc(String(c.url).replace(/^\//, '')) + '">' +
          esc(c.url) + '</a> · ' + new Date(c.created_at).toLocaleDateString() + '</small></div>';
      }).join('') : '<p class="kr-profile-empty">No comments yet.</p>';
    });
  };

  // Liked: only when the person made their likes public (or it's your own).
  function setupLiked() {
    getJSON(CHATTER_API + '/interact/users/' + encodeURIComponent(username) + '/liked?prefix=projects/&limit=24', { credentials: 'include' })
      .then(function (data) {
        if (!data) return;
        $('tab-liked-btn').hidden = false;
        loaders.liked = function () {
          var ids = (data.urls || []).map(function (u) { var m = /[?&]id=(\d+)/.exec(u); return m && m[1]; }).filter(Boolean);
          if (!ids.length) { $('liked-empty').hidden = false; return; }
          getJSON(PROJECTS_API + '/api/projects?limit=50&ids=' + ids.join(',')).then(function (list) {
            var order = {};
            ids.forEach(function (id, i) { order[id] = i; });
            list = (list || []).sort(function (a, b) { return order[a.id] - order[b.id]; });
            $('liked-empty').hidden = list.length > 0;
            Card.render($('liked-grid'), list);
          });
        };
      });
  }

  // --- Followers / following list ---------------------------------------------

  function openUserList(kind) {
    var title = $('user-list-title');
    var loading = $('user-list-loading');
    var list = $('user-list-list');
    var empty = $('user-list-empty');
    title.textContent = kind === 'followers' ? 'Followers' : 'Following';
    list.classList.add('d-none');
    empty.classList.add('d-none');
    loading.classList.remove('d-none');
    if (window.bootstrap && window.bootstrap.Modal) {
      window.bootstrap.Modal.getOrCreateInstance($('user-list-modal')).show();
    }
    getJSON(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/' + kind + '?limit=200').then(function (data) {
      loading.classList.add('d-none');
      var users = (data && data.users) || [];
      if (!users.length) { empty.classList.remove('d-none'); return; }
      list.classList.remove('d-none');
      list.innerHTML = users.map(function (u) {
        return '<li class="mb-2"><a class="text-decoration-none" href="/@' + encodeURIComponent(u.username) + '">@' + esc(u.username) + '</a></li>';
      }).join('');
    });
  }

  // --- Boot -------------------------------------------------------------------

  paintName(null);
  paintAvatar(null);
  setupTabs();
  setupShare();
  setupFollow();
  setupProjectControls();
  $('stat-followers-btn').addEventListener('click', function () { openUserList('followers'); });
  $('stat-following-btn').addEventListener('click', function () { openUserList('following'); });

  var chatterP = getJSON(CHATTER_API + '/profile/' + encodeURIComponent(username), { credentials: 'include' });
  var profileP = getJSON(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/profile');
  var projectsP = loadProjects();

  chatterP.then(function (c) { paintName(c); paintAvatar(c); });
  profileP.then(function (p) { paintProfile(p); loadBadges(p); });
  Promise.all([profileP, projectsP]).then(function (r) {
    totalLikes();
    drawPinned(r[0]);
  });
  Promise.all([chatterP, profileP]).then(function (r) {
    if (!r[0] && !r[1]) {
      $('profile-name').textContent = 'We couldn\'t find @' + username;
      document.title = 'Profile not found - kevsrobots';
    }
  });
  setupLiked();

  if (/^#(makes|activity|liked)$/.test(window.location.hash)) selectTab(window.location.hash.slice(1));
})();
