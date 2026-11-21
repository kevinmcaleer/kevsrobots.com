/* kr-project-card: the one project card used across kevsrobots.com
 * (homepage showcase, profiles, galleries).
 *
 *   KrProjectCard.render(container, projects, opts)  draw cards, then hydrate
 *   KrProjectCard.html(project, opts)                one card's markup
 *   KrProjectCard.hydrate(root)                      likes, follows, share
 *
 * opts: { showAuthor: true, showFollow: true, size: 'md' | 'lg' }
 *
 * Each card shows "Made with" badges (project.made_with from projects-api),
 * a heart you can toggle in place, the comment count, a Follow button for the
 * author and Share (the phone's own share sheet where there is one, otherwise
 * a small menu). Likes and comment counts for the whole grid come from one
 * Chatter call (POST /interact/like-status/batch); if that endpoint isn't
 * deployed yet it falls back to one /interact/like-status call per card.
 * Follow state comes from one projects-api call (who the viewer follows).
 *
 * Signed-out visitors who tap the heart or Follow go to the shared sign-in
 * and come back to the same page.
 */
(function () {
  'use strict';

  var PROJECTS_API = (window.KR_PROJECTS_API || 'https://projects.kevsrobots.com');
  var CHATTER = (window.KR_CHATTER_API || 'https://chatter.kevsrobots.com');
  var BATCH = 50;

  var APPS = {
    snakie: { label: 'Snakie', title: 'Wired and coded in Snakie' },
    bettercad: { label: 'BetterCAD', title: 'Designed in BetterCAD' }
  };

  var ICON = {
    heart: '<svg viewBox="0 0 24 24" aria-hidden="true" class="krc-ico krc-heart"><path d="M12 20.6s-7.4-4.5-9.3-9.1C1.3 8 3.5 4.4 7.1 4.4c2 0 3.4 1 4.9 2.8 1.5-1.8 2.9-2.8 4.9-2.8 3.6 0 5.8 3.6 4.4 7.1-1.9 4.6-9.3 9.1-9.3 9.1z"/></svg>',
    comment: '<svg viewBox="0 0 24 24" aria-hidden="true" class="krc-ico"><path d="M21 12a8 8 0 0 1-11.6 7.1L4 20l1-4.6A8 8 0 1 1 21 12z"/></svg>',
    share: '<svg viewBox="0 0 24 24" aria-hidden="true" class="krc-ico"><path d="M12 3v12M7 8l5-5 5 5M5 14v5h14v-5"/></svg>'
  };

  function esc(text) {
    return String(text == null ? '' : text)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
  }

  function viewer() {
    var m = document.cookie.match(/(?:^|;\s*)username=([^;]+)/);
    return m ? decodeURIComponent(m[1]) : null;
  }

  function signIn() {
    window.location.href = '/login?return_to=' + encodeURIComponent(window.location.href);
  }

  function projectsFetch(url, opts) {
    if (window.ProjectAuth && window.ProjectAuth.apiFetch) return window.ProjectAuth.apiFetch(url, opts);
    opts = opts || {};
    opts.credentials = 'include';
    return fetch(url, opts);
  }

  function chatterFetch(path, opts) {
    opts = opts || {};
    opts.credentials = 'include';
    return fetch(CHATTER + path, opts);
  }

  // Chatter keys project likes and comments by this URL (project-interactions.js).
  function likeKey(p) { return 'projects/view.html?id=' + p.id; }

  function projectHref(p) {
    return (p.slug && p.author_username)
      ? '/projects/' + encodeURIComponent(p.author_username) + '/' + encodeURIComponent(p.slug)
      : '/projects/view.html?id=' + encodeURIComponent(p.id);
  }

  function profileHref(username) { return '/@' + encodeURIComponent(username); }

  function hue(text) {
    var h = 0;
    for (var i = 0; i < text.length; i++) h = (h * 31 + text.charCodeAt(i)) % 360;
    return h;
  }

  function cover(p) {
    if (p.cover_image) {
      return '<img src="' + esc(p.cover_image) + '" alt="" loading="lazy" decoding="async" width="400" height="300">';
    }
    var h = hue(p.title || String(p.id));
    var initials = (p.title || 'P').split(/\s+/).map(function (w) { return w.charAt(0); }).join('').slice(0, 2).toUpperCase();
    return '<span class="krc-cover-fallback" style="--krc-h:' + h + '">' + esc(initials) + '</span>';
  }

  function badges(p) {
    var apps = (p.made_with || []).filter(function (a) { return APPS[a]; });
    if (!apps.length) return '';
    return '<span class="krc-made">' + apps.map(function (a) {
      return '<span class="krc-app krc-app-' + a + '" title="' + esc(APPS[a].title) + '">' + esc(APPS[a].label) + '</span>';
    }).join('') + '</span>';
  }

  function avatar(username) {
    return '<span class="krc-avatar" aria-hidden="true" style="--krc-h:' + hue(username) + '">' +
      esc(username.charAt(0).toUpperCase()) + '</span>';
  }

  function html(p, opts) {
    opts = opts || {};
    var showAuthor = opts.showAuthor !== false;
    var me = viewer();
    var author = p.author_username || '';
    var follow = (showAuthor && opts.showFollow !== false && author && author !== me)
      ? '<button type="button" class="krc-follow" data-krc-follow="' + esc(author) + '" aria-pressed="false">Follow</button>'
      : '';
    var by = showAuthor && author
      ? '<div class="krc-by"><a class="krc-author" href="' + profileHref(author) + '">' + avatar(author) +
        '<span class="krc-author-name">' + esc(author) + '</span></a>' + follow + '</div>'
      : '';
    var href = projectHref(p);
    return '' +
      '<article class="krc' + (opts.size === 'lg' ? ' krc-lg' : '') + '" data-krc-key="' + esc(likeKey(p)) + '"' +
        ' data-krc-title="' + esc(p.title) + '" data-krc-href="' + esc(href) + '">' +
        '<a class="krc-cover" href="' + esc(href) + '" tabindex="-1" aria-hidden="true">' + cover(p) + badges(p) + '</a>' +
        '<div class="krc-body">' +
          '<h3 class="krc-title"><a href="' + esc(href) + '">' + esc(p.title) + '</a></h3>' +
          (opts.size === 'lg' && p.short_description ? '<p class="krc-desc">' + esc(p.short_description) + '</p>' : '') +
          by +
          '<div class="krc-acts">' +
            '<button type="button" class="krc-like" aria-pressed="false" aria-label="Like ' + esc(p.title) + '">' +
              ICON.heart + '<span class="krc-like-n">0</span></button>' +
            '<a class="krc-comments" href="' + esc(href) + '#comments" aria-label="Comments">' +
              ICON.comment + '<span class="krc-comment-n">0</span></a>' +
            '<span class="krc-spacer"></span>' +
            '<button type="button" class="krc-share" aria-haspopup="true" aria-expanded="false" aria-label="Share ' + esc(p.title) + '">' +
              ICON.share + '</button>' +
          '</div>' +
        '</div>' +
      '</article>';
  }

  // --- likes ------------------------------------------------------------------

  function paintLike(btn, count, liked) {
    btn.setAttribute('aria-pressed', liked ? 'true' : 'false');
    btn.querySelector('.krc-like-n').textContent = count;
  }

  function applyStatus(cards, items) {
    cards.forEach(function (card) {
      var s = items[card.getAttribute('data-krc-key')];
      if (!s) return;
      paintLike(card.querySelector('.krc-like'), s.like_count || 0, !!s.user_has_liked);
      var cn = card.querySelector('.krc-comment-n');
      if (cn && s.comment_count != null) cn.textContent = s.comment_count;
    });
  }

  function loadOne(card) {
    var key = card.getAttribute('data-krc-key');
    return chatterFetch('/interact/like-status/' + encodeURIComponent(key))
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (d) {
        if (!d) return;
        var items = {};
        items[key] = { like_count: d.like_count || 0, user_has_liked: !!d.user_has_liked };
        applyStatus([card], items);
      })
      .catch(function () {});
  }

  function loadStatus(cards) {
    var chunks = [];
    for (var i = 0; i < cards.length; i += BATCH) chunks.push(cards.slice(i, i + BATCH));
    return Promise.all(chunks.map(function (chunk) {
      var urls = chunk.map(function (c) { return c.getAttribute('data-krc-key'); });
      return chatterFetch('/interact/like-status/batch', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ urls: urls })
      }).then(function (r) {
        if (r.ok) return r.json().then(function (d) { applyStatus(chunk, d.items || {}); });
        if (r.status === 404 || r.status === 405 || r.status === 422) return Promise.all(chunk.map(loadOne));
      }).catch(function () {});
    }));
  }

  function toggleLike(btn) {
    if (!viewer()) { signIn(); return; }
    var card = btn.closest('.krc');
    var was = btn.getAttribute('aria-pressed') === 'true';
    var n = parseInt(btn.querySelector('.krc-like-n').textContent, 10) || 0;
    paintLike(btn, Math.max(0, n + (was ? -1 : 1)), !was);
    btn.disabled = true;
    chatterFetch('/interact/like', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url: card.getAttribute('data-krc-key') })
    }).then(function (r) {
      if (r.status === 401) { signIn(); return null; }
      if (!r.ok) throw new Error('like failed');
      return r.json();
    }).then(function (d) {
      if (d) paintLike(btn, d.like_count, !!d.liked);
      document.dispatchEvent(new CustomEvent('krc:liked', { detail: { key: card.getAttribute('data-krc-key'), liked: d && d.liked } }));
    }).catch(function () {
      paintLike(btn, n, was);
    }).then(function () { btn.disabled = false; });
  }

  // --- follows ----------------------------------------------------------------

  var followingPromise = null;

  function following() {
    var me = viewer();
    if (!me) return Promise.resolve(null);
    if (!followingPromise) {
      followingPromise = projectsFetch(PROJECTS_API + '/api/users/' + encodeURIComponent(me) + '/following?limit=200')
        .then(function (r) { return r.ok ? r.json() : { users: [] }; })
        .then(function (d) { return new Set((d.users || []).map(function (u) { return u.username; })); })
        .catch(function () { return new Set(); });
    }
    return followingPromise;
  }

  function paintFollow(username, on) {
    document.querySelectorAll('.krc-follow').forEach(function (b) {
      if (b.getAttribute('data-krc-follow') !== username) return;
      b.setAttribute('aria-pressed', on ? 'true' : 'false');
      b.textContent = on ? 'Following' : 'Follow';
    });
  }

  function loadFollows(root) {
    var buttons = root.querySelectorAll('.krc-follow');
    if (!buttons.length) return;
    following().then(function (set) {
      if (!set) return;
      buttons.forEach(function (b) {
        paintFollow(b.getAttribute('data-krc-follow'), set.has(b.getAttribute('data-krc-follow')));
      });
    });
  }

  function toggleFollow(btn) {
    if (!viewer()) { signIn(); return; }
    var username = btn.getAttribute('data-krc-follow');
    var was = btn.getAttribute('aria-pressed') === 'true';
    paintFollow(username, !was);
    btn.disabled = true;
    projectsFetch(PROJECTS_API + '/api/users/' + encodeURIComponent(username) + '/follow', {
      method: was ? 'DELETE' : 'POST'
    }).then(function (r) {
      if (r.status === 401) { signIn(); return null; }
      if (!r.ok) throw new Error('follow failed');
      return r.json();
    }).then(function (d) {
      if (!d) return;
      paintFollow(username, !!d.following);
      following().then(function (set) { if (set) { if (d.following) set.add(username); else set.delete(username); } });
      document.dispatchEvent(new CustomEvent('krc:followed', { detail: { username: username, following: !!d.following } }));
    }).catch(function () {
      paintFollow(username, was);
    }).then(function () { btn.disabled = false; });
  }

  // --- share ------------------------------------------------------------------

  function closeMenus(except) {
    document.querySelectorAll('.krc-share-menu').forEach(function (m) {
      if (m === except) return;
      var btn = m.previousElementSibling;
      if (btn) btn.setAttribute('aria-expanded', 'false');
      m.remove();
    });
  }

  function shareLinks(url, title) {
    var u = encodeURIComponent(url);
    var t = encodeURIComponent(title);
    return [
      ['Mastodon', 'https://mastodon.social/share?text=' + t + '%20' + u],
      ['X', 'https://twitter.com/intent/tweet?text=' + t + '&url=' + u],
      ['LinkedIn', 'https://www.linkedin.com/sharing/share-offsite/?url=' + u],
      ['Reddit', 'https://www.reddit.com/submit?url=' + u + '&title=' + t]
    ];
  }

  function copy(text, el) {
    // Where the clipboard is refused, show the link so it can be selected.
    var shown = function () { el.textContent = text; el.classList.add('krc-share-url'); };
    if (!navigator.clipboard || !navigator.clipboard.writeText) { shown(); return; }
    navigator.clipboard.writeText(text).then(function () { el.textContent = 'Link copied'; }, shown);
  }

  function openShare(btn, url, title) {
    if (navigator.share && window.matchMedia('(pointer: coarse)').matches) {
      navigator.share({ title: title, url: url }).catch(function () {});
      return;
    }
    var open = btn.nextElementSibling && btn.nextElementSibling.classList.contains('krc-share-menu');
    closeMenus();
    if (open) return;
    var menu = document.createElement('div');
    menu.className = 'krc-share-menu';
    menu.setAttribute('role', 'menu');
    menu.innerHTML = '<button type="button" role="menuitem" class="krc-share-copy">Copy link</button>' +
      shareLinks(url, title).map(function (l) {
        return '<a role="menuitem" href="' + esc(l[1]) + '" target="_blank" rel="noopener noreferrer">' + l[0] + '</a>';
      }).join('');
    btn.insertAdjacentElement('afterend', menu);
    btn.setAttribute('aria-expanded', 'true');
    var first = menu.querySelector('button');
    first.addEventListener('click', function () { copy(url, first); });
    first.focus();
  }

  function shareCard(btn) {
    var card = btn.closest('.krc');
    var url = new URL(card.getAttribute('data-krc-href'), window.location.origin).href;
    openShare(btn, url, card.getAttribute('data-krc-title'));
  }

  // --- wiring -----------------------------------------------------------------

  var wired = false;

  function wire() {
    if (wired) return;
    wired = true;
    document.addEventListener('click', function (e) {
      var t = e.target.closest ? e.target : e.target.parentElement;
      var like = t.closest('.krc-like');
      if (like) { e.preventDefault(); toggleLike(like); return; }
      var follow = t.closest('.krc-follow');
      if (follow) { e.preventDefault(); toggleFollow(follow); return; }
      var share = t.closest('.krc-share');
      if (share) { e.preventDefault(); shareCard(share); return; }
      if (!t.closest('.krc-share-menu')) closeMenus();
    });
    document.addEventListener('keydown', function (e) { if (e.key === 'Escape') closeMenus(); });
  }

  function hydrate(root) {
    root = root || document;
    wire();
    var cards = Array.prototype.slice.call(root.querySelectorAll('.krc'));
    if (!cards.length) return Promise.resolve();
    loadFollows(root);
    return loadStatus(cards);
  }

  function render(container, projects, opts) {
    if (!container) return Promise.resolve();
    container.innerHTML = (projects || []).map(function (p) { return html(p, opts); }).join('');
    return hydrate(container);
  }

  window.KrProjectCard = {
    html: html,
    hydrate: hydrate,
    render: render,
    likeKey: likeKey,
    projectHref: projectHref,
    profileHref: profileHref,
    openShare: openShare,
    viewer: viewer,
    following: following
  };
})();
