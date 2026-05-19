/**
 * Project Interactions — comments and likes via Chatter API.
 * Depends on ProjectAuth (project-auth.js) for authenticated requests.
 */
var ProjectInteractions = (function () {
  var CHATTER = 'https://chatter.kevsrobots.com';

  /* ── helpers ────────────────────────────────────────────────── */

  function esc(text) {
    var d = document.createElement('div');
    d.textContent = text || '';
    return d.innerHTML;
  }

  function relativeTime(dateStr) {
    var now = Date.now();
    var then = new Date(dateStr).getTime();
    var diff = Math.floor((now - then) / 1000);
    if (diff < 60) return 'just now';
    if (diff < 3600) return Math.floor(diff / 60) + 'm ago';
    if (diff < 86400) return Math.floor(diff / 3600) + 'h ago';
    if (diff < 2592000) return Math.floor(diff / 86400) + 'd ago';
    return new Date(dateStr).toLocaleDateString();
  }

  function chatterFetch(path, opts) {
    opts = opts || {};
    opts.credentials = 'include';
    // DO NOT add `Authorization: Bearer <ProjectAuth.getDevToken()>` here.
    // That token is signed with the projects-api secret, not Chatter's, so
    // Chatter rejects it as an invalid Authorization header and (worse)
    // doesn't fall back to the session cookie — comments end up saved as
    // anonymous instead of the logged-in user. The Chatter cookie set on
    // .kevsrobots.com is the only identity Chatter accepts; cookies travel
    // via `credentials: 'include'` already.
    return fetch(CHATTER + path, opts);
  }

  /* ── likes ──────────────────────────────────────────────────── */

  function loadLikes(projectUrl, container) {
    if (!container) return;
    // Chatter's `/interact/likes/<key>` returns `{like_count}` only —
    // anonymous-safe but no user_has_liked. Use `/interact/like-status/`
    // which returns both `{like_count, user_has_liked}` in one call so the
    // heart renders in the right state on first paint.
    chatterFetch('/interact/like-status/' + encodeURIComponent(projectUrl))
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (data) {
        if (!data) return;
        var count = (data.like_count != null) ? data.like_count : (data.count || 0);
        var liked = (data.user_has_liked != null) ? data.user_has_liked
                  : (data.liked != null) ? data.liked : !!data.user_liked;
        renderLikeButton(container, projectUrl, count, liked);
      })
      .catch(function () {
        // Chatter unreachable — show a static heart with 0
        renderLikeButton(container, projectUrl, 0, false);
      });
  }

  function renderLikeButton(container, projectUrl, count, liked) {
    container.innerHTML =
      '<button class="btn btn-sm ' + (liked ? 'btn-danger' : 'btn-outline-danger') + ' like-btn" title="Like this project">' +
      '<i class="' + (liked ? 'fas' : 'far') + ' fa-heart"></i> ' +
      '<span class="like-count">' + count + '</span></button>';

    var btn = container.querySelector('.like-btn');
    btn.addEventListener('click', function () {
      toggleLike(projectUrl, container);
    });
  }

  function toggleLike(projectUrl, container) {
    var btn = container.querySelector('.like-btn');
    if (btn) {
      btn.disabled = true;
      btn.classList.add('like-pulse');
    }
    // Chatter's toggle endpoint is POST /interact/like (singular) with the
    // URL in the JSON body — NOT POST /interact/likes/<key>, which only
    // accepts GET and returns 405 on POST.
    chatterFetch('/interact/like', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url: projectUrl })
    })
      .then(function (r) {
        if (r.status === 401 || r.status === 403) {
          alert('Please log in to like projects.');
          return null;
        }
        return r.ok ? r.json() : null;
      })
      .then(function (data) {
        if (!data) { if (btn) btn.disabled = false; return; }
        // Chatter returns { like_count, liked } — accept the older
        // { count, user_liked } shape too in case anything else uses it.
        var count = (data.like_count != null) ? data.like_count : (data.count || 0);
        var liked = (data.liked != null) ? data.liked : !!data.user_liked;
        renderLikeButton(container, projectUrl, count, liked);
      })
      .catch(function () { if (btn) btn.disabled = false; });
  }

  /**
   * Fetch just the like count for a project URL (used on hub cards).
   * Returns a Promise that resolves to { count: N }.
   */
  function getLikeCount(projectUrl) {
    return chatterFetch('/interact/likes/' + encodeURIComponent(projectUrl))
      .then(function (r) { return r.ok ? r.json() : { count: 0 }; })
      .catch(function () { return { count: 0 }; });
  }

  /* ── comments ───────────────────────────────────────────────── */

  function loadComments(projectUrl, container) {
    if (!container) return;
    container.innerHTML =
      '<div class="text-center py-3"><div class="spinner-border spinner-border-sm text-secondary"></div></div>';

    chatterFetch('/interact/comments/' + encodeURIComponent(projectUrl))
      .then(function (r) { return r.ok ? r.json() : []; })
      .then(function (comments) { renderComments(container, projectUrl, comments || []); })
      .catch(function () { renderComments(container, projectUrl, []); });
  }

  function renderComments(container, projectUrl, comments) {
    // Build a tree from flat list
    var byId = {};
    var roots = [];
    comments.forEach(function (c) { byId[c.id] = c; c._children = []; });
    comments.forEach(function (c) {
      if (c.parent_comment_id && byId[c.parent_comment_id]) {
        byId[c.parent_comment_id]._children.push(c);
      } else {
        roots.push(c);
      }
    });

    // Sort newest first at root level
    roots.sort(function (a, b) { return new Date(b.created_at) - new Date(a.created_at); });

    var html = '';

    // Post form
    html += commentForm(projectUrl, null);

    // Count
    html += '<p class="text-muted small mb-3">' + comments.length + ' comment' + (comments.length !== 1 ? 's' : '') + '</p>';

    if (roots.length === 0) {
      html += '<p class="text-muted">No comments yet. Be the first!</p>';
    } else {
      roots.forEach(function (c) {
        html += renderCommentThread(c, projectUrl, 0);
      });
    }

    container.innerHTML = html;

    // Attach event handlers
    container.querySelectorAll('.comment-form').forEach(function (form) {
      form.addEventListener('submit', function (e) {
        e.preventDefault();
        var textarea = form.querySelector('textarea');
        var parentId = form.dataset.parentId || null;
        var content = textarea.value.trim();
        if (!content) return;
        var submitBtn = form.querySelector('button[type="submit"]');
        submitBtn.disabled = true;
        submitBtn.textContent = 'Posting...';
        postComment(projectUrl, content, parentId)
          .then(function (ok) {
            if (ok) {
              loadComments(projectUrl, container);
            } else {
              submitBtn.disabled = false;
              submitBtn.textContent = 'Post';
            }
          });
      });
    });

    // Reply toggle buttons
    container.querySelectorAll('.reply-toggle').forEach(function (btn) {
      btn.addEventListener('click', function () {
        var replyForm = container.querySelector('#reply-form-' + btn.dataset.commentId);
        if (replyForm) {
          replyForm.classList.toggle('d-none');
          if (!replyForm.classList.contains('d-none')) {
            replyForm.querySelector('textarea').focus();
          }
        }
      });
    });
  }

  function renderCommentThread(comment, projectUrl, depth) {
    var indent = depth > 0 ? ' ms-4 border-start ps-3' : '';
    // Chatter returns `username` (not `author_username`) on its
    // /interact/comments response. Accept both shapes so this works if
    // Chatter ever standardises the field name.
    var who = comment.username || comment.author_username || comment.author;
    var authorName = who || 'Anonymous';
    // Issue #111: link author to their profile when we have a username.
    var authorHtml = who
      ? '<a href="/profile/?u=' + encodeURIComponent(who) +
        '" class="text-decoration-none"><strong class="small">' + esc(authorName) + '</strong></a>'
      : '<strong class="small">' + esc(authorName) + '</strong>';
    var html =
      '<div class="comment-thread mb-3' + indent + '">' +
        '<div class="d-flex align-items-center mb-1">' +
          authorHtml +
          '<span class="text-muted small ms-2">' + relativeTime(comment.created_at) + '</span>' +
        '</div>' +
        '<div class="small mb-1">' + esc(comment.content) + '</div>' +
        '<button class="btn btn-link btn-sm p-0 text-muted reply-toggle" data-comment-id="' + comment.id + '">' +
          '<i class="fas fa-reply fa-xs"></i> Reply' +
        '</button>' +
        '<div id="reply-form-' + comment.id + '" class="d-none mt-2">' +
          commentForm(projectUrl, comment.id) +
        '</div>';

    // Render children
    if (comment._children && comment._children.length) {
      comment._children.forEach(function (child) {
        html += renderCommentThread(child, projectUrl, depth + 1);
      });
    }

    html += '</div>';
    return html;
  }

  function commentForm(projectUrl, parentId) {
    return (
      '<form class="comment-form mb-3" data-parent-id="' + (parentId || '') + '">' +
        '<div class="input-group">' +
          '<textarea class="form-control form-control-sm" rows="2" placeholder="Write a comment..." required></textarea>' +
        '</div>' +
        '<button type="submit" class="btn btn-sm btn-primary mt-1"><i class="fas fa-paper-plane me-1"></i> Post</button>' +
      '</form>'
    );
  }

  function postComment(projectUrl, content, parentId) {
    var body = { url: projectUrl, content: content };
    if (parentId) body.parent_comment_id = parentId;
    // Chatter accepts POST /interact/comment (singular) — the plural form
    // /interact/comments is GET-only (used for listing).
    return chatterFetch('/interact/comment', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    })
      .then(function (r) {
        if (r.status === 401 || r.status === 403) {
          alert('Please log in to post a comment.');
          return false;
        }
        return r.ok;
      })
      .catch(function () {
        alert('Could not post comment. Please try again later.');
        return false;
      });
  }

  /* ── public API ─────────────────────────────────────────────── */

  return {
    loadComments: loadComments,
    postComment: postComment,
    loadLikes: loadLikes,
    toggleLike: toggleLike,
    getLikeCount: getLikeCount
  };
})();
