/**
 * Like and Comment Component JavaScript
 * Handles like/unlike and comment posting functionality
 */

(function() {
  'use strict';

  const CHATTER_API = 'https://chatter.kevsrobots.com';

  // Wait for DOM to be ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

  function init() {
    const likeSection = document.querySelector('.like-comment-section');
    if (!likeSection) return; // Component not on this page

    const contentUrl = likeSection.dataset.url;
    let currentUserAuthenticated = false;
    let currentLikeId = null;

    // Check if user is authenticated
    currentUserAuthenticated = isAuthenticated();

    // Event listeners
    const likeBtn = document.getElementById('like-btn');
    const postCommentBtn = document.getElementById('post-comment-btn');
    const commentTextarea = document.getElementById('comment-textarea');

    if (likeBtn) likeBtn.addEventListener('click', () => toggleLike(contentUrl));
    if (postCommentBtn) postCommentBtn.addEventListener('click', () => postComment(contentUrl));

    if (commentTextarea) {
      commentTextarea.addEventListener('input', function() {
        if (postCommentBtn) {
          postCommentBtn.disabled = !this.value.trim();
        }
      });
    }

    // Load initial data
    loadLikeData(contentUrl);
    loadComments(contentUrl);
    logPageView(contentUrl);
  }

  // Log page view
  async function logPageView(contentUrl) {
    console.log('[PageView] Attempting to log page view for:', contentUrl);
    try {
      // Get IP address will be handled server-side via X-Forwarded-For or request.client.host
      const response = await fetch(`${CHATTER_API}/analytics/page-view`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          url: contentUrl,
          ip_address: 'server-detected', // Server will override this
          user_agent: navigator.userAgent
        })
      });
      if (response.ok) {
        const data = await response.json();
        console.log('[PageView] Successfully logged:', data);
      } else {
        console.error('[PageView] Failed with status:', response.status);
      }
    } catch (error) {
      // Silently fail - page view logging is not critical
      console.error('[PageView] Error logging page view:', error);
    }
  }

  // Check if user is authenticated
  function isAuthenticated() {
    const cookies = document.cookie.split(';');
    for (let cookie of cookies) {
      const [name, value] = cookie.trim().split('=');
      if (name === 'username' && value) {
        return true;
      }
    }
    return false;
  }

  // Get relative time string
  function getRelativeTime(dateString) {
    const now = new Date();
    const commentDate = new Date(dateString);
    const diffMs = now - commentDate;
    const diffSec = Math.floor(diffMs / 1000);
    const diffMin = Math.floor(diffSec / 60);
    const diffHour = Math.floor(diffMin / 60);
    const diffDay = Math.floor(diffHour / 24);
    const diffWeek = Math.floor(diffDay / 7);
    const diffMonth = Math.floor(diffDay / 30);
    const diffYear = Math.floor(diffDay / 365);

    if (diffSec < 60) return 'just now';
    if (diffMin < 60) return `${diffMin}m`;
    if (diffHour < 24) return `${diffHour}h`;
    if (diffDay < 7) return `${diffDay}d`;
    if (diffWeek < 4) return `${diffWeek}w`;
    if (diffMonth < 12) return `${diffMonth}mo`;
    return `${diffYear}y`;
  }

  // Load like count and user like status
  async function loadLikeData(contentUrl) {
    try {
      const countResponse = await fetch(`${CHATTER_API}/interact/likes/${encodeURIComponent(contentUrl)}`, {
        credentials: 'include'
      });
      const countData = await countResponse.json();
      const likeCountEl = document.getElementById('like-count');
      if (likeCountEl) {
        likeCountEl.textContent = countData.like_count;
      }

      // If user is authenticated, check if they've liked this
      if (isAuthenticated()) {
        try {
          const statusResponse = await fetch(`${CHATTER_API}/interact/user-like-status/${encodeURIComponent(contentUrl)}`, {
            credentials: 'include'
          });
          if (statusResponse.ok) {
            const statusData = await statusResponse.json();
            if (statusData.user_has_liked) {
              // Show filled heart
              const outlineHeart = document.querySelector('.heart-outline');
              const filledHeart = document.querySelector('.heart-filled');
              if (outlineHeart) outlineHeart.style.display = 'none';
              if (filledHeart) filledHeart.style.display = 'inline';
            }
          }
        } catch (err) {
          console.log('Could not check user like status:', err);
        }
      }
    } catch (error) {
      console.error('Error loading like data:', error);
    }
  }

  // Toggle like/unlike
  async function toggleLike(contentUrl) {
    if (!isAuthenticated()) {
      window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
      return;
    }

    try {
      const response = await fetch(`${CHATTER_API}/interact/like`, {
        method: 'POST',
        credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ url: contentUrl })
      });

      if (response.ok) {
        const data = await response.json();
        const outlineHeart = document.querySelector('.heart-outline');
        const filledHeart = document.querySelector('.heart-filled');

        if (data.liked) {
          if (outlineHeart) outlineHeart.style.display = 'none';
          if (filledHeart) filledHeart.style.display = 'inline';
        } else {
          if (outlineHeart) outlineHeart.style.display = 'inline';
          if (filledHeart) filledHeart.style.display = 'none';
        }

        loadLikeData(contentUrl);
      }
    } catch (error) {
      console.error('Error toggling like:', error);
    }
  }

  // Load comments
  async function loadComments(contentUrl) {
    try {
      const response = await fetch(`${CHATTER_API}/interact/comments/${encodeURIComponent(contentUrl)}`, {
        credentials: 'include'
      });

      if (response.ok) {
        const comments = await response.json();
        const container = document.getElementById('comments-container');
        const loading = document.getElementById('comments-loading');
        const noComments = document.getElementById('no-comments');

        if (loading) loading.style.display = 'none';

        if (comments.length === 0) {
          if (noComments) noComments.style.display = 'block';
          return;
        }

        if (noComments) noComments.style.display = 'none';
        if (container) container.innerHTML = '';

        comments.forEach(comment => {
          const commentEl = document.createElement('div');
          commentEl.className = 'comment-item mb-3 p-3 border rounded';
          commentEl.innerHTML = `
            <div class="d-flex justify-content-between align-items-start">
              <div class="flex-grow-1">
                <div class="d-flex align-items-center mb-1">
                  <strong class="me-2">${escapeHtml(comment.username)}</strong>
                  <span class="text-muted small">${getRelativeTime(comment.created_at)}</span>
                </div>
                <p class="mb-0">${escapeHtml(comment.content)}</p>
              </div>
              <div class="dropdown">
                <button class="btn btn-link btn-sm text-muted p-0" type="button" data-bs-toggle="dropdown">
                  <i class="fa-solid fa-ellipsis-vertical"></i>
                </button>
                <ul class="dropdown-menu dropdown-menu-end">
                  <li><a class="dropdown-item" href="#" onclick="reportComment(${comment.id}); return false;">
                    <i class="fa-solid fa-flag me-2"></i>Report
                  </a></li>
                </ul>
              </div>
            </div>
          `;
          if (container) container.appendChild(commentEl);
        });
      }
    } catch (error) {
      console.error('Error loading comments:', error);
      const loading = document.getElementById('comments-loading');
      if (loading) loading.textContent = 'Error loading comments';
    }
  }

  // Post comment
  async function postComment(contentUrl) {
    if (!isAuthenticated()) {
      window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
      return;
    }

    const textarea = document.getElementById('comment-textarea');
    const content = textarea ? textarea.value.trim() : '';

    if (!content) return;

    // Check for URLs (client-side warning)
    const urlPattern = /https?:\/\/|www\.|\.com|\.net|\.org|\.io/i;
    if (urlPattern.test(content)) {
      alert('Comments cannot contain URLs or links to other sites.');
      return;
    }

    try {
      const response = await fetch(`${CHATTER_API}/interact/comment`, {
        method: 'POST',
        credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ url: contentUrl, content: content })
      });

      if (response.ok) {
        if (textarea) textarea.value = '';
        const postBtn = document.getElementById('post-comment-btn');
        if (postBtn) postBtn.disabled = true;

        const successEl = document.getElementById('comment-success');
        if (successEl) {
          successEl.textContent = 'Comment posted successfully!';
          successEl.style.display = 'block';
          setTimeout(() => { successEl.style.display = 'none'; }, 3000);
        }

        loadComments(contentUrl);
      } else {
        const error = await response.json();
        const errorEl = document.getElementById('comment-error');
        if (errorEl) {
          errorEl.textContent = error.detail || 'Error posting comment';
          errorEl.style.display = 'block';
        }
      }
    } catch (error) {
      console.error('Error posting comment:', error);
      const errorEl = document.getElementById('comment-error');
      if (errorEl) {
        errorEl.textContent = 'Error posting comment';
        errorEl.style.display = 'block';
      }
    }
  }

  // Escape HTML
  function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
  }

  // Report comment (placeholder)
  window.reportComment = function(commentId) {
    if (!isAuthenticated()) {
      window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
      return;
    }
    alert('Report functionality coming soon (Issue #35)');
  };

})();
