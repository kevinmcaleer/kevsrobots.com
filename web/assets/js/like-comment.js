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
    loadPageViewData(contentUrl);
    loadLikeData(contentUrl);
    loadComments(contentUrl);
  }

  // Load page view data
  async function loadPageViewData(contentUrl) {
    try {
      const response = await fetch(`${CHATTER_API}/analytics/page-views/${encodeURIComponent(contentUrl)}`, {
        credentials: 'include'
      });
      if (response.ok) {
        const data = await response.json();
        const viewCountEl = document.getElementById('view-count');
        if (viewCountEl) {
          viewCountEl.textContent = data.view_count_formatted;
        }
        console.log('[PageView] Loaded view count:', data.view_count_formatted);
      }
    } catch (error) {
      console.error('[PageView] Error loading view data:', error);
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

  // Get current username from cookies
  function getCurrentUsername() {
    const cookies = document.cookie.split(';');
    for (let cookie of cookies) {
      const [name, value] = cookie.trim().split('=');
      if (name === 'username') {
        return decodeURIComponent(value);
      }
    }
    return null;
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

  // Convert @mentions to profile links
  function linkifyMentions(text) {
    // Match @username (alphanumeric and underscore)
    return text.replace(/@(\w+)/g, '<a href="https://www.kevsrobots.com/profile?username=$1" class="text-primary text-decoration-none fw-semibold">@$1</a>');
  }

  // Load like count and user like status (combined into single API call)
  async function loadLikeData(contentUrl) {
    try {
      const response = await fetch(`${CHATTER_API}/interact/like-status/${encodeURIComponent(contentUrl)}`, {
        credentials: 'include'
      });

      if (response.ok) {
        const data = await response.json();

        // Update like count
        const likeCountEl = document.getElementById('like-count');
        if (likeCountEl) {
          likeCountEl.textContent = data.like_count;
        }

        // Update heart icon based on user_has_liked
        if (data.user_has_liked) {
          const outlineHeart = document.querySelector('.heart-outline');
          const filledHeart = document.querySelector('.heart-filled');
          if (outlineHeart) outlineHeart.style.display = 'none';
          if (filledHeart) filledHeart.style.display = 'inline';
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
        const likeCountEl = document.getElementById('like-count');

        // Update UI from response data (no need to fetch again)
        if (data.liked) {
          if (outlineHeart) outlineHeart.style.display = 'none';
          if (filledHeart) filledHeart.style.display = 'inline';
        } else {
          if (outlineHeart) outlineHeart.style.display = 'inline';
          if (filledHeart) filledHeart.style.display = 'none';
        }

        // Update like count from response (eliminates 2 API calls)
        if (likeCountEl && data.like_count !== undefined) {
          likeCountEl.textContent = data.like_count;
        }
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

        const currentUsername = getCurrentUsername();

        comments.forEach(comment => {
          const commentEl = document.createElement('div');
          commentEl.className = 'comment-item mb-3 p-3 border rounded';
          commentEl.dataset.commentId = comment.id;

          // Check if current user is the author
          const isAuthor = currentUsername && currentUsername === comment.username;

          // Debug logging
          console.log(`Comment ${comment.id}: currentUser="${currentUsername}", author="${comment.username}", isAuthor=${isAuthor}`);

          // Build dropdown menu items
          let dropdownItems = '';
          if (isAuthor) {
            dropdownItems += `
              <li><a class="dropdown-item" href="#" onclick="editComment(${comment.id}); return false;">
                <i class="fa-solid fa-pen me-2"></i>Edit
              </a></li>
              <li><a class="dropdown-item text-danger" href="#" onclick="removeComment(${comment.id}); return false;">
                <i class="fa-solid fa-trash me-2"></i>Remove
              </a></li>
            `;
          }
          dropdownItems += `
            <li><a class="dropdown-item" href="#" onclick="reportComment(${comment.id}); return false;">
              <i class="fa-solid fa-flag me-2"></i>Report
            </a></li>
          `;

          // Show "edited" indicator if comment was edited
          const editedIndicator = comment.edited_at ?
            `<span class="text-muted small ms-1">(<a href="#" class="text-decoration-none" onclick="toggleVersionHistory(${comment.id}); return false;">edited ${getRelativeTime(comment.edited_at)}</a>)</span>` : '';

          // Build avatar HTML
          const avatarHtml = comment.profile_picture ?
            `<img src="https://chatter.kevsrobots.com/profile_pictures/${escapeHtml(comment.profile_picture)}" alt="${escapeHtml(comment.username)}" class="rounded-circle me-2" style="width: 32px; height: 32px; object-fit: cover;">` :
            `<div class="rounded-circle bg-secondary d-inline-flex align-items-center justify-content-center text-white me-2" style="width: 32px; height: 32px; font-size: 0.875rem;">${escapeHtml(comment.username[0].toUpperCase())}</div>`;

          commentEl.innerHTML = `
            <div class="d-flex justify-content-between align-items-start">
              <div class="d-flex flex-grow-1">
                ${avatarHtml}
                <div class="flex-grow-1">
                  <div class="d-flex align-items-center mb-1">
                    <a href="https://www.kevsrobots.com/profile?username=${escapeHtml(comment.username)}" class="text-decoration-none me-2">
                      <strong>${escapeHtml(comment.username)}</strong>
                    </a>
                    <span class="text-muted small">${getRelativeTime(comment.created_at)}</span>
                    ${editedIndicator}
                  </div>
                <div class="comment-content">
                  <p class="mb-0 comment-text">${linkifyMentions(escapeHtml(comment.content))}</p>
                </div>
                <div class="version-history mt-2" id="version-history-${comment.id}" style="display: none;">
                  <div class="text-muted small mb-1">
                    <i class="fa-solid fa-clock-rotate-left me-1"></i>Previous versions:
                  </div>
                  <div class="version-history-content" style="max-height: 200px; overflow-y: auto;">
                    <div class="text-muted small">Loading...</div>
                  </div>
                </div>
                </div>
              </div>
              <div class="dropdown">
                <button class="btn btn-link btn-sm text-muted p-0" type="button" data-bs-toggle="dropdown">
                  <i class="fa-solid fa-ellipsis-vertical"></i>
                </button>
                <ul class="dropdown-menu dropdown-menu-end">
                  ${dropdownItems}
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

  // Edit comment
  window.editComment = async function(commentId) {
    if (!isAuthenticated()) {
      window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
      return;
    }

    const commentEl = document.querySelector(`[data-comment-id="${commentId}"]`);
    if (!commentEl) return;

    const contentDiv = commentEl.querySelector('.comment-content');
    const textEl = commentEl.querySelector('.comment-text');
    if (!contentDiv || !textEl) return;

    // Get current text
    const currentText = textEl.textContent;

    // Replace with textarea
    contentDiv.innerHTML = `
      <div class="edit-comment-form">
        <textarea class="form-control mb-2" rows="3" id="edit-textarea-${commentId}">${escapeHtml(currentText)}</textarea>
        <div class="d-flex gap-2">
          <button class="btn btn-sm btn-primary" onclick="saveCommentEdit(${commentId}); return false;">Save</button>
          <button class="btn btn-sm btn-secondary" onclick="cancelCommentEdit(${commentId}, '${escapeHtml(currentText).replace(/'/g, "\\'")}'); return false;">Cancel</button>
        </div>
        <div id="edit-error-${commentId}" class="alert alert-danger mt-2" style="display: none;"></div>
      </div>
    `;

    // Focus the textarea
    const textarea = document.getElementById(`edit-textarea-${commentId}`);
    if (textarea) {
      textarea.focus();
      textarea.setSelectionRange(textarea.value.length, textarea.value.length);
    }
  };

  // Save comment edit
  window.saveCommentEdit = async function(commentId) {
    const textarea = document.getElementById(`edit-textarea-${commentId}`);
    if (!textarea) return;

    const newContent = textarea.value.trim();
    if (!newContent) {
      alert('Comment cannot be empty');
      return;
    }

    // Check for URLs (client-side warning)
    const urlPattern = /https?:\/\/|www\.|\.com|\.net|\.org|\.io/i;
    if (urlPattern.test(newContent)) {
      alert('Comments cannot contain URLs or links to other sites.');
      return;
    }

    try {
      const response = await fetch(`${CHATTER_API}/interact/comments/${commentId}`, {
        method: 'PUT',
        credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ content: newContent })
      });

      if (response.ok) {
        const likeSection = document.querySelector('.like-comment-section');
        const contentUrl = likeSection ? likeSection.dataset.url : null;
        if (contentUrl) {
          loadComments(contentUrl);
        }
      } else {
        const error = await response.json();
        const errorEl = document.getElementById(`edit-error-${commentId}`);
        if (errorEl) {
          errorEl.textContent = error.detail || 'Error updating comment';
          errorEl.style.display = 'block';
        }
      }
    } catch (error) {
      console.error('Error updating comment:', error);
      const errorEl = document.getElementById(`edit-error-${commentId}`);
      if (errorEl) {
        errorEl.textContent = 'Error updating comment';
        errorEl.style.display = 'block';
      }
    }
  };

  // Cancel comment edit
  window.cancelCommentEdit = function(commentId, originalText) {
    const commentEl = document.querySelector(`[data-comment-id="${commentId}"]`);
    if (!commentEl) return;

    const contentDiv = commentEl.querySelector('.comment-content');
    if (!contentDiv) return;

    // Restore original text
    contentDiv.innerHTML = `<p class="mb-0 comment-text">${originalText}</p>`;
  };

  // Toggle version history display
  window.toggleVersionHistory = async function(commentId) {
    const versionHistoryEl = document.getElementById(`version-history-${commentId}`);
    if (!versionHistoryEl) return;

    // Toggle visibility
    if (versionHistoryEl.style.display === 'none') {
      versionHistoryEl.style.display = 'block';

      // Load versions if not already loaded
      const contentEl = versionHistoryEl.querySelector('.version-history-content');
      if (contentEl && contentEl.textContent.includes('Loading')) {
        try {
          const response = await fetch(`${CHATTER_API}/interact/comments/${commentId}/versions`, {
            credentials: 'include'
          });

          if (response.ok) {
            const versions = await response.json();

            if (versions.length === 0) {
              contentEl.innerHTML = '<div class="text-muted small">No previous versions</div>';
            } else {
              contentEl.innerHTML = versions.map(version => `
                <div class="border-start border-2 ps-2 mb-2">
                  <div class="text-muted small mb-1">
                    ${new Date(version.edited_at).toLocaleString()}
                  </div>
                  <div class="small">${escapeHtml(version.content)}</div>
                </div>
              `).join('');
            }
          } else {
            contentEl.innerHTML = '<div class="text-danger small">Error loading versions</div>';
          }
        } catch (error) {
          console.error('Error loading version history:', error);
          contentEl.innerHTML = '<div class="text-danger small">Error loading versions</div>';
        }
      }
    } else {
      versionHistoryEl.style.display = 'none';
    }
  };

  // Remove comment
  window.removeComment = async function(commentId) {
    if (!isAuthenticated()) {
      window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
      return;
    }

    // Confirm removal
    if (!confirm('Are you sure you want to remove this comment? This action cannot be undone.')) {
      return;
    }

    try {
      const response = await fetch(`${CHATTER_API}/interact/comments/${commentId}`, {
        method: 'DELETE',
        credentials: 'include'
      });

      if (response.ok) {
        // Reload comments to remove the deleted one from view
        const likeSection = document.querySelector('.like-comment-section');
        const contentUrl = likeSection ? likeSection.dataset.url : null;
        if (contentUrl) {
          loadComments(contentUrl);
        }
      } else {
        const error = await response.json();
        alert(error.detail || 'Error removing comment');
      }
    } catch (error) {
      console.error('Error removing comment:', error);
      alert('Error removing comment');
    }
  };

  // Report comment (placeholder)
  window.reportComment = function(commentId) {
    if (!isAuthenticated()) {
      window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
      return;
    }
    alert('Report functionality coming soon (Issue #35)');
  };

})();
