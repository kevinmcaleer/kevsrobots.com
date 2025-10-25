/**
 * Like and Comment Summary Component
 * Loads like and comment counts for cards/galleries
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
    // Find all summary widgets on the page
    const summaries = document.querySelectorAll('.like-comment-summary');

    if (summaries.length === 0) return;

    // Load data for each summary
    summaries.forEach(summary => {
      const url = summary.dataset.summaryUrl;
      if (url) {
        loadSummaryData(summary, url);
      }
    });
  }

  async function loadSummaryData(summaryElement, contentUrl) {
    try {
      // Load view count, like count and comment count in parallel
      const [viewResponse, likeResponse, commentResponse] = await Promise.all([
        fetch(`${CHATTER_API}/analytics/page-views/${encodeURIComponent(contentUrl)}`, {
          credentials: 'include'
        }),
        fetch(`${CHATTER_API}/interact/likes/${encodeURIComponent(contentUrl)}`, {
          credentials: 'include'
        }),
        fetch(`${CHATTER_API}/interact/comments/${encodeURIComponent(contentUrl)}`, {
          credentials: 'include'
        })
      ]);

      // Update view count
      if (viewResponse.ok) {
        const viewData = await viewResponse.json();
        const viewCountEl = summaryElement.querySelector('.view-count-summary');
        if (viewCountEl) {
          viewCountEl.textContent = viewData.view_count_formatted || '0';
        }
      }

      // Update like count
      if (likeResponse.ok) {
        const likeData = await likeResponse.json();
        const likeCountEl = summaryElement.querySelector('.like-count-summary');
        if (likeCountEl) {
          likeCountEl.textContent = likeData.like_count || 0;
        }
      }

      // Update comment count
      if (commentResponse.ok) {
        const comments = await commentResponse.json();
        const commentCountEl = summaryElement.querySelector('.comment-count-summary');
        if (commentCountEl) {
          commentCountEl.textContent = comments.length || 0;
        }
      }
    } catch (error) {
      console.error('Error loading summary data for', contentUrl, error);
      // Keep the summary visible even on error (shows 0s)
    }
  }

})();
