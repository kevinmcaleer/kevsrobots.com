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
      // Load both like count and comment count in parallel
      const [likeResponse, commentResponse] = await Promise.all([
        fetch(`${CHATTER_API}/interact/likes/${encodeURIComponent(contentUrl)}`, {
          credentials: 'include'
        }),
        fetch(`${CHATTER_API}/interact/comments/${encodeURIComponent(contentUrl)}`, {
          credentials: 'include'
        })
      ]);

      if (likeResponse.ok && commentResponse.ok) {
        const likeData = await likeResponse.json();
        const comments = await commentResponse.json();

        // Update the summary display
        const likeCountEl = summaryElement.querySelector('.like-count-summary');
        const commentCountEl = summaryElement.querySelector('.comment-count-summary');

        if (likeCountEl) {
          likeCountEl.textContent = likeData.like_count || 0;
        }

        if (commentCountEl) {
          commentCountEl.textContent = comments.length || 0;
        }

        // Hide the summary if there are no likes and no comments
        if ((likeData.like_count === 0 || !likeData.like_count) &&
            (comments.length === 0 || !comments.length)) {
          summaryElement.style.display = 'none';
        }
      }
    } catch (error) {
      console.error('Error loading summary data for', contentUrl, error);
      // Hide the summary on error
      summaryElement.style.display = 'none';
    }
  }

})();
