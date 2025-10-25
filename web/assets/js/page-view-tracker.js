/**
 * Universal Page View Tracker
 * Logs page views for all pages on kevsrobots.com
 * Runs independently of other components
 */

(function() {
  'use strict';

  const CHATTER_API = 'https://chatter.kevsrobots.com';

  // Get current page URL (remove leading slash for consistency with API)
  const currentPath = window.location.pathname.substring(1) || 'index.html';

  // Log page view immediately when script loads
  logPageView(currentPath);

  async function logPageView(url) {
    try {
      const response = await fetch(`${CHATTER_API}/analytics/page-view`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          url: url,
          ip_address: 'server-detected', // Server will extract real IP
          user_agent: navigator.userAgent
        })
      });

      if (response.ok) {
        const data = await response.json();
        console.log('[Analytics] Page view logged:', data.url);
      } else {
        console.warn('[Analytics] Failed to log page view:', response.status);
      }
    } catch (error) {
      // Silently fail - analytics should never break the user experience
      console.debug('[Analytics] Page view logging error:', error);
    }
  }

})();
