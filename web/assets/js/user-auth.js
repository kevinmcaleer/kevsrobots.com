/**
 * User Authentication Menu Handler
 * Manages user login state display and logout functionality
 */

(function() {
  'use strict';

  const CHATTER_API = 'https://chatter.kevsrobots.com';
  const IS_LOCAL = window.location.hostname === 'local.kevsrobots.com';

  /**
   * Check if user is authenticated by checking for access_token OR username cookie
   * We check username cookie because httponly cookies aren't accessible to JavaScript
   */
  function isAuthenticated() {
    const cookies = document.cookie.split(';');
    for (let cookie of cookies) {
      const [name, value] = cookie.trim().split('=');
      // Check for username cookie (since access_token is httponly, we can't read it)
      if (name === 'username' && value) {
        return true;
      }
    }
    return false;
  }

  /**
   * Get username from the username cookie
   */
  function getUsername() {
    const cookies = document.cookie.split(';');
    for (let cookie of cookies) {
      const [name, value] = cookie.trim().split('=');
      if (name === 'username' && value) {
        return decodeURIComponent(value);
      }
    }
    return null;
  }

  /**
   * Update menu display based on authentication state
   */
  function updateMenu() {
    const guestDropdown = document.getElementById('navbarDropdownGuest');
    const userDropdown = document.getElementById('navbarDropdownUser');
    const usernameDisplay = document.getElementById('username-display');

    // Check if elements exist
    if (!guestDropdown || !userDropdown) {
      console.error('Navigation elements not found');
      return;
    }

    const authenticated = isAuthenticated();
    const username = getUsername();

    console.log('updateMenu called:', { authenticated, username });

    if (authenticated && username) {
      // User is logged in - show user menu
      console.log('Showing user menu for:', username);
      usernameDisplay.textContent = username;
      checkAdmin(username);
      guestDropdown.style.setProperty('display', 'none', 'important');
      userDropdown.style.setProperty('display', 'block', 'important');

      // Verify after setting
      console.log('After setting - Guest display:', guestDropdown.style.display);
      console.log('After setting - User display:', userDropdown.style.display);
    } else {
      // User is not logged in - show guest menu
      console.log('Showing guest menu');
      guestDropdown.style.setProperty('display', 'block', 'important');
      userDropdown.style.setProperty('display', 'none', 'important');

      // Verify after setting
      console.log('After setting - Guest display:', guestDropdown.style.display);
      console.log('After setting - User display:', userDropdown.style.display);
    }
  }

  /**
   * Show "Admin console" in the user menu for admins. The Projects API's
   * /api/auth/me says whether the user is an admin; the answer is kept
   * for the browser session so other pages don't ask again.
   */
  const PROJECTS_API = 'https://projects.kevsrobots.com';

  function showAdminLink(show) {
    const item = document.getElementById('admin-console-item');
    if (item) item.hidden = !show;
  }

  function adminCacheKey(username) {
    return 'kr_is_admin:' + username;
  }

  function readAdminCache(username) {
    try { return sessionStorage.getItem(adminCacheKey(username)); } catch (e) { return null; }
  }

  function writeAdminCache(username, isAdmin) {
    try { sessionStorage.setItem(adminCacheKey(username), isAdmin ? '1' : '0'); } catch (e) {}
  }

  async function checkAdmin(username) {
    const cached = readAdminCache(username);
    if (cached !== null) {
      showAdminLink(cached === '1');
      return;
    }
    try {
      const opts = { credentials: 'include' };
      let token = null;
      try { token = localStorage.getItem('dev_jwt_token'); } catch (e) {}
      if (token) opts.headers = { Authorization: 'Bearer ' + token };
      const r = await fetch(PROJECTS_API + '/api/auth/me', opts);
      if (!r.ok) return; // don't cache a failure; try again on the next page
      const me = await r.json();
      const isAdmin = !!(me && me.is_admin === true);
      writeAdminCache(username, isAdmin);
      showAdminLink(isAdmin);
    } catch (e) {
      // The API is unreachable: leave the link hidden.
    }
  }

  /**
   * Handle logout action
   */
  async function handleLogout(event) {
    event.preventDefault();

    const username = getUsername();
    if (username) {
      try { sessionStorage.removeItem(adminCacheKey(username)); } catch (e) {}
    }

    try {
      // Call logout endpoint
      const response = await fetch(`${CHATTER_API}/logout`, {
        credentials: 'include'
      });

      if (response.ok || response.redirected) {
        // Clear local state and redirect to home
        window.location.href = '/';
      } else {
        console.error('Logout failed');
        // Still redirect to home even if logout fails
        window.location.href = '/';
      }
    } catch (error) {
      console.error('Error during logout:', error);
      // Redirect anyway
      window.location.href = '/';
    }
  }

  /**
   * Add return URL to login and register links
   */
  function setupReturnURL() {
    const loginLink = document.getElementById('login-link');
    const registerLink = document.getElementById('register-link');
    const currentURL = encodeURIComponent(window.location.href);

    if (loginLink) {
      const newHref = `/login?return_to=${currentURL}`;
      console.log('Setting login link href from:', loginLink.href, 'to:', newHref);
      loginLink.href = newHref;
      console.log('After setting, href is:', loginLink.href);
    }

    if (registerLink) {
      const newHref = `/register?return_to=${currentURL}`;
      console.log('Setting register link href from:', registerLink.href, 'to:', newHref);
      registerLink.href = newHref;
      console.log('After setting, href is:', registerLink.href);
    }
  }

  /**
   * Initialize menu when DOM is ready
   */
  function init() {
    // Update menu on page load
    updateMenu();

    // Setup return URLs for login/register links
    setupReturnURL();

    // Attach logout handler
    const logoutBtn = document.getElementById('logout-btn');
    if (logoutBtn) {
      logoutBtn.addEventListener('click', handleLogout);
    }
  }

  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

})();
