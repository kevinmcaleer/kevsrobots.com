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
      return;
    }

    if (isAuthenticated()) {
      // User is logged in - show user menu
      const username = getUsername();

      if (username) {
        usernameDisplay.textContent = username;
        guestDropdown.style.display = 'none';
        userDropdown.style.display = 'inline-flex';
      } else {
        // Failed to get username, show guest menu
        guestDropdown.style.display = 'inline-flex';
        userDropdown.style.display = 'none';
      }
    } else {
      // User is not logged in - show guest menu
      guestDropdown.style.display = 'inline-flex';
      userDropdown.style.display = 'none';
    }
  }

  /**
   * Handle logout action
   */
  async function handleLogout(event) {
    event.preventDefault();

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
      loginLink.href = `${CHATTER_API}/login?return_to=${currentURL}`;
    }

    if (registerLink) {
      registerLink.href = `${CHATTER_API}/register?return_to=${currentURL}`;
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
