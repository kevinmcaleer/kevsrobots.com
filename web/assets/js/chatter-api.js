/**
 * Chatter API Library
 * JavaScript client for interacting with the Chatter authentication API
 */

const ChatterAPI = (function() {
  'use strict';

  const API_BASE = 'https://chatter.kevsrobots.com';
  const IS_LOCAL = window.location.hostname === 'local.kevsrobots.com';

  /**
   * Make an API request
   * @param {string} endpoint - API endpoint (e.g., '/api/login')
   * @param {object} options - Fetch options
   * @returns {Promise<object>} Response data
   */
  async function apiRequest(endpoint, options = {}) {
    const url = `${API_BASE}${endpoint}`;

    const defaultOptions = {
      credentials: 'include', // Send cookies
      headers: {
        'Content-Type': 'application/json',
        ...options.headers
      }
    };

    const response = await fetch(url, { ...defaultOptions, ...options });

    // Handle redirects (shouldn't happen for API endpoints, but just in case)
    if (response.redirected) {
      window.location.href = response.url;
      return;
    }

    const data = await response.json();

    if (!response.ok) {
      throw { status: response.status, ...data };
    }

    return data;
  }

  /**
   * Login user
   * @param {string} username
   * @param {string} password
   * @returns {Promise<object>} Access token and user info
   */
  async function login(username, password) {
    // API endpoint expects form data for OAuth2
    const formData = new URLSearchParams();
    formData.append('username', username);
    formData.append('password', password);

    const response = await fetch(`${API_BASE}/api/login`, {
      method: 'POST',
      credentials: 'include',
      headers: {
        'Content-Type': 'application/x-www-form-urlencoded'
      },
      body: formData
    });

    const data = await response.json();

    if (!response.ok) {
      throw { status: response.status, ...data };
    }

    return data;
  }

  /**
   * Register new user
   * @param {object} userData - User registration data
   * @returns {Promise<object>} Created user info
   */
  async function register(userData) {
    return apiRequest('/api/register', {
      method: 'POST',
      body: JSON.stringify(userData)
    });
  }

  /**
   * Logout user
   * @returns {Promise<void>}
   */
  async function logout() {
    try {
      await fetch(`${API_BASE}/logout`, {
        credentials: 'include'
      });
    } catch (error) {
      console.error('Logout error:', error);
    }

    // Clear local state regardless of API response
    window.location.href = '/login';
  }

  /**
   * Get current user info
   * @returns {Promise<object>} User data
   */
  async function getCurrentUser() {
    return apiRequest('/api/me', {
      method: 'GET'
    });
  }

  /**
   * Update user email
   * @param {string} email - New email address
   * @returns {Promise<object>} Updated user info
   */
  async function updateEmail(email) {
    return apiRequest('/api/me/email', {
      method: 'PATCH',
      body: JSON.stringify({ email })
    });
  }

  /**
   * Change password
   * @param {string} currentPassword
   * @param {string} newPassword
   * @returns {Promise<object>} Success message
   */
  async function changePassword(currentPassword, newPassword) {
    return apiRequest('/api/me/password', {
      method: 'PATCH',
      body: JSON.stringify({
        current_password: currentPassword,
        new_password: newPassword
      })
    });
  }

  /**
   * Delete account
   * @returns {Promise<object>} Success message
   */
  async function deleteAccount() {
    return apiRequest('/api/me', {
      method: 'DELETE'
    });
  }

  /**
   * Reset password with code
   * @param {string} username
   * @param {string} resetCode
   * @param {string} newPassword
   * @returns {Promise<object>} Success message
   */
  async function resetPassword(username, resetCode, newPassword) {
    return apiRequest('/api/reset-password', {
      method: 'POST',
      body: JSON.stringify({
        username,
        reset_code: resetCode,
        new_password: newPassword
      })
    });
  }

  /**
   * Get user activity (likes, comments)
   * @returns {Promise<object>} Activity data
   */
  async function getUserActivity() {
    return apiRequest('/api/me/activity', {
      method: 'GET'
    });
  }

  /**
   * Admin: Get all users
   * @returns {Promise<array>} List of users
   */
  async function getAllUsers() {
    return apiRequest('/api/admin/users', {
      method: 'GET'
    });
  }

  /**
   * Admin: Generate password reset code
   * @param {number} userId
   * @returns {Promise<object>} Reset code
   */
  async function generateResetCode(userId) {
    return apiRequest(`/api/admin/users/${userId}/reset-code`, {
      method: 'POST'
    });
  }

  /**
   * Admin: Force password reset
   * @param {number} userId
   * @returns {Promise<object>} Success message
   */
  async function forcePasswordReset(userId) {
    return apiRequest(`/api/admin/users/${userId}/force-reset`, {
      method: 'POST'
    });
  }

  /**
   * Check if user is authenticated
   * @returns {boolean}
   */
  function isAuthenticated() {
    // Check for username cookie (non-httponly, readable by JS)
    const cookies = document.cookie.split(';');
    for (let cookie of cookies) {
      const [name, value] = cookie.trim().split('=');
      if (name === 'username' && value) {
        return true;
      }
    }
    return false;
  }

  /**
   * Get username from cookie
   * @returns {string|null}
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
   * Display error message in form
   * @param {string} elementId - ID of error message element
   * @param {string|object} error - Error message or error object
   */
  function displayError(elementId, error) {
    const errorElement = document.getElementById(elementId);
    if (!errorElement) return;

    let message = 'An error occurred. Please try again.';

    if (typeof error === 'string') {
      message = error;
    } else if (error.detail) {
      message = error.detail;
    } else if (error.message) {
      message = error.message;
    }

    errorElement.textContent = message;
    errorElement.style.display = 'block';
  }

  /**
   * Hide error message
   * @param {string} elementId - ID of error message element
   */
  function hideError(elementId) {
    const errorElement = document.getElementById(elementId);
    if (errorElement) {
      errorElement.style.display = 'none';
    }
  }

  /**
   * Display success message
   * @param {string} elementId - ID of success message element
   * @param {string} message - Success message
   */
  function displaySuccess(elementId, message) {
    const successElement = document.getElementById(elementId);
    if (!successElement) return;

    successElement.textContent = message;
    successElement.style.display = 'block';
  }

  // Public API
  return {
    login,
    register,
    logout,
    getCurrentUser,
    updateEmail,
    changePassword,
    deleteAccount,
    resetPassword,
    getUserActivity,
    getAllUsers,
    generateResetCode,
    forcePasswordReset,
    isAuthenticated,
    getUsername,
    displayError,
    hideError,
    displaySuccess
  };
})();
