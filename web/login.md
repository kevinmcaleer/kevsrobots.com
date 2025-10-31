---
layout: content
title: Login
description: Login to your kevsrobots.com account
hide_likes_and_comments: true
---

# Login

<div class="container mt-4">
  <div class="row justify-content-center">
    <div class="col-md-6">
      <div class="card">
        <div class="card-body">
          <h2 class="card-title text-center mb-4">Login to Your Account</h2>

          <div id="error-message" class="alert alert-danger" style="display: none;" role="alert"></div>
          <div id="success-message" class="alert alert-success" style="display: none;" role="alert"></div>

          <form id="login-form">
            <div class="mb-3">
              <label for="username" class="form-label fw-bold">Username or Email</label>
              <input type="text" class="form-control" id="username" name="username" required autocomplete="username">
            </div>

            <div class="mb-3">
              <label for="password" class="form-label fw-bold">Password</label>
              <input type="password" class="form-control" id="password" name="password" required>
            </div>

            <button type="submit" class="btn btn-primary w-100 mb-3">
              <span id="login-spinner" class="spinner-border spinner-border-sm me-2" style="display: none;" role="status" aria-hidden="true"></span>
              <span id="login-text">Login</span>
            </button>
          </form>

          <div class="text-center">
            <p class="mb-2">Don't have an account? <a href="/register">Register here</a></p>
            <p class="mb-0"><a href="/reset-password">Forgot your password?</a></p>
          </div>
        </div>
      </div>
    </div>
  </div>
</div>

<script src="/assets/js/chatter-api.js"></script>
<script>
  // Check for error/success messages in URL
  const urlParams = new URLSearchParams(window.location.search);
  const error = urlParams.get('error');
  const reset = urlParams.get('reset');
  const registered = urlParams.get('registered');

  if (error === 'session_expired') {
    ChatterAPI.displayError('error-message', 'Your session has expired. Please login again.');
  } else if (reset === 'success') {
    ChatterAPI.displaySuccess('success-message', 'Password reset successful! You can now login with your new password.');
  } else if (registered === 'success') {
    ChatterAPI.displaySuccess('success-message', 'Account created successfully! You can now login.');
  }

  // Handle login form submission
  document.getElementById('login-form').addEventListener('submit', async (e) => {
    e.preventDefault();

    const username = document.getElementById('username').value;
    const password = document.getElementById('password').value;

    // Hide previous messages
    ChatterAPI.hideError('error-message');
    ChatterAPI.hideError('success-message');

    // Show loading spinner
    document.getElementById('login-spinner').style.display = 'inline-block';
    document.getElementById('login-text').textContent = 'Logging in...';

    try {
      const result = await ChatterAPI.login(username, password);

      // Success! Redirect to account page or return_to URL
      const returnTo = urlParams.get('return_to') || '/account';
      window.location.href = returnTo;
    } catch (error) {
      // Hide spinner
      document.getElementById('login-spinner').style.display = 'none';
      document.getElementById('login-text').textContent = 'Login';

      // Display error
      ChatterAPI.displayError('error-message', error);
    }
  });
</script>
