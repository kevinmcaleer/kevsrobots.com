---
layout: content
title: Reset Password
description: Reset your password using a reset code
---

# Reset Password

<div class="container mt-4">
  <div class="row justify-content-center">
    <div class="col-md-6">
      <div class="card">
        <div class="card-body">
          <h2 class="card-title text-center mb-4">Reset Your Password</h2>

          <div class="alert alert-info mb-4" role="alert">
            <i class="fa-solid fa-info-circle me-2"></i>
            <strong>Need a reset code?</strong> Contact an administrator to generate a one-time password reset code for your account.
          </div>

          <div id="error-message" class="alert alert-danger" style="display: none;" role="alert"></div>

          <form id="reset-password-form">
            <div class="mb-3">
              <label for="username" class="form-label fw-bold">Username</label>
              <input type="text" class="form-control" id="username" name="username" required>
              <div class="form-text">Your kevsrobots.com username</div>
            </div>

            <div class="mb-3">
              <label for="reset-code" class="form-label fw-bold">Reset Code</label>
              <input
                type="text"
                class="form-control text-uppercase"
                id="reset-code"
                name="reset-code"
                required
                maxlength="8"
                style="font-family: monospace; font-size: 1.1rem; letter-spacing: 2px;"
                placeholder="ABC12345"
              >
              <div class="form-text">8-character code provided by administrator (case-insensitive)</div>
            </div>

            <div class="mb-3">
              <label for="new-password" class="form-label fw-bold">New Password</label>
              <input type="password" class="form-control" id="new-password" name="new-password" required minlength="8">
              <div class="form-text">Minimum 8 characters</div>
            </div>

            <div class="mb-3">
              <label for="confirm-password" class="form-label fw-bold">Confirm New Password</label>
              <input type="password" class="form-control" id="confirm-password" name="confirm-password" required minlength="8">
            </div>

            <button type="submit" class="btn btn-primary w-100 mb-3">
              <span id="reset-spinner" class="spinner-border spinner-border-sm me-2" style="display: none;" role="status" aria-hidden="true"></span>
              <span id="reset-text">Reset Password</span>
            </button>
          </form>

          <div class="text-center">
            <p class="mb-0"><a href="/login">Back to Login</a></p>
          </div>
        </div>
      </div>

      <!-- Help Card -->
      <div class="card mt-4">
        <div class="card-body">
          <h5 class="card-title"><i class="fa-solid fa-question-circle me-2"></i>How This Works</h5>
          <ol class="mb-0">
            <li>Contact an administrator to verify your identity</li>
            <li>Administrator generates an 8-character reset code</li>
            <li>Enter your username, the reset code, and your new password</li>
            <li>Code expires after 24 hours and can only be used once</li>
          </ol>
        </div>
      </div>
    </div>
  </div>
</div>

<script src="/assets/js/chatter-api.js"></script>
<script>
  // Auto-uppercase reset code as user types
  document.getElementById('reset-code').addEventListener('input', (e) => {
    e.target.value = e.target.value.toUpperCase();
  });

  // Handle reset password form submission
  document.getElementById('reset-password-form').addEventListener('submit', async (e) => {
    e.preventDefault();

    const username = document.getElementById('username').value;
    const resetCode = document.getElementById('reset-code').value;
    const newPassword = document.getElementById('new-password').value;
    const confirmPassword = document.getElementById('confirm-password').value;

    // Hide previous messages
    ChatterAPI.hideError('error-message');

    // Validate passwords match
    if (newPassword !== confirmPassword) {
      ChatterAPI.displayError('error-message', 'Passwords do not match');
      return;
    }

    // Validate reset code format (8 alphanumeric characters)
    if (!/^[A-Z0-9]{8}$/.test(resetCode)) {
      ChatterAPI.displayError('error-message', 'Reset code must be 8 alphanumeric characters');
      return;
    }

    // Show loading spinner
    document.getElementById('reset-spinner').style.display = 'inline-block';
    document.getElementById('reset-text').textContent = 'Resetting password...';

    try {
      await ChatterAPI.resetPassword(username, resetCode, newPassword);

      // Success! Redirect to login page with success message
      window.location.href = '/login?reset=success';
    } catch (error) {
      // Hide spinner
      document.getElementById('reset-spinner').style.display = 'none';
      document.getElementById('reset-text').textContent = 'Reset Password';

      // Display error
      ChatterAPI.displayError('error-message', error);
    }
  });
</script>
