---
layout: content
title: Register
description: Create a new kevsrobots.com account
---

# Register

<div class="container mt-4">
  <div class="row justify-content-center">
    <div class="col-md-6">
      <div class="card">
        <div class="card-body">
          <h2 class="card-title text-center mb-4">Create Your Account</h2>

          <div id="error-message" class="alert alert-danger" style="display: none;" role="alert"></div>

          <form id="register-form">
            <div class="mb-3">
              <label for="username" class="form-label fw-bold">Username</label>
              <input type="text" class="form-control" id="username" name="username" required>
              <div class="form-text">Choose a unique username (letters, numbers, underscores only)</div>
            </div>

            <div class="mb-3">
              <label for="firstname" class="form-label fw-bold">First Name</label>
              <input type="text" class="form-control" id="firstname" name="firstname" required>
            </div>

            <div class="mb-3">
              <label for="lastname" class="form-label fw-bold">Last Name</label>
              <input type="text" class="form-control" id="lastname" name="lastname" required>
            </div>

            <div class="mb-3">
              <label for="email" class="form-label fw-bold">Email Address</label>
              <input type="email" class="form-control" id="email" name="email" required>
              <div class="form-text">We'll never share your email with anyone else</div>
            </div>

            <div class="mb-3">
              <label for="password" class="form-label fw-bold">Password</label>
              <input type="password" class="form-control" id="password" name="password" required minlength="8">
              <div class="form-text">Minimum 8 characters</div>
            </div>

            <div class="mb-3">
              <label for="confirm_password" class="form-label fw-bold">Confirm Password</label>
              <input type="password" class="form-control" id="confirm_password" name="confirm_password" required minlength="8">
            </div>

            <div class="mb-3">
              <label for="date_of_birth" class="form-label fw-bold">Date of Birth (Optional)</label>
              <input type="date" class="form-control" id="date_of_birth" name="date_of_birth">
            </div>

            <button type="submit" class="btn btn-primary w-100 mb-3">
              <span id="register-spinner" class="spinner-border spinner-border-sm me-2" style="display: none;" role="status" aria-hidden="true"></span>
              <span id="register-text">Register</span>
            </button>
          </form>

          <div class="text-center">
            <p class="mb-0">Already have an account? <a href="/login">Login here</a></p>
          </div>
        </div>
      </div>
    </div>
  </div>
</div>

<script src="/assets/js/chatter-api.js"></script>
<script>
  // Handle register form submission
  document.getElementById('register-form').addEventListener('submit', async (e) => {
    e.preventDefault();

    const username = document.getElementById('username').value;
    const firstname = document.getElementById('firstname').value;
    const lastname = document.getElementById('lastname').value;
    const email = document.getElementById('email').value;
    const password = document.getElementById('password').value;
    const confirmPassword = document.getElementById('confirm_password').value;
    const dateOfBirth = document.getElementById('date_of_birth').value || null;

    // Hide previous messages
    ChatterAPI.hideError('error-message');

    // Validate passwords match
    if (password !== confirmPassword) {
      ChatterAPI.displayError('error-message', 'Passwords do not match');
      return;
    }

    // Show loading spinner
    document.getElementById('register-spinner').style.display = 'inline-block';
    document.getElementById('register-text').textContent = 'Creating account...';

    try {
      const userData = {
        username: username,
        firstname: firstname,
        lastname: lastname,
        email: email,
        password: password,
        date_of_birth: dateOfBirth
      };

      const result = await ChatterAPI.register(userData);

      // Success! Redirect to login page
      window.location.href = '/login?registered=success';
    } catch (error) {
      // Hide spinner
      document.getElementById('register-spinner').style.display = 'none';
      document.getElementById('register-text').textContent = 'Register';

      // Display error
      ChatterAPI.displayError('error-message', error);
    }
  });
</script>
