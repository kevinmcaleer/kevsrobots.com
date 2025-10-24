---
layout: content
title: Admin Panel
description: Manage users and system settings
---

# Admin Panel

<div class="container mt-4">
  <div class="row">
    <div class="col-12">
      <div class="d-flex justify-content-between align-items-center mb-4">
        <h2>User Management</h2>
        <a href="/account" class="btn btn-secondary">
          <i class="fa-solid fa-arrow-left me-2"></i>Back to Account
        </a>
      </div>

      <div id="error-message" class="alert alert-danger" style="display: none;" role="alert"></div>
      <div id="success-message" class="alert alert-success" style="display: none;" role="alert"></div>

      <!-- Reset Code Display Alert -->
      <div id="reset-code-alert" class="alert alert-info alert-dismissible fade show" style="display: none;" role="alert">
        <h5 class="alert-heading"><i class="fa-solid fa-key me-2"></i>Password Reset Code Generated</h5>
        <p class="mb-2"><strong>User:</strong> <span id="reset-code-username"></span></p>
        <p class="mb-2"><strong>Code:</strong> <code class="fs-4 text-dark" id="reset-code-value"></code></p>
        <hr>
        <p class="mb-0 small">
          <i class="fa-solid fa-clock me-1"></i>This code expires in 24 hours.
          <i class="fa-solid fa-exclamation-triangle ms-3 me-1"></i>Give this code to the user securely.
        </p>
        <button type="button" class="btn-close" onclick="document.getElementById('reset-code-alert').style.display='none'"></button>
      </div>

      <!-- Users Table -->
      <div class="card">
        <div class="card-body">
          <div class="table-responsive">
            <table class="table table-striped table-hover">
              <thead>
                <tr>
                  <th>Username</th>
                  <th>Name</th>
                  <th>Email</th>
                  <th>Type</th>
                  <th>Status</th>
                  <th>Last Login</th>
                  <th>Created</th>
                  <th>Actions</th>
                </tr>
              </thead>
              <tbody id="users-table-body">
                <tr>
                  <td colspan="8" class="text-center">
                    <div class="spinner-border" role="status">
                      <span class="visually-hidden">Loading...</span>
                    </div>
                  </td>
                </tr>
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </div>
  </div>
</div>

<script src="/assets/js/chatter-api.js"></script>
<script>
  // Check if user is authenticated and is admin
  if (!ChatterAPI.isAuthenticated()) {
    window.location.href = '/login?return_to=/admin';
  }

  // Format date helper
  function formatDate(dateString) {
    if (!dateString) return 'Never';
    const date = new Date(dateString);
    return date.toLocaleDateString('en-US', {
      year: 'numeric',
      month: 'short',
      day: 'numeric'
    });
  }

  // Format datetime helper
  function formatDateTime(dateString) {
    if (!dateString) return 'Never';
    const date = new Date(dateString);
    return date.toLocaleString('en-US', {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit'
    });
  }

  // Load users
  async function loadUsers() {
    try {
      const users = await ChatterAPI.getAllUsers();

      const tbody = document.getElementById('users-table-body');
      tbody.innerHTML = '';

      users.forEach(user => {
        const row = document.createElement('tr');

        // Username with password reset warning badge
        const usernameCell = document.createElement('td');
        usernameCell.innerHTML = `
          ${user.username}
          ${user.force_password_reset ? '<span class="badge bg-warning text-dark ms-2" title="Password reset required"><i class="fa-solid fa-key"></i></span>' : ''}
        `;

        // Full name
        const nameCell = document.createElement('td');
        nameCell.textContent = `${user.firstname} ${user.lastname}`;

        // Email
        const emailCell = document.createElement('td');
        emailCell.textContent = user.email;

        // Type badge
        const typeCell = document.createElement('td');
        if (user.type === 1) {
          typeCell.innerHTML = '<span class="badge bg-danger">Admin</span>';
        } else {
          typeCell.innerHTML = '<span class="badge bg-secondary">User</span>';
        }

        // Status badge
        const statusCell = document.createElement('td');
        if (user.status === 'active') {
          statusCell.innerHTML = '<span class="badge bg-success">Active</span>';
        } else {
          statusCell.innerHTML = '<span class="badge bg-secondary">Inactive</span>';
        }

        // Last login
        const lastLoginCell = document.createElement('td');
        lastLoginCell.textContent = formatDateTime(user.last_login);

        // Created date
        const createdCell = document.createElement('td');
        createdCell.textContent = formatDate(user.created_at);

        // Actions
        const actionsCell = document.createElement('td');
        actionsCell.innerHTML = `
          <button class="btn btn-sm btn-primary me-1" onclick="generateResetCode(${user.id}, '${user.username}')" title="Generate password reset code">
            <i class="fa-solid fa-key"></i>
          </button>
          <button class="btn btn-sm btn-warning" onclick="forcePasswordReset(${user.id}, '${user.username}')" title="Force password reset on next login">
            <i class="fa-solid fa-lock"></i>
          </button>
        `;

        row.appendChild(usernameCell);
        row.appendChild(nameCell);
        row.appendChild(emailCell);
        row.appendChild(typeCell);
        row.appendChild(statusCell);
        row.appendChild(lastLoginCell);
        row.appendChild(createdCell);
        row.appendChild(actionsCell);

        tbody.appendChild(row);
      });
    } catch (error) {
      ChatterAPI.displayError('error-message', 'Failed to load users. Admin access required.');
      console.error('Error loading users:', error);
    }
  }

  // Generate reset code
  async function generateResetCode(userId, username) {
    if (!confirm(`Generate a password reset code for ${username}?`)) {
      return;
    }

    ChatterAPI.hideError('error-message');
    ChatterAPI.hideError('success-message');

    try {
      const result = await ChatterAPI.generateResetCode(userId);

      // Show reset code alert
      document.getElementById('reset-code-username').textContent = username;
      document.getElementById('reset-code-value').textContent = result.reset_code;
      document.getElementById('reset-code-alert').style.display = 'block';

      // Scroll to top to show alert
      window.scrollTo({ top: 0, behavior: 'smooth' });

      // Reload users to show any changes
      await loadUsers();
    } catch (error) {
      ChatterAPI.displayError('error-message', error);
    }
  }

  // Force password reset
  async function forcePasswordReset(userId, username) {
    if (!confirm(`Force ${username} to reset their password on next login?`)) {
      return;
    }

    ChatterAPI.hideError('error-message');
    ChatterAPI.hideError('success-message');

    try {
      await ChatterAPI.forcePasswordReset(userId);
      ChatterAPI.displaySuccess('success-message', `Password reset flag set for ${username}`);

      // Reload users to show updated badge
      await loadUsers();
    } catch (error) {
      ChatterAPI.displayError('error-message', error);
    }
  }

  // Make functions globally accessible
  window.generateResetCode = generateResetCode;
  window.forcePasswordReset = forcePasswordReset;

  // Load users on page load
  loadUsers();
</script>
