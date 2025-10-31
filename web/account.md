---
layout: content
title: Account
description: Manage your kevsrobots.com account
---

# My Account

<div class="container mt-4">
  <div class="row">
    <div class="col-md-8">
      <div class="card mb-4">
        <div class="card-body">
          <h2 class="card-title mb-4">Account Information</h2>

          <div id="error-message" class="alert alert-danger" style="display: none;" role="alert"></div>
          <div id="success-message" class="alert alert-success" style="display: none;" role="alert"></div>

          <div id="account-info">
            <!-- Profile Picture -->
            <div class="mb-4">
              <label class="form-label fw-bold">Profile Picture</label>
              <div class="d-flex align-items-center gap-3">
                <div id="profile-picture-preview">
                  <div class="rounded-circle bg-secondary d-inline-flex align-items-center justify-content-center text-white" style="width: 100px; height: 100px; font-size: 2rem;">
                    ?
                  </div>
                </div>
                <div>
                  <button type="button" class="btn btn-sm btn-primary" onclick="document.getElementById('profile-picture-input').click()">
                    <i class="fas fa-upload me-1"></i>Upload Picture
                  </button>
                  <button type="button" class="btn btn-sm btn-danger" id="remove-picture-btn" style="display: none;" onclick="deleteProfilePicture()">
                    <i class="fas fa-trash me-1"></i>Remove
                  </button>
                  <input type="file" id="profile-picture-input" accept="image/*" style="display: none;" onchange="uploadProfilePicture()">
                  <div id="upload-status" class="mt-2"></div>
                  <small class="text-muted d-block mt-1">Max 5MB. Will be resized to 400x400px.</small>
                </div>
              </div>
            </div>

            <div class="mb-3">
              <label class="form-label fw-bold">Username</label>
              <p class="form-control-plaintext" id="account-username">Loading...</p>
            </div>

            <div class="mb-3">
              <label class="form-label fw-bold">Full Name</label>
              <p class="form-control-plaintext" id="fullname-display">Loading...</p>
            </div>

            <div class="mb-3">
              <label for="email" class="form-label fw-bold">Email Address</label>
              <div class="input-group">
                <input type="email" class="form-control" id="email" name="email" readonly>
                <button class="btn btn-outline-secondary" type="button" id="edit-email-btn">Edit</button>
              </div>
            </div>

            <div id="email-edit-section" style="display: none;">
              <div class="mb-3">
                <label for="new-email" class="form-label fw-bold">New Email Address</label>
                <input type="email" class="form-control" id="new-email" name="new-email" required>
              </div>
              <button class="btn btn-primary me-2" id="save-email-btn">Save Email</button>
              <button class="btn btn-secondary" id="cancel-email-btn">Cancel</button>
            </div>

            <div class="mb-3">
              <label for="location" class="form-label fw-bold">Location</label>
              <input type="text" class="form-control" id="location" name="location" placeholder="e.g., United Kingdom">
              <small class="text-muted">Your location helps with timezone/localization</small>
            </div>

            <div class="mb-3">
              <label for="bio" class="form-label fw-bold">Bio</label>
              <textarea class="form-control" id="bio" name="bio" rows="3" maxlength="500" placeholder="Tell us about yourself..."></textarea>
              <small class="text-muted">Max 500 characters</small>
            </div>

            <button class="btn btn-primary mb-3" id="save-profile-btn">
              <i class="fas fa-save me-2"></i>Update Profile
            </button>

            <div class="mb-3">
              <label class="form-label fw-bold">Account Status</label>
              <p class="form-control-plaintext">
                <span class="badge bg-success" id="status-badge">Active</span>
              </p>
            </div>

            <div class="mb-3">
              <label class="form-label fw-bold">Member Since</label>
              <p class="form-control-plaintext" id="created-display">Loading...</p>
            </div>

            <div class="mb-3">
              <label class="form-label fw-bold">Last Login</label>
              <p class="form-control-plaintext" id="last-login-display">Loading...</p>
            </div>
          </div>
        </div>
      </div>

      <!-- Change Password Section -->
      <div class="card mb-4">
        <div class="card-body">
          <h3 class="card-title mb-4">Change Password</h3>

          <form id="password-form">
            <div class="mb-3">
              <label for="current-password" class="form-label fw-bold">Current Password</label>
              <input type="password" class="form-control" id="current-password" name="current-password" required>
            </div>

            <div class="mb-3">
              <label for="new-password" class="form-label fw-bold">New Password</label>
              <input type="password" class="form-control" id="new-password" name="new-password" required minlength="8">
              <div class="form-text">Minimum 8 characters</div>
            </div>

            <div class="mb-3">
              <label for="confirm-new-password" class="form-label fw-bold">Confirm New Password</label>
              <input type="password" class="form-control" id="confirm-new-password" name="confirm-new-password" required minlength="8">
            </div>

            <button type="submit" class="btn btn-primary">
              <span id="password-spinner" class="spinner-border spinner-border-sm me-2" style="display: none;" role="status" aria-hidden="true"></span>
              Change Password
            </button>
          </form>
        </div>
      </div>

      <!-- Delete Account Section -->
      <div class="card mb-4 border-danger">
        <div class="card-body">
          <h3 class="card-title text-danger mb-4">Delete Account</h3>
          <p class="text-muted">Once you delete your account, there is no going back. Please be certain.</p>
          <button class="btn btn-danger" id="delete-account-btn">Delete My Account</button>
        </div>
      </div>
    </div>

    <div class="col-md-4">
      <!-- Admin Panel Link (only shown for admins) -->
      <div class="card mb-4" id="admin-card" style="display: none;">
        <div class="card-body">
          <h3 class="card-title">Admin Panel</h3>
          <p class="text-muted">Manage users and system settings</p>
          <a href="/admin" class="btn btn-warning w-100">
            <i class="fa-solid fa-user-shield me-2"></i>Admin Panel
          </a>
        </div>
      </div>

      <!-- Activity Summary -->
      <div class="card mb-4">
        <div class="card-body">
          <h3 class="card-title mb-4">Activity Summary</h3>
          <div id="activity-summary">
            <p class="mb-2"><strong>Comments:</strong> <span id="comments-count">0</span></p>
            <p class="mb-0"><strong>Likes:</strong> <span id="likes-count">0</span></p>
          </div>
        </div>
      </div>
    </div>
  </div>
</div>

<script src="/assets/js/chatter-api.js"></script>
<script>
  let currentUser = null;

  // Check if user is authenticated
  if (!ChatterAPI.isAuthenticated()) {
    window.location.href = '/login?return_to=/account';
  }

  // Load user data
  async function loadUserData() {
    try {
      currentUser = await ChatterAPI.getCurrentUser();

      // Display user info
      document.getElementById('account-username').textContent = currentUser.username;
      document.getElementById('fullname-display').textContent = `${currentUser.firstname} ${currentUser.lastname}`;
      document.getElementById('email').value = currentUser.email;
      document.getElementById('status-badge').textContent = currentUser.status.charAt(0).toUpperCase() + currentUser.status.slice(1);

      // Format dates
      const createdDate = new Date(currentUser.created_at);
      document.getElementById('created-display').textContent = createdDate.toLocaleDateString('en-US', {
        year: 'numeric', month: 'long', day: 'numeric'
      });

      if (currentUser.last_login) {
        const lastLoginDate = new Date(currentUser.last_login);
        document.getElementById('last-login-display').textContent = lastLoginDate.toLocaleString('en-US', {
          year: 'numeric', month: 'long', day: 'numeric', hour: '2-digit', minute: '2-digit'
        });
      } else {
        document.getElementById('last-login-display').textContent = 'Never';
      }

      // Show admin panel link if user is admin
      if (currentUser.type === 1) {
        document.getElementById('admin-card').style.display = 'block';
      }

      // Load profile picture
      updateProfilePictureDisplay(currentUser.profile_picture);

      // Load location and bio
      document.getElementById('location').value = currentUser.location || '';
      document.getElementById('bio').value = currentUser.bio || '';

      // Load activity data
      loadActivity();
    } catch (error) {
      ChatterAPI.displayError('error-message', 'Failed to load account information');
      console.error('Error loading user data:', error);
    }
  }

  // Load activity data
  async function loadActivity() {
    try {
      const activity = await ChatterAPI.getUserActivity();
      document.getElementById('comments-count').textContent = activity.comments_count || 0;
      document.getElementById('likes-count').textContent = activity.likes_count || 0;
    } catch (error) {
      console.error('Error loading activity:', error);
    }
  }

  // Email editing
  document.getElementById('edit-email-btn').addEventListener('click', () => {
    document.getElementById('email').readOnly = false;
    document.getElementById('edit-email-btn').style.display = 'none';
    document.getElementById('email-edit-section').style.display = 'block';
    document.getElementById('new-email').value = document.getElementById('email').value;
  });

  document.getElementById('cancel-email-btn').addEventListener('click', () => {
    document.getElementById('email').readOnly = true;
    document.getElementById('edit-email-btn').style.display = 'inline-block';
    document.getElementById('email-edit-section').style.display = 'none';
    document.getElementById('email').value = currentUser.email;
    ChatterAPI.hideError('error-message');
    ChatterAPI.hideError('success-message');
  });

  document.getElementById('save-email-btn').addEventListener('click', async () => {
    const newEmail = document.getElementById('new-email').value;

    ChatterAPI.hideError('error-message');
    ChatterAPI.hideError('success-message');

    try {
      await ChatterAPI.updateEmail(newEmail);
      ChatterAPI.displaySuccess('success-message', 'Email updated successfully!');
      document.getElementById('email').value = newEmail;
      document.getElementById('email').readOnly = true;
      document.getElementById('edit-email-btn').style.display = 'inline-block';
      document.getElementById('email-edit-section').style.display = 'none';
      currentUser.email = newEmail;
    } catch (error) {
      ChatterAPI.displayError('error-message', error);
    }
  });

  // Change password form
  document.getElementById('password-form').addEventListener('submit', async (e) => {
    e.preventDefault();

    const currentPassword = document.getElementById('current-password').value;
    const newPassword = document.getElementById('new-password').value;
    const confirmPassword = document.getElementById('confirm-new-password').value;

    ChatterAPI.hideError('error-message');
    ChatterAPI.hideError('success-message');

    // Validate passwords match
    if (newPassword !== confirmPassword) {
      ChatterAPI.displayError('error-message', 'New passwords do not match');
      return;
    }

    // Show spinner
    document.getElementById('password-spinner').style.display = 'inline-block';

    try {
      await ChatterAPI.changePassword(currentPassword, newPassword);
      ChatterAPI.displaySuccess('success-message', 'Password changed successfully!');

      // Clear form
      document.getElementById('password-form').reset();
    } catch (error) {
      ChatterAPI.displayError('error-message', error);
    } finally {
      document.getElementById('password-spinner').style.display = 'none';
    }
  });

  // Delete account
  document.getElementById('delete-account-btn').addEventListener('click', async () => {
    if (!confirm('Are you absolutely sure you want to delete your account? This action cannot be undone.')) {
      return;
    }

    if (!confirm('This will permanently delete all your data. Are you really sure?')) {
      return;
    }

    try {
      await ChatterAPI.deleteAccount();
      alert('Your account has been deleted.');
      window.location.href = '/';
    } catch (error) {
      ChatterAPI.displayError('error-message', error);
    }
  });

  // Update profile picture display
  function updateProfilePictureDisplay(profilePicture) {
    const preview = document.getElementById('profile-picture-preview');
    const removeBtn = document.getElementById('remove-picture-btn');

    if (profilePicture) {
      preview.innerHTML = `<img src="https://chatter.kevsrobots.com/profile_pictures/${profilePicture}" alt="Profile" class="rounded-circle" style="width: 100px; height: 100px; object-fit: cover;">`;
      removeBtn.style.display = 'inline-block';
    } else {
      const initial = currentUser ? currentUser.username[0].toUpperCase() : '?';
      preview.innerHTML = `<div class="rounded-circle bg-secondary d-inline-flex align-items-center justify-content-center text-white" style="width: 100px; height: 100px; font-size: 2rem;">${initial}</div>`;
      removeBtn.style.display = 'none';
    }
  }

  // Upload profile picture
  async function uploadProfilePicture() {
    const fileInput = document.getElementById('profile-picture-input');
    const statusDiv = document.getElementById('upload-status');
    const file = fileInput.files[0];

    if (!file) return;

    // Validate file size (5MB)
    if (file.size > 5 * 1024 * 1024) {
      statusDiv.innerHTML = '<div class="alert alert-danger alert-sm mt-2">File too large. Max 5MB.</div>';
      return;
    }

    statusDiv.innerHTML = '<div class="text-muted"><i class="fas fa-spinner fa-spin me-2"></i>Uploading...</div>';

    const formData = new FormData();
    formData.append('file', file);

    try {
      const response = await fetch('https://chatter.kevsrobots.com/profile/picture', {
        method: 'POST',
        credentials: 'include',
        body: formData
      });

      const data = await response.json();

      if (response.ok) {
        statusDiv.innerHTML = '<div class="alert alert-success alert-sm mt-2">Picture uploaded successfully!</div>';
        // Update display
        currentUser.profile_picture = data.profile_picture_url.split('/').pop();
        updateProfilePictureDisplay(currentUser.profile_picture);
        setTimeout(() => { statusDiv.innerHTML = ''; }, 3000);
      } else {
        statusDiv.innerHTML = `<div class="alert alert-danger alert-sm mt-2">${data.detail || 'Upload failed'}</div>`;
      }
    } catch (error) {
      console.error('Upload error:', error);
      statusDiv.innerHTML = '<div class="alert alert-danger alert-sm mt-2">Upload failed. Please try again.</div>';
    }
  }

  // Delete profile picture
  async function deleteProfilePicture() {
    if (!confirm('Are you sure you want to remove your profile picture?')) {
      return;
    }

    const statusDiv = document.getElementById('upload-status');
    statusDiv.innerHTML = '<div class="text-muted"><i class="fas fa-spinner fa-spin me-2"></i>Removing...</div>';

    try {
      const response = await fetch('https://chatter.kevsrobots.com/profile/picture', {
        method: 'DELETE',
        credentials: 'include'
      });

      const data = await response.json();

      if (response.ok) {
        statusDiv.innerHTML = '<div class="alert alert-success alert-sm mt-2">Picture removed successfully!</div>';
        currentUser.profile_picture = null;
        updateProfilePictureDisplay(null);
        setTimeout(() => { statusDiv.innerHTML = ''; }, 3000);
      } else {
        statusDiv.innerHTML = `<div class="alert alert-danger alert-sm mt-2">${data.detail || 'Delete failed'}</div>`;
      }
    } catch (error) {
      console.error('Delete error:', error);
      statusDiv.innerHTML = '<div class="alert alert-danger alert-sm mt-2">Delete failed. Please try again.</div>';
    }
  }

  // Update profile (location and bio)
  document.getElementById('save-profile-btn').addEventListener('click', async () => {
    const location = document.getElementById('location').value.trim();
    const bio = document.getElementById('bio').value.trim();

    ChatterAPI.hideError('error-message');
    ChatterAPI.hideError('success-message');

    try {
      const response = await fetch('https://chatter.kevsrobots.com/profile', {
        method: 'PUT',
        credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ location, bio })
      });

      const data = await response.json();

      if (response.ok) {
        ChatterAPI.displaySuccess('success-message', 'Profile updated successfully!');
        currentUser.location = location;
        currentUser.bio = bio;
      } else {
        ChatterAPI.displayError('error-message', data.detail || 'Failed to update profile');
      }
    } catch (error) {
      console.error('Update error:', error);
      ChatterAPI.displayError('error-message', 'Failed to update profile. Please try again.');
    }
  });

  // Load data on page load
  loadUserData();
</script>
