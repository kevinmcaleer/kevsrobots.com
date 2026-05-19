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
            <!-- Profile Picture (Chatter-backed, unchanged) -->
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
              <div class="input-group" style="max-width: 500px;">
                <input type="text" class="form-control form-control-sm" id="profile-link" readonly>
                <button class="btn btn-sm btn-outline-secondary" type="button" onclick="copyProfileLink()">
                  <i class="fas fa-copy me-1"></i>Copy Link
                </button>
              </div>
              <small class="text-muted">Share your public profile with others</small>
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

            <!-- Public profile section (projects-api backed) -->
            <hr>
            <h3 class="h5 mb-3"><i class="fas fa-id-card me-2 text-primary"></i>Public Profile</h3>
            <p class="text-muted small">These fields are shown on your <a id="public-profile-link" href="/profile.html">public profile page</a>.</p>

            <!-- Lazy backfill prompt: appears when projects-api side is empty
                 but Chatter has values. Click the button to sync to projects-api. -->
            <div id="backfill-prompt" class="alert alert-info d-none" role="alert">
              <i class="fas fa-info-circle me-2"></i>
              <strong>Sync your existing bio to your public profile?</strong>
              We found a bio/location on your Chatter account but nothing on your public profile yet.
              The form below has been pre-filled — click <em>Save Public Profile</em> to copy it over.
            </div>

            <div class="mb-3">
              <label for="location" class="form-label fw-bold">Location</label>
              <input type="text" class="form-control" id="location" name="location" maxlength="120" placeholder="e.g., Glasgow, UK">
              <small class="text-muted">Your location helps with timezone/localization</small>
            </div>

            <div class="mb-3">
              <label for="bio" class="form-label fw-bold">Bio</label>
              <textarea class="form-control" id="bio" name="bio" rows="3" maxlength="500" placeholder="Tell us about yourself..."></textarea>
              <div class="form-text d-flex justify-content-between">
                <span>Max 500 characters</span>
                <span><span id="bio-count">0</span>/500</span>
              </div>
            </div>

            <div class="mb-3">
              <label for="website_url" class="form-label fw-bold">Website</label>
              <input type="url" class="form-control" id="website_url" name="website_url" maxlength="200" placeholder="https://example.com">
              <small class="text-muted">Must start with <code>http://</code> or <code>https://</code> (max 200 chars).</small>
            </div>

            <fieldset class="mb-3">
              <legend class="h6 fw-bold">Social Links</legend>
              <div class="mb-2">
                <label for="social_github" class="form-label small mb-1"><i class="fa-brands fa-github me-1"></i> GitHub</label>
                <input id="social_github" type="url" class="form-control" maxlength="200" placeholder="https://github.com/your-handle">
              </div>
              <div class="mb-2">
                <label for="social_twitter" class="form-label small mb-1"><i class="fa-brands fa-x-twitter me-1"></i> Twitter / X</label>
                <input id="social_twitter" type="url" class="form-control" maxlength="200" placeholder="https://x.com/your-handle">
              </div>
              <div class="mb-2">
                <label for="social_youtube" class="form-label small mb-1"><i class="fa-brands fa-youtube me-1"></i> YouTube</label>
                <input id="social_youtube" type="url" class="form-control" maxlength="200" placeholder="https://youtube.com/@your-channel">
              </div>
              <div class="mb-2">
                <label for="social_mastodon" class="form-label small mb-1"><i class="fa-brands fa-mastodon me-1"></i> Mastodon</label>
                <input id="social_mastodon" type="url" class="form-control" maxlength="200" placeholder="https://mastodon.social/@your-handle">
              </div>
            </fieldset>

            <!-- Issue #150: Display currency. ``Auto-detect from browser``
                 saves null on the server, which the parts/projects pages
                 treat as "show native". The supported list matches the
                 SUPPORTED_CURRENCIES allow-list in projects-api/fx.py. -->
            <div class="mb-3">
              <label for="preferred_currency" class="form-label fw-bold">Display currency</label>
              <select class="form-select" id="preferred_currency" name="preferred_currency">
                <option value="">Auto-detect from browser</option>
                <option value="GBP">GBP — British Pound (&pound;)</option>
                <option value="USD">USD — US Dollar ($)</option>
                <option value="EUR">EUR — Euro (&euro;)</option>
                <option value="JPY">JPY — Japanese Yen (&yen;)</option>
                <option value="AUD">AUD — Australian Dollar (A$)</option>
                <option value="CAD">CAD — Canadian Dollar (C$)</option>
              </select>
              <small class="text-muted">Used to show converted prices on parts and projects. Source prices remain in the original currency.</small>
            </div>

            <!-- Featured badges picker. Hidden if the badges endpoint returns
                 nothing — gracefully degrades when #106 is not yet live. -->
            <fieldset class="mb-3 d-none" id="featured-badges-fieldset">
              <legend class="h6 fw-bold">Featured Badges</legend>
              <p class="form-text">Pick up to 3 badges to showcase on your public profile.</p>
              <div id="featured-badges-options" class="d-flex flex-wrap gap-2"></div>
            </fieldset>

            <button class="btn btn-primary mb-3" id="save-profile-btn">
              <i class="fas fa-save me-2"></i>Save Public Profile
            </button>

            <hr>

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

<script src="/assets/js/chatter-api.js?v={{ site.time | date: '%s' }}"></script>
<script src="/assets/js/project-auth.js?v={{ site.time | date: '%s' }}"></script>
<script>
  let currentUser = null;
  // Cached projects-api profile (the source of truth for public profile fields).
  let projectsProfile = null;
  // Badge picker state — slugs the user has selected (max 3).
  let pickedSlugs = [];

  const PROJECTS_API = 'https://projects.kevsrobots.com';

  // Check if user is authenticated
  if (!ChatterAPI.isAuthenticated()) {
    window.location.href = '/login?return_to=/account';
  }

  // Bio character counter
  const bioInput = document.getElementById('bio');
  const bioCount = document.getElementById('bio-count');
  function updateBioCount() {
    bioCount.textContent = String((bioInput.value || '').length);
  }
  bioInput.addEventListener('input', updateBioCount);

  // Load user data
  async function loadUserData() {
    try {
      currentUser = await ChatterAPI.getCurrentUser();

      // Display user info
      document.getElementById('account-username').textContent = currentUser.username;
      document.getElementById('profile-link').value = `https://www.kevsrobots.com/profile?username=${currentUser.username}`;
      document.getElementById('public-profile-link').href = `/profile.html?u=${encodeURIComponent(currentUser.username)}`;
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

      // Load projects-api public profile (bio/location/website/socials/badges).
      // This is the source of truth — Chatter values are only used as a
      // lazy-backfill fallback if projects-api is empty.
      await loadPublicProfile();

      // Load activity data
      loadActivity();
    } catch (error) {
      // Check if it's an authentication error (expired or invalid token)
      if (error.status === 401) {
        // Redirect to login with return URL
        window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
        return;
      }
      ChatterAPI.displayError('error-message', 'Failed to load account information');
      console.error('Error loading user data:', error);
    }
  }

  // Fetch the public profile from projects-api and populate the public-profile
  // form fields. Implements the lazy-backfill prompt: if both bio and location
  // on the projects-api side are empty AND Chatter has a non-empty value for
  // either, pre-fill the form with the Chatter value and show the sync banner.
  // The banner short-circuits as soon as either side has content for both
  // fields — no banner is shown if projects-api already has any data.
  async function loadPublicProfile() {
    if (!currentUser || !currentUser.username) return;
    try {
      const r = await fetch(
        `${PROJECTS_API}/api/users/${encodeURIComponent(currentUser.username)}/profile`
      );
      projectsProfile = r.ok ? await r.json() : null;
    } catch (_) {
      projectsProfile = null;
    }

    const apiBio = (projectsProfile && projectsProfile.bio) || '';
    const apiLoc = (projectsProfile && projectsProfile.location) || '';
    const chatterBio = currentUser.bio || '';
    const chatterLoc = currentUser.location || '';

    // Decide what to populate the editable fields with.
    let useBio = apiBio;
    let useLoc = apiLoc;
    let needsBackfill = false;

    if (!apiBio && chatterBio) {
      useBio = chatterBio;
      needsBackfill = true;
    }
    if (!apiLoc && chatterLoc) {
      useLoc = chatterLoc;
      needsBackfill = true;
    }

    document.getElementById('bio').value = useBio;
    document.getElementById('location').value = useLoc;
    updateBioCount();

    document.getElementById('website_url').value =
      (projectsProfile && projectsProfile.website_url) || '';
    const social = (projectsProfile && projectsProfile.social_links) || {};
    document.getElementById('social_github').value = social.github || '';
    document.getElementById('social_twitter').value = social.twitter || '';
    document.getElementById('social_youtube').value = social.youtube || '';
    document.getElementById('social_mastodon').value = social.mastodon || '';

    // Issue #150: preferred display currency. Empty string = "auto-detect".
    const ccyEl = document.getElementById('preferred_currency');
    if (ccyEl) {
      ccyEl.value = (projectsProfile && projectsProfile.preferred_currency) || '';
    }

    if (needsBackfill) {
      document.getElementById('backfill-prompt').classList.remove('d-none');
    }

    loadBadgePicker(currentUser.username, projectsProfile);
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
      if (error.status === 401) {
        window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
        return;
      }
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
      if (error.status === 401) {
        window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
        return;
      }
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
      if (error.status === 401) {
        window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
        return;
      }
      ChatterAPI.displayError('error-message', error);
    }
  });

  // Copy profile link to clipboard
  function copyProfileLink() {
    const profileLinkInput = document.getElementById('profile-link');
    profileLinkInput.select();
    profileLinkInput.setSelectionRange(0, 99999); // For mobile devices

    try {
      navigator.clipboard.writeText(profileLinkInput.value).then(() => {
        // Visual feedback
        const btn = event.target.closest('button');
        const originalHTML = btn.innerHTML;
        btn.innerHTML = '<i class="fas fa-check me-1"></i>Copied!';
        btn.classList.remove('btn-outline-secondary');
        btn.classList.add('btn-success');

        setTimeout(() => {
          btn.innerHTML = originalHTML;
          btn.classList.remove('btn-success');
          btn.classList.add('btn-outline-secondary');
        }, 2000);
      });
    } catch (err) {
      // Fallback for older browsers
      document.execCommand('copy');
      alert('Profile link copied to clipboard!');
    }
  }

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
      } else if (response.status === 401) {
        // Authentication error - redirect to login
        window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
        return;
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
      } else if (response.status === 401) {
        // Authentication error - redirect to login
        window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
        return;
      } else {
        statusDiv.innerHTML = `<div class="alert alert-danger alert-sm mt-2">${data.detail || 'Delete failed'}</div>`;
      }
    } catch (error) {
      console.error('Delete error:', error);
      statusDiv.innerHTML = '<div class="alert alert-danger alert-sm mt-2">Delete failed. Please try again.</div>';
    }
  }

  // --- Featured badges picker ---------------------------------------------
  // Defensive: hides itself if the badges endpoint returns nothing (i.e.
  // #106 is not deployed). Mirrors the pattern from the retired profile-edit.js.

  function renderBadgePicker(badges, pinned) {
    const fieldset = document.getElementById('featured-badges-fieldset');
    const wrap = document.getElementById('featured-badges-options');
    if (!badges || badges.length === 0) return;
    fieldset.classList.remove('d-none');
    pickedSlugs = (pinned || []).slice(0, 3);
    wrap.innerHTML = badges.map(function (b) {
      const checked = pickedSlugs.some(function (s) {
        return (s || '').toLowerCase() === (b.key || '').toLowerCase();
      });
      return '<label class="form-check form-check-inline">' +
               '<input class="form-check-input badge-pick" type="checkbox" value="' +
                 (b.key || '') + '"' + (checked ? ' checked' : '') + '>' +
               '<span class="badge bg-' + (b.color || 'primary') + ' p-2 ms-1">' +
                 '<i class="fas ' + (b.icon || 'fa-circle') + ' me-1"></i>' + (b.name || '') +
               '</span>' +
             '</label>';
    }).join('');

    wrap.addEventListener('change', function (e) {
      if (!e.target.classList.contains('badge-pick')) return;
      const slug = e.target.value;
      if (e.target.checked) {
        const checkedNow = wrap.querySelectorAll('.badge-pick:checked');
        if (checkedNow.length > 3) {
          e.target.checked = false;
          alert('You can only pin 3 badges. Uncheck one first.');
          return;
        }
        pickedSlugs.push(slug);
      } else {
        pickedSlugs = pickedSlugs.filter(function (s) {
          return (s || '').toLowerCase() !== (slug || '').toLowerCase();
        });
      }
    });
  }

  function loadBadgePicker(username, profile) {
    if (!username) return;
    fetch(`${PROJECTS_API}/api/users/${encodeURIComponent(username)}/badges`)
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (data) {
        if (!data || !data.badges || data.badges.length === 0) return;
        renderBadgePicker(data.badges, (profile && profile.featured_badge_slugs) || []);
      })
      .catch(function () {});
  }

  // --- URL validation helper ------------------------------------------------
  function validateUrl(value, field) {
    if (!value) return '';
    const trimmed = value.trim();
    if (trimmed === '') return '';
    if (!/^https?:\/\/[^\s]+$/i.test(trimmed)) {
      throw new Error(field + ' must start with http:// or https://');
    }
    if (trimmed.length > 200) {
      throw new Error(field + ' is too long (max 200 chars)');
    }
    return trimmed;
  }

  // Save the public profile fields (bio, location, website, socials,
  // featured badges) to projects-api. Email + password + picture stay on
  // Chatter via the other handlers above.
  document.getElementById('save-profile-btn').addEventListener('click', async () => {
    ChatterAPI.hideError('error-message');
    ChatterAPI.hideError('success-message');

    const bio = (document.getElementById('bio').value || '').trim();
    if (bio.length > 500) {
      ChatterAPI.displayError('error-message', 'Bio is too long (max 500 chars).');
      return;
    }

    let payload;
    try {
      // Issue #150: empty string from the "Auto-detect" option becomes
      // null on the wire, which clears the stored preference.
      const ccyRaw = (document.getElementById('preferred_currency').value || '').trim();
      const preferredCurrency = ccyRaw === '' ? null : ccyRaw.toUpperCase();

      payload = {
        bio: bio,
        location: (document.getElementById('location').value || '').trim(),
        website_url: validateUrl(document.getElementById('website_url').value, 'Website'),
        social_links: {
          github: validateUrl(document.getElementById('social_github').value, 'GitHub URL'),
          twitter: validateUrl(document.getElementById('social_twitter').value, 'Twitter URL'),
          youtube: validateUrl(document.getElementById('social_youtube').value, 'YouTube URL'),
          mastodon: validateUrl(document.getElementById('social_mastodon').value, 'Mastodon URL')
        },
        featured_badge_slugs: pickedSlugs.slice(0, 3),
        preferred_currency: preferredCurrency
      };
    } catch (err) {
      ChatterAPI.displayError('error-message', err.message);
      return;
    }

    const saveBtn = document.getElementById('save-profile-btn');
    saveBtn.disabled = true;

    try {
      const response = await ProjectAuth.apiFetch(`${PROJECTS_API}/api/users/me/profile`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      if (response.status === 401 || response.status === 403) {
        window.location.href = `/login?return_to=${encodeURIComponent(window.location.pathname)}`;
        return;
      }

      if (response.ok) {
        const data = await response.json().catch(() => null);
        projectsProfile = data || projectsProfile;
        ChatterAPI.displaySuccess('success-message', 'Public profile updated successfully!');
        // Backfill done — hide the prompt if it was visible.
        document.getElementById('backfill-prompt').classList.add('d-none');
      } else {
        const data = await response.json().catch(() => ({}));
        ChatterAPI.displayError('error-message', data.detail || `Failed to update profile (${response.status})`);
      }
    } catch (error) {
      console.error('Update error:', error);
      ChatterAPI.displayError('error-message', 'Failed to update profile. Please try again.');
    } finally {
      saveBtn.disabled = false;
    }
  });

  // Load data on page load
  loadUserData();
</script>
