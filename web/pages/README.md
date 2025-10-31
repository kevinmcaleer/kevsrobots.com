# Profile Page - Static HTML Version

## Overview

This directory contains a static HTML version of the user profile page that can be easily edited without rebuilding the Chatter Docker container.

## Usage

### Accessing Profiles

The profile page uses URL parameters to load user data:

```
https://www.kevsrobots.com/pages/profile.html?username=kev
```

Replace `kev` with any username to view that user's profile.

### How It Works

1. The page reads the `username` parameter from the URL
2. Fetches profile data from the Chatter API: `GET https://chatter.kevsrobots.com/profile/{username}`
3. Fetches recent comments from: `GET https://chatter.kevsrobots.com/profile/{username}/comments?limit=10`
4. Dynamically populates the page with the user's information

### Features

- **Profile Picture**: Shows user's uploaded picture or a letter avatar
- **User Info**: Name, username, location, bio
- **Join Date**: When the user created their account
- **Comment Count**: Total number of comments
- **Recent Comments**: Last 10 comments with links to the pages
- **Edit Button**: Appears for users viewing their own profile

### Customization

You can easily customize the styling and layout by editing `profile.html`:

- **Styling**: Modify Bootstrap classes or add custom CSS
- **Layout**: Rearrange sections or add new ones
- **Content**: Change text, icons, or add new fields
- **Comments Limit**: Change `?limit=10` to show more/fewer comments

### API Endpoints Used

1. `GET /profile/{username}` - Returns:
   ```json
   {
     "username": "kev",
     "firstname": "Kevin",
     "lastname": "McAleer",
     "location": "United Kingdom",
     "bio": "Robotics enthusiast...",
     "profile_picture_url": "https://chatter.kevsrobots.com/profile_pictures/123.jpg",
     "created_at": "2025-10-23T08:19:19.397493",
     "comment_count": 14,
     "is_own_profile": false
   }
   ```

2. `GET /profile/{username}/comments?limit=10` - Returns:
   ```json
   {
     "username": "kev",
     "comments": [
       {
         "id": 39,
         "url": "ideas/halloween.html",
         "content": "Happy Halloween!",
         "created_at": "2025-10-30T16:53:16.343641",
         "edited_at": null
       }
     ]
   }
   ```

### Integration Tips

You can link to profiles from anywhere on your site:

```html
<!-- Link to a profile -->
<a href="/pages/profile.html?username=kev">View Profile</a>

<!-- Link with user's display name -->
<a href="/pages/profile.html?username={{ username }}">
  {{ firstname }} {{ lastname }}
</a>
```

### Jekyll Integration

If you want to use Jekyll's templating features, you can convert this to a Jekyll layout or include. The current version is standalone HTML with JavaScript that works without Jekyll processing.

## Issue Reference

Created as part of **Issue #44**: User Profiles feature
