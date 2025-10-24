# Like and Comment Component

A reusable Jekyll include component that adds like and comment functionality to any page.

## Features

- **Like Button**: Heart-shaped button that shows outline when not liked, filled (red) when liked
- **Like Count**: Displays the number of likes next to the heart icon
- **Comment Input**: Text area for posting comments with "Post your comment" placeholder
- **Comments List**: Shows all comments with username and relative time (e.g., "2h", "3d")
- **Report Menu**: Hamburger menu on each comment for reporting (placeholder for issue #35)
- **Authentication**: Automatically redirects to login page if user tries to like/comment without being logged in
- **Responsive**: Works on mobile and desktop
- **Real-time**: Uses Chatter API to fetch and update likes/comments

## Usage

Simply add this line to any Jekyll page where you want likes and comments:

```liquid
{% include like-comment.html %}
```

The component will automatically use `page.url` as the unique identifier for likes and comments.

## Example

```markdown
---
layout: content
title: My Blog Post
---

# My Blog Post

This is my awesome blog post content...

{% include like-comment.html %}
```

## Requirements

- User must be logged in to like or comment
- All users (logged in or not) can see like counts and comments
- Requires Chatter API to be running at `https://chatter.kevsrobots.com`
- Requires Bootstrap 5.3.3 for styling
- Requires Font Awesome 6 for icons

## API Endpoints Used

- `GET /interact/likes/{url}` - Get like count
- `GET /interact/user-like-status/{url}` - Check if user has liked (auth required)
- `POST /interact/like` - Toggle like/unlike (auth required)
- `POST /interact/comment` - Post a comment (auth required)
- `GET /interact/comments/{url}` - Get all comments

## Styling

The component includes inline styles for:
- Like button hover/active states
- Comment card styling
- Input focus states

You can override these styles in your site's CSS if needed.

## Future Enhancements

- Report comment functionality (issue #35)
- Edit/delete own comments
- Like animation effects
- Comment pagination for posts with many comments
- Markdown support in comments
