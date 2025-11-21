# Stats Dashboard Include Files

This directory contains the include files for the KevsRobots.com statistics dashboard.

## Files Created

1. **stats_dashboard.html** - Main HTML structure for the dashboard
2. **stats_dashboard.js** - JavaScript logic for fetching and rendering data
3. **stats_dashboard.css** - Styling for the dashboard components

## Usage

To add the stats dashboard to a page, simply include the HTML file:

```liquid
---
title: Your Page Title
layout: content
---

<style>
{% include stats_dashboard.css %}
</style>

{% include stats_dashboard.html %}
```

## Features

- **Real-time data** - Fetches from https://stats.kevsrobots.com/api/dashboard
- **Auto-refresh** - Updates every 5 minutes
- **Responsive design** - Works on mobile and desktop
- **No PII** - All data is aggregated and anonymized

## Charts Included

1. Line Chart - Searches over time (last year)
2. Pie Charts - Device, OS, and content type breakdowns
3. Lists - Top searches, trending searches, popular pages

## Customization

The CSS uses CSS variables for theming. You can customize colors by defining:

- `--card-bg` - Background color for cards
- `--text-primary` - Primary text color
- `--text-secondary` - Secondary text color
- `--accent-color` - Accent color for stats
- `--link-color` - Link color
- `--border-color` - Border color

## API Endpoint

The dashboard fetches data from:
```
https://stats.kevsrobots.com/api/dashboard
```

This endpoint is updated hourly by the DuckDB analytics container.

## Dependencies

- Chart.js 4.4.0 (loaded from CDN)
- Modern browser with ES6 support

## Example Page

See `/web/stats.md` for a complete example of how to use the dashboard.
