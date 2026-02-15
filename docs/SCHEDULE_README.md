# Show Schedule Management

This directory contains `schedule.yml` which drives the show schedule page and ICS calendar file.

## How It Works

1. **Data File**: `/web/_data/schedule.yml` - Edit this to update your show schedule
2. **Display Page**: `/web/schedule.md` - Shows the schedule on the website
3. **Calendar File**: `/web/schedule.ics` - Auto-generated ICS file for downloads

## Updating the Schedule

### Edit schedule.yml

```yaml
- name: "Show Name"
  description: "Description of the show"
  frequency: "Monthly - First Saturday"  # Human-readable frequency
  time: "14:00"                          # 24-hour format (HH:MM)
  duration: "2h"                         # e.g., "1h 30m", "2h", "45m"
  timezone: "Europe/London"              # IANA timezone
  youtube_channel: "https://www.youtube.com/@kevinmcaleer28"
  status: "active"                       # "active" or "inactive"
  category: "livestream"                 # Category for filtering
  next_dates:                            # Upcoming show dates (YYYY-MM-DD)
    - "2026-03-01"
    - "2026-04-05"
    - "2026-05-03"
```

### Adding Future Dates

Update the `next_dates` array whenever you schedule new shows:

```yaml
next_dates:
  - "2026-03-01"  # Add new dates here
  - "2026-04-05"
  - "2026-05-03"
  - "2026-06-07"  # Keep adding future dates
```

### Deactivating a Show

Change `status` to inactive:

```yaml
status: "inactive"  # Show won't appear on website or in ICS
```

## The ICS File

The ICS file is **automatically generated** by Jekyll when the site builds:

- **No Python script needed!**
- **Always in sync** with `schedule.yml`
- **Updates automatically** when you edit the YAML file
- Users can download from: `https://kevsrobots.com/schedule.ics`

### How It Works

1. You edit `schedule.yml`
2. Jekyll reads the YAML during build
3. `schedule.ics` uses Liquid templating to generate proper ICS format
4. File is ready for download immediately

## Duration Format

Durations can be:
- `"1h"` - 1 hour
- `"30m"` - 30 minutes
- `"1h 30m"` - 1 hour 30 minutes
- `"2h"` - 2 hours

The ICS generator automatically calculates end times.

## Time Zones

The site timezone is configured in `/web/_config.yml` as `site_timezone: "Europe/London"` (GMT/BST). The ICS file includes full timezone rules for automatic DST handling.

To change the timezone:
1. Update `site_timezone` in `/web/_config.yml`
2. All ICS files and schedule pages will use the new timezone automatically

## Testing

1. Update `schedule.yml`
2. Rebuild Jekyll: `cd stacks && docker-compose restart`
3. Visit: `http://localhost:4000/schedule`
4. Download: `http://localhost:4000/schedule.ics`
5. Import ICS file to your calendar app to verify

## Workflow

**Regular maintenance:**
1. Every month, add next month's dates to `next_dates` array
2. Remove old past dates (optional - they won't show on site anyway)
3. Commit and deploy
4. ICS file updates automatically

**One-time shows:**
Just add a new entry with a single date:

```yaml
- name: "Special Event"
  description: "One-time special livestream"
  frequency: "One-time event"
  time: "19:00"
  duration: "2h"
  timezone: "Europe/London"
  youtube_channel: "https://www.youtube.com/@kevinmcaleer28"
  status: "active"
  category: "special"
  next_dates:
    - "2026-03-15"
```

## Calendar Import Instructions

Users can import the ICS file into:

- **Google Calendar**: Settings → Import & Export → Import
- **Apple Calendar**: Double-click the .ics file
- **Outlook**: File → Import/Export → Import an iCalendar file
- **Most other apps**: Support standard .ics import

## Troubleshooting

**ICS file not updating?**
- Clear Jekyll cache: `rm -rf web/_site`
- Rebuild: `docker-compose down && docker-compose up -d`

**Invalid dates in calendar?**
- Check date format is `YYYY-MM-DD`
- Check time format is `HH:MM` (24-hour)
- Verify YAML syntax is correct

**Shows not appearing?**
- Verify `status: "active"`
- Check that `next_dates` array has future dates
- Ensure proper YAML indentation
