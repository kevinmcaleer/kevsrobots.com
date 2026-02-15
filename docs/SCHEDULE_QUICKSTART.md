# Show Schedule - Quick Start Guide

## What Was Created

✅ **Data File**: `web/_data/schedule.yml` - Your show schedule database
✅ **Display Page**: `web/schedule.md` - Public-facing schedule page
✅ **ICS Generator**: `web/schedule.ics` - Auto-generated calendar file
✅ **Navigation**: Added "Show Schedule" to Videos menu
✅ **Documentation**: `web/_data/SCHEDULE_README.md` - Full maintenance guide

## How It Works

```
schedule.yml  ───┐
                 ├──> Jekyll Build ───┐
                 │                     ├──> schedule page (HTML)
                 └───────────────────> schedule.ics file
```

**No Python scripts needed!** The ICS file is generated automatically by Jekyll using Liquid templates.

## Quick Update Workflow

### 1. Add a New Show

Edit `web/_data/schedule.yml`:

```yaml
- name: "Your Show Name"
  description: "What the show is about"
  frequency: "Weekly - Thursdays"
  time: "19:00"                      # 24-hour format
  duration: "1h 30m"
  timezone: "Europe/London"
  youtube_channel: "https://www.youtube.com/@kevinmcaleer28"
  status: "active"
  category: "livestream"
  next_dates:
    - "2026-02-20"                   # Add upcoming dates here
    - "2026-02-27"
    - "2026-03-06"
```

### 2. Update Existing Show Dates

Just edit the `next_dates` array:

```yaml
next_dates:
  - "2026-03-01"
  - "2026-04-05"
  - "2026-05-03"
  - "2026-06-07"  # Add new dates as you schedule them
```

### 3. Test Locally

```bash
cd stacks
docker-compose up -d
```

Visit:
- Schedule page: http://localhost:4000/schedule
- ICS file: http://localhost:4000/schedule.ics

### 4. Deploy

```bash
git add web/_data/schedule.yml
git commit -m "Update show schedule"
git push
```

The ICS file updates automatically when Jekyll rebuilds!

## URLs

- **Schedule Page**: https://kevsrobots.com/schedule
- **ICS Download**: https://kevsrobots.com/schedule.ics
- **Navigation**: Videos menu → "Show Schedule"

## Sample Data Included

The `schedule.yml` file includes a sample "Robotlab Hangout Live" entry with dates through August 2026. Update this with your actual show details.

## Calendar Subscription

Users can:
1. Download the ICS file from your site
2. Import it into Google Calendar, Apple Calendar, Outlook, etc.
3. Get automatic reminders for your shows

## Maintenance Tips

**Monthly routine:**
1. Open `web/_data/schedule.yml`
2. Add next month's show dates to `next_dates`
3. Remove old past dates (optional)
4. Commit and push

**That's it!** Jekyll handles the rest automatically.

## File Locations

```
web/
├── _data/
│   ├── schedule.yml              ← Edit this to update schedule
│   └── SCHEDULE_README.md        ← Full documentation
├── schedule.md                   ← Display page
├── schedule.ics                  ← ICS generator (auto-updates)
└── assets/img/schedule/          ← Add cover image here (optional)
```

## Example: Adding a One-Time Special Event

```yaml
- name: "Pi Day Special - Build a Raspberry Pi Robot"
  description: "Special Pi Day livestream building a robot from scratch"
  frequency: "One-time event"
  time: "14:00"
  duration: "3h"
  timezone: "Europe/London"
  youtube_channel: "https://www.youtube.com/@kevinmcaleer28"
  status: "active"
  category: "special-event"
  next_dates:
    - "2026-03-14"
```

## Troubleshooting

**ICS file not updating?**
```bash
cd stacks
docker-compose restart
```

**Shows not appearing?**
- Check `status: "active"`
- Verify future dates in `next_dates`
- Check YAML syntax (indentation matters!)

**Invalid calendar import?**
- Dates must be `YYYY-MM-DD`
- Times must be `HH:MM` (24-hour)
- Duration format: `"1h"`, `"30m"`, or `"1h 30m"`

## Need Help?

See full documentation in `web/_data/SCHEDULE_README.md`
