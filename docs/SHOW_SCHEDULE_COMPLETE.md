# ✅ Show Schedule Implementation Complete

## What's Live

Your show schedule system is now fully operational! Here's what was created:

### 📁 Files Created

1. **`web/_data/schedule.yml`** - Your show schedule database
   - Easy YAML format for adding/updating shows
   - Already populated with your two shows:
     - Robotlab Hangout Live (Monthly, Sundays, 19:00, 1h)
     - Code to Creation Series (Fortnightly, Sundays, 19:00, 20m)

2. **`web/schedule.md`** - Public schedule page
   - Beautiful Bootstrap card layout
   - Shows upcoming dates (next 3 per show)
   - Links to YouTube
   - Download button for ICS file
   - URL: https://kevsrobots.com/schedule

3. **`web/schedule.ics`** - Auto-generated calendar file
   - Valid ICS format compatible with all major calendar apps
   - Includes timezone support (Europe/London with DST rules)
   - Updates automatically when Jekyll rebuilds
   - URL: https://kevsrobots.com/schedule.ics

4. **Navigation updated** - Added to Videos menu
   - Shows as "Show Schedule" between Livestreams and Videos

5. **Documentation**
   - `SCHEDULE_QUICKSTART.md` - Quick reference guide
   - `web/_data/SCHEDULE_README.md` - Full maintenance documentation

## 🎯 How to Use

### Add a New Show

Edit `web/_data/schedule.yml`:

```yaml
- name: "New Show Name"
  description: "What the show is about"
  frequency: "Weekly - Fridays"
  time: "20:00"
  duration: "1h 30m"
  timezone: "Europe/London"
  youtube_channel: "https://www.youtube.com/@kevinmcaleer28"
  status: "active"
  category: "workshop"
  next_dates:
    - "2026-02-21"
    - "2026-02-28"
```

### Update Show Dates

Just edit the `next_dates` array in `schedule.yml`:

```yaml
next_dates:
  - "2026-03-01"
  - "2026-04-05"
  - "2026-05-03"  # Add new dates as needed
```

### Test Locally

```bash
cd stacks
docker-compose up -d
```

Visit:
- http://localhost:4000/schedule
- http://localhost:4000/schedule.ics

### Deploy

```bash
git add web/_data/schedule.yml
git commit -m "Update show schedule"
git push
```

The ICS file updates automatically - no scripts to run!

## ✨ Features

- **Zero Maintenance**: No Python scripts needed
- **Always in Sync**: ICS file auto-generates from YAML
- **User-Friendly**: Download or subscribe to calendar
- **Professional**: Valid ICS format with timezone support
- **Flexible**: Supports any duration format:
  - `"30m"` - minutes only
  - `"2h"` - hours only
  - `"1h 30m"` - hours and minutes

## 📅 Duration Calculation

The system automatically calculates end times:
- Start: 19:00, Duration: "20m" → End: 19:20 ✓
- Start: 19:00, Duration: "1h" → End: 20:00 ✓
- Start: 14:00, Duration: "1h 30m" → End: 15:30 ✓

## 🌐 URLs

- **Schedule Page**: https://kevsrobots.com/schedule
- **ICS Download**: https://kevsrobots.com/schedule.ics
- **Navigation**: Videos → Show Schedule

## 📱 Calendar Import

Users can import your ICS file into:
- Google Calendar (Settings → Import)
- Apple Calendar (Double-click file)
- Outlook (File → Import)
- Any other calendar app (standard .ics format)

## 🔄 Workflow

**Your monthly routine:**
1. Open `web/_data/schedule.yml`
2. Add next month's dates to `next_dates` arrays
3. Commit and push
4. Done! Jekyll automatically updates the ICS file

## 📊 Current Schedule

### Robotlab Hangout Live
- **When**: Monthly - First Sunday
- **Time**: 19:00 GMT/BST
- **Duration**: 1 hour
- **Upcoming**: Mar 1, Apr 5, May 3, Jun 7, Jul 5, Aug 2

### Code to Creation Series
- **When**: Fortnightly - Sundays
- **Time**: 19:00 GMT/BST
- **Duration**: 20 minutes
- **Upcoming**: Feb 22, Mar 8, Mar 22, Apr 5, Apr 19

## 🎨 Optional Enhancement

Add a cover image for the schedule page:
- Create: `web/assets/img/schedule/schedule-cover.jpg`
- Size: 1200x630px
- Optimize: Run `python3 optimize_images.py`
- Update `web/schedule.md` frontmatter with `cover:` field

## 🐛 Troubleshooting

**ICS not updating?**
```bash
cd stacks && docker-compose restart
```

**Shows not appearing?**
- Verify `status: "active"`
- Check dates are in future
- Validate YAML syntax (indentation matters)

**Wrong times in calendar?**
- Check format: `"HH:MM"` (24-hour, with quotes)
- Duration format: `"1h"`, `"30m"`, or `"1h 30m"`

## 🚀 Next Steps

1. **Test the ICS file**: Download from localhost and import to your calendar
2. **Add cover image**: Optional but makes the page look nicer
3. **Deploy to production**: Commit and push when ready
4. **Share the link**: Tell your audience about the schedule page

## 📝 Architecture

```
User edits schedule.yml
         ↓
    Jekyll reads YAML
         ↓
    ┌────┴────┐
    ↓         ↓
schedule.md  schedule.ics
    ↓         ↓
  HTML Page  Calendar File
```

**No Python scripts** - Pure Jekyll/Liquid templating!

## 🎉 Success!

Your show schedule system is complete and ready to use. It's:
- ✅ Easy to maintain (just edit YAML)
- ✅ Automatic (ICS generates on every build)
- ✅ Professional (valid calendar format)
- ✅ User-friendly (download or subscribe)
- ✅ Integrated (shows in navigation)

See `SCHEDULE_QUICKSTART.md` for quick reference or `web/_data/SCHEDULE_README.md` for full documentation.
