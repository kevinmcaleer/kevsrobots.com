# ✅ Events Calendar Auto-Generation Complete

## What Was Created

The `/events.ics` file now **auto-generates** from `events.yml` - just like the schedule calendar!

### How It Works

```
events.yml (your events database)
     ↓
  Jekyll Build
     ↓
  events.ics (auto-generated)
```

**No Python scripts needed!** Pure Jekyll/Liquid templating.

## What Gets Included

The ICS file automatically includes:
- ✅ Events with `status: confirmed`
- ✅ Events with `status: expected`
- ❌ Skips events with `status: closed` (past events)

### Currently Generating 7 Events

1. **Open Hardware Summit 2025** - May 30-31, Edinburgh
2. **Makers Central 2025** - May 17-18, Birmingham
3. **Liverpool Makefest 2025** - July 5, Liverpool
4. **Open Sauce 2025** - July 18-20, San Francisco
5. **Makers Faire Rome 2025** - Oct 17-19, Rome
6. **CamJam Raspberry Pi Birthday Party 2025** - Mar 1, Cambridge
7. **SMRRF 2026** - Mar 7-8, Manchester

## ICS File Features

**Proper All-Day Events:**
- Uses `DTSTART;VALUE=DATE` format (not datetime)
- Correctly adds +1 day to end date (ICS spec requirement)
- Events span correctly in calendar apps

**Event Details:**
- Event name (SUMMARY)
- Location (LOCATION field + description)
- Event URL (URL field + description)
- Status: CONFIRMED or TENTATIVE
- Categories: MAKER-EVENT, CONFERENCE

**Calendar Metadata:**
- Calendar name: "KevsRobots Maker Events"
- Description: "Maker faires, conferences, and events featuring KevsRobots"
- Unique UIDs for each event

## Example ICS Output

```ics
BEGIN:VEVENT
UID:open-hardware-summit-2025-20250530@kevsrobots.com
DTSTART;VALUE=DATE:20250530
DTEND;VALUE=DATE:20250601
SUMMARY:Open Hardware Summit 2025
DESCRIPTION:Location: University of Edinburgh, UK

Event link: https://2025.oshwa.org/
LOCATION:University of Edinburgh, UK
URL:https://2025.oshwa.org/
STATUS:CONFIRMED
TRANSP:TRANSPARENT
CATEGORIES:MAKER-EVENT,CONFERENCE
END:VEVENT
```

## Status Mapping

**events.yml → ICS:**
- `status: confirmed` → `STATUS:CONFIRMED`
- `status: expected` → `STATUS:TENTATIVE`
- `status: closed` → Not included (filtered out)

## How to Update

### Add a New Event

Edit `web/_data/events.yml`:

```yaml
- event: New Maker Event 2026
  start: "2026-08-15"
  end: "2026-08-16"
  location: "Your City, Country"
  link: https://example.com
  status: confirmed
  image: /assets/img/events/new-event.jpg
```

### Update Event Status

```yaml
# Change from expected to confirmed when registration opens
- event: Maker Faire 2026
  status: confirmed  # Was: expected
```

### Mark Event as Closed

```yaml
# After event ends, mark as closed
- event: Past Event
  status: closed  # Will no longer appear in ICS
```

## File Locations

```
web/
├── _data/
│   └── events.yml           ← Edit this to update events
└── events.ics               ← Auto-generated (don't edit directly)
```

## URLs

- **ICS Download:** https://kevsrobots.com/events.ics
- **Schedule ICS:** https://kevsrobots.com/schedule.ics (livestreams)

## Use Cases

**Users can:**
1. Download `/events.ics` to import into their calendar
2. Subscribe to the URL for automatic updates
3. See all upcoming maker events you're attending/exhibiting at

**Calendar Apps:**
- Google Calendar (Import or subscribe)
- Apple Calendar (Double-click to import)
- Outlook (Import ICS file)
- Any standard calendar app

## Workflow

**Your monthly routine:**
1. Update `events.yml` when you confirm new events
2. Change `status: expected` → `confirmed` when ready
3. Mark past events as `closed` after they finish
4. Commit and push

**That's it!** Jekyll automatically regenerates `events.ics` on every build.

## All-Day Event Handling

**Important:** The ICS file correctly handles all-day events:

```liquid
{% comment %}Add +1 day to end date (ICS spec requirement){% endcomment %}
{% assign end_timestamp = event.end | date: "%s" | plus: 86400 %}
{% assign event_end_plus_one = end_timestamp | date: "%Y%m%d" %}
```

**Why:** ICS spec requires all-day events to have an exclusive end date (day after the last day).

**Example:**
- Event: March 1-2
- YAML: `start: "2026-03-01"`, `end: "2026-03-02"`
- ICS: `DTSTART:20260301`, `DTEND:20260303` (adds +1 day)

## Comparison: Schedule vs Events

**schedule.ics:**
- Livestream shows
- Timed events (19:00 GMT/BST)
- Recurring schedule
- Short duration (20m-2h)

**events.ics:**
- Maker faires and conferences
- All-day events
- One-time events
- Multi-day duration

**Both:**
- Auto-generated from YAML
- Update automatically
- Downloadable calendars
- No Python scripts needed

## Testing

**Test locally:**
```bash
cd stacks && docker-compose up -d
curl http://localhost:4000/events.ics
```

**Verify:**
- [ ] Only confirmed/expected events appear
- [ ] Closed events are filtered out
- [ ] All-day dates are correct
- [ ] Location and URL are included
- [ ] File ends with END:VCALENDAR

**Import test:**
- [ ] Download events.ics
- [ ] Import to your calendar app
- [ ] Verify events appear on correct dates
- [ ] Check multi-day events span correctly

## Benefits

**Before:**
- Static ICS file
- Manual updates needed
- Could get out of sync with events.yml

**After:**
- ✅ Auto-generated from YAML
- ✅ Always in sync with events.yml
- ✅ No manual updates needed
- ✅ Filter by status automatically
- ✅ Proper all-day event formatting
- ✅ Professional calendar metadata

## Technical Details

**Liquid Logic:**
1. Loop through all events in `events.yml`
2. Filter: only `status: confirmed` or `expected`
3. Convert dates to ICS format (YYYYMMDD)
4. Add +1 day to end date for all-day events
5. Generate unique UID from event name + date
6. Set STATUS based on event status
7. Include all metadata (location, URL, etc.)

**Date Calculation:**
```liquid
{% assign end_timestamp = event.end | date: "%s" | plus: 86400 %}
{% assign event_end_plus_one = end_timestamp | date: "%Y%m%d" %}
```

Converts date to Unix timestamp, adds 86400 seconds (1 day), converts back to YYYYMMDD format.

## Next Steps

1. **Test the ICS file:** Download and import to your calendar
2. **Update events.yml:** Add any new upcoming events
3. **Deploy:** Commit and push when ready

## Summary

**events.ics is now:**
- ✅ Auto-generated from events.yml
- ✅ Filters out closed events
- ✅ Properly formatted all-day events
- ✅ Includes location and links
- ✅ Professional calendar metadata
- ✅ Zero maintenance overhead

Just like `schedule.ics`, it's **fully automatic** and always in sync with your data!
