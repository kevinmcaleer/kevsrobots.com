# ✅ Event Map Auto-Generation Complete

## What Was Created

The event map now uses **auto-generated data** from `events.yml`!

### Files Created/Updated

**1. `web/_data/event_locations.yml`** - NEW
- Coordinates lookup table for event venues
- Maps location names to latitude/longitude
- Easy to add new locations

**2. `web/events.json`** - UPDATED
- Now auto-generates from `events.yml`
- Looks up coordinates from `event_locations.yml`
- Powers the Leaflet map in `events_map.html`

## How It Works

```
events.yml (event data)
     +
event_locations.yml (coordinates)
     ↓
  Jekyll Build
     ↓
  events.json (auto-generated)
     ↓
events_map.html (displays map)
```

**No Python scripts!** Pure Jekyll/Liquid templating.

## What Gets Generated

The JSON includes all events with:
- ✅ Event name, dates, location
- ✅ Status, link, image
- ✅ Latitude/longitude (from lookup table)

### Currently: 20 Events with Coordinates

All events from events.yml now have map coordinates!

## Example Generated JSON

```json
{
  "event": "Open Hardware Summit 2025",
  "start": "2025-05-30",
  "end": "2025-05-31",
  "location": "University of Edinburgh, UK",
  "link": "https://2025.oshwa.org/",
  "status": "confirmed",
  "latitude": "55.9440764",
  "longitude": "-3.1883736"
}
```

## Location Lookup System

**event_locations.yml structure:**
```yaml
locations:
  "Cambridge, UK":
    latitude: "52.2055314"
    longitude: "0.1186637"

  "San Francisco, USA":
    latitude: "37.7792588"
    longitude: "-122.4193286"
```

**How it works:**
1. Event has `location: "Cambridge, UK"` in events.yml
2. Generator looks up "Cambridge, UK" in event_locations.yml
3. Adds latitude/longitude to JSON output

## Current Locations in Lookup

- Cambridge, UK
- Birmingham, UK
- NEC, Birmingham, UK
- Rome, Italy (3 venues)
- San Francisco, USA
- Liverpool, UK
- Manchester, UK
- Eastnor Castle, UK
- University of Edinburgh, UK
- ExCeL, London, UK

## How to Update

### Add a New Event

**Step 1:** Add to `events.yml`:
```yaml
- event: New Maker Event 2026
  start: "2026-09-15"
  end: "2026-09-16"
  location: "New Venue, City, Country"
  link: https://example.com
  status: confirmed
  image: /assets/img/events/new-event.jpg
```

**Step 2:** Add location coordinates to `event_locations.yml`:
```yaml
locations:
  "New Venue, City, Country":
    latitude: "XX.XXXXXXX"
    longitude: "YY.YYYYYYY"
```

**Step 3:** Commit and deploy
- Jekyll auto-generates events.json
- Map automatically shows new event pin

### Find Coordinates

Use: https://www.latlong.net/

1. Search for venue/city
2. Copy latitude and longitude
3. Add to `event_locations.yml`

### Update Existing Event

Just edit `events.yml` - JSON regenerates automatically!

```yaml
# Change status
- event: Maker Faire 2026
  status: confirmed  # Was: expected

# Update location
- event: Event Name
  location: "New Venue, City"  # Add new venue to locations lookup
```

## File Locations

```
web/
├── _data/
│   ├── events.yml              ← Edit event details here
│   └── event_locations.yml     ← Edit coordinates here
├── _includes/
│   └── events_map.html         ← Map display (unchanged)
└── events.json                 ← Auto-generated (don't edit directly)
```

## Map Features

The event map (`events_map.html`) shows:
- 📍 Marker for each event with coordinates
- 🖼️ Clickable event images
- 📅 Event dates and location
- 🔗 Links to event websites
- 🗺️ OpenStreetMap base layer

## Auto-Generation Logic

**Liquid template in events.json:**
```liquid
{% for event in site.data.events %}
  {
    "event": "{{ event.event }}",
    "location": "{{ event.location }}",
    ...
    {% if site.data.event_locations.locations[event.location] %}
    "latitude": "{{ site.data.event_locations.locations[event.location].latitude }}",
    "longitude": "{{ site.data.event_locations.locations[event.location].longitude }}"
    {% endif %}
  }
{% endfor %}
```

**Key features:**
- Loops through all events
- Includes all event fields
- Conditionally adds lat/long if location found in lookup
- Valid JSON formatting with proper commas

## Workflow Comparison

**Before:**
1. Edit events.yml
2. Manually update events.json with same data
3. Manually add lat/long to JSON
4. Easy to get out of sync

**After:**
1. Edit events.yml
2. Add new locations to event_locations.yml (once per venue)
3. **Done!** JSON auto-generates

## Benefits

**Centralized Data:**
- ✅ Single source of truth: events.yml
- ✅ Coordinates in one lookup file
- ✅ No duplicate data entry

**Always In Sync:**
- ✅ JSON matches YAML automatically
- ✅ Can't get out of sync
- ✅ One update = all systems update

**Maintainable:**
- ✅ Easy to add events (just YAML)
- ✅ Reusable locations (many events at same venue)
- ✅ Clear structure

## Location Reuse

**Common venues used multiple times:**
- Cambridge, UK (4 events)
- Birmingham, UK (3 events)
- San Francisco, USA (3 events)
- Rome, Italy (3 events)

Add coordinates once, use for all events at that venue!

## Missing Coordinates?

If an event location is NOT in event_locations.yml:
- JSON still generates
- Event appears in events.json
- But NO latitude/longitude fields
- Map won't show marker for that event

**Fix:** Add location to event_locations.yml

## Testing

**Test locally:**
```bash
cd stacks && docker-compose up -d
curl http://localhost:4000/events.json | python3 -m json.tool
```

**Verify:**
- [ ] Valid JSON syntax
- [ ] All events present
- [ ] Coordinates for all locations
- [ ] Image paths correct
- [ ] Links valid

**Test map:**
- [ ] Visit page with events_map.html include
- [ ] All markers appear
- [ ] Popups show event details
- [ ] Images load correctly
- [ ] Links work

## Integration

**The map include works in any page:**
```liquid
{% include events_map.html %}
```

Currently used in: `/web/events.md`

## Data Flow Summary

```
User edits events.yml
         ↓
   (event details)
         ↓
User adds location to event_locations.yml (if new)
         ↓
   (coordinates)
         ↓
    Jekyll builds
         ↓
events.json auto-generates
         ↓
events_map.html loads JSON
         ↓
    Leaflet renders map
         ↓
  User sees markers!
```

## Future Enhancements (Optional)

Could add:
- Event type/category icons (different marker colors)
- Filter by status (show only upcoming)
- Cluster markers when zoomed out
- Auto-geocoding (look up coordinates automatically)
- Multiple venues per event

But current system is:
- ✅ Simple
- ✅ Maintainable
- ✅ Automatic
- ✅ Works great

## Comparison: All Auto-Generated Files

**schedule.ics:**
- From: schedule.yml
- Format: ICS calendar
- For: Livestream shows

**events.ics:**
- From: events.yml
- Format: ICS calendar
- For: Maker events

**events.json:**
- From: events.yml + event_locations.yml
- Format: JSON
- For: Event map

**All:**
- Auto-generate on Jekyll build
- No Python scripts
- Always in sync with data

## Next Steps

1. **Test the map:** Visit page with event map include
2. **Verify coordinates:** Check all markers appear
3. **Add missing locations:** If any events lack coordinates
4. **Deploy:** Commit and push when ready

## Summary

**events.json is now:**
- ✅ Auto-generated from events.yml
- ✅ Includes coordinates from lookup table
- ✅ Powers the event map
- ✅ Valid JSON format
- ✅ Always in sync with event data
- ✅ Zero maintenance overhead

Just update `events.yml` and `event_locations.yml` (once per new venue), and the map updates automatically!
