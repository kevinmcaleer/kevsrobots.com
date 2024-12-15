import yaml
from ics import Calendar, Event
from datetime import datetime

EVENTS_YAML_FILE = "web/_data/events.yml"
EVENTS_ICAL_FILE = "web/events.ics"

# Load the YAML file
with open(EVENTS_YAML_FILE, 'r') as file:
    events = yaml.safe_load(file)

# Create a new Calendar
calendar = Calendar()

# Process each event
for item in events:
    event = Event()
    if event.begin == "tbc" or event.begin is None:
       continue
    event.name = item['event']
    event.begin = datetime.fromisoformat(item['start'])
    event.end = datetime.fromisoformat(item['end'])

    if event.location != "tbc":
        event.location = item['location']
    event.url = item['link']
    event.description = f"Status: {item['status']}"
    calendar.events.add(event)

# Export to an ical file
with open(EVENTS_ICAL_FILE, 'w') as file:
    file.writelines(calendar)
