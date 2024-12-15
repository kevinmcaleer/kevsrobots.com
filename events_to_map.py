import yaml
import requests
import json

def geocode_location(location):
    response = requests.get(f"https://nominatim.openstreetmap.org/search?format=json&q={location}")
    try:
        data = response.json()
    except json.decoder.JSONDecodeError:
        print(f"there was an error in {data}")
    if data:
        return data[0]['lat'], data[0]['lon']
    else:
        return None, None

def process_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        events = yaml.safe_load(file)
    
    processed_events = []
    for event in events:
        if event['location'] == "tbc":
            break
        lat, lon = geocode_location(event['location'])
        if lat and lon:
            event['latitude'] = lat
            event['longitude'] = lon
            processed_events.append(event)

    with open('web/events.json', 'w') as outfile:
        json.dump(processed_events, outfile)

process_yaml('web/_data/events.yml')
