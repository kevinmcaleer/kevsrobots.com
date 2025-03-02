import yaml
import requests
import json
import time

def geocode_location(location):
    location_string = f"https://nominatim.openstreetmap.org/search?format=json&q={location}"
    headers = {
        "User-Agent": "event_to_map/1.0 (kevinmcaleer@gmail.com)"  # Replace with your app info
    }

    print(f'geocoding {location} with {location_string}')
    try:
        response = requests.get(location_string, headers=headers)
        response.raise_for_status() # Check for HTTP errors

        try:
            data = response.json()
            if data:
                return data[0]['lat'], data[0]['lon']
            else:
                print(f"No results found for {location}")
                return None, None
        except json.JSONDecodeError as e:
            print(f"there was an error in {e} for location {location}. Response Text: {response.text}")
            if response.status_code == 403:
                print("Received 403 error, waiting then retrying")
                time.sleep(10)
                return geocode_location(location)
            return None, None

    except requests.exceptions.RequestException as e:
        print(f"Error during request for {location}: {e}")
        if response.status_code == 403:
            print("Received 403 error, waiting then retrying")
            time.sleep(10)
            return geocode_location(location)
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
        time.sleep(1) #respect rate limits

    with open('web/events.json', 'w') as outfile:
        json.dump(processed_events, outfile, indent=4) #Added indent for readability

process_yaml('web/_data/events.yml')