import requests
import yaml
import time

# Cloudflare API Settings
CLOUDFLARE_API_TOKEN = "K_F7Ab3rtoE-6wdjKKTNrovLqFBVNf-3jCFB39a8"
ZONE_ID = "e06ab6d949dd97494778641a47a50c6b"  # Get this from Cloudflare Dashboard

# Headers for the request
HEADERS = {
    "Authorization": f"Bearer {CLOUDFLARE_API_TOKEN}",
    "Content-Type": "application/json"
}

# GraphQL query for analytics
QUERY = {
    "query": """
    query {
      viewer {
        zones(filter: { zoneTag: "%s" }) {
          httpRequestsAdaptiveGroups(
            limit: 10000,
            filter: { datetime_geq: "%s", datetime_leq: "%s" },
            orderBy: [count_DESC],
            groupBy: [clientRequestPath]
          ) {
            count
            dimensions {
              clientRequestPath
            }
          }
        }
      }
    }
    """
}

# Get date range (last 7 days)
def get_date_range():
    today = time.strftime("%Y-%m-%dT00:00:00Z")
    last_week = time.strftime("%Y-%m-%dT00:00:00Z", time.gmtime(time.time() - 7 * 86400))
    return last_week, today

# Fetch data from Cloudflare
def fetch_popularity():
    start_date, end_date = get_date_range()
    query = QUERY["query"] % (ZONE_ID, start_date, end_date)
    response = requests.post(
        "https://api.cloudflare.com/client/v4/graphql",
        headers=HEADERS,
        json={"query": query}
    )
    
    if response.status_code != 200:
        print("Error fetching data:", response.json())
        return []
    
    data = response.json()
    return data["data"]["viewer"]["zones"][0]["httpRequestsAdaptiveGroups"]

# Process data into categories
def categorize_pages(data):
    categories = {
        "blog": {},
        "projects": {},
        "reviews": {},
        "learn": {}
    }
    
    for entry in data:
        path = entry["dimensions"]["clientRequestPath"]
        count = entry["count"]
        
        if path.startswith("/blog/"):
            categories["blog"][path] = count
        elif path.startswith("/projects/"):
            categories["projects"][path] = count
        elif path.startswith("/reviews/"):
            categories["reviews"][path] = count
        elif path.startswith("/learn/"):
            categories["learn"][path] = count
    
    return categories

# Save data to YAML for Jekyll
def save_to_yaml(categories):
    with open("web/_data/popularity.yaml", "w") as f:
        yaml.dump(categories, f, default_flow_style=False)
    print("Popularity data saved to _data/popularity.yaml")

# Run the script
if __name__ == "__main__":
    print("Fetching popular pages from Cloudflare...")
    raw_data = fetch_popularity()
    
    if raw_data:
        categorized_data = categorize_pages(raw_data)
        save_to_yaml(categorized_data)
        print("Done! Update Jekyll templates to use popularity data.")
