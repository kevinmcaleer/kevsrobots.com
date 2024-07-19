import yaml
import os
from datetime import datetime
import PyRSS2Gen

# Function to read YAML files
def read_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

# Function to convert date string to datetime object
def parse_date(date_str):
    return datetime.strptime(date_str, '%Y-%m-%d')

# Function to generate RSS feed
def generate_rss(content):
    rss_items = []
    for entry in content:
        item = PyRSS2Gen.RSSItem(
            title=entry['name'],
            link=f"https://www.example.com{entry['link']}",
            description=entry['description'],
            guid=PyRSS2Gen.Guid(f"https://www.example.com{entry['link']}"),
            pubDate=parse_date(entry['date'])
        )
        rss_items.append(item)

    rss = PyRSS2Gen.RSS2(
        title="Your Website Title",
        link="https://www.example.com",
        description="Description of your website",
        lastBuildDate=datetime.now(),
        items=rss_items
    )
    return rss

# Paths to the YAML files
yaml_files = [
    'web/data/blogs.yml',
    'web/data/projects.yml',
    'web/datarobots.yml',
    'web/datavideos.yml',
    'web/datacourses.yml',
    'web/datareviews.yml'
]

# Read content from YAML files
content = []
for file_path in yaml_files:
    content.extend(read_yaml(file_path))

# Generate and save RSS feed
rss = generate_rss(content)
rss.write_xml(open("path/to/save/rss.xml", "w"))

print("RSS feed generated and saved successfully.")
