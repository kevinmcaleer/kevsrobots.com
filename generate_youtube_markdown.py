import os
import yaml
from datetime import datetime

# Path to the YAML file
YAML_PATH = 'web/_data/youtube.yml'
# Output directory for markdown files
OUTPUT_DIR = 'youtube_markdown'

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Read YAML file
with open(YAML_PATH, 'r') as f:
    videos = yaml.safe_load(f)


for video in videos:
    video_id = video.get('video_id')
    title = video.get('title', 'Untitled')
    published = video.get('published', '')
    # Always output date as yyyy-mm-dd, handling both dash and slash separators
    formatted_date = ''
    if published:
        try:
            # If it's a date/datetime object
            if hasattr(published, 'strftime'):
                formatted_date = published.strftime("%Y-%m-%d")
            else:
                published_str = str(published).replace("/", "-")
                dt = datetime.strptime(published_str, "%Y-%m-%d")
                formatted_date = dt.strftime("%Y-%m-%d")
        except Exception:
            formatted_date = str(published)
    cover = f"https://img.youtube.com/vi/{video_id}/hqdefault.jpg"
    filename = f"{video_id}.md"
    filepath = os.path.join(OUTPUT_DIR, filename)

    # Create markdown content
    md_content = f"""---
title: "{title}"
video_id: {video_id}
published_date: {formatted_date}
cover: {cover}
tags: [youtube]
---

# {title}

[Watch on YouTube](https://www.youtube.com/watch?v={video_id})

![Cover Image]({cover})
"""

    # Write to markdown file
    with open(filepath, 'w') as md_file:
        md_file.write(md_content)

print(f"Created {len(videos)} markdown files in '{OUTPUT_DIR}' directory.")
