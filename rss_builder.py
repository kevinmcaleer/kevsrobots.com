# Kevin McAleer
# April 2023
# RSS Builder

import datetime
from feedgen.feed import FeedGenerator
import yaml

"""
This module builds the RSS feed for the site, including:

blog articles
videos
courses
Reviews
Gear List
Projects
Robots

"""

YOUTUBE_YAML = 'web/_data/youtube.yml'

def generate_from_youtube():
    """ Generate the RSS feed from the youtube yaml file """

    # read the youtube yaml file
    with open(YOUTUBE_YAML, 'r') as file:
        data = yaml.safe_load(file)

    # create the RSS feed
    rss = FeedGenerator(
        title='Kevin McAleer - YouTube',
        link='https://www.youtube.com/channel/UC4WgR8Kv7VxMx0e5cVWYg8Q',
        description='YouTube videos by Kevin McAleer',
        lastBuildDate=datetime.datetime.now()
    )

    # add the items
    for item in data:
        rss.add_item(
            title=item['title'],
            link=f"https://www.youtube.com/watch={item['video_id']}",
            pubDate=item['published']
        ) 

    # write the RSS feed
    with open('web/feeds/youtube.xml', 'w') as file:
        file.write(rss.to_xml())
    