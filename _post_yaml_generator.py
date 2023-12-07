# Generate a yaml file from the _posts directory

import yaml
import os
import sys
import re
import datetime

def read_front_matter(post):
    with open(post, 'r') as f:
        front_matter = f.readline()
        front_matter = front_matter.replace('---', '')
        front_matter = front_matter.replace('


# Get the path to the _posts directory
posts_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '_posts')

posts = os.dir(posts_dir)

for post in posts:
    front_matter = read_front_matter(post)
    print(front_matter)



