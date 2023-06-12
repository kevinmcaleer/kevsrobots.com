import os
import re
import yaml

POSTS_FOLDER = 'web/_posts'  # replace with your actual directory path
OUTPUT_FILE = 'tags.yml'

def extract_tags(filepath):
    with open(filepath, 'r') as file:
        content = file.read()
        matches = re.search(r'tags:\n(.*?)(\n---|\Z)', content, re.DOTALL)
        if matches:
            tags_section = matches.group(1)
            tags = re.findall(r'-\s(.*?)\n', tags_section)
            return tags
        return []

def main():
    
    all_tags = []

    for root, dirs, files in os.walk(POSTS_FOLDER):
        for file in files:
            if file.endswith('.md'):
                filepath = os.path.join(root, file)
                tags = extract_tags(filepath)
                all_tags.extend(tags)

    # remove duplicate tags and sort the list
    all_tags = sorted(list(set(all_tags)))

    with open(OUTPUT_FILE, 'w') as file:
        yaml.dump({'tags': all_tags}, file)

if __name__ == '__main__':
    main()
