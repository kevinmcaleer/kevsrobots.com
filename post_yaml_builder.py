import os
import re
import yaml

def extract_front_matter(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    front_matter_regex = re.compile(r'^---\n(.*?)\n---\n', re.DOTALL | re.MULTILINE)
    match = front_matter_regex.search(content)

    if match:
        return yaml.safe_load(match.group(1))
    else:
        return None

def main():
    input_folder = 'web/_posts'
    output_file = 'web/_data/posts.yaml'

    metadata_list = []

    for filename in os.listdir(input_folder):
        if (filename.endswith('.md') or filename.endswith('.markdown')) and not filename.startswith('_') :
            file_path = os.path.join(input_folder, filename)

            front_matter = extract_front_matter(file_path)

            if front_matter:
                metadata_list.append({
                    'title': front_matter.get('title', 'Untitled'),
                    'url': front_matter.get('url', f'/{filename}'),
                    'date': front_matter.get('date', 'unknown'),
                })

    with open(output_file, 'w', encoding='utf-8') as file:
        yaml.dump({'posts': metadata_list}, file, sort_keys=False)

if __name__ == '__main__':
    main()
