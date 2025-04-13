import os
import re
import yaml

def extract_front_matter(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    front_matter_regex = re.compile(r'^---\n(.*?)\n---\n', re.DOTALL | re.MULTILINE)
    match = front_matter_regex.search(content)

    if match:
        yaml_string_to_parse = match.group(1)
        # print(f"--- DEBUG: Processing file: {file_path} ---")
        # print(f"--- DEBUG: String passed to yaml.safe_load: ---")
        # print(yaml_string_to_parse)
        # print(f"--- END DEBUG ---")
        return yaml.safe_load(yaml_string_to_parse)
    else:
        # print(f"--- DEBUG: No front matter match found in: {file_path} ---")
        return None
        # return yaml.safe_load(match.group(1))

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
