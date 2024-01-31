import os
import yaml

def extract_front_matter(markdown_text):
    """Extracts the YAML front matter from a markdown file."""
    if markdown_text.startswith("---"):
        _, front_matter, _ = markdown_text.split("---", 2)
        return front_matter
    return None

def save_to_yaml(data, filename):
    """Saves data to a YAML file."""
    with open(filename, 'w') as file:
        yaml.dump(data, file)

# Path to the reviews folder
reviews_folder = 'web/_reviews'

# Initialize an empty list to store the extracted front matter
extracted_data = []

# Iterate over files in the reviews folder
for filename in os.listdir(reviews_folder):
    if filename.endswith(".md"):
        with open(os.path.join(reviews_folder, filename), 'r') as file:
            front_matter = extract_front_matter(file.read())
            if front_matter:
                extracted_data.append(yaml.safe_load(front_matter))

# Save the extracted data to a YAML file
save_to_yaml(extracted_data, 'web/_data/reviews.yml')
