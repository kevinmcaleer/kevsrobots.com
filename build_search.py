import yaml

def read_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def write_yaml(data, file_path):
    with open(file_path, 'w') as file:
        yaml.safe_dump(data, file)

def convert_data(data):
    new_data = []
    
    for item in data:
        new_item = {
            'title': item['title'],
            'keywords': item['description'],  
            'link': item['link']
        }
        if 'tags' in item:
                new_item['keywords'] += item['term']
        new_data.append(new_item)  
    return new_data

if __name__ == '__main__':
    input_file_path = ['web/_data/courses.yml','web/_data/glossary.yml']
    output_file_path = 'web/assets/data/data.yml'

    for file in input_file_path:
        print("Reading YAML file:", file)
        input_data = read_yaml(file)
        new_data = convert_data(input_data)
    write_yaml(new_data, output_file_path)
    print("Converted YAML file saved to:", output_file_path)
