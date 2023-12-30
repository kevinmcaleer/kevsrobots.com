import os
from bs4 import BeautifulSoup
from search.database import insert_document  # Replace with your actual module

def parse_html_file(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        soup = BeautifulSoup(file, 'lxml')

        title = soup.title.string if soup.title else 'No Title'
        content = soup.get_text()

        url = file_path.replace('web/_site/', '', 1).replace('../', '', 1)

        og_image_tag = soup.find('meta', {'property': 'og:image'})
        cover_image = og_image_tag['content'] if og_image_tag and 'content' in og_image_tag.attrs else 'DefaultImagePath'

        # Correct the cover image URL if necessary
        base_url = 'http://www.kevsrobots.com'
        if not cover_image.startswith(('http://', 'https://')):
            cover_image = base_url + ('/' if not cover_image.startswith('/') else '') + cover_image

        h1_tag = soup.find('h1')
        page_title = h1_tag.get_text().strip() if h1_tag else 'Default Page Title'

        insert_document(title, content, url, cover_image, page_title)

def main():
    for root, dirs, files in os.walk('../web/_site'):
        for file in files:
            if file.endswith('.html'):
                file_path = os.path.join(root, file)
                parse_html_file(file_path)

if __name__ == "__main__":
    main()
