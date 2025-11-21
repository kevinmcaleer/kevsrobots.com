import os
from bs4 import BeautifulSoup
from search.database import insert_document  # Replace with your actual module

def parse_html_file(file_path):
    try:
        # Try UTF-8 first
        with open(file_path, 'r', encoding='utf-8') as file:
            soup = BeautifulSoup(file, 'lxml')
    except UnicodeDecodeError:
        # If UTF-8 fails, try latin-1 (which accepts all byte values)
        try:
            with open(file_path, 'r', encoding='latin-1') as file:
                soup = BeautifulSoup(file, 'lxml')
        except Exception as e:
            print(f"Warning: Could not decode {file_path}: {e}")
            return False  # Indicate failure
    except Exception as e:
        print(f"Warning: Error parsing {file_path}: {e}")
        return False  # Indicate failure

    try:
        title = soup.title.string if soup.title else 'No Title'
        content = soup.get_text()

        url = file_path.replace('web/_site/', '', 1).replace('../', '', 1)

        og_image_tag = soup.find('meta', {'property': 'og:image'})
        cover_image = og_image_tag['content'] if og_image_tag and 'content' in og_image_tag.attrs else 'DefaultImagePath'

        description_tag = soup.find('meta', {'property': 'description'}) 
        description = description_tag['content'] if description_tag and 'content' in description_tag.attrs else 'No Description'
        date_tag = soup.find('meta', {'property': 'date'})
        date  = date_tag['content'] if date_tag and 'content' in date_tag.attrs else '2023-30-12'
        author_tag = soup.find('meta', {'property': 'author'})
        author = author_tag['content'] if author_tag and 'content' in author_tag.attrs else 'Kevin McAleer'

        # Correct the cover image URL if necessary
        base_url = 'https://www.kevsrobots.com'
        if cover_image.startswith('http://'):
            cover_image = cover_image.replace('http://', 'https://')
        if not cover_image.startswith(('http://', 'https://')):
            cover_image = base_url + ('/' if not cover_image.startswith('/') else '') + cover_image

        cover_image = cover_image.replace(".com//",".com/")

        h1_tag = soup.find('h1')
        # page_title = h1_tag.get_text().strip() if h1_tag else 'Untitled Page'
        page_title = title

        insert_document(title, content, url, cover_image, page_title, description, date, author)
        return True  # Successfully indexed
    except Exception as e:
        print(f"Warning: Error processing content from {file_path}: {e}")
        return False  # Indicate failure

def main():
    total_files = 0
    processed_files = 0
    skipped_files = 0

    print("Scanning for HTML files...")

    # Count total files first
    html_files = []
    for root, dirs, files in os.walk('../web/_site'):
        for file in files:
            if file.endswith('.html'):
                html_files.append(os.path.join(root, file))

    total_files = len(html_files)
    print(f"Found {total_files} HTML files to index\n")

    # Process each file
    for i, file_path in enumerate(html_files, 1):
        if i % 100 == 0:
            print(f"Progress: {i}/{total_files} files processed...")

        success = parse_html_file(file_path)
        if success:
            processed_files += 1
        else:
            skipped_files += 1

    print(f"\n{'='*60}")
    print(f"Indexing Summary:")
    print(f"  Total HTML files: {total_files}")
    print(f"  Successfully indexed: {processed_files}")
    print(f"  Skipped (errors): {skipped_files}")
    if skipped_files > 0:
        print(f"\n  Note: {skipped_files} files had encoding or parsing errors")
        print(f"        Check warnings above for details")
    print(f"{'='*60}\n")

if __name__ == "__main__":
    main()
