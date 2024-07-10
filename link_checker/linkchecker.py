import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import os
import csv
import argparse
import concurrent.futures
from itertools import islice

class LinkTracker:
    def __init__(self, base_url, max_depth=2, output_file=None):
        self.base_url = base_url
        self.base_domain = urlparse(self.base_url).netloc
        self.max_depth = max_depth
        self.visited = set()
        self.broken_links = set()
        self.all_links = []
        self.output_file = output_file
        if self.output_file:
            self.initialize_csv()

    def initialize_csv(self):
        # Wipe and initialize the CSV file
        with open(self.output_file, 'w', newline='') as csvfile:
            fieldnames = ['Source URL', 'Broken Link', 'Status', 'Line Number']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def append_to_csv(self, source_url, broken_link, status, line_number):
        with open(self.output_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([source_url, broken_link, status, line_number])

    def check_link(self, url, retries=3):
        parsed_url = urlparse(url)
        if parsed_url.scheme in ('http', 'https'):
            attempt = 0
            while attempt < retries:
                try:
                    response = requests.get(url, timeout=5)
                    if response.status_code != 200:
                        return False, response.status_code
                    return True, response.status_code
                except requests.exceptions.RequestException as e:
                    attempt += 1
                    if attempt == retries:
                        return False, str(e)
        elif os.path.isfile(url):  # Local file
            if os.path.exists(url):
                return True, "File exists"
            else:
                return False, "File not found"
        else:
            return False, "Unsupported URL scheme"

    def collect_links(self, url, depth):
        if depth > self.max_depth or url in self.visited:
            return

        print(f"Collecting links from: {url} at depth: {depth}")
        self.visited.add(url)
        parsed_url = urlparse(url)

        if parsed_url.scheme in ('http', 'https'):
            try:
                response = requests.get(url, timeout=5)
                response.raise_for_status()
                soup = BeautifulSoup(response.text, 'html.parser')
                links = soup.find_all('a', href=True)
                for link in links:
                    link_url = urljoin(url, link['href'])
                    self.all_links.append((url, link_url))
                    link_parsed_url = urlparse(link_url)
                    if link_parsed_url.netloc == self.base_domain or link_parsed_url.netloc == '':
                        self.collect_links(link_url, depth + 1)
            except requests.exceptions.RequestException as e:
                if url not in self.broken_links:
                    print(f"Error collecting links from URL: {url} - Exception: {e}")
                    self.broken_links.add((url, str(e)))
                    if self.output_file:
                        self.append_to_csv(url, url, str(e), 'N/A')
        elif os.path.isfile(parsed_url.path):  # Local file
            try:
                file_path = os.path.abspath(parsed_url.path)
                with open(file_path, 'r') as file:
                    soup = BeautifulSoup(file, 'html.parser')
                    links = soup.find_all('a', href=True)
                    for link in links:
                        link_url = os.path.abspath(os.path.join(os.path.dirname(file_path), link['href']))
                        self.all_links.append((file_path, link_url))
                        link_parsed_url = urlparse(link_url)
                        if link_parsed_url.netloc == self.base_domain or link_parsed_url.netloc == '':
                            self.collect_links(link_url, depth + 1)
            except Exception as e:
                if url not in self.broken_links:
                    print(f"Error collecting links from file: {url} - Exception: {e}")
                    self.broken_links.add((url, str(e)))
                    if self.output_file:
                        self.append_to_csv(url, url, str(e), 'N/A')

    def process_links_in_batches(self, batch_size=10):
        total_links = len(self.all_links)
        for i in range(0, total_links, batch_size):
            batch = self.all_links[i:i + batch_size]
            with concurrent.futures.ThreadPoolExecutor(max_workers=batch_size) as executor:
                futures = {executor.submit(self.check_and_record_link, source_url, link_url): (source_url, link_url) for source_url, link_url in batch}
                for future in concurrent.futures.as_completed(futures):
                    source_url, link_url = futures[future]
                    try:
                        future.result()
                    except Exception as exc:
                        print(f'Generated an exception: {exc}')
            print(f'Processed {min(i + batch_size, total_links)} of {total_links} links')

    def check_and_record_link(self, source_url, link_url):
        is_valid, status = self.check_link(link_url)
        if not is_valid and link_url not in self.broken_links:
            line_number = self.find_line_number(source_url, link_url)
            print(f"Broken link found: {link_url} - Status: {status} - Line: {line_number}")
            self.broken_links.add((link_url, status))
            if self.output_file:
                self.append_to_csv(source_url, link_url, status, line_number)

    def find_line_number(self, source_url, link_url):
        # Find the line number of the element in the BeautifulSoup object
        try:
            if urlparse(source_url).scheme in ('http', 'https'):
                response = requests.get(source_url, timeout=5)
                soup = BeautifulSoup(response.text, 'html.parser')
            else:
                with open(source_url, 'r') as file:
                    soup = BeautifulSoup(file, 'html.parser')
                    
            links = soup.find_all('a', href=True)
            for i, link in enumerate(links):
                if urljoin(source_url, link['href']) == link_url:
                    lines = str(soup).split('\n')
                    element_str = str(link)
                    for j, line in enumerate(lines):
                        if element_str in line:
                            return j + 1
        except:
            return 'N/A'
        return 'N/A'

    def start(self):
        print(f"Starting crawl at: {self.base_url}")
        self.collect_links(self.base_url, 0)
        print(f"Collected {len(self.all_links)} links to check")
        self.process_links_in_batches()
        print("Crawl completed.")

    def print_broken_links(self):
        if self.broken_links:
            print("\nBroken links:")
            for url, status in self.broken_links:
                print(f"Broken link: {url} - Status: {status}")
        else:
            print("\nNo broken links found.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Track and check links on a website or local files.')
    parser.add_argument('url', help='The base URL or local file to start scanning')
    parser.add_argument('-d', '--depth', type=int, default=2, help='The maximum depth to scan')
    parser.add_argument('-o', '--output', help='CSV file to save broken links', default=None)
    
    args = parser.parse_args()
    
    tracker = LinkTracker(args.url, max_depth=args.depth, output_file=args.output)
    tracker.start()
    tracker.print_broken_links()

    if args.output:
        print(f"Broken links saved to {args.output}")