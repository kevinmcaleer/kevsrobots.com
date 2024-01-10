import os
import xml.etree.ElementTree as ET
from urllib.parse import urljoin

def generate_sitemap(site_url, output_dir):
    root = ET.Element("urlset", xmlns="http://www.sitemaps.org/schemas/sitemap/0.9")
    for foldername, subfolders, filenames in os.walk(output_dir):
        for filename in filenames:
            if filename.endswith('.html'):
                filepath = os.path.join(foldername, filename)
                url = urljoin(site_url, os.path.relpath(filepath, output_dir))
                url_element = ET.SubElement(root, "url")
                ET.SubElement(url_element, "loc").text = url

    tree = ET.ElementTree(root)
    tree.write("web/sitemap.xml", xml_declaration=True, encoding='utf-8')

# Example usage
generate_sitemap("https://www.kevsrobots.com", "web/_site")
