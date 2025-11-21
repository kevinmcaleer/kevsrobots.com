#!/usr/bin/env python3
"""
Test URL cleaning for development URLs in cover images.
"""

def clean_cover_url(cover_image):
    """Clean development URLs from cover images."""
    base_url = 'https://www.kevsrobots.com'

    if cover_image:
        # Replace development server URLs with production
        cover_image = cover_image.replace('http://0.0.0.0:4000', base_url)
        cover_image = cover_image.replace('https://0.0.0.0:4000', base_url)
        cover_image = cover_image.replace('http://localhost:4000', base_url)
        cover_image = cover_image.replace('https://localhost:4000', base_url)
        cover_image = cover_image.replace('http://127.0.0.1:4000', base_url)
        cover_image = cover_image.replace('https://127.0.0.1:4000', base_url)

        # Replace http with https
        if cover_image.startswith('http://'):
            cover_image = cover_image.replace('http://', 'https://')

        # Add base URL if relative path
        if not cover_image.startswith(('http://', 'https://')):
            cover_image = base_url + ('/' if not cover_image.startswith('/') else '') + cover_image

        # Clean up double slashes
        cover_image = cover_image.replace(".com//", ".com/")

    return cover_image


# Test cases
test_urls = [
    'https://0.0.0.0:4000/learn/python/assets/1.png',
    'http://0.0.0.0:4000/learn/python/assets/1.png',
    'http://localhost:4000/assets/cover.jpg',
    'https://localhost:4000/assets/cover.jpg',
    'http://127.0.0.1:4000/images/test.png',
    '/learn/python/assets/1.png',
    'http://www.kevsrobots.com/assets/image.jpg',
    'https://www.kevsrobots.com/assets/image.jpg',
]

print("=" * 80)
print("URL Cleaning Test")
print("=" * 80)
print()

for url in test_urls:
    cleaned = clean_cover_url(url)
    status = "✅" if cleaned.startswith('https://www.kevsrobots.com') else "⚠️"
    print(f"{status} Input:  {url}")
    print(f"   Output: {cleaned}")
    print()

print("=" * 80)
