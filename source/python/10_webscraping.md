---
title: Web Scraping with Python
description: Understand the basics of web scraping using Python, including the use of libraries like Beautiful Soup and requests.
layout: lesson
cover: assets/3.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

In this lesson, we will introduce the concept of web scraping, which is a method of extracting information from websites. Python offers several libraries for web scraping, including Beautiful Soup and requests.

---

## Learning Objectives

- Understand what web scraping is.
- Learn how to use the requests library to download web pages.
- Learn how to use the Beautiful Soup library to parse HTML and extract information.

---

### What is Web Scraping?

Web scraping is the process of extracting information directly from a web page. It involves making a request to a web page, downloading its HTML content, and parsing that content to extract the information you need.

---

### Downloading Web Pages with `requests`

The `requests` library allows you to send HTTP requests using Python. You can use it to download web pages.

```python
import requests

# Make a request to a web page
response = requests.get('https://www.example.com')

# Print the status code (200 means success)
print(response.status_code)

# Print the first 500 characters of the HTML content
print(response.text[:500])
```

---

### Parsing HTML with `Beautiful Soup`

Beautiful Soup is a library for parsing HTML and XML documents. It provides methods and Pythonic idioms for iterating, searching, and modifying the parse tree.

```python
from bs4 import BeautifulSoup
import requests

# Make a request to a web page
response = requests.get('https://www.example.com')

# Create a Beautiful Soup object
soup = BeautifulSoup(response.text, 'html.parser')

# Find the title tag
title_tag = soup.find('title')

# Print the text of the title tag
print(title_tag.text)
```

---

## Summary

In this lesson, you've learned about web scraping with Python. We've covered how to use the requests library to download web pages and the Beautiful Soup library to parse HTML and extract information. Web scraping is a powerful tool for gathering data from the internet.

---
