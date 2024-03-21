---
layout: lesson
title: Building and Testing Your Site Locally
author: Kevin McAleer
type: page
cover: /learn/jekyll/assets/building_testing_locally.jpg
date: 2024-03-21
previous: 08_customizing_themes_and_plugins.html
next: 10_deploying_to_github_pages.html
description: Learn how to build your Jekyll site locally, test it for errors, and
  ensure it's ready for deployment, maintaining high quality and reliability.
percent: 90
duration: 2
navigation:
- name: Mastering Markdown for Documentation with Jekyll
- content:
  - section: Introduction to Markdown and Jekyll
    content:
    - name: Overview of Jekyll and Static Sites
      link: 01_overview.html
    - name: Introduction to Markdown
      link: 02_markdown_basics.html
    - name: Setting Up Your Environment
      link: 03_setup_environment.html
  - section: Creating Content with Markdown and Jekyll
    content:
    - name: Writing Content in Markdown
      link: 04_writing_content_in_markdown.html
    - name: Working with Jekyll Layouts and Includes
      link: 05_working_with_layouts_and_includes.html
    - name: Managing Static Assets
      link: 06_managing_static_assets.html
  - section: Enhancing Your Site
    content:
    - name: Using Liquid Templating for Dynamic Content
      link: 07_using_liquid_templating.html
    - name: Customizing Themes and Plugins
      link: 08_customizing_themes_and_plugins.html
  - section: Publishing Your Site
    content:
    - name: Building and Testing Your Site Locally
      link: 09_building_and_testing_locally.html
    - name: Deploying to GitHub Pages
      link: 10_deploying_to_github_pages.html
---


![Building and Testing Your Site Locally cover image]({{ page.cover }}){:class="cover"}

## Introduction

Before deploying your site to the public, it's essential to build and test it locally. This process helps you catch and fix any issues, ensuring your site operates correctly and provides a good user experience.

## Building Your Site Locally

Jekyll comes with a built-in development server that regenerates your site every time a file is updated, making it easy to see changes in real-time.

1. **Start the Development Server:**
   Open your terminal, navigate to your Jekyll project directory, and run:
   ```bash
   bundle exec jekyll serve
   ```
2. **Access Your Site:**
   By default, your site will be available at `http://localhost:4000`. Open this address in your web browser to see your Jekyll site in action.

## Testing for Errors

As you develop your site, you may encounter various errors, such as broken links, missing images, or layout issues. Here's how to spot and fix common problems:

- **Check the Terminal Output:** Jekyll prints errors and warnings to the terminal. Pay attention to these messages as they can provide clues on what needs fixing.
- **Use Browser Developer Tools:** Modern browsers come with developer tools that can help diagnose issues with HTML, CSS, and JavaScript.

## Enhancing Your Site's Performance

Performance is key to providing a good user experience. Here are a few tips to improve your site's speed:

- **Optimize Images:** Ensure your images are appropriately sized and consider using compression tools to reduce their file size without significantly affecting quality.
- **Minimize CSS and JavaScript:** Use tools to minify your CSS and JavaScript files, reducing their size and load times.

## Practice Exercise

1. Build your Jekyll site locally and navigate through each page to ensure everything loads correctly.
2. Identify and fix any errors or warnings reported by Jekyll or discovered during your review.
3. Optimize at least one image and one CSS file to improve your site's performance.

## Additional Resources

- [Jekyll's Documentation on Running Jekyll Locally](https://jekyllrb.com/docs/)
- [Web Performance Optimization](https://web.dev/performance/)

---
