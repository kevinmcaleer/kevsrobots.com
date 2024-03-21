---
layout: lesson
title: Deploying to GitHub Pages
author: Kevin McAleer
type: page
cover: /learn/jekyll/assets/deploying_to_github_pages.jpg
date: 2024-03-21
previous: 09_building_and_testing_locally.html
description: Step-by-step guide on how to deploy your Jekyll site to GitHub Pages,
  making your site accessible to anyone on the internet.
percent: 100
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


![Deploying to GitHub Pages cover image]({{ page.cover }}){:class="cover"}

## Introduction

Deploying your Jekyll site to GitHub Pages is a straightforward way to make your content accessible on the internet. GitHub Pages provides a free hosting service that integrates directly with GitHub repositories, making it an ideal platform for hosting Jekyll-based sites.

## Preparing Your Site for GitHub Pages

Before deploying, ensure your site is compatible with GitHub Pages:

1. **GitHub Pages Dependency Versions:** Check that your Jekyll version and any plugins are supported by GitHub Pages.
2. **Repository Setup:** Your site should be in a GitHub repository, either in the root of the `master` branch, the `gh-pages` branch, or in a `/docs` folder on the `master` branch.

## Deploying Your Site

1. **GitHub Pages Activation:**
   - Go to your repository's settings on GitHub.
   - Find the "GitHub Pages" section.
   - Select the branch and folder you want to deploy from, then click "Save".

2. **Custom Domain (Optional):**
   If you have a custom domain, you can configure it in the GitHub Pages settings.

3. **Wait for Deployment:**
   GitHub automatically builds your site when you push changes to the configured branch. The process may take a few minutes.

4. **Access Your Site:**
   Once deployed, GitHub will provide a URL to access your site, such as `https://<username>.github.io/<repository>`.

## Updating Your Site

Any changes you make to your site can be pushed to the configured branch in your GitHub repository. GitHub Pages will automatically rebuild and redeploy your site.

## Practice Exercise

1. Ensure your Jekyll site is ready for deployment, following the GitHub Pages requirements.
2. Deploy your site to GitHub Pages, using either the `master`, `gh-pages` branch, or a `/docs` folder.
3. Customize your site with a custom domain, if available.
4. Make a change to your site, push the change to GitHub, and verify that your site updates automatically.

## Additional Resources

- [GitHub Pages Documentation](https://pages.github.com/)
- [Jekyll on GitHub Pages](https://jekyllrb.com/docs/github-pages/)

---
