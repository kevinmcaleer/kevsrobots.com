---
layout: lesson
title: Setting Up Your Environment
author: Kevin McAleer
type: page
cover: assets/3.png
date: 2024-03-21
previous: 02_markdown_basics.html
next: 04_writing_content_in_markdown.html
description: Step-by-step guide to installing Jekyll and preparing your development
  environment for creating Jekyll sites.
percent: 30
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


![Environment Setup cover image]({{ page.cover }}){:class="cover"}

## Introduction

Before diving into creating content and customizing your Jekyll site, it's essential to set up your local development environment. This lesson will guide you through installing Jekyll, Ruby, and other necessary tools.

## Prerequisites

- **Ruby:** Jekyll is built with Ruby. Ensure you have Ruby and the RubyGem package manager installed.
- **Git:** Version control is crucial for managing your site's source code.

## Installing Ruby

The installation process varies depending on your operating system.

### Windows

1. Download and install Ruby using the RubyInstaller for Windows.
2. During installation, check the option to add Ruby to your PATH.

### macOS

macOS comes with Ruby installed, but you may need to update it. Use Homebrew:

```bash
brew install ruby
```

Add Ruby to your PATH using the instructions provided by Homebrew.

### Linux

Use your package manager to install Ruby. For example, on Ubuntu:

```bash
sudo apt-get install ruby-full
```

## Installing Jekyll

With Ruby installed, you can install Jekyll and Bundler:

```bash
gem install jekyll bundler
```

## Setting Up a Jekyll Project

Create a new Jekyll site in your desired directory:

```bash
jekyll new my-awesome-site
```

Change into your new directory:

```bash
cd my-awesome-site
```

Build the site and make it available on a local server:

```bash
bundle exec jekyll serve
```

Open your web browser and go to `http://localhost:4000` to see your new site.

## Version Control with Git

Initialize a Git repository in your project folder:

```bash
git init
```

Add and commit your Jekyll project:

```bash
git add .
git commit -m "Initial commit"
```

## Practice Exercise

1. Install Ruby, Jekyll, and Bundler following the steps above.
2. Create a new Jekyll project named `learn-jekyll`.
3. Run your site locally and take a screenshot of your browser displaying the site.

## Additional Resources

- [Jekyll Installation Documentation](https://jekyllrb.com/docs/installation/)
- [Ruby Official Website](https://www.ruby-lang.org/en/downloads/)
- [Git Official Website](https://git-scm.com/)

---
