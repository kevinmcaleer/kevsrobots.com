---
title: Setting Up Your Environment
description: Step-by-step guide to installing Jekyll and preparing your development environment for creating Jekyll sites.
layout: lesson
type: page
cover: /learn/jekyll/assets/setup_environment.jpg
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
