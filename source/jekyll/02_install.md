---
title: Installation and Setup
description: A step-by-step guide to installing Jekyll and setting up your first static site, with practical insights.
layout: lesson
type: page
---

## Introduction

Welcome to the second lesson in our Jekyll course. In this lesson, we will walk you through the installation process of Jekyll, a popular static site generator. By the end of this lesson, you'll have Jekyll installed and running on your computer, ready to create your first static website.

---

## What You'll Need

Before we begin, make sure you have the following:

- A computer running Windows, macOS, or Linux.
- Internet connection for downloading necessary software.
- Basic familiarity with using command line interfaces.

---

## Step 1: System Requirements and Preparations

Jekyll is built on Ruby, so we need to ensure that your system has Ruby installed along with a few other tools.

### Checking for Ruby

- **Windows**: Ruby might not be installed by default. We'll use RubyInstaller in the next steps.
- **macOS/Linux**: Ruby usually comes pre-installed. You can check by typing `ruby -v` in your terminal.

---

### Installing Ruby (If Necessary)

- **Windows**: Download RubyInstaller from [rubyinstaller.org](https://rubyinstaller.org/). Follow the installation instructions provided on the website.
- **macOS**: Use Homebrew to install Ruby by running `brew install ruby` in the terminal.
- **Linux**: Use your distribution’s package manager. For example, on Ubuntu, you can run `sudo apt-get install ruby-full`.

---

## Step 2: Installing Jekyll

With Ruby installed, we can now install Jekyll.

---

### Installing Jekyll and Bundler

- Open your terminal or command prompt.
- Run the command `gem install jekyll bundler`.
- This command installs both Jekyll and Bundler, a dependency manager for Ruby.

---

### Verifying the Installation

- After installation, verify by typing `jekyll -v`. You should see the Jekyll version number.

---

## Step 3: Creating Your First Jekyll Site

Now, let's create your first static site with Jekyll.

---

### Creating a New Site

- Choose a directory where you want your site to be.
- Run `jekyll new my-awesome-site`. Replace `my-awesome-site` with your desired site name.
- Jekyll will create a new directory with the necessary files.

---

### Exploring the Site Structure

- Navigate to your new site's directory.
- You'll find several files and folders. The key ones are:
  - `_posts`: Where your blog posts will be stored.
  - `_config.yml`: The configuration file for your site.
  - `index.md`: The main page of your site.

---

## Step 4: Running Your Site Locally

It's time to see your site in action.

---

### Starting Jekyll Server

- In your site's directory, run `jekyll serve`.
- This command builds your site and starts a local server.

---

### Viewing Your Site

- Open your browser and go to `http://localhost:4000`.
- You should see your new Jekyll site. Any changes you make to the files will automatically update the site.

---

## Troubleshooting

If you encounter any issues:

- Check for error messages in the terminal. They usually provide insights into what's wrong.
- Ensure that you're using compatible versions of Ruby and Jekyll.

---

## Conclusion

Congratulations on setting up Jekyll and creating your first site! Experiment with editing the markdown files and watch how your site updates. In our next lesson, we will dive into customizing your Jekyll site.

---

## Additional Resources

- [Jekyll’s Documentation](https://jekyllrb.com/docs/)
- [RubyInstaller for Windows](https://rubyinstaller.org/)

---

## Assignment

Try adding a new post to your site. Create a markdown file in the `_posts` directory and see how it appears on your local site.

---
