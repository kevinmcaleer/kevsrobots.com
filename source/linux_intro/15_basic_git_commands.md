---
title: Basic Git Commands
description: Learn essential Git commands like init, add, commit, and clone to manage your repositories.
layout: lesson
cover: /learn/linux_intro/assets/git.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Git provides a range of commands to help you manage your repositories and track changes. In this lesson, you’ll learn the essential Git commands needed to start managing your projects.

---

## Learning Objectives

- Initialize a new Git repository.
- Add and commit changes to the repository.
- Clone an existing repository.

---

### Initializing a Git Repository

To start tracking a project with Git, navigate to the project directory and initialize a repository:

```bash
git init
```

---

## Adding and Committing Changes

After making changes to your files, you need to add them to the staging area and commit them:

```bash
git add .
git commit -m "Initial commit"
```

The add command stages your changes, and commit saves them to the repository with a descriptive message.

---

## Cloning a Repository

If you want to work on an existing project, you can clone its repository:

        git clone https://github.com/user/repo.git

This command downloads the project and its entire history to your local machine.

---

## Summary

In this lesson, you learned how to initialize a Git repository, add and commit changes, and clone existing repositories. These basic commands are the foundation for managing your projects with Git.

---
