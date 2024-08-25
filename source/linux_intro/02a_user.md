---
title: Managing Users and Using `sudo`
description: Learn how to manage users, use the `sudo` command, and change passwords in a Unix-like system.
layout: lesson
cover: /learn/linux_intro/assets/usermgr.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

In a Unix-like operating system, managing users and permissions is a critical task, especially when multiple people use the same system. This lesson covers how to add and manage users, use the `sudo` command to execute commands with superuser privileges, and change user passwords.

---

## Learning Objectives

- Understand what the `sudo` command does and how to use it.
- Learn how to add, delete, and manage users.
- Change user passwords using the `passwd` command.

---

### Understanding `sudo`

The `sudo` command, short for "**superuser do**," allows a permitted user to execute a command as the superuser (or another user), as specified by the security policy. The superuser, also known as "root," has unrestricted access to the system, making `sudo` a powerful and necessary tool for performing administrative tasks.

#### **Using `sudo`:**

To use `sudo`, simply prepend it to the command you want to run with elevated privileges. For example:

```bash
sudo apt update
```

This command updates the package list for the system, but since it requires superuser privileges, `sudo` is used. You will usually be prompted to enter your password when using `sudo` to verify your identity.

#### **Why Use `sudo` Instead of Logging in as Root?**

- **Safety:** Using `sudo` reduces the risk of making critical errors because commands are run as superuser only when necessary.
- **Security:** It allows for better auditing of commands executed with superuser privileges, as each command is associated with the user who ran it.
- **Control:** Admins can control which users can execute which commands by configuring the `sudoers` file.

---

### Adding and Managing Users

#### **1. Adding a New User:**

To add a new user to the system, use the `adduser` command:

```bash
sudo adduser username
```

You will be prompted to enter and confirm a password for the new user, as well as provide some additional information like the full name (which is optional).

#### **2. Deleting a User:**

To delete a user and optionally remove their home directory, use the `deluser` command:

```bash
sudo deluser username
```

If you also want to remove the user’s home directory, use:

```bash
sudo deluser --remove-home username
```

#### **3. Adding a User to the `sudo` Group:**

If you want a user to have the ability to use `sudo`, you need to add them to the `sudo` group:

```bash
sudo usermod -aG sudo username
```

The `-aG` option appends the user to the group without removing them from any other groups they belong to.

---

### Changing Passwords

#### **1. Changing Your Own Password:**

To change your own password, use the `passwd` command:

```bash
passwd
```

You will be prompted to enter your current password followed by the new password twice.

#### **2. Changing Another User’s Password:**

If you have `sudo` privileges, you can change another user’s password:

```bash
sudo passwd username
```

After entering your own password (for `sudo`), you will be prompted to enter a new password for the specified user.

#### **3. Forcing a Password Change on Next Login:**

To force a user to change their password the next time they log in, use:

```bash
sudo passwd -e username
```

This command expires the user's password, requiring them to update it upon their next login.

---

### Managing Groups

Groups are a way to manage permissions for multiple users at once. You can add or remove users from groups to grant or revoke permissions.

- **Create a New Group:**
  ```bash
  sudo groupadd groupname
  ```

- **Add a User to a Group:**
  ```bash
  sudo usermod -aG groupname username
  ```

- **Remove a User from a Group:**
  ```bash
  sudo deluser username groupname
  ```

---

## Summary

In this lesson, you learned how to use the `sudo` command to execute commands with superuser privileges, manage users and groups, and change passwords. These skills are essential for securely administering a Unix-like system, especially when multiple users are involved.

---

## Practice Exercise

Try the following tasks to practice what you've learned:

1. **Add a new user:** Create a user called `student` and give them `sudo` privileges.
2. **Change your password:** Update your current password using the `passwd` command.
3. **Manage groups:** Create a new group called `developers` and add the `student` user to this group.

---
