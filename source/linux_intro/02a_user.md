---
title: Managing Users and Using `sudo`
description: Learn how to manage users, use the `sudo` command, and change passwords in a Unix-like system with simple and easy-to-understand instructions.
layout: lesson
cover: /learn/linux_intro/assets/usermgr.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

When using a Unix-like system, like Linux on your Raspberry Pi, it's important to know how to manage users and their permissions. This lesson will teach you how to add and manage users, how to use the `sudo` command to run important tasks, and how to change passwords.

---

## Learning Objectives

- Understand what the `sudo` command does and how to use it.
- Learn how to add, delete, and manage users.
- Change user passwords using the `passwd` command.

---

### Understanding `sudo`

The `sudo` command (which stands for "superuser do") lets you run commands with special permissions. This is useful when you need to do things that require more control over the system, like installing software or changing system settings.

#### **How to Use `sudo`:**

To use `sudo`, you put it in front of the command you want to run with higher permissions. For example:

```bash
sudo apt update
```

This command updates the list of software packages on your system, but since it needs special permissions, you use `sudo` first. Usually, you’ll be asked to type in your password to confirm that you have permission to use `sudo`.

#### **Why Use `sudo` Instead of Logging in as Root?**

- **Safety:** Using `sudo` helps prevent mistakes because you only use special permissions when needed.
- **Security:** It keeps track of who uses `sudo`, so you know who did what on the system.
- **Control:** You can decide which users are allowed to use `sudo` by setting up permissions.

---

### Adding and Managing Users

#### **1. Adding a New User:**

To add a new user to your system, you use the `adduser` command:

```bash
sudo adduser username
```

Replace `username` with the name you want to give the new user. The system will ask you to set a password for the new user and give some extra information (like the full name), but you can skip these extra details if you want.

#### **2. Deleting a User:**

To remove a user from the system, use the `deluser` command:

```bash
sudo deluser username
```

If you also want to delete the user’s home directory (where their files are stored), use this command:

```bash
sudo deluser --remove-home username
```

#### **3. Giving a User `sudo` Permissions:**

If you want a user to be able to use `sudo`, you need to add them to the `sudo` group:

```bash
sudo usermod -aG sudo username
```

This command adds the user to the group that’s allowed to use `sudo`. The `-aG` part means “append to group,” so the user stays in their other groups too.

---

### Changing Passwords

#### **1. Changing Your Own Password:**

To change your own password, use the `passwd` command:

```bash
passwd
```

The system will ask you to enter your current password and then type your new password twice to confirm it.

#### **2. Changing Another User’s Password:**

If you have `sudo` permissions, you can change another user’s password by typing:

```bash
sudo passwd username
```

After entering your own password (to use `sudo`), the system will let you set a new password for the user.

#### **3. Making a User Change Their Password on Next Login:**

If you want to force a user to change their password the next time they log in, use this command:

```bash
sudo passwd -e username
```

This expires the current password, so the user has to set a new one the next time they log in.

---

### Managing Groups

Groups are a way to organize users, so you can give several users the same permissions at once. Here’s how you can manage groups:

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

In this lesson, you learned how to use the `sudo` command to do tasks that need special permissions, how to manage users on your system, and how to change passwords. These are important skills for keeping your system safe and making sure everyone has the right access.

---

## Practice Exercise

Try these tasks to practice what you’ve learned:

1. **Add a new user:** Create a user called `student` and give them `sudo` permissions.
2. **Change your password:** Update your current password using the `passwd` command.
3. **Manage groups:** Create a new group called `developers` and add the `student` user to this group.

---
