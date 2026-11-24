---
layout: lesson
title: Final Project Steps
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/banners/17_final_project_steps.jpg
date: 2024-08-23
previous: 16_final_project_overview.html
next: 18_resources_next_steps.html
description: Build, test and schedule pi-checkup.sh step by step, then commit it with
  Git so your Pi is ready for Docker.
percent: 80
duration: 6
quiz_total: 85
date_updated: 2026-11-24
navigation:
- name: Introduction to the Linux Command Line on Raspberry Pi OS
- content:
  - section: Getting Started with the Command Line
    content:
    - name: Course Overview
      link: 00_intro.html
    - name: Getting Help with `man`
      link: 01a_man.html
    - name: Basic Command Line Navigation
      link: 02_basic_navigation.html
    - name: Managing Users and Using `sudo`
      link: 02a_user.html
  - section: Managing Files and Directories
    content:
    - name: Working with Files
      link: 03_working_with_files.html
    - name: Working with Directories
      link: 04_working_with_directories.html
    - name: File Permissions and Ownership
      link: 05_file_permissions.html
  - section: Managing Software
    content:
    - name: Installing Software
      link: 06_installing_software.html
    - name: Managing Software
      link: 07_managing_software.html
  - section: System Monitoring and Management
    content:
    - name: Monitoring System Performance
      link: 08_monitoring_performance.html
    - name: Managing Processes
      link: 09_managing_processes.html
    - name: Services and systemctl
      link: 09a_services.html
  - section: Networking Basics
    content:
    - name: Networking Commands
      link: 10_networking_commands.html
    - name: Transferring Files Over a Network
      link: 11_transferring_files.html
  - section: Scripting Basics
    content:
    - name: Introduction to Shell Scripting
      link: 12_intro_scripting.html
    - name: Basic Scripting Constructs
      link: 13_basic_scripting_constructs.html
  - section: Working with Git (Optional Advanced Topic)
    content:
    - name: Introduction to Git
      link: 14_intro_git.html
    - name: Basic Git Commands
      link: 15_basic_git_commands.html
  - section: 'Final Project: Building a Simple Command Line Application'
    content:
    - name: Final Project Overview
      link: 16_final_project_overview.html
    - name: Final Project Steps
      link: 17_final_project_steps.html
  - section: Summary and Next Steps
    content:
    - name: Summary and Next Steps
      link: 18_resources_next_steps.html
    - name: Linux Command Line Cheatsheet
      link: 19_cheatsheet.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write, test and schedule a complete health-check and backup script for your Pi, and save it in Git.

## Let's build it

Following the plan from the last lesson, we will build `pi-checkup.sh` in small pieces, running it after each one. Open the file:

```bash
$ cd ~/homelab/scripts
$ nano pi-checkup.sh
```

## Step 1: the header and the report line

```bash
#!/bin/bash
# pi-checkup.sh - a quick health report and backup for a Raspberry Pi
# Usage: ./pi-checkup.sh [warning-percent]

WARN=${1:-80}
PROJECTS="$HOME/homelab/projects"
BACKUPS="$HOME/homelab/backups"

echo "=== Checkup for $(hostname) at $(date '+%Y-%m-%d %H:%M') ==="
echo "Uptime: $(uptime -p)"
```

Save, make it executable and run it. Always test the first piece before adding more:

```bash
$ chmod +x pi-checkup.sh
$ ./pi-checkup.sh
=== Checkup for raspberrypi at 2026-11-24 11:19 ===
Uptime: up 2 days, 3 hours, 12 minutes
```

Settings go at the top as variables, so that if you move your folders you only change one line.

## Step 2: the disk check

Add this to the end of the file:

```bash
# Disk: df prints a header line, so take the last line and keep only the digits
USED=$(df --output=pcent / | tail -n 1 | tr -dc '0-9')
if [ "$USED" -ge "$WARN" ]; then
    echo "WARNING: the main disk is ${USED}% full"
else
    echo "Disk OK: ${USED}% used"
fi
```

Test both branches. Run it normally, then with a tiny limit to force the warning:

```bash
$ ./pi-checkup.sh 5 | tail -n 1
WARNING: the main disk is 19% full
```

## Step 3: memory, temperature and services

```bash
# Memory: pick the "Mem:" row from free and print two of its columns
free -h | awk '/^Mem:/ {print "Memory: " $3 " used of " $2}'

# Temperature: only the Pi has vcgencmd, so check it exists first
if command -v vcgencmd > /dev/null; then
    echo "CPU temperature: $(vcgencmd measure_temp | cut -d= -f2)"
fi

# Services that have failed
FAILED=$(systemctl --failed --no-legend 2>/dev/null | wc -l)
echo "Failed services: $FAILED"
```

`awk` picks columns from text: on the line starting `Mem:`, `$3` is the used memory and `$2` the total. `cut -d= -f2` splits `temp=48.3'C` at the `=` and keeps the second half. `command -v` asks whether a command exists, and `> /dev/null` hides its output.

## Step 4: the backup

```bash
# Back up the projects folder
if [ -d "$PROJECTS" ]; then
    mkdir -p "$BACKUPS"
    rsync -a "$PROJECTS/" "$BACKUPS/projects/"
    echo "Backup copied to $BACKUPS/projects/"
else
    echo "No projects folder yet, skipping the backup"
fi
```

Note the trailing slashes from the file-transfer lesson, and that there is no `--delete`: the backup only ever adds or updates files. If `rsync` is missing, run `sudo apt install rsync`.

## The finished run

Add a test file so there is something to back up, then run the lot:

```bash
$ echo "version: 1" > ~/homelab/projects/hello-docker/compose.yaml
$ ./pi-checkup.sh
=== Checkup for raspberrypi at 2026-11-24 11:30 ===
Uptime: up 2 days, 3 hours, 22 minutes
Disk OK: 19% used
Memory: 412Mi used of 3.7Gi
CPU temperature: 48.3'C
Failed services: 0
Backup copied to /home/pi/homelab/backups/projects/
```

Your numbers will differ. If `Failed services` is more than 0, run `systemctl --failed` and then `journalctl -u NAME -n 20` to see why.

## Run it automatically with cron

The `cron` service from the services lesson runs commands on a schedule. Edit your own schedule:

```bash
$ crontab -e
```

The first time, it asks which editor to use: choose `nano`. Add this line at the bottom:

```bash
0 7 * * 1 $HOME/homelab/scripts/pi-checkup.sh >> $HOME/homelab/checkup.log 2>&1
```

The five fields are minute, hour, day of month, month and day of week, so this means "at 07:00 every Monday". The `>>` adds the report to a log file, and `2>&1` sends error messages there too. This is the answer to "how will it tell me when I am not watching": read the log with `tail -n 20 ~/homelab/checkup.log`. See your schedule with `crontab -l`.

Cron runs with a smaller environment than your terminal, so if the script works by hand but not in cron, check the log, and use full paths for anything unusual.

## Save it with Git

```bash
$ cd ~/homelab/scripts
$ git add pi-checkup.sh
$ git commit -m "Add pi-checkup.sh"
```

Finally, document it with a short `README.md` (use `nano`) saying what it does, how to run it, and what each warning means. Future you will be grateful.

Pi progress: your Pi is tidy, current, monitored and backed up, with SSH and a script that watches over it. It is ready for Docker.

## Try it Yourself

1. Run the script with a limit of 5 and with a limit of 95, and check which message appears each time.
2. Rename your `projects` folder temporarily and run the script, to see the "skipping" message. Then rename it back.
3. Add a line that prints the Pi's IP address with `hostname -I`.
4. Challenge: save the report to a dated file in `~/homelab/backups`, using `$(date +%F)` in the file name, and add a check that warns if the temperature is over 70.

## Common Issues

- **Problem**: `rsync: command not found`.
- **Solution**: Run `sudo apt install rsync`.
- **Why**: rsync is a separate package that may not be on your image.

- **Problem**: The script works by hand but nothing appears in the log from cron.
- **Solution**: Check the cron line with `crontab -l`, confirm the script is executable, and run `systemctl is-active cron`.
- **Why**: Cron needs the script path to be exact and the service to be running.

- **Problem**: `[: : integer expression expected`.
- **Solution**: The disk percentage came out empty. Run the `df ... | tr` line on its own to see what it prints.
- **Why**: The number comparison needs digits, so if the pipeline returns nothing, the test fails.

## Check your understanding

{% quiz %}
? What does the cron line `0 7 * * 1` mean?
- [ ] Every day at 07:00
- [x] At 07:00 every Monday
- [ ] Every 7 minutes
> The fields are minute, hour, day of month, month and day of week, and 1 means Monday.

? Why does the script set WARN, PROJECTS and BACKUPS as variables at the top?
- [x] So you only change one place if something moves
- [ ] bash requires it
- [ ] To make the script faster
> Settings in one place are easier to change and test.

? What does `>> checkup.log 2>&1` do in the cron line?
- [ ] Deletes the log
- [x] Adds the output and any errors to the log file
- [ ] Sends an email
> `>>` appends the normal output, and `2>&1` sends the errors to the same place.

? Why is there no `--delete` in the rsync command?
- [ ] It does not exist
- [x] A health check should never remove files
- [ ] It makes rsync slower
> The backup only adds and updates files, so a wrong path cannot erase anything.
{% endquiz %}
