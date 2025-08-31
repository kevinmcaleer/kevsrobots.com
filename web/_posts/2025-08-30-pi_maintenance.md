---
title: "How to Keep your Raspberry Pi happy"
description: >-
  Tips and tricks to maintain your Raspberry Pi for optimal performance and longevity.
excerpt: >-
  Discover essential maintenance tips to keep your Raspberry Pi running smoothly and efficiently.
layout: showcase
date: 2025-08-30
date_updated: 2025-08-30
difficulty: beginner
cover: /assets/img/blog/pi-clean/cover.jpg
hero: /assets/img/blog/pi-clean/hero.png
mode: light
tags:
  - raspberry pi
  - software
groups:
  - raspberrypi
  - software
  - productivity
---

# 10 Raspberry Pi Tips for a Happier Pi

1. Removing large files
2. Keeping your system updated
3. Backing up
4. Monitoring system performance with htop
5. Cleaning up the file system - force the maintenance scripts to run immediately, and a bit of background about what the maintenance scripts are, how often they run, what they do and what they leave behind
6. Power Supply - use a good quality power supply to avoid issues
7. Cooling - consider using heatsinks or fans to keep your Raspberry Pi cool, especially under heavy load
8. Use surge protection – Plug your Pi into a surge protector to protect it from power spikes and outages.
9. Change default passwords – Always change the default password for your Pi to keep it secure from unauthorized access.
10. Document your setup – Keep notes on your configurations, installed software, and any changes you make. This helps with troubleshooting and future upgrades.
11. Bonus Ansible script – Automate your Raspberry Pi setup and configuration with Ansible for easier management and deployment.

---

## 1. Removing large files

**What’s the problem?**  
Over time, your Raspberry Pi can fill up with large or unnecessary files, which can slow down your system or even cause it to stop working if the SD card is full.

---

**How might it present itself?**

- “No space left on device” errors  
- Slow performance  
- Inability to save files or install updates

---

**How to resolve it:**  
Find and remove large files you no longer need.

---

**How often?**  
Check every 1–2 months, or if you notice storage issues.

---

**Step-by-step instructions:**

1. Install ncdu: `sudo apt install ncdu`
2. Run ncdu in your home directory: `ncdu ~`
3. Use arrow keys to browse and identify large files/folders
4. Press `d` to delete unwanted files
5. Empty the trash if you use a desktop environment

---

## 2. Keeping your system updated

**What’s the problem?**  
Outdated software can have security vulnerabilities and bugs. Hackers may use known vulnerabilities to exploit your system, hack it and potentially hijack your computer - the so called ransomware attack.

---

**How might it present itself?**  

- Security warnings  
- Software crashes  
- Incompatibility with new software

---

**How to resolve it:**  
Regularly update your system packages.

---

**How often?**  
At least once a month, or when prompted.

**Step-by-step instructions:**

1. Open a terminal
2. Run: `sudo apt update`
3. Then: `sudo apt upgrade -y`
4. Reboot if the system asks you to

---

## 3. Backing up

**What’s the problem?**  
SD cards can fail, and accidental deletions or corruption can cause data loss.

---

**How might it present itself?**  

- Pi won’t boot  
- Missing files  
- Lost configurations

---

**How to resolve it:**  
Regularly back up your SD card or important files to another device or cloud storage.

---

**How often?**  
Monthly, or before making major changes.

---

**Step-by-step instructions:**

1. To back up the whole SD card, shut down the Pi and remove the card.
2. Use a card reader on your PC/Mac.
3. Use Raspberry Pi Imager or `dd` to create an image of the card.
4. For file backups, use `rsync` or copy important folders to another drive or cloud.

---

**Backing up configuration files:**

To backup the configuration files, you can use the following command:

```bash
sudo tar -czvf backup.tar.gz /etc
```

**Scheduling a backup with crontab**

To schedule regular backups, you can use `cron`. Here’s how to set it up:

1. Open the crontab editor:
   ```bash
   crontab -e
   ```

2. Add a line to schedule the backup (e.g., daily at 2 AM):
   ```bash
   0 2 * * * sudo tar -czvf /path/to/backup/backup_$(date +\%F).tar.gz /etc
   ```

3. Save and exit the editor.

---

> # Using Crontab
>
> Crontab is a time-based job scheduler in Unix-like operating systems. You can use it to schedule tasks to run at specific intervals.
>
> For example, you can use crontab to schedule backups, system updates, or any other repetitive task.
>
> Crontab format:
> ```
> * * * * * command_to_execute
> ```
> - The five asterisks represent different time and date fields:
>   1. Minute (0-59)
>   2. Hour (0-23)
>   3. Day of the month (1-31)
>   4. Month (1-12)
>   5. Day of the week (0-7) (Sunday is both 0 and 7)
>
> Crontab runs using the system's cron daemon, which checks the crontab files and executes the scheduled tasks at the specified times. The user account under which the cron job runs may affect its permissions and environment.

---

## 4. Monitoring system performance with htop

**What’s the problem?**  
Resource-heavy processes can slow down your Pi or cause it to freeze.

---

**How might it present itself?**  

- Slow response  
- High CPU temperature  
- Unresponsive programs

---

**How to resolve it:**  
Monitor system resources and identify problematic processes.

---

**How often?**  
Check when you notice slowness or monthly as a checkup.

---

**Step-by-step instructions:**

1. Install htop: `sudo apt install htop`
2. Run: `htop`
3. Look for processes using high CPU or memory
4. Use F9 to kill a process if needed

---

## 5. Cleaning up the file system

**What’s the problem?**  
Temporary files and file system fragmentation can waste space and slow down your Pi.

---

**How might it present itself?**  

- Disk space shrinking over time  
- Slower file access

---

**How to resolve it:**  
Run file system maintenance scripts to clean up and optimize storage.

---

**How often?**  
Every 1–2 months, or if you notice storage issues.

---

**Step-by-step instructions:**

1. To trim unused blocks (on SSDs/SD cards): `sudo fstrim -av`
2. To clear apt cache: `sudo apt clean`
3. To remove old logs: `sudo journalctl --vacuum-time=2weeks`
4. To run all maintenance scripts: `sudo systemctl start fstrim.timer`

---

> # Journalctl
>
> Journalctl is a command-line tool for querying and displaying messages from the journal, managed by systemd. It provides a way to view logs from the systemd journal, which includes logs from the kernel, system services, and user applications.
>
> The vacuum command is used to remove old journal logs and free up disk space. It normally runs automatically, every time the system boots.

---

## 6. Power Supply

**What’s the problem?**  
Using a poor-quality or underpowered supply can cause instability and hardware issues.

---

**How might it present itself?**  

- Random reboots  
- SD card corruption  
- Warning icons on the desktop
- Slow performance

---

**How to resolve it:**  
Use a high-quality, official power supply and check for voltage issues.

---

**How often?**  
Check if you see instability or after adding new peripherals.

---

**Step-by-step instructions:**

1. Use the official Raspberry Pi power supply
2. If you see a lightning bolt icon, check your power source
3. Run `vcgencmd get_throttled` to check for voltage problems

---

## 7. Cooling

**What’s the problem?**  
The Pi can overheat, especially in warm environments or under heavy load.

---

**How might it present itself?**

- Throttled performance  
- High temperature warnings  
- System shutdowns

---

**How to resolve it:**  
Add cooling solutions like heatsinks, fans, or better airflow.

---

**How often?**  
Check temperatures during heavy use or in hot weather.

---

**Step-by-step instructions:**

1. Monitor temperature: `vcgencmd measure_temp`
2. If above 70°C, consider adding heatsinks or a fan
3. Ensure your case has good ventilation

---

## 8. Use surge protection

**What’s the problem?**  
Power surges or outages can damage your Pi or corrupt data.

---

**How might it present itself?**  

- Pi won’t boot after a storm  
- Corrupted SD card  
- Hardware failure

---

**How to resolve it:**  
Use a surge protector or UPS (uninterruptible power supply).

---

**How often?**  
Always keep your Pi plugged into surge protection.

---

**Step-by-step instructions:**

1. Plug your Pi’s power supply into a surge protector
2. For extra safety, use a UPS for battery backup

---

## 9. Change default passwords

**What’s the problem?**  
Leaving the default password makes your Pi vulnerable to attacks.

---

**How might it present itself?**  

- Unauthorized access  
- Changed settings or files  
- Compromised system

---

**How to resolve it:**  
Change the default password and consider using SSH keys.

---

**How often?**  
Immediately after setup, and whenever you suspect a breach.

---

**Step-by-step instructions:**

1. Open a terminal
2. Run: `passwd` and follow the prompts
3. For SSH, set up key-based authentication for extra security

---

## 10. Document your setup

**What’s the problem?**  
Forgetting what you’ve changed or installed makes troubleshooting and upgrades difficult.

---

**How might it present itself?**  

- Can’t remember how you set something up  
- Hard to fix issues or replicate setups

---

**How to resolve it:**  
Keep notes or use version control to track changes.

---

**How often?**  
Update your documentation whenever you make changes.

---

**Step-by-step instructions:**

1. Create a Markdown or text file for notes
2. Record installed packages: `dpkg --get-selections > packages.txt`
3. Save configuration files and changes
4. Optionally, use Git to version control your notes and configs

---

## 11. Bonus Ansible script

**What’s the problem?**  
Setting up multiple Pis or repeating setups is time-consuming and error-prone.

---

**How might it present itself?**  

- Inconsistent configurations  
- Wasted time on manual setup

---

**How to resolve it:**  
Automate your setup with Ansible playbooks.

---

**How often?**  
Whenever you set up a new Pi or make major changes.

---

**Step-by-step instructions:**

1. Install Ansible on your main computer: `sudo apt install ansible`
2. Write a playbook to install packages and configure your Pi
3. Run the playbook: `ansible-playbook -i <pi_ip>, playbook.yml`
4. Update your playbook as your setup evolves

---
