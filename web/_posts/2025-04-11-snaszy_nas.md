---
title: "Snaszy NAS a 3D printed NAS for Raspberry Pi"
description: >- 
    Snaszy NAS is a 3D printed NAS for Raspberry Pi, designed for 2.5 inch drives. It uses BTRFS for RAID and is powered by a Raspberry Pi 5.
excerpt: >-
    Snaszy NAS is a 3D printed NAS for Raspberry Pi, designed for 2.5 inch drives. It uses BTRFS for RAID and is powered by a Raspberry Pi 5.
layout: showcase
date: 2025-04-11
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/snaszy_nas/cover.jpg
hero:  /assets/img/blog/snaszy_nas/hero.png
mode: light
videos:
  - CjX5USkejTI
tags:
  - NAS
  - raspberrypi
groups:
  - raspberrypi
stl:
  - name: "Bottom - snaszy_bottom.stl"
    link: "/assets/stl/snaszy_nas/snaszy_bottom.stl"
  - name: "Top - snaszy_top.stl"
    link: "/assets/stl/snaszy_nas/snaszy_top.stl"
  - name: "Back - snaszy_back.stl"
    link: "/assets/stl/snaszy_nas/snaszy_back.stl"
  - name: "Front - snaszy_front.stl"
    link: "/assets/stl/snaszy_nas/snaszy_front.stl"
  - name: "Pi 5 Tray - pi_5_tray.stl"
    link: "/assets/stl/snaszy_nas/pi_5_tray.stl"
  - name: "Frame - snaszy_frame.stl"
    link: "/assets/stl/snaszy_nas/snaszy_frame.stl"
  - name: "Flap - snaszy_flap.stl"
    link: "/assets/stl/snaszy_nas/snaszy_flap.stl"
  - name: "Lid - snaszy_lid.stl"
    link: "/assets/stl/snaszy_nas/snaszy_lid.stl"
  - name: "Base - snaszy_base.stl"
    link: "/assets/stl/snaszy_nas/snaszy_base.stl"
---

## What is Snaszy NAS?

Snaszy NAS is a 3D printed NAS for Raspberry Pi, designed for 2.5 inch drives. It uses [BTRFS](#what-is-btrfs) for RAID and is powered by a Raspberry Pi 5. The design is compact and easy to assemble, making it a great option for those looking to build their own, affordable NAS.

It looks awesome, too.

---

## Features

* 3D printed case
* 2.5 inch drive support
* BTRFS RAID support
* Powered by Raspberry Pi 5
* Compact design
* Easy to assemble
* Affordable
* Open source design
* Compatible with Raspberry Pi OS

---

## Gallery

Here are some pictures of Snaszy NAS in various stages of design and construction.

{% include gallery.html images="/assets/img/blog/snaszy_nas/nas01.jpg,/assets/img/blog/snaszy_nas/nas02.jpg,/assets/img/blog/snaszy_nas/nas03.jpg,/assets/img/blog/snaszy_nas/nas04.jpg,/assets/img/blog/snaszy_nas/nas05.jpg,/assets/img/blog/snaszy_nas/nas06.jpg,/assets/img/blog/snaszy_nas/nas07.jpg,/assets/img/blog/snaszy_nas/nas09.jpg,/assets/img/blog/snaszy_nas/nas10.jpg,/assets/img/blog/snaszy_nas/nas11.jpg,/assets/img/blog/snaszy_nas/nas12.jpg,/assets/img/blog/snaszy_nas/nas13.jpg,/assets/img/blog/snaszy_nas/nas14.jpg,/assets/img/blog/snaszy_nas/nas15.jpg" titles="It fits!, Tray Prototype, Drive bays, Finished Tray, Prusa Core One ,Printing Drive bay holder, Top view, Pi Powered, Dramatic Pose, Closeup, Chihuahua for scale, Rear view, Synology for scale,Power Routing" cols=4 small_title=true%}

---

> ## What is BTRFS?
>
> Btrfs (B-tree file system) is a modern copy-on-write (CoW) filesystem for Linux. 
>
> It is designed to address the shortcomings of traditional filesystems like ext4, providing advanced features such as snapshots, subvolumes, and built-in RAID support.
> Btrfs is particularly well-suited for use in NAS (Network Attached Storage) systems, as it allows for easy management of multiple drives and provides data redundancy through RAID configurations.
> Btrfs supports various RAID levels, including RAID 0, RAID 1, RAID 5, RAID 6, and RAID 10. It also allows for dynamic resizing of volumes and the ability to add or remove drives from a RAID array without downtime.
> Btrfs is a powerful and flexible filesystem that is ideal for use in NAS systems, providing advanced features and data protection capabilities.

## Design

Snaszy NAS is designed to be compact and easy to assemble. The case is 3D printed and can be printed using a standard FDM printer. The design is open source and can be modified to suit your needs.

The NAS has 4 drive bays for 2.5 inch drives either the slimmer 7mm style or full 15mm drives.

![Snaszy NAS](/assets/img/blog/snaszy_nas/components.jpg){:class="img-fluid w-100 rounded-3"}

---

## Bill of Materials

To build your own Snaszy NAS, you will need the following parts:

Item         | Description                                   |   Price |  Qty  |   Total
-------------|-----------------------------------------------|--------:|:-----:|-------:
Raspberry Pi | Raspberry Pi 5 4Gb                            |  £57.30 |   1   |  £57.30
Fan          | Active cooling fan                            |   £4.50 |   1   |   £4.50
5Tb Drive    | LaCie Rugged Mini 5Tb - or whatever you have! | £159.99 |   2   | £319.98
Power        | 45W USB-C Power Supply                        |  £14.40 |   1   |  £14.40
SSD to USB   | USB to SATA Adapter                           |   £6.00 |   2   |  £12.00
             |                                               |         | Total | £407.18
{:class="table table-striped"}

The hard disks are the most expensive part of the build, but you can use any 2.5 inch drives you have lying around. The LaCie drives are rugged and portable, making them a great option for a NAS.

---

## Setting up the NAS

Next we'll setup the NAS using BTRFS and Samba. This will allow you to share files over the network and access them from other devices.

---

### Install BTFRS

```bash
# Install BTFRS
sudo apt install btrfs-progs
```

---

### Setup a drive
```bash
sudo fdisk /dev/sda
```

Inside `fdisk`:
- Press `g` to create a new empty GPT partition table.
- Press `n` to add a new partition (accept defaults to use the whole disk).
- Press `w` to write the table to disk and exit.

---

### Format drive

```bash
# Double-check this is the correct device/partition before running! 
sudo mkfs.btrfs /dev/sda1
```

or add a label for easier identification

```bash
sudo mkfs.btrfs -L MyBtrfsDrive /dev/sda1
```

---

### Mount drive

```bash
sudo mount /dev/sda1 /mnt/my_btrfs_drive
```

---

### Create a sub volumne (for snapshoting)

``` bash
# List existing subvolumes
sudo btrfs subvolume list /mnt/my_btrfs_drive
```

```bash
#create a subvolume
sudo btrfs subvolume create /mnt/my_btrfs_drive/data
```

---

### Create a snapshot

```bash
# Optional: Create a directory to hold snapshots if it doesn't exist 
sudo mkdir/mnt/my_btrfs_drive/snapshots 

# Create the snapshot 
sudo btrfs subvolume snapshot /mnt/my_btrfs_drive/data /mnt/my_btrfs_drive/snapshots/data_$(date +%Y-%m-%d_%H-%M-%S)

```

---

### Add a second drive

```bash
sudo btrfs device add /dev/sdb /mnt/mybtrfs
```

```bash
# Validate the additional drive
sudo btrfs filesystem show /mnt/mybtrfs
```

---

### Balance drive

**Balance and Convert to RAID 1:** Now, start a balance operation to convert both data (`-d`) and metadata (`-m`) chunks from the current profile (`single`) to `raid1`.

* `-dconvert=raid1`: Convert data chunks to RAID 1 profile.
* `-mconvert=raid1`: Convert metadata chunks to RAID 1 profile.
* This process reads existing data/metadata chunks and writes them back according to the new profile across the available devices.
* The filesystem remains **online and usable** during the balance, although performance might be impacted.

```bash
sudo btrfs balance start -dconvert=raid1 -mconvert=raid1 /mnt/mybtrfs
```

**Monitor the Balance (Optional but Recommended):** Balances can take a long time, depending on the amount of data and disk speed. You can check the status in another terminal:

```bash
sudo btrfs balance status /mnt/mybtrfs
```

### Add a 3rd Drive and set to RAID5

```bash
sudo btrfs device add /dev/sdc /mnt/mybtrfs
```

Convert the mirror (RAID1) to RAID5 and balance the drive

```bash
sudo btrfs balance start -dconvert=raid5 -mconvert=raid1 /mnt/mybtrfs
```

---

## Making snapshots immutable

### What is an immutable snapshot?

An immutable snapshot is a snapshot that cannot be modified or deleted. This is useful for protecting important data and ensuring that it remains unchanged.

Let's say you created a snapshot of /home like this:

```bash
# sudo btrfs subvolume snapshot /home /.snapshots/home_backup_20250413

# To make it read-only (immutable):
sudo btrfs property set /.snapshots/home_backup_20250413 ro true
```

You can check that the snapshot is read-only by running:

```bash
sudo btrfs property get /.snapshots/home_backup_20250413 ro
```

This should return `ro = true`, indicating that the snapshot is read-only.
You can also check the status of the snapshot by running:

```bash
sudo btrfs subvolume show /.snapshots/home_backup_20250413
```

This will show you the details of the snapshot, including its ID, creation time, and whether it is read-only or not.


This will make the snapshot read-only, preventing any changes to it. You can still access the snapshot and read its contents, but you cannot modify or delete it.

---

## Setup Filesharing with Samba

`Samba` is a free software re-implementation of the SMB networking protocol, and was originally developed by Andrew Tridgell. It allows for file and print sharing between computers running Windows and those running Unix or Linux.

### Install Samba

To install Samba on your Raspberry Pi, run the following command:

```bash
sudo apt update && sudo apt upgrade
sudo apt install samba
```

---

### Add to Samba

To add a new share to Samba, open the Samba configuration file in a text editor:

```bash
sudo nano /etc/samba/smb.conf
```
Add the following lines to the end of the file:

```bash
[MyBtrfsDrive]
   path = /mnt/my_btrfs_drive
   valid users = pi
   read only = no
   browsable = yes
   public = yes
```

This will create a new share called `MyBtrfsDrive` that points to the `/mnt/my_btrfs_drive` directory. The `valid users` line specifies which users are allowed to access the share (so make sure that matches your user setup). The `read only` line specifies whether the share is read-only or not. The `browsable` line specifies whether the share should be visible in the network browser. The `public` line specifies whether the share is public or not.

To save the changes, press `CTRL + X`, then `Y`, and then `Enter`.

### Restart Samba

To apply the changes, restart the Samba service:

```bash
sudo systemctl restart samba
```

You can check the status of the Samba service with the following command:

```bash
sudo systemctl status samba
```

---

## Cockpit

Cockpit is a web-based server management tool that makes it easy to manage your server from a web browser. It provides a user-friendly interface for managing system resources, services, and configurations.

---

### Install Cockpit

To install Cockpit on your Raspberry Pi, run the following command:

```bash
sudo apt install cockpit
```

---

### What is Cockpit?

Cockpit is a web-based server management tool that makes it easy to manage your server from a web browser. It provides a user-friendly interface for managing system resources, services, and configurations.

---

### Accessing Cockpit

To access Cockpit, open a web browser and navigate to `http://<your-raspberry-pi-ip>:9090`. You will be prompted to log in with your Raspberry Pi username and password.

---

## Next Steps

Now that you have your Snaszy NAS up and running, you can start using it to store and share files. You can also explore other features of BTRFS, such as snapshots and subvolumes, to further enhance your NAS experience.

---
