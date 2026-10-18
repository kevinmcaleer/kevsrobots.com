---
title: Installing MongoDB
description: Get a MongoDB 8.0 server running with Docker, the free Atlas cloud tier, or a native install, then check it works
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/03_installing_mongodb.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Time to get a real MongoDB server running. There are three ways to do it, and you only need **one**:

| Option | Best for | What you install |
|---|---|---|
| 1. Docker (recommended) | Most people, including Raspberry Pi 5 owners | Docker, then one command |
| 2. MongoDB Atlas | No install at all, or an older Raspberry Pi | Just `mongosh` (or nothing, using the web shell) |
| 3. Native install | People who prefer not to use Docker | MongoDB and `mongosh` directly |
{:class="table table-single"}

Whichever you choose, you will finish this lesson at the same place: a `mongosh` prompt connected to a MongoDB 8.0 server.

---

## Option 1 - Docker (recommended)

**Docker** runs programs inside **containers**: neat, self-contained boxes that bring everything they need with them. It means you can run MongoDB without installing it into your system, and remove it cleanly afterwards. If Docker is new to you, our [Docker course](/learn/docker/) covers installing it on every platform.

With Docker installed, start MongoDB with one command:

```bash
docker run -d --name mongodb -p 27017:27017 -v mongodb_data:/data/db mongo:8.0
```

Here is what each part does:

| Part | Meaning |
|---|---|
| `-d` | Run in the background (detached) |
| `--name mongodb` | Call the container `mongodb` so we can refer to it easily |
| `-p 27017:27017` | Connect port 27017 on your computer to port 27017 in the container |
| `-v mongodb_data:/data/db` | Keep the data in a Docker **volume** so it survives restarts |
| `mongo:8.0` | The official MongoDB 8.0 image |
{:class="table table-single"}

The first run downloads the image, which takes a minute. Then open the shell inside the container:

```bash
docker exec -it mongodb mongosh
```

The official image comes with `mongosh` already installed, so there is nothing else to set up. You should see some startup messages and then a prompt:

```
test>
```

That means you are in. Skip ahead to [Check it works](#check-it-works).

When you reboot, start the container again with `docker start mongodb`. Your data is still in the volume.

---

## Option 2 - MongoDB Atlas (no install)

**MongoDB Atlas** is MongoDB's cloud service, and it has a free tier called **M0** with 512 MB of storage, which is far more than our Parts Bin will ever need. The server runs on MongoDB's computers and you connect over the internet.

The Atlas website changes its layout now and then, so here are the steps in outline:

1. Sign up at [mongodb.com/atlas](https://www.mongodb.com/atlas) and create a new project.
2. **Create a cluster** and choose the free **M0** option. Pick a cloud region near you.
3. **Create a database user.** This is a username and password for connecting to the database, separate from your Atlas login. Write them down.
4. **Set up network access.** Atlas blocks every connection until you allow it. Add your current IP address to the **IP access list**.
5. Click **Connect**, choose **Shell**, and copy the connection string. It looks something like `mongodb+srv://cluster0.abcde.mongodb.net/`.

A **connection string** is the address of your database, written as a single line. Install `mongosh` on your computer from the [MongoDB Shell download page](https://www.mongodb.com/try/download/shell), then connect with your own string and username:

```bash
mongosh "mongodb+srv://cluster0.abcde.mongodb.net/" --apiVersion 1 --username kev
```

It will ask for your database user's password, then show a prompt. Atlas also has a shell built into its website if you would rather not install anything.

Throughout the course, wherever you see `mongodb://localhost:27017`, Atlas users swap in their own connection string.

---

## Option 3 - Native install

If you would rather install MongoDB directly, here are the short versions. MongoDB's [installation docs](https://www.mongodb.com/docs/manual/installation/) have the full details for every platform.

### macOS (Homebrew)

```bash
brew tap mongodb/brew
brew install mongodb-community@8.0
brew services start mongodb-community@8.0
```

The last line starts MongoDB now and every time you log in. Then type `mongosh` to connect. If `mongosh` is not found, run `brew install mongosh`.

### Windows

Download the **MongoDB Community Server** MSI installer from the [download centre](https://www.mongodb.com/try/download/community) and run it, choosing to install MongoDB as a service so it starts automatically. The server installer does **not** include the shell, so also install **mongosh** from the [shell download page](https://www.mongodb.com/try/download/shell). Then open a new terminal and type `mongosh`.

### Ubuntu and Debian

MongoDB isn't in the standard package lists, so you add MongoDB's own package repository first. The steps differ slightly between versions, so follow the [official Ubuntu guide](https://www.mongodb.com/docs/manual/tutorial/install-mongodb-on-ubuntu/) or [Debian guide](https://www.mongodb.com/docs/manual/tutorial/install-mongodb-on-debian/). Once installed, start it with `sudo systemctl start mongod` and connect with `mongosh`.

---

## A note for Raspberry Pi owners

MongoDB 5.0 and later need a processor that supports **ARMv8.2-A**.

- **Raspberry Pi 5** has one, so it runs MongoDB 8.0 happily. Use 64-bit Raspberry Pi OS and Option 1 (the `mongo:8.0` Docker image).
- **Raspberry Pi 4 and earlier** don't, so the newest MongoDB they can run is 4.4, which is long out of support. Use **Atlas** (Option 2) instead and connect from the Pi with `mongosh` or Python.

---

## Optional - MongoDB Compass

**MongoDB Compass** is a free desktop app for browsing your data with a mouse instead of typing commands. It is a nice way to peek at your documents, and you can download it from the [Compass page](https://www.mongodb.com/try/download/compass). Connect it to `mongodb://localhost:27017` (or your Atlas string). We will stick to `mongosh` in this course, because commands are easier to learn from and to repeat.

---

## Check it works

At the `test>` prompt, ask the server if it is alive. This sends a "ping" command:

```javascript
db.runCommand({ ping: 1 })
```

```
{ ok: 1 }
```

`ok: 1` means the server heard you and replied. Now check which version you are running:

```javascript
db.version()
```

```
8.0.34
```

The last number may be different for you. Anything starting with `8.0` is perfect.

---

## Try it Yourself

1. Find out which version of `mongosh` itself you have. Hint: try `version()`.
2. Docker users: type `exit` to leave the shell, run `docker stop mongodb`, then `docker start mongodb` and reconnect. Is the ping still `ok: 1`?
3. Run `db.runCommand({ ping: 1 })` again but misspell `ping` as `pong`. What does the server say?

<details><summary>Show answers</summary>

1. `version()` prints the shell's version, for example `2.13.0`. That is separate from the server version shown by `db.version()`.
2. Yes - stopping and starting the container keeps everything, because the data lives in the `mongodb_data` volume.
3. If you type `pong` you get `MongoServerError[CommandNotFound]: no such command: 'pong'`. The server only understands commands it knows about, and it is strict about spelling.

</details>

---

## Common Issues

**Problem:** `MongoNetworkError: connect ECONNREFUSED 127.0.0.1:27017`  
**Solution:** The server isn't running. Docker users run `docker start mongodb`; Homebrew users run `brew services start mongodb-community@8.0`; Linux users run `sudo systemctl start mongod`.  
**Why:** `mongosh` is only a client. It needs a server listening on port 27017 to talk to.

**Problem:** `docker: Error response from daemon: Conflict. The container name "/mongodb" is already in use`  
**Solution:** You already created it. Use `docker start mongodb` instead of `docker run`.  
**Why:** `docker run` creates a new container each time, and names must be unique.

**Problem:** Atlas connection hangs and then times out.  
**Solution:** Check your IP address is on the Atlas **IP access list**. Your home IP can change, so you may need to add it again.  
**Why:** Atlas silently refuses connections from addresses it doesn't recognise.

**Problem:** `mongosh: command not found`  
**Solution:** Install `mongosh` separately, or use `docker exec -it mongodb mongosh` if you went the Docker route.  
**Why:** The server and the shell are separate programs, and some installers only include the server.

---

## Recap

- Docker is the quickest route: one `docker run` command and `docker exec -it mongodb mongosh`
- Atlas gives you a free M0 server in the cloud, ideal for older Raspberry Pis
- Native installs work on macOS, Windows and Linux, but install `mongosh` too
- `db.runCommand({ ping: 1 })` returning `{ ok: 1 }` proves you are connected

Next, we will take our first steps in `mongosh` and create the workshop database.
