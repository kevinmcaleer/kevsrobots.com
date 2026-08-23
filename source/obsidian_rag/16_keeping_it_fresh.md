---
title: Keeping the Index Fresh
description: Re-index on a schedule, watch the vault for changes, and decide which approach suits how you actually work
layout: lesson
type: page
cover: /learn/obsidian_rag/assets/banners/16_keeping_it_fresh.jpg
date_updated: 2026-08-23
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

An index is only useful if it matches your notes. Right now you have to remember to run `amv index`, and you will not.

Because indexing is incremental, keeping up to date is cheap - a no-change run on a big vault takes well under a second. That opens up options that would be silly if a rebuild took a minute.

---

## Three approaches

| Approach | Latency | Complexity | Good for |
|---|---|---|---|
| Run it before you ask | Seconds | None | Everyone, honestly |
| Scheduled job | Up to an hour | Low | Vaults you edit all day |
| File watcher | Instant | Medium | If instant genuinely matters |
{:class="table table-single"}

Start at the top. Move down only when the one above stops being good enough.

---

## Just index before you ask

The simplest thing that works. Add a flag:

```python
ask_parser.add_argument("--fresh", action="store_true",
                        help="Re-index before answering")
```

```python
def cmd_ask(args) -> int:
    from .answer import check_citations, stream_answer

    if args.fresh:
        index_vault(VAULT_PATH, args.db, verbose=False)
    ...
```

On a vault where nothing changed, `index_vault` reads the hashes, compares, and returns. You will not notice it happen.

If you find yourself passing `--fresh` every time, just make it the default and add `--stale` for the rare case where you want to skip it.

---

## A scheduled job

For a vault you edit throughout the day, run it on a timer. On macOS or Linux, `cron` is the least effort:

```bash
crontab -e
```

```cron
# Re-index the vault every 30 minutes
*/30 * * * * /Users/kev/ask-my-vault/.venv/bin/amv index --quiet >> /tmp/amv.log 2>&1
```

Two things that catch people out with cron:

**Use the absolute path to the venv's `amv`.** Cron does not run your shell profile, so your virtualenv is not active and `amv` is not on the PATH.

**Cron has almost no environment.** `VAULT_PATH` will not be set, so either put the default in `config.py` or set it in the crontab:

```cron
VAULT_PATH=/Users/kev/Obsidian/MyVault
*/30 * * * * /Users/kev/ask-my-vault/.venv/bin/amv index --quiet >> /tmp/amv.log 2>&1
```

Redirecting to a log file is worth doing. A cron job that fails silently is a cron job you will discover three weeks later when you wonder why the answers are stale.

On a Raspberry Pi that runs all the time, this is the right answer - index every half hour and forget about it.

---

## A file watcher

If you want the index updated the moment you save a note, watch the filesystem. This needs one more dependency:

```bash
pip install watchdog
```

```python
# vault_rag/watch.py
"""Watch the vault and re-index when notes change."""

import time
from pathlib import Path

from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

from .config import DB_PATH, VAULT_PATH
from .index import index_vault

DEBOUNCE_SECONDS = 5.0


class VaultHandler(FileSystemEventHandler):
    """Note when a markdown file changes; the main loop does the work."""

    def __init__(self):
        self.dirty_since: float | None = None

    def on_any_event(self, event):
        if event.is_directory:
            return
        if not str(event.src_path).endswith(".md"):
            return
        self.dirty_since = time.time()


def watch(vault_path: Path = VAULT_PATH, db_path: Path = DB_PATH):
    """Re-index a few seconds after the last change settles."""
    handler = VaultHandler()
    observer = Observer()
    observer.schedule(handler, str(Path(vault_path).expanduser()), recursive=True)
    observer.start()
    print(f"Watching {vault_path}. Ctrl-C to stop.")

    try:
        while True:
            time.sleep(1)
            if handler.dirty_since is None:
                continue
            if time.time() - handler.dirty_since < DEBOUNCE_SECONDS:
                continue

            handler.dirty_since = None
            stats = index_vault(vault_path, db_path, verbose=False)
            if stats.added or stats.updated or stats.removed:
                print(f"  {stats.summary()}")
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
```

**The debounce is the important part.** Obsidian saves on a timer while you type, so a single paragraph produces a dozen write events. Without debouncing you would re-index a dozen times. Waiting five seconds after the *last* change means you index once, when you pause.

**We do not track which file changed.** We could, but incremental indexing already figures that out from the hashes, and a "something changed, go look" signal is far simpler than maintaining a queue of paths. Let each layer do its own job.

**Only printing when something happened** keeps the terminal quiet. A watcher that logs "0 changed" every five seconds is a watcher you will kill.

Add it as a command:

```python
def cmd_watch(args) -> int:
    from .watch import watch
    watch(args.vault, args.db)
    return 0
```

---

## Running the watcher in the background

On macOS, a launch agent at `~/Library/LaunchAgents/com.kevsrobots.amv.plist`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<plist version="1.0">
<dict>
    <key>Label</key>
    <string>com.kevsrobots.amv</string>
    <key>ProgramArguments</key>
    <array>
        <string>/Users/kev/ask-my-vault/.venv/bin/amv</string>
        <string>watch</string>
    </array>
    <key>RunAtLoad</key>
    <true/>
    <key>KeepAlive</key>
    <true/>
    <key>StandardErrorPath</key>
    <string>/tmp/amv.err</string>
</dict>
</plist>
```

```bash
launchctl load ~/Library/LaunchAgents/com.kevsrobots.amv.plist
```

On Linux, a systemd user unit at `~/.config/systemd/user/amv.service`:

```ini
[Unit]
Description=Ask My Vault indexer

[Service]
ExecStart=/home/kev/ask-my-vault/.venv/bin/amv watch
Restart=always
Environment=VAULT_PATH=/home/kev/Obsidian/MyVault

[Install]
WantedBy=default.target
```

```bash
systemctl --user enable --now amv
```

---

## What about sync?

If your vault syncs across devices with Obsidian Sync, iCloud or Dropbox, the watcher fires when a *sync* writes a note, not just when you do. That is correct behaviour - the note really did change - but it means the index updates from edits made on your phone too, which is rather nice.

The one thing to avoid is putting the Chroma database inside the synced vault. It is a binary SQLite file plus index data; syncing it between machines will corrupt it, and re-indexing locally is fast anyway. Keep it in `~/.ask-my-vault/`, as `config.py` already does.

---

## Try it Yourself

1. Time `amv index` on your vault with nothing changed. Under a second? Then `--fresh` on every ask costs you nothing.
2. Set up the cron job and check `/tmp/amv.log` the next day. Did it run? Did it find anything?
3. Run the watcher, edit a note in Obsidian, and watch how long the debounce takes to fire. Tune `DEBOUNCE_SECONDS` to how fast you type.
4. Remove the debounce entirely and type a paragraph. Count the re-index messages.
5. Add a `--fresh` flag to `search` as well as `ask`. Do you actually want it there? Searching is often something you do *while* editing.

---

## Common Issues

- **Problem**: The cron job never runs.
- **Solution**: Check `/tmp/amv.log`. If it is empty, cron is not firing at all - on macOS, grant `cron` Full Disk Access in System Settings.
- **Why**: Recent macOS versions block cron from reading Documents and iCloud folders until you allow it explicitly.

- **Problem**: The watcher fires constantly with no edits.
- **Solution**: Check for a sync client or a plugin writing into the vault - some write cache files on a timer.
- **Why**: We already filter to `.md`, but a plugin writing `.md` cache files will still trigger it. Add its folder to `SKIP_DIRS` and filter the event path too.

- **Problem**: `OSError: inotify watch limit reached` on Linux.
- **Solution**: Raise it: `sudo sysctl fs.inotify.max_user_watches=524288`.
- **Why**: Recursive watching costs one inotify watch per directory, and the default limit is low for a big vault.

- **Problem**: The database gets corrupted every few days.
- **Solution**: Move it out of the synced vault folder.
- **Why**: Sync clients merge file changes byte-wise, which is fatal for a SQLite database being written from two machines.
