---
layout: content
title: Connect KevsRobots to your AI assistant
description: Add KevsRobots search to Claude, or any MCP-capable AI app, so it can answer questions from the site's tutorials, courses and project write-ups - with real links.
date_published: 2026-08-27
cover: /assets/img/og_image.jpg
---

{% include breadcrumbs.html %}

# <i class="fas fa-plug me-3 text-primary"></i> Connect KevsRobots to your AI assistant

Ask an AI assistant about a robot build and it will often half-remember something, or make it up entirely. This lets it search this site properly instead - so its answers come from the actual tutorials, with links you can click and check.

It uses **MCP** (Model Context Protocol), an open standard for connecting AI apps to outside tools. Setup is one line, it is free, and it is read-only - there is no account and nothing to sign up for.

---

## The endpoint

```
https://search.kevsrobots.com/mcp
```

That is all most apps need.

---

## Claude Code

In your terminal:

```bash
claude mcp add --transport http kevsrobots https://search.kevsrobots.com/mcp
```

Then ask it something like *"search kevsrobots for how to wire a DRV8833"*.

---

## Claude Desktop

1. Open **Settings** and go to **Connectors**
2. Choose **Add custom connector**
3. Name it `KevsRobots` and paste in `https://search.kevsrobots.com/mcp`
4. Save, then start a new chat

---

## Any other MCP client

Most other apps take a JSON config. The endpoint uses the **streamable HTTP** transport:

```json
{
  "mcpServers": {
    "kevsrobots": {
      "type": "http",
      "url": "https://search.kevsrobots.com/mcp"
    }
  }
}
```

Some clients still expect the older `command`/`args` shape for local servers. If yours has no HTTP option, it will need a bridge such as `mcp-remote` - check your app's own MCP documentation.

---

## What your assistant can do with it

Two tools become available:

| Tool | What it does |
|---|---|
| `search_kevsrobots` | Searches the whole site by meaning, not just keywords. Filters by content type, tag or course |
| `get_kevsrobots_page` | Reads a full page, for when an excerpt is not enough |

The search covers everything on the site: courses and lessons, blog posts, project write-ups, robot builds, hardware reviews and how-it-works explainers.

Things worth trying once it is connected:

- *"Search kevsrobots for a beginner MicroPython course and summarise what it covers"*
- *"What does kevsrobots say about wiring a PCA9685 to a Pico?"*
- *"Find the parts list for the SMARS robot"*

---

## Good to know

**It is read-only.** The tools search and read public pages. Nothing writes anything, and no account or key is involved.

**Answers should carry links.** The tools return a URL with every result, and assistants are asked to cite them. If you get an answer with no link, be sceptical - it may not have used the search at all.

**It searches, it does not think.** The assistant does the reasoning; this just gives it the right pages to reason from. If the site does not cover something, the honest answer is that there is nothing here about it.

**It cannot tell you a page is wrong.** If a tutorial has a mistake in it, the assistant will faithfully repeat that mistake with a citation. The link is there so you can check - use it.

---

## Trouble

- **Assistant does not use the tools** - ask explicitly: *"search kevsrobots for..."*. Some apps only reach for a tool when the request clearly calls for it.
- **Connection fails** - check the URL has no trailing slash and starts with `https`.
- **No results** - try fewer words, or drop any filters. A full question generally works better than one keyword.

If something is broken, [open an issue](https://github.com/kevinmcaleer/kevsrobots.com/issues) and let me know.
