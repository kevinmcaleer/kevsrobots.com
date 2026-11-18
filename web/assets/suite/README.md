# @kevsrobots/suite

The shared look of the kevsrobots suite: **Learn** (kevsrobots.com), **Projects**, **Snakie by kevsrobots** and **BetterCAD by kevsrobots** (kevsrobots.com #581).

- `kr-tokens.css`: the `--kr-*` custom properties. Neutrals, status colours, the suite gold (#D9A441), one accent per app (kevsrobots red #C22032, Snakie green #26A269, BetterCAD violet #7C3AED), radii, spacing, shadows, focus ring and font stacks.
- `kr-suite-bar.js`: `<kr-suite-bar>`, a dependency-free web component with the kevsrobots mark, the app switcher, the app name, and optional search and account slot.

The files in this folder are the package. kevsrobots.com serves them as they are, and the same files are published to npm, so the two never drift.

## Use it from kevsrobots.com

```html
<link rel="stylesheet" href="https://www.kevsrobots.com/assets/suite/kr-tokens.css">
<script src="https://www.kevsrobots.com/assets/suite/kr-suite-bar.js" defer></script>
```

## Bundle it (Snakie, BetterCAD, desktop and offline)

```sh
npm install @kevsrobots/suite
```

```js
import '@kevsrobots/suite'                 // defines <kr-suite-bar>
import '@kevsrobots/suite/kr-tokens.css'
```

## The bar

```html
<html data-kr-app="snakie" data-kr-theme="auto">
…
<kr-suite-bar app="snakie" theme="auto" context="burgerbot / main.py"
              search="https://www.kevsrobots.com/search-results.html?query={q}&page=1">
  <button slot="account">KM</button>
</kr-suite-bar>
```

| Attribute | Values |
| --- | --- |
| `app` | `learn` (default), `projects`, `snakie`, `bettercad`. Sets the accent and marks the current app in the switcher. |
| `app-name` | The label beside the mark. Defaults to the app's name. |
| `context` | Smaller text after the name: a lesson, file or project. |
| `variant` | `bar` (default): the full 48 px bar with the accent stripe. `brand`: only the mark, switcher and name, inline, for a page that keeps its own navbar. |
| `theme` | `light` (default), `dark`, `auto`. |
| `home` | Where the mark links. Defaults to `https://www.kevsrobots.com/`. |
| `mark-src` | An image for the mark instead of the "KR" tile. |
| `search` | A URL with `{q}` where the query goes. Without it there is no search box. |
| `search-placeholder` | Placeholder text for search. |
| `hide-name` | Shows the app name to screen readers only. |

Slots: `actions` and `account`, at the right-hand end.

Events, both cancellable with `preventDefault()`:

- `kr-suite-search` (`detail: { query }`) before search navigates.
- `kr-suite-switch` (`detail: { app, href }`) before the switcher navigates. A desktop app can open its own window instead.

Set the `apps` property to your own list (`{ id, name, href, blurb, mark }`) to point the switcher somewhere else, such as a local dev server.

Keyboard: the switcher opens with Enter or Space, arrow keys, Home and End move through the apps, Escape closes it, and `/` jumps to search.

## Tokens

Put `data-kr-app` and `data-kr-theme` on `<html>` (the app's root). The light set is the default, so a light-only page can load the tokens safely. `data-kr-theme="dark"` turns the dark set on, and `"auto"` follows the system.

## Publishing

From this folder: bump `version` in `package.json`, then `npm publish`.
