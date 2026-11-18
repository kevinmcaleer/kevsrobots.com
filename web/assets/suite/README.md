# @kevsrobots/suite

The kevsrobots suite's app switcher and account button, as two dependency-free
web components shared by kevsrobots.com, Snakie and BetterCAD (#581).

```html
<script type="module" src="https://www.kevsrobots.com/assets/suite/kr-suite.js"></script>

<kr-app-switcher app="snakie" target="_blank"></kr-app-switcher>
<kr-account app="snakie"><span slot="note">Only needed to send wiring to kevsrobots.</span></kr-account>
```

- `app`: `learn`, `projects`, `snakie` or `bettercad`. Marks where you are and picks the accent.
- `theme`: `light` or `dark`. Without it, the system setting.
- `target` (switcher): passed to the links, e.g. `_blank` in an editor.
- `.user` (account): `{ name, username, avatarUrl }` when signed in, else `null`.
  Fires `kr-sign-in` and `kr-sign-out`; the app does the signing in. `busy` disables the buttons.
- Colours come from `--kr-ink`, `--kr-ink-soft`, `--kr-line`, `--kr-panel`, `--kr-sunk`,
  `--kr-shadow`, `--kr-accent`, `--kr-on-accent`, `--kr-focus` and `--kr-font-ui` when set.

This folder is the canonical copy. Snakie (`src/renderer/src/vendor/kr-suite.js`) and
BetterCAD (`web/src/vendor/kr-suite.js`) bundle the same file so they work offline and on
the desktop. Change it here, bump `KR_SUITE_VERSION` and `version`, then copy it across.
Tests: `node --test scripts/tests/kr-suite.test.js`.
