# The project editor

How a user project on kevsrobots.com is written, stored and shown, for
anyone changing the editor (`web/projects/editor.html`) or the project page
(`web/projects/view.html`). The design came from epic #254, "document a
robot build as easily as writing a note".

## One Markdown document

A project is one Markdown document, `projects.content_md`. Everything a
reader sees in the write-up is in that text: the sections, the Bill of
Materials, wiring and diagram images, videos, code. The structured data
around it (images, files, BOM rows, wiring, diagrams) is either referenced
from the text or kept in step with it. When the two disagree, the text wins.

Projects in `content_mode = builder` (the Instruction Builder,
`web/projects/instructions/edit.html`) are the exception: their steps are
stored separately and the editor and project page render them with
`builder-renderer.js`. Nothing in the new editor changes them.

## The editor

`editor.html` mounts markdown-studio's editor bundle (the Notes editor),
vendored in `web/assets/js/markdown-editor/` by
`scripts/update-markdown-editor.sh`. Its `VERSION` file names the
markdown-studio commit. Re-vendor after a markdown-studio change merges,
and merge the markdown-studio PR first.

`project-editor.js` wires the bundle to the Projects API
(`stacks/projects-api`, https://projects.kevsrobots.com) and adds the
project's own pieces. The instance is `window.projectMarkdownEditor`, and
`project-markdown-editor:ready` fires once it is mounted.

| Piece | Where |
| --- | --- |
| `/` commands: canonical sections (Overview, Bill of Materials, Assembly instructions, Wiring, Code repository, Videos…), image, table, file, video, part, diagram, mermaid | `project-slash-commands.js`, section names in `project-sections.js` |
| "On this page" nav, the same on editor and page | `project-toc.js` |
| Details row (difficulty, time, tags), Draft/Published switch | `editor.html`, `project-editor.js` |
| Completeness meter (bottom right) | `project-editor.js` (`COMPLETENESS_ITEMS`) |
| Autosave and local drafts (`projects.<id>`) | `project-editor.js`, the bundle's draft keeper |

### Images, files and models

Images live in the gallery (`/api/projects/{id}/images`); the first is the
cover. The text links them as `![alt](…/images/{id}/view)`, with
`{full|half|cover|small}` size modifiers and image grids from the bundle.
Dropping a folder anywhere on the page sorts files: images to the gallery, a
README into the text, models to Files & Models (`project-editor.js`,
`processDroppedFiles`). STL thumbnails are drawn in the browser
(`model-thumbnails.js`); older models get theirs at
`/admin/projects/thumbnails/`.

### Graphics (Layers)

`/Graphic` opens Layers, markdown-studio's Canva-style graphic editor
(`packages/layers`), in the page in place of the picture, with a Full screen
button. Save uploads a PNG to the gallery and writes `![Title](…/images/{id}/view#layers)`:
the PNG carries its layers (an `iTXt` chunk), and `#layers` marks it, so
"Edit graphic" (hover or double-click) and "Draw on it" in a picture's
popover open it again. Nothing new on the server. The bundle's `layers`
option in `project-editor.js` loads `web/assets/js/layers/layers.js`,
vendored by `scripts/update-layers.sh` (re-run it with
`update-markdown-editor.sh` after markdown-studio Layers changes).

### Bill of Materials

The BOM is a table under the Bill of Materials heading. `bom-sync.js`
parses it (`BomSync.parse`) and after each save the editor sends the rows to
`PUT /api/projects/{id}/bom/sync`, so cost estimates and the parts catalog's
"used in projects" keep working. Parts link to the catalog
(`/parts/view.html?slug=…`) and show as chips (`part-links.js`). The editor
keeps a Total column and a **Total** row (`BomSync.totalsChanges`). Without a
table in the text nothing is synced.

### Wiring and diagrams

- Wiring is drawn in Snakie's electronics workspace (`project-wiring.js`,
  `snakie-host.js`, the embed bundle in `web/assets/js/snakie-parts/`).
  It is stored in `project_wiring` and the text shows its render,
  `/api/projects/{id}/wiring.svg`.
- `/diagram` is the Canva-lite diagram tool (`diagram-editor.js`, Fabric),
  stored in `project_diagrams` with an SVG the text links to.
- ```` ```mermaid ```` blocks stay as text and are drawn by Mermaid Factory's
  mermaid-tools (`web/assets/js/mermaid-tools/`, `scripts/update-mermaid-tools.sh`),
  themed by `project-mermaid-theme.js`.

## The project page

`view.html` renders `content_md` with marked and the same helpers
(callouts, image sizes and grids, video cards, part links, Mermaid, code
highlighting), so what the author sees in the editor is what readers see.
Structured sections the page draws itself (images, files, a stored BOM
without a table, links, journal, remixes) sit under the author's matching
heading when there is one.

## The look

Both pages share the Notes look of markdown-studio: warm paper, serif
headings, one terracotta accent.

- `web/assets/css/project-notes.css`: the tokens (`--bg`, `--text`,
  `--border`, `--accent`, `--font-serif`…) on `.kr-notes`, the page
  container, and the shared components: quiet sections (`.kr-section`) in
  place of Bootstrap cards, buttons, notices (`.kr-notice`), and the popovers
  and pickers. Dark tokens are there under `.kr-notes[data-theme="dark"]`
  but not switched on, because the rest of the site is light only.
- `web/assets/css/project-content.css`: the document on the project page,
  matching the bundle's live view sizes.
- `web/assets/css/project-editor.css`: the editor page's own pieces.

Keep the tokens in step with markdown-studio (`web/src/styles.css` and the
bundle's `standalone.css`).

## Older projects (issue #273)

Nothing in the new editor rewrites a project's text when it opens, apart
from writing in what used to be stored elsewhere: BOM totals, a Code
repository section for a stored `code_repo_url`, and a Videos section for
stored videos. Text saved with Windows line endings comes back with `\n`.

`/admin/projects/migration/` (API: `routers/admin_migration.py`) has:

- **Bill of Materials not in the text yet**: projects with stored BOM rows
  and no table. Owners see an "Insert BOM table" bar in the editor; the
  admin button appends the table for all of them under a
  `<!-- … generated … -->` comment, without changing `updated_at`.
- **Projects using the old schematic editor**: `project_schematics` rows on
  projects without wiring. They keep rendering on the project page (a
  "Schematic" section, rasterised by `builder-renderer.js`), and the editor
  links to `/projects/schematic/edit.html` until the owner redraws the
  wiring. When the count reaches zero the schematic and symbol editors can
  be removed.
- **Open-and-save check**: loads every project's text into the editor bundle
  in the browser and reads it back, without saving.

## Tests

- `node --test scripts/tests/*.test.js`: the BOM parser, totals and parts catalog.
- `cd stacks/projects-api && python -m pytest`: the API, including
  `tests/test_admin_migration.py`.
