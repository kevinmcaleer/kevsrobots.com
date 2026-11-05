# Project versions (epic #433)

A `projects` row is the author's **working copy**. The editor autosaves into
it and into the rows that hang off it (tags, BOM, videos, links, wiring,
diagrams, schematic, images, files). Readers never see it.

**Publish** takes a snapshot of all of that, stores it as an immutable
`project_versions` row, and points `projects.published_version_id` at it.
Everything public serves that snapshot. The code is in
`projects_api/versions.py` (model, snapshot, restore, what a request is
shown) and `projects_api/routers/versions.py` (endpoints).

## Tables

| Column | Meaning |
|---|---|
| `project_versions.number` | 1, 2, 3 … per project, never reused (`projects.last_version_number` remembers the highest) |
| `project_versions.kind` | `published` (went live when made) or `unpublished` (the working copy, saved before a restore) |
| `project_versions.snapshot` | JSON: the project's fields, sorted tags, BOM / video / link rows, wiring `{data, svg_path, svg_hash}`, diagrams `{id: {title, data, svg_path, png_path, …}}`, schematic, and the images and files it uses (`{id, caption, sort_order}`, `{id, description, print_*}`) |
| `project_versions.published_at` | when it went live (NULL for unpublished) |
| `project_versions.note` | optional "what changed", ≤ 200 chars |
| `projects.published_version_id` | the live version; NULL = nothing live (status `draft`) |
| `project_images.removed_at`, `project_files.removed_at` | removed from the working copy but still used by a version |

A never-published project has no version rows; the editor calls it **v0**.

## Endpoints (owner only)

- `POST /api/projects/{id}/publish` `{note?}` → `{version, project}`. 409 when the working copy is the live version already.
- `POST /api/projects/{id}/unpublish` — nothing live, status `draft`, versions kept.
- `POST /api/projects/{id}/archive`, `/unarchive` — the live version stays live.
- `GET /api/projects/{id}/versions` → `{live_number, has_unpublished_changes, versions: [...]}` newest first.
- `GET /api/projects/{id}/versions/{n}` → one version with its snapshot.
- `POST /api/projects/{id}/versions/{n}/restore` → `{project, restored, saved_as}`. Saves the working copy as an unpublished version first, unless some version already has exactly that content.
- `DELETE /api/projects/{id}/versions/{n}` — 409 for the live version. Removes the version's picture copies and removed images/files nothing else uses.
- `PUT /api/projects/{id}` takes `expected_updated_at`; a row that changed since → 409 `stale`.

`PUT … {status}` still works the old way for older editors: `completed` publishes, `draft` unpublishes, `archived` archives.

## What a request is shown (`versions.resolve_view`)

Every public read (`GET /api/projects/{id}`, by-slug, `/bom`, `/images`, `/files`,
`/links`, `/videos`, `/wiring`, `/wiring.svg`, `/diagrams…`, `/schematic`) takes
`?version=`:

| Caller | no `version` | `live` | `working` / `N` |
|---|---|---|---|
| author | working copy | live version (`no-store`) | that state (`no-store`) |
| anyone else | live version | live version | 404 |

Someone else's draft is a 404. A public project with no version (only possible
for a row an old API client created as `wip`) reads as its working copy, as it
did before versions.

Image views, file downloads and thumbnails are public only when the live
version uses them; otherwise they are the author's alone and sent `no-store`.

Listings (hub, search, tags, featured, staff picks, popular, remixes, OG cards)
read the live version's title, description, cover, difficulty and tags
(`versions.LiveView`). `/my/list` is the author's working copies with
`live_number` and `has_unpublished_changes`.

Readers never see version numbers: the response has `published_at` (first
publish) and `updated_at` (when the live version was published).

## Pictures

The wiring SVG and diagram SVG/PNG are rewritten in place on every save, so a
version keeps its own copies under `projects/versions/{project_id}/`, named by
content hash: a picture that didn't change between versions is stored once.
Restore copies a version's pictures back into the working files.

Images and files are a shared pool referenced by id. Deleting one a version
uses only sets `removed_at` (it leaves the gallery; the live page and older
versions keep it). Restoring a version clears `removed_at` on the ones it uses.

## Slugs and activity

The slug follows the title only while nothing is live. Once a version is live
it changes on publish, from the published title, with the old slug kept in
`project_slug_history` (301). Publish logs `project_published` for the first
version and `project_updated` after; saves log nothing.

## Backfill

`backfill_published_versions` (once, `applied_migrations` name
`434-published-versions`): every `completed` / `wip` / `archived` project with
no live version gets v1 from its current row (`published_at` = its
`updated_at`), and `wip` becomes `completed`. Drafts are left alone.

Not versioned: journal entries, makes, and the Instruction Builder's steps
(they stay project-level).

## Layers graphic renders

Saving or updating a Layers graphic (`*.layers.png`) uploads a new render with
`replaces=<old picture id>` (`POST /images`). Renders of one graphic share
`graphic_id`; the replaced one gets `superseded_at` and leaves the gallery
(`GET /images` and new snapshots leave it out unless the text still shows it).

On the next save of the text, a superseded render older than an hour (time to
undo) that the working copy doesn't show is deleted for good, or soft-removed
(`removed_at`) when a version's page shows it. Those kept renders are the
graphic's history: `GET /images/history` (author only) lists them per current
render, with the versions that show each, for the editor's History button.

"A version's page shows it" means its `content_md` or cover links to it, or,
for a version with no `::: gallery` block, it is in the version's pictures
(the page lists those under the text).

`POST /images/tidy-graphics` (author only, `?dry_run=true` to count; body
`{"text": …}` with the editor's unsaved text) clears renders made before they
were linked: graphics the text doesn't show, deleted unless a version shows
them (then kept for it, hidden). The "pictures aren't on your page" dialog
offers it as "Tidy them away". Code: `projects_api/graphics.py`.
