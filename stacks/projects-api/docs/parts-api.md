# Parts catalog API

Phase 1 (issue #121) endpoints. Mounted under `/api/parts`. All mutation
endpoints (POST/PUT/restore) require a logged-in user **and** an account
that is at least 14 days old. Read endpoints are public.

Auth: send the Chatter JWT either via the `access_token` cookie
(domain `.kevsrobots.com`, value `Bearer <jwt>`) or an
`Authorization: Bearer <jwt>` header.

The 14-day gate looks the username up against Chatter's `/api/me`
endpoint and caches the result in-process for 1 hour. If Chatter is
unreachable or omits `account_created_at`, the gate fails closed with
HTTP 403 `"Account age cannot be verified"`.

---

## GET `/api/parts`

Substring search across `name`, `sku`, `mpn`, and any registered alias.

### Query params

| name  | type   | default | description                                        |
| ----- | ------ | ------- | -------------------------------------------------- |
| `q`   | string | _none_  | search term. Empty / omitted returns popular parts |
| `limit` | int  | `10`    | 1 – 50                                             |

### Response — `200 OK`

```json
[
  {
    "id": 12,
    "slug": "sg90-micro-servo",
    "name": "SG90 Micro Servo",
    "sku": "SG90",
    "status": "draft",
    "usage_count": 4,
    "primary_supplier_url": "https://thepihut.com/products/sg90-micro-servo"
  }
]
```

`primary_supplier_url` is the first registered supplier for the part, or
`null` if none.

---

## POST `/api/parts`

Create a new draft part. Auto-slugifies the name (with `-2`, `-3` etc.
suffixes on collision) and writes an initial revision with
`change_summary = "Initial draft"`.

### Request body

```json
{
  "name": "SG90 Micro Servo",
  "sku": "SG90",
  "mpn": null,
  "description_md": null,
  "image_url": null,
  "supplier_url": "https://thepihut.com/products/sg90-micro-servo",
  "supplier_name": "ThePiHut",
  "tags": ["servo", "rc"]
}
```

Only `name` is required. `supplier_url` is optional; if provided, a single
`PartSupplier` row is created with `supplier_name` (optional).

### Response — `201 Created`

Returns a [PartDetail](#partdetail-shape). See `GET /api/parts/{slug}`.

### Errors

* `401 Unauthorized` — no / invalid JWT.
* `403 Forbidden` — account younger than 14 days, or account age can't be
  verified (Chatter down / no `account_created_at` field).
* `422 Unprocessable Entity` — validation error (name too short, etc.).

---

## GET `/api/parts/{slug}`

Returns the full part incl. suppliers and the 10 most recent revisions.

### Response — `200 OK` — PartDetail shape

```json
{
  "id": 12,
  "slug": "sg90-micro-servo",
  "name": "SG90 Micro Servo",
  "sku": "SG90",
  "mpn": null,
  "description_md": "Plastic-geared hobby servo …",
  "image_url": "https://…/sg90.jpg",
  "tags": ["servo", "rc"],
  "status": "draft",
  "created_by": "kev",
  "created_at": "2026-05-15T11:22:33",
  "updated_at": "2026-05-15T11:22:33",
  "current_revision_id": 87,
  "usage_count": 4,
  "suppliers": [
    {
      "id": 31,
      "name": "ThePiHut",
      "url": "https://thepihut.com/products/sg90-micro-servo",
      "last_checked_at": null,
      "last_status": null
    }
  ],
  "recent_revisions": [
    {
      "id": 87,
      "author": "kev",
      "created_at": "2026-05-15T11:22:33",
      "change_summary": "Fix supplier URL"
    }
  ]
}
```

Timestamps are naive UTC ISO-8601 (no timezone suffix).

### Errors

* `404 Not Found` — slug doesn't exist.

---

## PUT `/api/parts/{slug}`

Write a new revision. The server diffs the request against the current
part state; if **no field actually changes**, returns `400 Bad Request`
with detail `"No fields changed"`. Otherwise:

* Writes a new `PartRevision` snapshot.
* Updates `parts.current_revision_id` and the part's editable fields.
* If `suppliers` is provided, replaces the supplier list en-bloc.

### Request body

```json
{
  "name": "SG90 Micro Servo (9g)",
  "sku": "SG90",
  "mpn": null,
  "description_md": "Updated description …",
  "image_url": null,
  "tags": ["servo", "rc", "9g"],
  "suppliers": [
    { "name": "ThePiHut", "url": "https://thepihut.com/products/sg90-micro-servo" },
    { "name": "AliExpress", "url": "https://aliexpress.com/item/…" }
  ],
  "change_summary": "Add second supplier and update spec"
}
```

All editable fields are optional **except** `change_summary` (1 – 200
chars). Omitting a field means "leave it alone". Omitting `suppliers`
means "leave supplier list alone"; passing `[]` means "remove all
suppliers".

### Response — `200 OK`

Returns the updated [PartDetail](#partdetail-shape).

### Errors

* `400 Bad Request` — diff is empty (no fields changed).
* `401 / 403` — see POST.
* `404 Not Found` — slug doesn't exist.

---

## GET `/api/parts/{slug}/revisions`

Paginated revision list.

### Query params

| name     | type | default | description |
| -------- | ---- | ------- | ----------- |
| `limit`  | int  | `20`    | 1 – 100     |
| `offset` | int  | `0`     | ≥ 0         |

### Response — `200 OK`

```json
[
  {
    "id": 87,
    "author": "kev",
    "created_at": "2026-05-15T11:22:33",
    "change_summary": "Fix supplier URL"
  }
]
```

---

## GET `/api/parts/{slug}/revisions/{rev_id}`

Returns a single revision snapshot. Useful for the "diff vs current"
view on the history page.

### Response — `200 OK`

```json
{
  "id": 87,
  "author": "kev",
  "created_at": "2026-05-15T11:22:33",
  "change_summary": "Fix supplier URL",
  "name": "SG90 Micro Servo",
  "sku": "SG90",
  "mpn": null,
  "description_md": "Plastic-geared hobby servo …",
  "image_url": null,
  "tags": ["servo", "rc"],
  "suppliers": [
    { "name": "ThePiHut", "url": "https://thepihut.com/products/sg90-micro-servo" }
  ]
}
```

---

## POST `/api/parts/{slug}/revisions/{rev_id}/restore`

Writes a **new** revision whose snapshot equals `rev_id`. History stays
linear (the old revision is not deleted). The new revision's
`change_summary` is `"Restored from revision #<rev_id>"`.

### Response — `200 OK`

Returns the resulting [PartDetail](#partdetail-shape) (now pointing at
the newly-written revision).

### Errors

* `401 / 403` — same as POST.
* `404 Not Found` — slug or rev_id doesn't exist.

---

## Snakie parts.yml (issue #266)

A part carries the fields of Snakie's `parts.yml`. `manufacturer`,
`package` (`THT`/`SMD`), `voltage`, `pin_spacing` (mm), `dimensions_mm`
(`{width, height}`) and `datasheet_url` are editable through
`PUT /api/parts/{slug}` (present → set, `""`/`null` clears). The rest of the
Snakie `PartDefinition` (headers of pins, geometry, electrical model) lives
in `definition`. Every revision snapshots all of them.

- `GET /api/parts/{slug}/parts.yml` serialises the part in Snakie's format.
  Catalog columns win over the stored definition; `image` and `mesh` are
  absolute URLs.
- `PUT /api/parts/{slug}/definition` takes the JSON the Snakie part editor
  saves and writes a revision. Its manufacturer, package, voltage, pin
  spacing and dimensions update the part's own; name and description don't.
- `GET /api/parts/{slug}/mesh` serves the 3D mesh when the part has one.
- Photos carry `source` and `licence` (`permission-granted:adafruit`,
  `permission-granted:pimoroni`, `permission-granted:digikey`, `cc-by`,
  `own`). A source with no licence sets `needs_review`, and the photo is
  listed at `GET /api/admin/parts/photos/needs-review` for an admin to set
  one with `PUT /api/parts/{slug}/photos/{id}`.

### Seeding from the Snakie standard library

```bash
docker exec -it <projects-api container> \
  python -m projects_api.import_snakie_parts /path/to/Snakie/examples/parts/snakie-standard
```

The command matches each part by slug, then name, then part number. Parts
that match nothing are created as drafts. An existing part gets the Snakie
fields refreshed, while its name, description, part number, category and
tags are only filled in where empty. Changes are recorded as revisions by
`snakie-import`. A second run changes nothing. The library's images have no
recorded licence, so they arrive flagged for review.

## Vendors, price history and availability (issues #202, #267)

- A part's suppliers carry a `kind`: `shop` (somewhere to buy it) or a
  technical link (`datasheet`, `driver`, `support`, `docs`). `PUT
  /api/parts/{slug}` matches suppliers to existing rows **by URL**, so a
  supplier keeps its id (and the BOM rows and price history that point at
  it) across edits and restores.
- Every new or changed shop price writes a `part_price_snapshots` row: on
  an edit or restore, and in the daily supplier job (which also picks up
  any price that changed without one). `GET /api/parts/{slug}/price-history`
  returns each shop's points, oldest first.
- `POST /api/parts/{slug}/suppliers/{id}/unavailable` (`{"note": …}`), by
  any signed-in user, reports the part unavailable at that shop. `DELETE`
  on the same path clears it, for the reporter, the part's creator or an
  admin. Supplier responses carry `unavailable_*` and the `last_price*`
  last recorded.
- BOM rows: a chosen supplier's price is used while that supplier is
  available. Otherwise, and for linked rows with no price of their own,
  the row uses the cheapest available shop (`price_source: "cheapest"`).
  `suppliers_unavailable` is true when every shop is broken or unavailable.

## GET `/api/parts/_resolve?names=…` (issue #263)

Looks up each `names` value (repeat the parameter, up to 100) as an exact,
case-insensitive match on a part's name, an alias, its SKU or its part
number. Returns one `{name, slug, part_name, cover_url}` per distinct name,
with `slug` null when nothing matches. The project editor's "Link all"
uses it to link Bill of Materials rows that already name a catalog part.

## BOM integration

`BOMItemCreate` / `BOMItemResponse` now include an optional `part_id`
(nullable int). When set on a BOM row, `parts.usage_count` is kept in
sync as `COUNT(DISTINCT project_id) FROM project_bom_items WHERE
part_id = ?`. The free-form fields (`name`, `quantity`, `unit`,
`unit_cost`, `supplier_url`) still work and remain authoritative — the
frontend should populate them from the chosen part for display, but the
catalog row is the source of truth for future revisions.

### PUT `/api/projects/{id}/bom/sync` (issue #264)

The project's Bill of Materials lives in its Markdown as a table. On each
save the editor parses it (`web/assets/js/bom-sync.js`) and sends
`{rows: [{name, quantity, part_slug, unit_cost?, currency_code?}]}`.
The stored rows then follow the table in order: each reuses a stored row
for the same part, else the same name, so the pinned revision, chosen
supplier and unit survive; rows the table no longer has are deleted and
usage counts follow. `unit_cost` is left out when the table has no Price
column, which keeps the stored price. Owner only; returns the list as
`GET /bom` does. With no table in the text the editor doesn't call it.

### Example BOM body

```json
{
  "name": "SG90 Micro Servo",
  "quantity": 4,
  "unit": "qty",
  "unit_cost": 2.50,
  "supplier_url": "https://thepihut.com/products/sg90-micro-servo",
  "sort_order": 0,
  "part_id": 12
}
```

If `part_id` points at a non-existent catalog row, the server silently
sets it to `null` rather than rejecting the BOM mutation.
