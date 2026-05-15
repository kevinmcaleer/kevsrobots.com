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

## BOM integration

`BOMItemCreate` / `BOMItemResponse` now include an optional `part_id`
(nullable int). When set on a BOM row, `parts.usage_count` is kept in
sync as `COUNT(DISTINCT project_id) FROM project_bom_items WHERE
part_id = ?`. The free-form fields (`name`, `quantity`, `unit`,
`unit_cost`, `supplier_url`) still work and remain authoritative — the
frontend should populate them from the chosen part for display, but the
catalog row is the source of truth for future revisions.

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
