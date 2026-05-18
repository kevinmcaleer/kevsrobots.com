# Feedback API spec (issue #138)

This document is the contract between the KevsRobots.com Jekyll frontend and
the backend at `projects.kevsrobots.com`. The frontend (floating widget +
admin inbox at `/admin/feedback`) is already shipped and will call these
endpoints as soon as the backend exposes them.

The backend lives in a separate repository. Until the listed endpoints are
live the frontend degrades gracefully: the widget silently never opens for
guests, the admin inbox shows a "service is being set up" banner.

---

## 1. Auth model

Reuse the existing Projects Hub auth — there is nothing new here.

- **Production:** HTTP-only `access_token` cookie, sent via
  `credentials: 'include'`. The session is decoded server-side; routes
  derive `user_id` / `username` / `email` / `is_admin` from it.
- **Local dev:** a JWT in `localStorage` keyed `dev_jwt_token`, forwarded
  as `Authorization: Bearer <token>`. The frontend wrapper
  `ProjectAuth.apiFetch` already attaches it on every call — backend should
  accept both transports identically.

**Roles**

| Role  | Required for                                       |
| ----- | -------------------------------------------------- |
| guest | nothing — feedback endpoints are auth-only         |
| user  | `POST /api/feedback`                               |
| admin | every `/api/admin/feedback*` route                 |

Guests calling user-only routes must receive `401`. Non-admins calling
admin routes must receive `403`.

---

## 2. Endpoints

### 2.1 `GET /api/auth/me`

Used by the widget to decide whether to show the pill. **Optional but
strongly recommended** — without it the widget falls back to probing
`GET /api/projects/my/list` which is brittle.

| Auth     | Required (any logged-in user) |
| -------- | ----------------------------- |
| Method   | `GET`                         |

**Response (200, JSON)**

```json
{
  "id": 42,
  "username": "kev",
  "email": "kev@example.com",
  "is_admin": true
}
```

`email` may be `null` if the account has none. `is_admin` is informational
— the frontend never trusts it for gating (the admin inbox proves admin
status by attempting `GET /api/admin/feedback`).

**Errors**

- `401` — not authenticated.

### 2.2 `POST /api/feedback`

Create a new feedback record. Called by the widget on Send.

| Auth   | Required (any logged-in user) |
| ------ | ----------------------------- |
| Method | `POST`                        |

**Request — JSON variant** (`Content-Type: application/json`, no
screenshot):

```json
{
  "sentiment": "love | like | issue | idea",
  "message": "string, 10–2000 chars",
  "email": "optional reply-to, validated as email",
  "page_url": "https://kevsrobots.com/projects/...",
  "referrer": "string or empty",
  "user_agent": "string",
  "viewport": "1280x720"
}
```

**Request — multipart variant** (`Content-Type: multipart/form-data`,
with screenshot):

Same fields as form parts, plus a file part named `screenshot`. The file
part MUST:

- accept `image/png`, `image/jpeg`, `image/webp`, `image/gif`
- reject anything else with `415`
- enforce a max upload size of **4 MB** — reject with `413` if larger
- be stored either on the local filesystem (e.g.
  `/var/lib/projects-api/feedback/<uuid>.<ext>`) or in object storage. A
  signed/public URL is then persisted in `feedback.screenshot_url`.

**Response (201, JSON)**

```json
{
  "id": 123,
  "status": "unread",
  "created_at": "2026-05-18T12:34:56Z"
}
```

**Errors**

- `400` — missing/invalid field (`sentiment` not in enum, message too
  short or too long, email present but invalid).
- `401` — not authenticated.
- `413` — screenshot too large.
- `415` — screenshot MIME not allowed.
- `429` — rate limited (suggested: 1 feedback per 10s per user, 30 per
  hour per user; values for the backend team to pick).

### 2.3 `GET /api/admin/feedback`

List feedback for the admin inbox.

| Auth   | Admin |
| ------ | ----- |
| Method | `GET` |

**Query parameters** (all optional):

| Name        | Type   | Notes                                                      |
| ----------- | ------ | ---------------------------------------------------------- |
| `status`    | enum   | `unread` / `read` / `archived`. Omitted = all (incl. archived). |
| `sentiment` | enum   | `love` / `like` / `issue` / `idea`. Repeatable / CSV.      |
| `q`         | string | Case-insensitive substring match on message or page_url.   |
| `limit`     | int    | Default 50, max 200.                                       |
| `offset`    | int    | Default 0.                                                 |

**Response (200, JSON)** — flat array (preferred) OR `{items, total}`:

```json
[
  {
    "id": 123,
    "sentiment": "issue",
    "message": "Code block in step 3 is missing the import",
    "email": "you@example.com",
    "username": "kev",
    "user_id": 42,
    "status": "unread",
    "page_url": "https://kevsrobots.com/projects/view.html?id=18",
    "referrer": "https://kevsrobots.com/projects/",
    "user_agent": "Mozilla/5.0 ...",
    "viewport": "1280x720",
    "screenshot_url": "https://projects.kevsrobots.com/feedback/abc.png",
    "created_at": "2026-05-18T12:34:56Z",
    "updated_at": "2026-05-18T12:34:56Z"
  }
]
```

The frontend handles either shape, but the array form is the canonical
contract.

**Pagination semantics**

- `limit` + `offset` — simple offset pagination. Newest first
  (`ORDER BY created_at DESC`).
- Backend MAY add `?cursor=` later; the frontend will ignore it until
  asked.

**Errors**: `401`, `403`.

### 2.4 `GET /api/admin/feedback/:id`

Fetch a single record (used when opening the detail pane). Same row
shape as the list endpoint.

**Errors**: `401`, `403`, `404`.

### 2.5 `PATCH /api/admin/feedback/:id`

Mutate a record. Currently only `status` is supported.

**Body**

```json
{ "status": "unread | read | archived" }
```

**Response (200)** — the updated row, same shape as 2.3.

**Errors**: `400` (invalid status), `401`, `403`, `404`.

### 2.6 `DELETE /api/admin/feedback/:id`

Hard delete. The frontend confirms with the operator before calling.

**Response (204)** — empty.

**Errors**: `401`, `403`, `404`.

### 2.7 `GET /api/admin/feedback/counts`

Used by the admin inbox header strip.

**Response (200)**

```json
{
  "total": 42,
  "unread": 7,
  "read": 30,
  "archived": 5,
  "by_sentiment": { "love": 10, "like": 12, "issue": 15, "idea": 5 }
}
```

`by_sentiment` is optional (the current frontend doesn't render it yet
but will once #138-followup lands).

**Errors**: `401`, `403`.

---

## 3. Database schema (suggested)

```sql
CREATE TABLE feedback (
  id              INTEGER PRIMARY KEY AUTOINCREMENT,
  user_id         INTEGER NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  sentiment       TEXT    NOT NULL CHECK (sentiment IN ('love','like','issue','idea')),
  message         TEXT    NOT NULL CHECK (length(message) BETWEEN 10 AND 2000),
  email           TEXT,
  status          TEXT    NOT NULL DEFAULT 'unread'
                          CHECK (status IN ('unread','read','archived')),
  page_url        TEXT    NOT NULL,
  referrer        TEXT,
  user_agent      TEXT,
  viewport        TEXT,
  screenshot_path TEXT,                       -- on-disk or object-store key
  screenshot_url  TEXT,                       -- public URL returned to admins
  created_at      TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at      TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_feedback_status_created ON feedback(status, created_at DESC);
CREATE INDEX idx_feedback_sentiment      ON feedback(sentiment);
CREATE INDEX idx_feedback_user           ON feedback(user_id, created_at DESC);
-- Optional FTS index on message + page_url for the ?q= search.
```

Indexes are sized for the admin inbox's two main access patterns:
"newest first within a status" and "histogram by sentiment".

---

## 4. Storage notes

- Path on disk: `/var/lib/projects-api/feedback/<yyyy>/<mm>/<uuid>.<ext>`
  (sharded to keep dir size manageable).
- Object store: any bucket the projects API already uses for user
  uploads is fine. Set `screenshot_url` to a long-lived public URL.
- The screenshot is never re-encoded — store the bytes as received,
  trusting the MIME sniff at upload time.
- Cleanup: a record's screenshot file MUST be removed when the row is
  hard-deleted via 2.6.

---

## 5. Minimal viable backend

The frontend won't error-toast on first paint as long as **two**
endpoints exist:

1. **`POST /api/feedback` returns `201`** — even if it does nothing else,
   the widget shows the success state instead of the red error message.
2. **`GET /api/admin/feedback/counts` returns `200`** with
   `{ total: 0, unread: 0 }` for admin callers — this lets the inbox
   page render cleanly with the "service is being set up" banner instead
   of breaking.

Once those two stubs are live, the remaining endpoints can land in any
order. The admin inbox uses 404 → banner as its signal that the rest of
the service isn't ready yet.

---

## 6. Open questions for backend team

- **Spam control.** Suggested: rate-limit + minimum message length
  (already enforced client-side at 10 chars). Re-validate server-side.
- **PII.** `user_agent` and `viewport` are diagnostic; treat them as
  log-grade data. `email` is user-volunteered; keep it on the row but
  never expose it on any public endpoint.
- **Notifications.** Out of scope for this issue, but a Slack/email
  hook on new `issue`-sentiment feedback would be a fast win.
