---
published_date: 2025-08-24
---
```base
filters:
  and:
    - title.lower().contains("raspberry")
formulas:
  File: File.asLink(video_id)
properties:
  formula.File:
    displayName: Video_id
views:
  - type: table
    name: Table
    order:
      - title
      - formula.File
      - published_date
    sort:
      - property: title.asLink()
        direction: ASC
    columnSize:
      note.title: 431
      formula.File: 126

```
