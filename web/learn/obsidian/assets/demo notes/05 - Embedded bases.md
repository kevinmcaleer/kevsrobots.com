You can embed views by adding the base name in a 
`![[basename.base]]` block.

You can even specify which view to use by adding a hash and the view name:

`![[basename.base#new]]

To embed a new view use the code block below:

````markdown
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
````

```base 
views:
  - type: table
    name: table
    order:
      - file.name
      - file.mtime
    sort:
      - property: file.name
        direction: DESC
      - property: file.mtime
        direction: ASC

```

