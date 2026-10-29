---
layout: lesson
title: Importing and Exporting Data
type: page
description: Learn how to import data from various file formats into Pandas Data Frames and export Data Frames back into these formats for efficient data handling.
cover: /learn/pandas_and_numpy/assets/pandas05.png
---

![Cover photo]({{page.cover}}){:class="cover"}

## Overview

This lesson is centered on importing and exporting data using Pandas. We'll cover how to read data from various sources like CSV, Excel, and YAML files into Pandas Data Frames, and similarly, how to export Data Frames into these file formats. Mastering these skills is essential for efficient data handling and analysis.

---

## Importing Data

### Reading from CSV

To read data from a CSV file into a Data Frame, use `pd.read_csv()`:

```python
import pandas as pd

# Reading from a CSV file
df = pd.read_csv('path/to/your/file.csv')
print(df)
```

---

### Reading from Excel

Reading from an Excel file is just as straightforward, though you'll need the `openpyxl` library installed (`pip install openpyxl`):

```python
# Reading from an Excel file
df = pd.read_excel('path/to/your/file.xlsx')
print(df)
```

---

### Reading from YAML

To read YAML data, you'll need an additional library, `PyYAML` (`pip install pyyaml`). The YAML file should hold a list of records, one per row:

```python
import yaml
import pandas as pd

# Reading from a YAML file
with open('path/to/your/file.yaml', 'r') as file:
    yaml_data = yaml.safe_load(file)

df = pd.DataFrame(yaml_data)
print(df)
```

---

## Exporting Data

### Writing to CSV

You can export a Data Frame to a CSV file using `df.to_csv()`:

```python
# Writing to a CSV file
df.to_csv('path/to/your/newfile.csv', index=False)
```

`index=False` stops Pandas writing the row numbers as an extra, unnamed column. Leave it out if your index holds useful labels.

---

### Writing to Excel

Similarly, to export to an Excel file:

```python
# Writing to an Excel file
df.to_excel('path/to/your/newfile.xlsx', index=False)
```

---

{% quiz %}
? Why pass `index=False` to `to_csv()`?
- [ ] To make the file smaller by removing headers
- [x] To stop the row numbers being saved as an extra column
- [ ] To sort the rows
> Without it, reading the file back in gives you an unwanted "Unnamed: 0" column.
{% endquiz %}

---

## Summary

In this lesson, we've covered the fundamentals of importing and exporting data using Pandas. You've learned how to work with different file formats, which is a key part of the data analysis workflow.

---
