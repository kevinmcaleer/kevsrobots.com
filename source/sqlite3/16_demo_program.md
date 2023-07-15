---
title: Demo program
description: This is an example program highlighting some of the simple concepts covered in this course
layout: lesson
---

```python
import sqlite3

# Create a database and establish a connection
connection = sqlite3.connect('mydatabase.db')

# Create a table
connection.execute('''
    CREATE TABLE IF NOT EXISTS books (
        id INTEGER PRIMARY KEY,
        title TEXT,
        author TEXT,
        year INTEGER
    )
''')

# Insert data into the table
books_data = [
    ("The Great Gatsby", "F. Scott Fitzgerald", 1925),
    ("To Kill a Mockingbird", "Harper Lee", 1960),
    ("Pride and Prejudice", "Jane Austen", 1813)
]

connection.executemany('INSERT INTO books (title, author, year) VALUES (?, ?, ?)', books_data)

# Query data from the table
result = connection.execute('SELECT * FROM books')
data = result.fetchall()

# Display the retrieved data
for row in data:
    print(f"Title: {row[1]}")
    print(f"Author: {row[2]}")
    print(f"Year: {row[3]}")
    print()

# Close the database connection
connection.close()
```

In this demo program, we create a database named "mydatabase.db" and establish a connection to it using the `sqlite3` module. We then create a table named "books" with columns for book title, author, and year. Next, we insert some sample data into the table using the `executemany()` method.

After inserting the data, we retrieve it from the table using a SELECT statement. The `fetchall()` method returns all rows as a list, which we iterate over and display the title, author, and year of each book.

Finally, we close the database connection to ensure proper cleanup.

You can run this program to see the database creation, data insertion, and retrieval in action. Make sure to have the `sqlite3` module installed and adjust the database file path as needed.

---
