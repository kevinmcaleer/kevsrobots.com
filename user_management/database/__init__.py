import sqlite3

connection = sqlite3.connect('users.db')

with connection:
    connection.execute("""
        CREATE TABLE IF NOT EXISTS users (
            id INTEGER PRIMARY KEY,
            username TEXT UNIQUE NOT NULL,
            email TEXT UNIQUE NOT NULL,
            profile_picture BLOB
        );
    """)
