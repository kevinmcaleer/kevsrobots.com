from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
import sqlite3
from typing import List, Optional

app = FastAPI()

# Set up CORS to allow all origins (Not recommended for production)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# The rest of your FastAPI application code...

# Function to connect to the SQLite database
def get_db_connection():
    conn = sqlite3.connect('/Users/Kev/Web/kevsrobots.com/courses.db')
    conn.row_factory = sqlite3.Row  # This allows us to return dictionary-like rows
    return conn

# Route to fetch all courses with optional tag filtering
@app.get("/courses/")
def get_courses(tags: Optional[List[str]] = Query(None)):
    conn = get_db_connection()
    c = conn.cursor()

    if tags:
        # Generate the SQL query to filter by tags and include all tags for each course
        placeholders = ','.join('?' for _ in tags)
        query = f'''
            SELECT courses.course_id, courses.course_name, courses.course_url, 
                   GROUP_CONCAT(tags.tag_name, ', ') AS course_tags
            FROM courses
            JOIN tag_link ON courses.course_id = tag_link.course_id
            JOIN tags ON tags.tag_id = tag_link.tag_id
            WHERE tags.tag_name IN ({placeholders})
            GROUP BY courses.course_id
            HAVING COUNT(DISTINCT tags.tag_name) = ?
        '''
        c.execute(query, tags + [len(tags)])
    else:
        # If no tags provided, return all courses with their associated tags
        query = '''
            SELECT courses.course_id, courses.course_name, courses.course_url, 
                   GROUP_CONCAT(tags.tag_name, ', ') AS course_tags
            FROM courses
            LEFT JOIN tag_link ON courses.course_id = tag_link.course_id
            LEFT JOIN tags ON tags.tag_id = tag_link.tag_id
            GROUP BY courses.course_id
        '''
        c.execute(query)
    
    courses = c.fetchall()
    conn.close()

    # Convert rows to a list of dictionaries
    return [dict(course) for course in courses]

# Route to fetch all available tags
@app.get("/tags/")
def get_tags():
    conn = get_db_connection()
    c = conn.cursor()
    c.execute("SELECT * FROM tags ORDER BY tag_name")
    tags = c.fetchall()
    conn.close()

    # Convert rows to a list of dictionaries
    return [dict(tag) for tag in tags]

