from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
import sqlite3
from typing import List, Optional

app = FastAPI()

DATABASE_FILE = 'courses.db'

# Set up CORS to allow all origins (Not recommended for production)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

def get_db_connection():
    conn = sqlite3.connect(DATABASE_FILE)
    print(f"Opened database successfully {DATABASE_FILE}")
    conn.row_factory = sqlite3.Row  # This allows us to return dictionary-like rows
    return conn

# Route to fetch all courses with optional tag filtering
@app.get("/courses/")
def get_courses(tags: Optional[List[str]] = Query(None)):
    conn = get_db_connection()
    c = conn.cursor()

    if tags:
        placeholders = ','.join('?' for _ in tags)
        query = f'''
            SELECT courses.course_id, courses.course_name, courses.course_url, 
                   tags.tag_name
            FROM courses
            JOIN tag_link ON courses.course_id = tag_link.course_id
            JOIN tags ON tags.tag_id = tag_link.tag_id
            WHERE courses.course_id IN (
                SELECT courses.course_id 
                FROM courses 
                JOIN tag_link ON courses.course_id = tag_link.course_id
                JOIN tags ON tags.tag_id = tag_link.tag_id
                WHERE tags.tag_name IN ({placeholders})
                GROUP BY courses.course_id
                HAVING COUNT(DISTINCT tags.tag_name) = ?
            )
            GROUP BY courses.course_id, tags.tag_name
        '''
        c.execute(query, tags + [len(tags)])
    else:
        query = '''
            SELECT courses.course_id, courses.course_name, courses.course_url, 
                   tags.tag_name
            FROM courses
            LEFT JOIN tag_link ON courses.course_id = tag_link.course_id
            LEFT JOIN tags ON tags.tag_id = tag_link.tag_id
            GROUP BY courses.course_id, tags.tag_name
        '''
        c.execute(query)
    
    rows = c.fetchall()
    conn.close()

    # Group courses by course_id and collect tags into a list
    courses_dict = {}
    for row in rows:
        course_id = row["course_id"]
        if course_id not in courses_dict:
            courses_dict[course_id] = {
                "course_id": course_id,
                "course_name": row["course_name"],
                "course_url": row["course_url"],
                "course_tags": []
            }
        # Only append the tag if it's not None
        if row["tag_name"]:
            courses_dict[course_id]["course_tags"].append(row["tag_name"])

    # Convert the dictionary to a list of courses
    courses = list(courses_dict.values())
    
    return courses

# Route to fetch all available tags
@app.get("/tags/")
def get_tags():
    conn = get_db_connection()
    c = conn.cursor()
    c.execute("SELECT * FROM tags ORDER BY tag_name")
    tags = c.fetchall()
    conn.close()

    return [dict(tag) for tag in tags]
