import sqlite3
import yaml

DATABASE_FILE = 'course_app/courses.db'

def delete_database():
    conn = sqlite3.connect(DATABASE_FILE)
    c = conn.cursor()
    c.execute('''DROP TABLE IF EXISTS courses''')
    c.execute('''DROP TABLE IF EXISTS tags''')
    c.execute('''DROP TABLE IF EXISTS tag_link''')
    conn.commit()
    conn.close()

def build_database():
    conn = sqlite3.connect(DATABASE_FILE)
    c = conn.cursor()
    c.execute('''CREATE TABLE courses
             (course_id INTEGER PRIMARY KEY, 
              course_name TEXT,
              course_url TEXT, 
              course_cover TEXT,
              course_author TEXT,
              course_duration TEXT,
              course_date TEXT
              )''')
    c.execute('''CREATE TABLE tags
                (tag_id INTEGER PRIMARY KEY, 
                tag_name TEXT UNIQUE
              )''')
    c.execute('''CREATE TABLE tag_link
             (tag_link_id INTEGER PRIMARY KEY, 
              course_id INTEGER,
              tag_id INTEGER,
              FOREIGN KEY (course_id) REFERENCES courses(course_id),
              FOREIGN KEY (tag_id) REFERENCES tags(tag_id)
              )''')
    conn.commit()
    conn.close()

def read_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def insert_data(data):
    conn = sqlite3.connect(DATABASE_FILE)
    c = conn.cursor()
    courses = data
    for course in courses:
        # Insert the course into the courses table
        c.execute("INSERT INTO courses (course_name, course_url, course_cover, course_author, course_duration, course_date) VALUES (?, ?, ?, ?, ?, ?)", 
                  (course['name'], course['link'], course['cover'], course['author'], course['duration'], course['date_published']))
        
        # Get the course_id of the last inserted course
        course_id = c.lastrowid

        # Insert each tag into the tags table and link it to the course
        for tag in course['groups']:
            # Insert the tag if it doesn't already exist
            c.execute("INSERT OR IGNORE INTO tags (tag_name) VALUES (?)", (tag,))
            
            # Get the tag_id of the inserted or existing tag
            c.execute("SELECT tag_id FROM tags WHERE tag_name = ?", (tag,))
            tag_id = c.fetchone()[0]
            
            # Insert into the tag_link table
            c.execute("INSERT INTO tag_link (course_id, tag_id) VALUES (?, ?)", 
                      (course_id, tag_id))
    
    conn.commit()
    conn.close()

delete_database()
build_database()
data = read_yaml('web/_data/courses.yml')
insert_data(data)
