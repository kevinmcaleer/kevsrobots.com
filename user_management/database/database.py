from fastapi import FastAPI, HTTPException, File, UploadFile
import sqlite3
from typing import List
import shutil
from . import UserCreate, UserDisplay

app = FastAPI()

@app.post("/users/", response_model=UserDisplay)
def create_user(user: UserCreate):
    with sqlite3.connect('users.db') as conn:
        cursor = conn.cursor()
        try:
            cursor.execute("INSERT INTO users (username, email, profile_picture) VALUES (?, ?, ?)", 
                           (user.username, user.email, user.profile_picture))
            conn.commit()
            return cursor.execute("SELECT id, username, email FROM users WHERE username = ?", (user.username,)).fetchone()
        except sqlite3
