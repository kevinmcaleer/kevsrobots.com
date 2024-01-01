from pydantic import BaseModel, EmailStr
from typing import Optional

class UserCreate(BaseModel):
    username: str
    email: EmailStr
    profile_picture: Optional[bytes] = None

class UserDisplay(BaseModel):
    id: int
    username: str
    email: EmailStr
