from datetime import datetime
from pydantic import BaseModel, EmailStr, PositiveInt
from typing import Optional

class UserCreate(BaseModel):
    username: str
    email: EmailStr
    profile_picture: Optional[bytes] = None
class User(BaseModel):
    id: int  
    name: str = 'John Doe'  
    signup_ts: datetime | None  
    tastes: dict[str, PositiveInt]
    username: str
    email: EmailStr