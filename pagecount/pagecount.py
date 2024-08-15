from fastapi import FastAPI, Request
from sqlalchemy import create_engine, Column, String, DateTime, Integer
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime

# Set up PostgreSQL database connection
DATABASE_URL = "postgresql://kev:mysecretpassword@192.168.2.1:5433/pagecount"
engine = create_engine(DATABASE_URL)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

class PageVisit(Base):
    __tablename__ = "page_visits"
    id = Column(Integer, primary_key=True, index=True)
    page_url = Column(String, index=True)
    ip_address = Column(String, index=True)
    timestamp = Column(DateTime, default=datetime.utcnow)

Base.metadata.create_all(bind=engine)

app = FastAPI()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

@app.post("/log_visit/")
async def log_visit(request: Request, db: SessionLocal = next(get_db())):
    data = await request.json()
    page_url = data.get("page_url")
    client_ip = request.headers.get("X-Forwarded-For", request.client.host)
    new_visit = PageVisit(page_url=page_url, ip_address=client_ip)
    db.add(new_visit)
    db.commit()
    return {"message": "Logged successfully"}
