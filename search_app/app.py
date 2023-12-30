from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from search.database import insert_document, query_documents

app = FastAPI()

# Set up CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://0.0.0.0:4000"],  # List of allowed origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

@app.post("/documents/")
def create_document(title: str, content: str, url: str):
    insert_document(title, content, url)
    return {"message": "Document created successfully"}

@app.put("/documents/{id}")
def update_document_api(id: int, title: str, content: str, url: str):
    update_document(id, title, content, url)
    return {"message": "Document updated successfully"}

@app.get("/search/")
def search_documents(query: str):
    results = query_documents(query)
    return {"results": results}

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
