from fastapi import FastAPI, Request
from datetime import datetime
import logging
from fastapi.middleware.cors import CORSMiddleware
from search.database import insert_document, query_documents

app = FastAPI()

# Setup logger
logging.basicConfig(filename='search-logs.log', level=logging.INFO)


# Set up CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://0.0.0.0:4000", "https://www.kevsrobots.com", "http://www.kevsrobots.com"],  # List of allowed origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

@app.post("/documents/")
def create_document(title: str, content: str, url: str):
    insert_document(title, content, url)
    return {"message": "Document created successfully"}

@app.get("/search/")
def search_documents(request:Request, query: str):
    client_ip = request.client.host
    current_time = datetime.now().isoformat()
    log_entry = f"{current_time} - IP: {client_ip} - Query: {query}"
    
    # Log the search query, IP, and timestamp
    logging.info(log_entry)


    results = query_documents(query)
    return {"results": results}
