from fastapi import FastAPI, Request, requests
from datetime import datetime
import logging
from fastapi.middleware.cors import CORSMiddleware
from search.database import insert_document, query_documents, total_results
from typing import Optional
import time

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
async def search_documents(request:Request, query: str, page: Optional[int] = 1, page_size: Optional[int] = 10):
    start_time = time.time()
    client_ip = request.client.host
    current_time = datetime.now().isoformat()
    log_entry = f"{current_time} - IP: {client_ip} - Query: {query}"
    
    # Log the search query, IP, and timestamp
    logging.info(log_entry)

    results = query_documents(query,offset = (page - 1) * page_size, limit = page_size)
    execution_time = time.time() - start_time

     # Assuming `total_results` is a function that returns the total count of results
    total_count = total_results(query)
    print('results found: ', total_count)

    return {
        "results": results,
        "total_count": total_count,
        "total_pages": total_count // page_size + 1,
        "page": page,
        "page_size": page_size,
        "execution_time": round(execution_time, 3)
    }


# @app.get("/search/")
async def search(request:Request, query: str, page: Optional[int] = 1, page_size: Optional[int] = 10):
    
    # For example, fetching results from a database using offset and limit
    offset = (page - 1) * page_size
    limit = page_size
    # Assuming `search_query` is a function that performs the actual search
    results = search_documents(request, query, offset=offset, limit=limit)

   


    return {
        "results": results,
        "total_count": total_count,
        "page": page,
        "page_size": page_size,
        "execution_time": execution_time
    }