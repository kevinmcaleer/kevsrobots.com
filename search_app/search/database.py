import sqlite3

SEARCH_DB = 'search.db'

def initialize_database():
    conn = get_db_connection()
    conn.execute('''
        CREATE VIRTUAL TABLE IF NOT EXISTS documents_fts 
        USING fts5(title, content, url, cover_image, page_title);
    ''')
    conn.commit()
    conn.close()


def get_db_connection():
    conn = sqlite3.connect(SEARCH_DB)
    conn.row_factory = sqlite3.Row
    return conn

def insert_document(title, content, url, cover_image, page_title):
    conn = get_db_connection()
    conn.execute('''
        INSERT INTO documents_fts (title, content, url, cover_image, page_title) 
        VALUES (?, ?, ?, ?, ?)
    ''', (title, content, url, cover_image, page_title))
    conn.commit()
    conn.close()

def query_documents(query):
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # Modify the query to group by URL and return distinct results
    cursor.execute('''
        SELECT DISTINCT url, cover_image, page_title FROM documents_fts 
        WHERE documents_fts MATCH ? 
        GROUP BY url
        ORDER BY rank 
        LIMIT 10
    ''', (query,))
    
    results = [{'url': row['url'], 'cover_image': row['cover_image'], 'page_title': row['page_title']} for row in cursor.fetchall()]
    conn.close()
    return results



if __name__ == '__main__':
    initialize_database()
