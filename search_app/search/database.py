import sqlite3

SEARCH_DB = 'search.db'

def initialize_database():
    conn = get_db_connection()
    conn.execute('''
        CREATE VIRTUAL TABLE IF NOT EXISTS documents_fts 
        USING fts5(title, content, url, cover_image, page_title, description, date, author);
    ''')
    conn.commit()
    conn.close()


def get_db_connection():
    conn = sqlite3.connect(SEARCH_DB)
    conn.row_factory = sqlite3.Row
    return conn

def insert_document(title, content, url, cover_image, page_title, description, date, author):
    if author == None:
        author = "Kevin McAleer"
    if date == None:
        date = "2023-30-12"
    if description == None:
        description = "No description"
    # print(f'title: {title}, url: {url}, cover_image: {cover_image}, page_title: {page_title}, description: {description}, date: {date}, author: {author}')
    conn = get_db_connection()
    conn.execute('''
        INSERT INTO documents_fts (title, content, url, cover_image, page_title, description, date, author) 
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    ''', (title, content, url, cover_image, page_title, description, date, author))
    conn.commit()
    conn.close()

def total_results(query):
    conn = get_db_connection()
    cursor = conn.cursor()

    # Count query for full-text search
    count_query = '''
        SELECT COUNT(*) FROM documents_fts
        WHERE documents_fts MATCH ?
    '''

    cursor.execute(count_query, (query,))
    total_count = cursor.fetchone()[0]

    conn.close()
    return total_count

def query_documents(query, offset:int=None, limit:int=None):
    """ Query the documents table for the given query string."""
    if offset == None: offset = 0
    if limit == None: limit = 10
    conn = get_db_connection()
    cursor = conn.cursor()
    
    print(f'query: {query}, offset: {offset}, limit: {limit}')

    sql_query = '''
        SELECT DISTINCT url, cover_image, page_title, description, date, author FROM documents_fts 
        WHERE documents_fts MATCH ? 
        GROUP BY url
        ORDER BY rank 
        LIMIT ? OFFSET ?
    '''

    # Modify the query to group by URL and return distinct results
    cursor.execute(sql_query, (str(query),int(limit), int(offset)))
    
    results = [{'url': row['url'], 'cover_image': row['cover_image'], 'page_title': row['page_title'], 'description':row['description'], 'date':row['date'], 'author':row['author']} for row in cursor.fetchall()]
    conn.close()
    return results

if __name__ == '__main__':
    initialize_database()
