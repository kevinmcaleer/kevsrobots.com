import sqlite3
import re

SEARCH_DB = 'search.db'

def initialize_database():
    conn = get_db_connection()
    conn.execute('''
        CREATE VIRTUAL TABLE IF NOT EXISTS documents_fts
        USING fts5(title, content, url, cover_image, page_title, description, date, author, page_type);
    ''')
    conn.commit()
    conn.close()


def get_db_connection():
    conn = sqlite3.connect(SEARCH_DB)
    conn.row_factory = sqlite3.Row
    return conn

def insert_document(title, content, url, cover_image, page_title, description, date, author, page_type="page"):
    if author is None:
        author = "Kevin McAleer"
    if date is None:
        date = "2023-12-30"
    if description is None:
        description = "No description"
    if page_type is None:
        page_type = "page"
    conn = get_db_connection()
    conn.execute('''
        INSERT INTO documents_fts (title, content, url, cover_image, page_title, description, date, author, page_type)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
    ''', (title, content, url, cover_image, page_title, description, date, author, page_type))
    conn.commit()
    conn.close()


def sanitize_fts5_query(query):
    """Sanitize a user query for safe use with FTS5 MATCH.

    Strips FTS5 special operators, balances quotes, and returns a safe
    query string. Returns None if the query is empty after sanitization.
    """
    if not query or not query.strip():
        return None

    # Remove FTS5 special characters/operators
    # Keep alphanumeric, spaces, hyphens, and basic punctuation
    sanitized = re.sub(r'[*^{}()\[\]:!]', ' ', query)

    # Balance double quotes — if odd number, strip them all
    if sanitized.count('"') % 2 != 0:
        sanitized = sanitized.replace('"', '')

    # Collapse whitespace
    sanitized = ' '.join(sanitized.split())

    if not sanitized.strip():
        return None

    return sanitized.strip()


def total_results(query):
    sanitized = sanitize_fts5_query(query)
    if sanitized is None:
        return 0

    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        count_query = '''
            SELECT COUNT(*) FROM documents_fts
            WHERE documents_fts MATCH ?
        '''

        cursor.execute(count_query, (sanitized,))
        total_count = cursor.fetchone()[0]

        conn.close()
        return total_count
    except Exception as e:
        print(f"Error counting results: {e}")
        return 0


def query_documents(query, offset: int = None, limit: int = None, sort: str = "relevance"):
    """Query the documents table for the given query string.

    Args:
        query: Search query string.
        offset: Number of results to skip.
        limit: Maximum number of results to return.
        sort: "relevance" for BM25 ranking, "recent" for date descending.
    """
    if offset is None:
        offset = 0
    if limit is None:
        limit = 10

    sanitized = sanitize_fts5_query(query)
    if sanitized is None:
        return []

    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        # Column order: title, content, url, cover_image, page_title, description, date, author, page_type
        # Weights:      10      1       0    0            10          5            0     2       0
        if sort == "recent":
            order_clause = "ORDER BY date DESC"
        else:
            order_clause = "ORDER BY bm25(documents_fts, 10, 1, 0, 0, 10, 5, 0, 2, 0)"

        sql_query = f'''
            SELECT url, cover_image, page_title, description, date, author, page_type,
                   snippet(documents_fts, 1, '<mark>', '</mark>', '...', 32) AS snippet
            FROM documents_fts
            WHERE documents_fts MATCH ?
            {order_clause}
            LIMIT ? OFFSET ?
        '''

        cursor.execute(sql_query, (sanitized, int(limit), int(offset)))

        results = [{
            'url': row['url'],
            'cover_image': row['cover_image'],
            'page_title': row['page_title'],
            'description': row['description'],
            'date': row['date'],
            'author': row['author'],
            'page_type': row['page_type'],
            'snippet': row['snippet'],
        } for row in cursor.fetchall()]

        conn.close()
        return results
    except Exception as e:
        print(f"Error querying documents: {e}")
        return []

if __name__ == '__main__':
    initialize_database()
