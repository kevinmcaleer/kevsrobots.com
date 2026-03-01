import sqlite3
import re
from datetime import datetime, timezone

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
        date = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S+00:00')
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


def get_facet_counts(query):
    """Return {page_type: count} for all matching documents, ignoring any type filter."""
    sanitized = sanitize_fts5_query(query)
    if sanitized is None:
        return {}

    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute(
            '''SELECT page_type, COUNT(*) as cnt
               FROM documents_fts
               WHERE documents_fts MATCH ?
               GROUP BY page_type''',
            (sanitized,),
        )
        facets = {row['page_type']: row['cnt'] for row in cursor.fetchall()}
        conn.close()
        return facets
    except Exception as e:
        print(f"Error getting facet counts: {e}")
        return {}


def _build_type_filter(page_types):
    """Build a SQL clause and params for filtering by page_type list."""
    if not page_types:
        return '', []
    placeholders = ','.join('?' for _ in page_types)
    return f' AND page_type IN ({placeholders})', list(page_types)


def total_results(query, page_types=None):
    sanitized = sanitize_fts5_query(query)
    if sanitized is None:
        return 0

    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        type_clause, type_params = _build_type_filter(page_types)

        count_query = f'''
            SELECT COUNT(*) FROM documents_fts
            WHERE documents_fts MATCH ?{type_clause}
        '''

        cursor.execute(count_query, [sanitized] + type_params)
        total_count = cursor.fetchone()[0]

        conn.close()
        return total_count
    except Exception as e:
        print(f"Error counting results: {e}")
        return 0


def query_documents(query, offset: int = None, limit: int = None, sort: str = "relevance", page_types=None):
    """Query the documents table for the given query string.

    Args:
        query: Search query string.
        offset: Number of results to skip.
        limit: Maximum number of results to return.
        sort: "relevance" for BM25 ranking, "recent" for date descending.
        page_types: Optional list of page_type values to filter by.
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

        type_clause, type_params = _build_type_filter(page_types)

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
            WHERE documents_fts MATCH ?{type_clause}
            {order_clause}
            LIMIT ? OFFSET ?
        '''

        cursor.execute(sql_query, [sanitized] + type_params + [int(limit), int(offset)])

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
