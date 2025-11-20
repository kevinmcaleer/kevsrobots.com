"""
Search Logger Module for PostgreSQL

This module provides functionality to log search queries to a PostgreSQL database.
It tracks search queries, IP addresses, timestamps, and execution metrics.
"""

import os
from datetime import datetime
from typing import Optional
from urllib.parse import unquote
import psycopg2
from psycopg2.extras import RealDictCursor
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class SearchLogger:
    """Handles logging of search queries to PostgreSQL database."""

    def __init__(self):
        """Initialize database connection parameters from environment variables."""
        # URL-decode credentials if they are URL-encoded
        db_user = os.getenv('DB_USER', '')
        db_password = os.getenv('DB_PASSWORD', '')

        self.db_config = {
            'host': os.getenv('DB_HOST', '192.168.2.3'),
            'port': os.getenv('DB_PORT', '5433'),
            'database': os.getenv('DB_NAME', 'searchlogs'),
            'user': unquote(db_user) if db_user else None,
            'password': unquote(db_password) if db_password else None
        }
        self.initialize_table()

    def get_connection(self):
        """
        Create and return a database connection.

        Returns:
            psycopg2.connection: Database connection object

        Raises:
            psycopg2.Error: If connection fails
        """
        return psycopg2.connect(**self.db_config)

    def initialize_table(self):
        """
        Create the search_logs table if it doesn't exist.

        Table schema:
            - id: Auto-incrementing primary key
            - timestamp: When the search occurred
            - client_ip: IP address of the client
            - query: The search query string
            - results_count: Number of results returned
            - execution_time: Time taken to execute search (seconds)
            - page: Page number requested
            - page_size: Number of results per page
        """
        try:
            conn = self.get_connection()
            cursor = conn.cursor()

            create_table_query = """
            CREATE TABLE IF NOT EXISTS search_logs (
                id SERIAL PRIMARY KEY,
                timestamp TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
                client_ip VARCHAR(45) NOT NULL,
                query TEXT NOT NULL,
                results_count INTEGER,
                execution_time FLOAT,
                page INTEGER DEFAULT 1,
                page_size INTEGER DEFAULT 10,
                user_agent TEXT,
                referer TEXT
            );

            CREATE INDEX IF NOT EXISTS idx_search_logs_timestamp ON search_logs(timestamp);
            CREATE INDEX IF NOT EXISTS idx_search_logs_client_ip ON search_logs(client_ip);
            CREATE INDEX IF NOT EXISTS idx_search_logs_query ON search_logs(query);
            """

            cursor.execute(create_table_query)
            conn.commit()
            cursor.close()
            conn.close()

        except psycopg2.Error as e:
            print(f"Error initializing search_logs table: {e}")
            raise

    def log_search(
        self,
        client_ip: str,
        query: str,
        results_count: int,
        execution_time: float,
        page: int = 1,
        page_size: int = 10,
        user_agent: Optional[str] = None,
        referer: Optional[str] = None,
        timestamp: Optional[datetime] = None
    ) -> Optional[int]:
        """
        Log a search query to the database.

        Args:
            client_ip: IP address of the client making the search
            query: The search query string
            results_count: Number of results returned
            execution_time: Time taken to execute the search in seconds
            page: Page number requested (default: 1)
            page_size: Number of results per page (default: 10)
            user_agent: User agent string from the request (optional)
            referer: Referer URL from the request (optional)
            timestamp: Specific timestamp for the log entry (optional, defaults to current time)

        Returns:
            int: The ID of the inserted log entry, or None if failed
        """
        try:
            conn = self.get_connection()
            cursor = conn.cursor()

            # If timestamp provided, include it in INSERT; otherwise use default CURRENT_TIMESTAMP
            if timestamp:
                insert_query = """
                INSERT INTO search_logs
                    (timestamp, client_ip, query, results_count, execution_time, page, page_size, user_agent, referer)
                VALUES
                    (%s, %s, %s, %s, %s, %s, %s, %s, %s)
                RETURNING id;
                """
                cursor.execute(
                    insert_query,
                    (timestamp, client_ip, query, results_count, execution_time, page, page_size, user_agent, referer)
                )
            else:
                insert_query = """
                INSERT INTO search_logs
                    (client_ip, query, results_count, execution_time, page, page_size, user_agent, referer)
                VALUES
                    (%s, %s, %s, %s, %s, %s, %s, %s)
                RETURNING id;
                """
                cursor.execute(
                    insert_query,
                    (client_ip, query, results_count, execution_time, page, page_size, user_agent, referer)
                )

            log_id = cursor.fetchone()[0]
            conn.commit()
            cursor.close()
            conn.close()

            return log_id

        except psycopg2.Error as e:
            print(f"Error logging search: {e}")
            return None

    def get_recent_searches(self, limit: int = 100) -> list:
        """
        Retrieve recent search queries.

        Args:
            limit: Maximum number of results to return (default: 100)

        Returns:
            list: List of recent search log entries as dictionaries
        """
        try:
            conn = self.get_connection()
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            query = """
            SELECT * FROM search_logs
            ORDER BY timestamp DESC
            LIMIT %s;
            """

            cursor.execute(query, (limit,))
            results = cursor.fetchall()

            cursor.close()
            conn.close()

            return [dict(row) for row in results]

        except psycopg2.Error as e:
            print(f"Error retrieving recent searches: {e}")
            return []

    def get_popular_searches(self, limit: int = 20, days: int = 7) -> list:
        """
        Get the most popular search queries within a time period.

        Args:
            limit: Maximum number of results to return (default: 20)
            days: Number of days to look back (default: 7)

        Returns:
            list: List of popular queries with counts
        """
        try:
            conn = self.get_connection()
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            query = """
            SELECT
                query,
                COUNT(*) as search_count,
                AVG(execution_time) as avg_execution_time,
                AVG(results_count) as avg_results_count
            FROM search_logs
            WHERE timestamp >= NOW() - INTERVAL '%s days'
            GROUP BY query
            ORDER BY search_count DESC
            LIMIT %s;
            """

            cursor.execute(query, (days, limit))
            results = cursor.fetchall()

            cursor.close()
            conn.close()

            return [dict(row) for row in results]

        except psycopg2.Error as e:
            print(f"Error retrieving popular searches: {e}")
            return []

    def get_search_stats(self, days: int = 30) -> dict:
        """
        Get aggregate statistics about searches.

        Args:
            days: Number of days to look back (default: 30)

        Returns:
            dict: Statistics including total searches, unique IPs, avg execution time
        """
        try:
            conn = self.get_connection()
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            query = """
            SELECT
                COUNT(*) as total_searches,
                COUNT(DISTINCT client_ip) as unique_ips,
                COUNT(DISTINCT query) as unique_queries,
                AVG(execution_time) as avg_execution_time,
                AVG(results_count) as avg_results_count,
                MAX(timestamp) as last_search
            FROM search_logs
            WHERE timestamp >= NOW() - INTERVAL '%s days';
            """

            cursor.execute(query, (days,))
            result = cursor.fetchone()

            cursor.close()
            conn.close()

            return dict(result) if result else {}

        except psycopg2.Error as e:
            print(f"Error retrieving search stats: {e}")
            return {}


# Global instance
_search_logger = None

def get_search_logger() -> SearchLogger:
    """
    Get or create the global SearchLogger instance.

    Returns:
        SearchLogger: The global search logger instance
    """
    global _search_logger
    if _search_logger is None:
        _search_logger = SearchLogger()
    return _search_logger
