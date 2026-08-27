#!/usr/bin/env python3
"""
Verify search.db database has correct data and can be queried.
"""

import sqlite3
import sys
import os

def verify_database(db_path='search.db'):
    """Verify the database has data and can be searched."""

    if not os.path.exists(db_path):
        print(f"❌ Database not found: {db_path}")
        return False

    print("=" * 80)
    print(f"Verifying Database: {db_path}")
    print("=" * 80)
    print()

    # Check file size
    size_mb = os.path.getsize(db_path) / (1024 * 1024)
    print(f"📊 File size: {size_mb:.2f} MB")
    print()

    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Check tables exist
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = cursor.fetchall()
        print(f"📁 Tables found: {[t[0] for t in tables]}")
        print()

        # Check if documents_fts exists
        if not any('documents_fts' in str(t) for t in tables):
            print("❌ documents_fts table not found!")
            return False

        # Count total entries
        cursor.execute("SELECT COUNT(*) FROM documents_fts")
        total = cursor.fetchone()[0]
        print(f"📚 Total entries: {total}")

        # Count unique URLs
        cursor.execute("SELECT COUNT(DISTINCT url) FROM documents_fts")
        unique = cursor.fetchone()[0]
        print(f"🔗 Unique URLs: {unique}")
        print()

        if total == 0:
            print("❌ Database is EMPTY!")
            return False

        # Test searches
        print("🔍 Testing searches:")
        print("-" * 80)

        test_queries = ['python', 'robot', 'micropython', 'raspberry']

        for query in test_queries:
            cursor.execute("SELECT COUNT(*) FROM documents_fts WHERE documents_fts MATCH ?", (query,))
            count = cursor.fetchone()[0]

            # Get sample result
            cursor.execute("SELECT url, page_title FROM documents_fts WHERE documents_fts MATCH ? LIMIT 1", (query,))
            sample = cursor.fetchone()

            if count > 0:
                print(f"  ✅ '{query}': {count} results")
                if sample:
                    print(f"      Example: {sample[1]}")
                    print(f"      URL: {sample[0]}")
            else:
                print(f"  ❌ '{query}': NO RESULTS (PROBLEM!)")
            print()

        # Show some sample entries
        print("-" * 80)
        print("📝 Sample entries (first 5):")
        print()
        cursor.execute("SELECT url, page_title FROM documents_fts LIMIT 5")
        for i, row in enumerate(cursor.fetchall(), 1):
            print(f"  {i}. {row[1]}")
            print(f"     {row[0]}")
            print()

        conn.close()

        print("=" * 80)
        print("✅ Database verification complete")
        print("=" * 80)
        return True

    except Exception as e:
        print(f"❌ Error: {e}")
        return False


if __name__ == '__main__':
    db_path = sys.argv[1] if len(sys.argv) > 1 else 'search.db'

    success = verify_database(db_path)
    sys.exit(0 if success else 1)
