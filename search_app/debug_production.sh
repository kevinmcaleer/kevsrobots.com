#!/bin/bash

echo "=================================="
echo "Production Search Debug Script"
echo "=================================="
echo ""

# Check if search.db exists
if [ -f "search.db" ]; then
    echo "✅ search.db exists"
    ls -lh search.db
    echo ""
else
    echo "❌ search.db NOT FOUND in current directory"
    echo "Current directory: $(pwd)"
    echo ""
    echo "Searching for search.db files:"
    find . -name "search.db" 2>/dev/null
    echo ""
    exit 1
fi

# Check permissions
echo "📋 File permissions:"
ls -l search.db
echo ""

# Check if it's readable
if [ -r "search.db" ]; then
    echo "✅ File is readable"
else
    echo "❌ File is NOT readable"
    exit 1
fi
echo ""

# Verify database using Python
echo "🔍 Verifying database contents..."
python3 << 'EOF'
import sqlite3
import os

db_path = 'search.db'

if not os.path.exists(db_path):
    print(f"❌ {db_path} not found")
    exit(1)

size_mb = os.path.getsize(db_path) / (1024 * 1024)
print(f"📊 Database size: {size_mb:.2f} MB")

conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Check table exists
cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='documents_fts'")
if cursor.fetchone():
    print("✅ documents_fts table exists")
else:
    print("❌ documents_fts table NOT FOUND")
    exit(1)

# Count entries
cursor.execute("SELECT COUNT(*) FROM documents_fts")
total = cursor.fetchone()[0]
print(f"📚 Total entries: {total}")

if total == 0:
    print("❌ DATABASE IS EMPTY!")
    exit(1)

# Test python search
cursor.execute("SELECT COUNT(*) FROM documents_fts WHERE documents_fts MATCH 'python'")
python_count = cursor.fetchone()[0]
print(f"🔍 Search 'python': {python_count} results")

if python_count == 0:
    print("❌ NO RESULTS FOR 'python' - DATABASE ISSUE!")
    exit(1)

# Show sample
cursor.execute("SELECT url, page_title FROM documents_fts WHERE documents_fts MATCH 'python' LIMIT 3")
print(f"\n📝 Sample results:")
for url, title in cursor.fetchall():
    print(f"   • {title}")
    print(f"     {url}")

conn.close()
print("\n✅ Database verification PASSED")
EOF

RESULT=$?

echo ""
if [ $RESULT -eq 0 ]; then
    echo "✅ Database is OK - problem might be in app.py"
    echo ""
    echo "Next steps:"
    echo "1. Check app.py is using correct database path"
    echo "2. Restart the container: docker-compose restart search-api"
    echo "3. Check logs: docker logs search-api"
    echo "4. Test API: curl 'http://localhost:8000/search/?query=python'"
else
    echo "❌ Database has issues - needs to be replaced"
    echo ""
    echo "Replace database:"
    echo "1. scp search.db to production server"
    echo "2. Stop container: docker-compose down"
    echo "3. Copy search.db to correct location"
    echo "4. Start container: docker-compose up -d"
fi
