#!/bin/bash

# KevsRobots Search Index Rebuild Script
# Automatically rebuilds the SQLite FTS search index from Jekyll _site

set -e  # Exit on error

echo "========================================"
echo "KevsRobots Search Index Rebuild"
echo "========================================"
echo ""

# Step 0: Check Python dependencies
echo "🔍 Checking Python dependencies..."
if ! python3 -c "import bs4, lxml" 2>/dev/null; then
    echo "⚠️  Missing dependencies: beautifulsoup4 and/or lxml"
    echo ""
    echo "Install with:"
    echo "  pip install beautifulsoup4 lxml"
    echo ""
    echo "Or if using system Python on macOS:"
    echo "  python3 -m pip install --break-system-packages beautifulsoup4 lxml"
    echo ""
    exit 1
fi
echo "✅ Dependencies OK"
echo ""

# Step 1: Check if Jekyll site exists
if [ ! -d "../web/_site" ]; then
    echo "❌ ERROR: Jekyll site not found at ../web/_site"
    echo ""
    echo "Build the Jekyll site first:"
    echo "  cd ~/kevsrobots.com/stacks"
    echo "  docker-compose up -d"
    echo ""
    exit 1
fi

# Step 2: Count HTML files
HTML_COUNT=$(find ../web/_site -name "*.html" | wc -l | tr -d ' ')
echo "📄 Found $HTML_COUNT HTML files to index"
echo ""

# Step 3: Backup existing database
if [ -f "search.db" ]; then
    BACKUP_NAME="search.db.backup.$(date +%Y%m%d_%H%M%S)"
    echo "💾 Backing up existing database to: $BACKUP_NAME"
    cp search.db "$BACKUP_NAME"
    echo ""

    # Keep only last 5 backups
    BACKUP_COUNT=$(ls -1 search.db.backup.* 2>/dev/null | wc -l | tr -d ' ')
    if [ "$BACKUP_COUNT" -gt 5 ]; then
        echo "🧹 Cleaning old backups (keeping 5 most recent)..."
        ls -t search.db.backup.* | tail -n +6 | xargs rm -f
    fi
    echo ""
fi

# Step 4: Remove old database
if [ -f "search.db" ]; then
    echo "🗑️  Removing old database..."
    rm search.db
    echo ""
fi

# Step 5: Initialize new database
echo "🔧 Initializing new database..."
python3 -c "from search.database import initialize_database; initialize_database()" 2>&1
echo ""

# Step 6: Run indexer
echo "📚 Indexing HTML files..."
echo "   This may take 1-2 minutes..."
echo ""

# Run indexer and capture output
if python3 index.py 2>&1; then
    echo ""
    echo "✅ Indexing complete"
else
    echo ""
    echo "❌ ERROR: Indexing failed"
    exit 1
fi

# Step 7: Verify index
echo ""
echo "🔍 Verifying index..."
DOC_COUNT=$(sqlite3 search.db "SELECT COUNT(DISTINCT url) FROM documents_fts;")
TOTAL_ENTRIES=$(sqlite3 search.db "SELECT COUNT(*) FROM documents_fts;")
DB_SIZE=$(du -h search.db | cut -f1)

echo "   📊 Statistics:"
echo "      • Unique documents: $DOC_COUNT"
echo "      • Total entries: $TOTAL_ENTRIES"
echo "      • Database size: $DB_SIZE"
echo ""

# Step 8: Test searches
echo "🧪 Testing search functionality..."

test_search() {
    QUERY=$1
    RESULT_COUNT=$(sqlite3 search.db "SELECT COUNT(*) FROM documents_fts WHERE documents_fts MATCH '$QUERY';")
    if [ "$RESULT_COUNT" -gt 0 ]; then
        echo "   ✅ '$QUERY': found $RESULT_COUNT results"
    else
        echo "   ⚠️  '$QUERY': no results (this might be ok)"
    fi
}

test_search "robot"
test_search "micropython"
test_search "raspberry"

echo ""
echo "========================================"
echo "✅ Rebuild Complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo ""
echo "1. Test the API locally:"
echo "   curl \"http://localhost:8000/search/?query=robot\""
echo ""
echo "2. Deploy to production:"
echo "   • Copy search.db to production server"
echo "   • Or rebuild on production server"
echo "   • Or mount as Docker volume"
echo ""
echo "3. Restart API if running:"
echo "   docker-compose restart search-api"
echo ""
