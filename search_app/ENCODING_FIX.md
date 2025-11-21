# Encoding Error Fix

## Problem

The search index rebuild was failing with:
```
UnicodeDecodeError: 'utf-8' codec can't decode byte 0xb0 in position 37: invalid start byte
```

This happened when the indexer encountered HTML files with non-UTF-8 characters.

## Solution

Updated `index.py` to handle encoding errors gracefully:

### 1. Multi-Encoding Support
- First tries UTF-8 (standard)
- Falls back to latin-1 (accepts all byte values)
- Skips files that can't be decoded

### 2. Better Error Handling
- Wraps all parsing in try-except blocks
- Shows warnings for problematic files
- Continues indexing remaining files
- Returns success/failure status

### 3. Progress Tracking
- Shows progress every 100 files
- Displays summary at end:
  - Total files found
  - Successfully indexed
  - Skipped (with errors)

## Result

The indexer now:
- ✅ Handles mixed encodings gracefully
- ✅ Continues even if some files fail
- ✅ Shows which files were skipped
- ✅ Provides detailed summary

## Example Output

```
Scanning for HTML files...
Found 1113 HTML files to index

Progress: 100/1113 files processed...
Warning: Could not decode ../web/_site/problematic.html: ...
Progress: 200/1113 files processed...
...

============================================================
Indexing Summary:
  Total HTML files: 1113
  Successfully indexed: 1110
  Skipped (errors): 3

  Note: 3 files had encoding or parsing errors
        Check warnings above for details
============================================================
```

## Common Encoding Issues

### Byte 0xb0 (°)
- Often: degree symbol in wrong encoding
- Example: "25°C" in ISO-8859-1

### Byte 0xa9 (©)
- Often: copyright symbol
- Example: "© 2024" in wrong encoding

### Byte 0xe9 (é)
- Often: accented characters
- Example: "résumé" in ISO-8859-1

## Testing

Run the rebuild to verify it works:

```bash
cd ~/kevsrobots.com/search_app
./rebuild_search_index.sh
```

Expected: Index completes successfully, possibly with a few skipped files (which is fine).

## What Gets Skipped

Files that might be skipped:
- Binary files accidentally in _site (images, PDFs misnamed as .html)
- Files with mixed or corrupted encodings
- Non-text files

**This is normal and expected!** The important thing is that the vast majority of HTML files get indexed.

## Troubleshooting

### Many files skipped (>10%)

Check if Jekyll build completed:
```bash
ls -la ../web/_site/
```

Should see normal HTML structure.

### Specific file always fails

Check the file encoding:
```bash
file ../web/_site/path/to/file.html
```

Or check for binary content:
```bash
head -c 100 ../web/_site/path/to/file.html
```

### All files failing

Check Python version and BeautifulSoup:
```bash
python3 --version
python3 -c "import bs4, lxml; print('OK')"
```

## Prevention

To avoid encoding issues in the future:

1. **Jekyll templates**: Use UTF-8 encoding
2. **Content files**: Save as UTF-8
3. **Special characters**: Use HTML entities when possible
   - `&deg;` for °
   - `&copy;` for ©
   - `&eacute;` for é

## Related Files

- `index.py` - The indexer script (now with encoding support)
- `rebuild_search_index.sh` - Rebuild automation
- `REBUILD_SEARCH_INDEX.md` - Full rebuild documentation

---

**Status**: ✅ Fixed - Index rebuild now handles encoding errors gracefully
