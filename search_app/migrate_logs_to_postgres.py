#!/usr/bin/env python3
"""
Migrate historical search logs from search-logs.log to PostgreSQL database.

This script reads the old log file format and inserts records into the
PostgreSQL search_logs table, preserving timestamps and query data.

Log format: INFO:root:2023-12-30T23:10:09.904838 - IP: 127.0.0.1 - Query: smars

Usage:
    python3 migrate_logs_to_postgres.py [--dry-run] [--log-file search-logs.log]
"""

import re
import argparse
from datetime import datetime
from typing import Optional, Tuple
import os
import sys

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from search.search_logger import get_search_logger
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


class LogMigrator:
    """Migrates search logs from file to PostgreSQL."""

    def __init__(self, dry_run: bool = False):
        """
        Initialize migrator.

        Args:
            dry_run: If True, don't insert into database, just show what would be done
        """
        self.dry_run = dry_run
        self.logger = get_search_logger() if not dry_run else None
        self.log_pattern = re.compile(
            r'INFO:root:(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d+) - IP: ([\d\.]+) - Query: (.+)'
        )
        self.stats = {
            'total_lines': 0,
            'parsed': 0,
            'skipped': 0,
            'inserted': 0,
            'errors': 0
        }

    def parse_log_line(self, line: str) -> Optional[Tuple[datetime, str, str]]:
        """
        Parse a log line and extract timestamp, IP, and query.

        Args:
            line: Log line string

        Returns:
            Tuple of (timestamp, ip_address, query) or None if parse fails
        """
        match = self.log_pattern.match(line.strip())
        if not match:
            return None

        timestamp_str, ip_address, query = match.groups()

        # Parse timestamp
        try:
            timestamp = datetime.fromisoformat(timestamp_str)
        except ValueError:
            return None

        return timestamp, ip_address, query

    def migrate_file(self, log_file: str) -> dict:
        """
        Migrate all logs from file to PostgreSQL.

        Args:
            log_file: Path to log file

        Returns:
            dict: Migration statistics
        """
        if not os.path.exists(log_file):
            print(f"ERROR: Log file not found: {log_file}")
            return self.stats

        print(f"{'[DRY RUN] ' if self.dry_run else ''}Migrating logs from: {log_file}")
        print("-" * 80)

        with open(log_file, 'r') as f:
            for line_num, line in enumerate(f, 1):
                self.stats['total_lines'] += 1

                # Parse log line
                parsed = self.parse_log_line(line)
                if not parsed:
                    self.stats['skipped'] += 1
                    if self.stats['skipped'] <= 5:  # Show first 5 skipped lines
                        print(f"SKIP (line {line_num}): {line.strip()[:80]}")
                    continue

                self.stats['parsed'] += 1
                timestamp, ip_address, query = parsed

                # Insert into database
                if self.dry_run:
                    print(f"WOULD INSERT: {timestamp} | {ip_address:15} | {query}")
                    self.stats['inserted'] += 1
                else:
                    try:
                        log_id = self.logger.log_search(
                            client_ip=ip_address,
                            query=query,
                            results_count=None,  # Historical data doesn't have this
                            execution_time=None,  # Historical data doesn't have this
                            page=1,
                            page_size=10,
                            user_agent=None,
                            referer=None
                        )
                        if log_id:
                            self.stats['inserted'] += 1
                            if self.stats['inserted'] % 100 == 0:
                                print(f"Inserted {self.stats['inserted']} records...")
                    except Exception as e:
                        self.stats['errors'] += 1
                        if self.stats['errors'] <= 5:  # Show first 5 errors
                            print(f"ERROR (line {line_num}): {e}")

        return self.stats

    def print_summary(self):
        """Print migration summary statistics."""
        print("\n" + "=" * 80)
        print("MIGRATION SUMMARY")
        print("=" * 80)
        print(f"Total lines read:     {self.stats['total_lines']:,}")
        print(f"Successfully parsed:  {self.stats['parsed']:,}")
        print(f"Skipped (invalid):    {self.stats['skipped']:,}")
        print(f"Inserted to DB:       {self.stats['inserted']:,}")
        print(f"Errors:               {self.stats['errors']:,}")
        print("=" * 80)

        if self.stats['inserted'] > 0:
            success_rate = (self.stats['inserted'] / self.stats['parsed'] * 100) if self.stats['parsed'] > 0 else 0
            print(f"Success rate: {success_rate:.1f}%")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Migrate historical search logs to PostgreSQL',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Dry run to see what would be migrated
  python3 migrate_logs_to_postgres.py --dry-run

  # Actually migrate the logs
  python3 migrate_logs_to_postgres.py

  # Migrate specific log file
  python3 migrate_logs_to_postgres.py --log-file /path/to/search-logs.log
        """
    )

    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be migrated without inserting to database'
    )

    parser.add_argument(
        '--log-file',
        default='search-logs.log',
        help='Path to log file (default: search-logs.log)'
    )

    args = parser.parse_args()

    # Check environment variables
    if not args.dry_run:
        required_vars = ['DB_HOST', 'DB_PORT', 'DB_NAME', 'DB_USER', 'DB_PASSWORD']
        missing = [var for var in required_vars if not os.getenv(var)]
        if missing:
            print("ERROR: Missing required environment variables:")
            for var in missing:
                print(f"  - {var}")
            print("\nMake sure .env file exists with database credentials.")
            sys.exit(1)

    # Run migration
    migrator = LogMigrator(dry_run=args.dry_run)

    try:
        stats = migrator.migrate_file(args.log_file)
        migrator.print_summary()

        if args.dry_run:
            print("\nThis was a DRY RUN. No data was inserted.")
            print("Run without --dry-run to actually migrate the data.")
        else:
            print("\nMigration complete!")
            print("\nVerify with:")
            print("  psql -h $DB_HOST -p $DB_PORT -U $DB_USER -d $DB_NAME")
            print("  SELECT COUNT(*) FROM search_logs;")

    except KeyboardInterrupt:
        print("\n\nMigration interrupted by user.")
        migrator.print_summary()
        sys.exit(1)

    except Exception as e:
        print(f"\n\nERROR: {e}")
        migrator.print_summary()
        sys.exit(1)


if __name__ == '__main__':
    main()
