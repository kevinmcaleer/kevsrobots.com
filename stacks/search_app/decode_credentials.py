#!/usr/bin/env python3
"""
Quick utility to decode URL-encoded credentials from .env file

Usage:
    python3 decode_credentials.py
"""

import os
from urllib.parse import unquote
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def decode_and_display():
    """Decode and display database credentials from .env file."""

    db_user = os.getenv('DB_USER', '')
    db_password = os.getenv('DB_PASSWORD', '')
    db_host = os.getenv('DB_HOST', '')
    db_port = os.getenv('DB_PORT', '')
    db_name = os.getenv('DB_NAME', '')

    # Decode credentials
    decoded_user = unquote(db_user) if db_user else 'NOT SET'
    decoded_password = unquote(db_password) if db_password else 'NOT SET'

    # Mask password for security
    masked_password = '*' * (len(decoded_password) - 4) + decoded_password[-4:] if len(decoded_password) > 4 else '****'

    print("=" * 60)
    print("Database Credential Decoder")
    print("=" * 60)
    print()
    print("ENCODED VALUES (from .env):")
    print(f"  DB_USER     : {db_user}")
    print(f"  DB_PASSWORD : {db_password[:4]}...{db_password[-4:] if len(db_password) > 8 else '****'}")
    print()
    print("DECODED VALUES (used by app):")
    print(f"  DB_USER     : {decoded_user}")
    print(f"  DB_PASSWORD : {masked_password}")
    print()
    print("CONNECTION INFO:")
    print(f"  Host        : {db_host}")
    print(f"  Port        : {db_port}")
    print(f"  Database    : {db_name}")
    print()
    print("CONNECTION STRING (for testing):")
    print(f"  psql -h {db_host} -p {db_port} -U {decoded_user} -d {db_name}")
    print()
    print("=" * 60)
    print()

    # Test if values changed after decoding
    if db_user == decoded_user:
        print("✓ DB_USER is NOT URL-encoded (plain text)")
    else:
        print("✓ DB_USER is URL-encoded and will be decoded automatically")
        print(f"  Original: {db_user}")
        print(f"  Decoded:  {decoded_user}")

    print()

    if db_password == decoded_password:
        print("✓ DB_PASSWORD is NOT URL-encoded (plain text)")
    else:
        print("✓ DB_PASSWORD is URL-encoded and will be decoded automatically")
        print(f"  Original: {db_password[:4]}...{db_password[-4:]}")
        print(f"  Decoded:  {masked_password}")

    print()
    print("=" * 60)

if __name__ == "__main__":
    if not os.path.exists('.env'):
        print("ERROR: .env file not found in current directory")
        print("Please run this script from the search_app directory")
        exit(1)

    decode_and_display()
