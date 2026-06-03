"""kevsrobots status service.

A tiny FastAPI app that polls every kevsrobots web service ``/health``
endpoint every 15 minutes, stores results in SQLite (``/data/status.db``),
exposes a public read-only API, and serves a public dashboard inspired
by status.claude.ai.

Per issue #203.
"""
