"""File storage via Synology NAS (SMB) with local fallback.

Adapted from chatter's storage.py. Stores project files and images
on the NAS at projects/files/{project_id}/ and projects/images/{project_id}/.
Falls back to local disk if NAS is unreachable.
"""

from __future__ import annotations

import logging
import uuid
from pathlib import Path
from typing import Optional, Tuple

from .config import get_settings

logger = logging.getLogger(__name__)

LOCAL_STORAGE_PATH = Path("/tmp/projects_uploads")
LOCAL_STORAGE_PATH.mkdir(parents=True, exist_ok=True)

ALLOWED_FILE_EXTENSIONS = {
    ".py", ".cpp", ".h", ".ino", ".md", ".txt", ".pdf",
    ".stl", ".obj", ".gcode", ".3mf", ".step", ".stp", ".dxf", ".dwg", ".svg",
    ".json", ".xml", ".yaml", ".yml", ".csv", ".toml",
    ".zip", ".tar", ".gz", ".7z", ".rar",
    ".scad", ".kicad_pcb", ".kicad_sch", ".brd", ".sch",
    ".f3d", ".fcstd",
}
ALLOWED_IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".gif", ".webp", ".svg"}

def _check_nas() -> bool:
    """Check NAS connectivity. Retries every time — no caching of failures."""

    settings = get_settings()
    if not all([settings.nas_host, settings.nas_username, settings.nas_password]):
        logger.warning("NAS credentials not configured")
        return False

    try:
        from smbprotocol.connection import Connection
        from smbprotocol.session import Session

        conn = Connection(uuid.uuid4(), settings.nas_host, 445)
        conn.connect(timeout=5)
        sess = Session(conn, settings.nas_username, settings.nas_password)
        sess.connect()
        conn.disconnect()
        return True
    except Exception as e:
        logger.warning("NAS unavailable: %s", e)
        return False


def validate_file(filename: str, content: bytes, max_size: int) -> Tuple[bool, Optional[str]]:
    ext = Path(filename).suffix.lower()
    allowed = ALLOWED_FILE_EXTENSIONS | ALLOWED_IMAGE_EXTENSIONS
    if ext not in allowed:
        return False, f"File type {ext} not allowed"
    if len(content) > max_size:
        return False, f"File too large ({len(content)} bytes, max {max_size})"
    return True, None


def is_image(filename: str) -> bool:
    return Path(filename).suffix.lower() in ALLOWED_IMAGE_EXTENSIONS


def generate_filename(original: str, project_id: int) -> str:
    ext = Path(original).suffix.lower()
    unique = uuid.uuid4().hex[:8]
    stem = Path(original).stem[:50]
    return f"p{project_id}_{stem}_{unique}{ext}"


def _nas_path(project_id: int, file_type: str) -> str:
    return f"projects\\{file_type}\\{project_id}"


def _local_path(project_id: int, file_type: str) -> Path:
    p = LOCAL_STORAGE_PATH / "projects" / file_type / str(project_id)
    p.mkdir(parents=True, exist_ok=True)
    return p


def save_file(content: bytes, filename: str, project_id: int, file_type: str) -> Optional[str]:
    """Save file to NAS, with local fallback for test/dev environments."""

    if _check_nas():
        if _save_to_nas(content, filename, project_id, file_type):
            return f"nas:projects/{file_type}/{project_id}/{filename}"
        return None

    if _save_to_local(content, filename, project_id, file_type):
        logger.warning("NAS unavailable — saved to local storage")
        return f"local:projects/{file_type}/{project_id}/{filename}"

    return None


def read_file(file_path: str) -> Optional[bytes]:
    """Read file from NAS or local storage."""

    if file_path.startswith("nas:"):
        return _read_from_nas(file_path[4:])
    elif file_path.startswith("local:"):
        local = LOCAL_STORAGE_PATH / file_path[6:]
        if local.exists():
            return local.read_bytes()
    return None


def delete_file(file_path: str) -> bool:
    if file_path.startswith("nas:"):
        return _delete_from_nas(file_path[4:])
    elif file_path.startswith("local:"):
        local = LOCAL_STORAGE_PATH / file_path[6:]
        if local.exists():
            local.unlink()
            return True
    return False


def _save_to_nas(content: bytes, filename: str, project_id: int, file_type: str) -> bool:
    try:
        from smbprotocol.connection import Connection
        from smbprotocol.session import Session
        from smbprotocol.tree import TreeConnect
        from smbprotocol.open import (
            Open, CreateDisposition, FilePipePrinterAccessMask,
            ImpersonationLevel, FileAttributes, ShareAccess, CreateOptions,
        )

        settings = get_settings()
        conn = Connection(uuid.uuid4(), settings.nas_host, 445)
        conn.connect(timeout=10)
        sess = Session(conn, settings.nas_username, settings.nas_password)
        sess.connect()
        tree = TreeConnect(sess, f"\\\\{settings.nas_host}\\{settings.nas_share_name}")
        tree.connect()

        nas_dir = _nas_path(project_id, file_type)
        for part_count in range(1, len(nas_dir.split("\\")) + 1):
            partial = "\\".join(nas_dir.split("\\")[:part_count])
            try:
                d = Open(tree, partial)
                d.create(
                    ImpersonationLevel.Impersonation,
                    FilePipePrinterAccessMask.GENERIC_READ,
                    FileAttributes.FILE_ATTRIBUTE_DIRECTORY,
                    ShareAccess.FILE_SHARE_READ | ShareAccess.FILE_SHARE_WRITE,
                    CreateDisposition.FILE_OPEN_IF,
                    CreateOptions.FILE_DIRECTORY_FILE, None,
                )
                d.close()
            except Exception:
                pass

        fp = f"{nas_dir}\\{filename}"
        f = Open(tree, fp)
        f.create(
            ImpersonationLevel.Impersonation,
            FilePipePrinterAccessMask.GENERIC_WRITE,
            FileAttributes.FILE_ATTRIBUTE_NORMAL,
            ShareAccess.FILE_SHARE_READ,
            CreateDisposition.FILE_OVERWRITE_IF,
            CreateOptions.FILE_NON_DIRECTORY_FILE, None,
        )
        f.write(content, 0)
        f.close()
        tree.disconnect()
        conn.disconnect()
        return True
    except Exception as e:
        logger.error("NAS save failed: %s", e)
        return False


def _save_to_local(content: bytes, filename: str, project_id: int, file_type: str) -> bool:
    try:
        path = _local_path(project_id, file_type) / filename
        path.write_bytes(content)
        return True
    except Exception as e:
        logger.error("Local save failed: %s", e)
        return False


def _read_from_nas(rel_path: str) -> Optional[bytes]:
    try:
        from smbprotocol.connection import Connection
        from smbprotocol.session import Session
        from smbprotocol.tree import TreeConnect
        from smbprotocol.open import (
            Open, CreateDisposition, FilePipePrinterAccessMask,
            ImpersonationLevel, FileAttributes, ShareAccess, CreateOptions,
        )

        settings = get_settings()
        conn = Connection(uuid.uuid4(), settings.nas_host, 445)
        conn.connect(timeout=10)
        sess = Session(conn, settings.nas_username, settings.nas_password)
        sess.connect()
        tree = TreeConnect(sess, f"\\\\{settings.nas_host}\\{settings.nas_share_name}")
        tree.connect()

        smb_path = rel_path.replace("/", "\\")
        f = Open(tree, smb_path)
        f.create(
            ImpersonationLevel.Impersonation,
            FilePipePrinterAccessMask.GENERIC_READ,
            FileAttributes.FILE_ATTRIBUTE_NORMAL,
            ShareAccess.FILE_SHARE_READ,
            CreateDisposition.FILE_OPEN,
            CreateOptions.FILE_NON_DIRECTORY_FILE, None,
        )
        data = f.read(0, f.end_of_file)
        f.close()
        tree.disconnect()
        conn.disconnect()
        return data
    except Exception as e:
        logger.error("NAS read failed: %s", e)
        return None


def _delete_from_nas(rel_path: str) -> bool:
    try:
        from smbprotocol.connection import Connection
        from smbprotocol.session import Session
        from smbprotocol.tree import TreeConnect
        from smbprotocol.open import (
            Open, CreateDisposition, FilePipePrinterAccessMask,
            ImpersonationLevel, FileAttributes, ShareAccess, CreateOptions,
        )

        settings = get_settings()
        conn = Connection(uuid.uuid4(), settings.nas_host, 445)
        conn.connect(timeout=10)
        sess = Session(conn, settings.nas_username, settings.nas_password)
        sess.connect()
        tree = TreeConnect(sess, f"\\\\{settings.nas_host}\\{settings.nas_share_name}")
        tree.connect()

        smb_path = rel_path.replace("/", "\\")
        f = Open(tree, smb_path)
        f.create(
            ImpersonationLevel.Impersonation,
            FilePipePrinterAccessMask.DELETE,
            FileAttributes.FILE_ATTRIBUTE_NORMAL,
            ShareAccess.FILE_SHARE_READ | ShareAccess.FILE_SHARE_WRITE | ShareAccess.FILE_SHARE_DELETE,
            CreateDisposition.FILE_OPEN,
            CreateOptions.FILE_NON_DIRECTORY_FILE | CreateOptions.FILE_DELETE_ON_CLOSE,
            None,
        )
        f.close()
        tree.disconnect()
        conn.disconnect()
        return True
    except Exception as e:
        logger.error("NAS delete failed: %s", e)
        return False
