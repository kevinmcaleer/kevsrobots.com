"""Snakie's "Share project" (Snakie's Share sheet, kevsrobots.com#582).

Two routes an app may call with a "Sign in with kevsrobots" token holding
``projects:write`` (Chatter#80), next to ``PUT …/wiring/send``:

- ``POST /api/projects/app/new``: make a private draft project for a Snakie
  project that has no kevsrobots page yet, so nobody has to leave Snakie to
  make an empty one first.
- ``PUT /api/projects/{id}/robot/send``: the robot model (the URDF and its
  STL meshes) as project files, plus an optional cover picture. A file
  replaces the working copy's file of the same name, so sending again
  updates the model instead of adding copies. The page's robot frame finds
  the URDF and meshes among the project's files by name.

Code is never sent: ``.py`` and other code files are refused here, and the
project links to GitHub instead.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from fastapi import APIRouter, Depends, File, HTTPException, UploadFile, status
from pydantic import BaseModel, Field
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from .. import versions as v
from ..auth import require_scope
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectFile, ProjectImage
from ..slugs import slugify_title, unique_slug_for_author
from ..storage import (
    STORED_GZ,
    compress_for_storage,
    delete_file,
    file_category,
    generate_filename,
    save_file,
)
from .files import storage_used
from .wiring import GITHUB_REPO, SITE_PROJECTS

router = APIRouter(prefix="/api/projects", tags=["app-send"])

EDITOR = "https://www.kevsrobots.com/projects/editor.html?id="
#: What a robot model is made of. Everything else (code above all) is refused.
ROBOT_EXTENSIONS = {".urdf", ".stl"}
COVER_EXTENSIONS = {".png", ".jpg", ".jpeg", ".webp"}
#: The cover Snakie sends keeps this name, so a re-send replaces it.
COVER_NAME = "snakie-cover"

write_scope = require_scope("projects:write", terms=True)


class AppProjectCreate(BaseModel):
    title: str = Field(..., min_length=5, max_length=200)
    short_description: Optional[str] = Field(None, max_length=500)
    code_repo_url: Optional[str] = Field(None, max_length=500)


class AppProject(BaseModel):
    id: int
    title: str
    slug: Optional[str] = None
    status: str
    has_unpublished_changes: bool = False
    url: Optional[str] = None
    editor_url: str


class SentFile(BaseModel):
    id: int
    filename: str
    replaced: bool = False


class RobotSent(BaseModel):
    project_id: int
    title: str
    url: Optional[str] = None
    editor_url: str
    files: list[SentFile] = Field(default_factory=list)
    cover: bool = False


def _page(project: Project) -> Optional[str]:
    return f"{SITE_PROJECTS}/{project.author_username}/{project.slug}" if project.slug else None


def _check_repo(url: Optional[str]) -> Optional[str]:
    url = (url or "").strip() or None
    if url is not None and not GITHUB_REPO.match(url):
        raise HTTPException(
            status.HTTP_422_UNPROCESSABLE_CONTENT,
            detail="code_repo_url must be a GitHub address, like https://github.com/you/robot",
        )
    return url


@router.post("/app/new", response_model=AppProject, status_code=201)
async def create_from_app(
    body: AppProjectCreate,
    session: AsyncSession = Depends(get_session),
    user: str = Depends(write_scope),
) -> AppProject:
    """A private draft for the author to write up and publish from the editor."""
    repo_url = _check_repo(body.code_repo_url)
    slug = await unique_slug_for_author(session, author_username=user, base_slug=slugify_title(body.title))
    project = Project(
        title=body.title.strip(),
        slug=slug,
        short_description=body.short_description,
        code_repo_url=repo_url,
        content_mode="markdown",
        status="draft",
        author_username=user,
    )
    session.add(project)
    await session.commit()
    await session.refresh(project)
    return AppProject(
        id=project.id,
        title=project.title,
        slug=project.slug,
        status=project.status,
        url=_page(project),
        editor_url=f"{EDITOR}{project.id}",
    )


async def _owned(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="That project isn't on kevsrobots any more.")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="That project isn't yours.")
    return project


def _name(upload: UploadFile) -> str:
    # Only the base name: a mesh is found by name, and no path gets stored.
    return Path((upload.filename or "").replace("\\", "/")).name


@router.put("/{project_id}/robot/send", response_model=RobotSent)
async def send_robot(
    project_id: int,
    files: list[UploadFile] = File(default_factory=list),
    cover: Optional[UploadFile] = File(None),
    session: AsyncSession = Depends(get_session),
    user: str = Depends(write_scope),
) -> RobotSent:
    project = await _owned(session, project_id, user)
    settings = get_settings()

    incoming: list[tuple[str, bytes]] = []
    seen: set[str] = set()
    for upload in files:
        name = _name(upload)
        ext = Path(name).suffix.lower()
        if not name or ext not in ROBOT_EXTENSIONS:
            raise HTTPException(
                status.HTTP_400_BAD_REQUEST,
                detail=f"{name or 'A file'} isn't part of a robot model. Send .urdf and .stl files only.",
            )
        if name.lower() in seen:
            continue
        seen.add(name.lower())
        data = await upload.read()
        if len(data) > settings.max_file_size:
            raise HTTPException(
                status.HTTP_400_BAD_REQUEST,
                detail=f"{name} is over the {settings.max_file_size // (1024 * 1024)} MB a file can be.",
            )
        incoming.append((name, data))

    cover_data: Optional[bytes] = None
    cover_ext = ""
    if cover is not None:
        cover_ext = Path(_name(cover)).suffix.lower()
        if cover_ext not in COVER_EXTENSIONS:
            raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="The cover picture must be a PNG, JPEG or WebP.")
        cover_data = await cover.read()
        if len(cover_data) > settings.max_image_size:
            raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="The cover picture is too big.")

    # The working copy's files these replace, by name (case-insensitive, as
    # the robot frame matches meshes).
    current = (
        await session.scalars(
            select(ProjectFile).where(ProjectFile.project_id == project_id, ProjectFile.removed_at.is_(None))
        )
    ).all()
    by_name = {f.filename.lower(): f for f in current}
    replacing = [by_name[n.lower()] for n, _ in incoming if n.lower() in by_name]

    stored: list[tuple[str, bytes, bytes, bool]] = []
    for name, data in incoming:
        packed_data, packed = compress_for_storage(name, data)
        stored.append((name, data, packed_data, packed))
    used = await storage_used(session, project_id)
    freed = sum((f.stored_size or f.file_size) for f in replacing)
    adding = sum(len(p) for _, _, p, _ in stored)
    if used - freed + adding > settings.max_project_storage:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"The model is over the {settings.max_project_storage // (1024 * 1024)} MB a project can hold.",
        )

    sent: list[SentFile] = []
    for name, data, packed_data, packed in stored:
        filename = generate_filename(name, project_id)
        path = save_file(packed_data, filename + STORED_GZ if packed else filename, project_id, "files")
        if path is None:
            raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"{name} couldn't be stored.")
        old = by_name.get(name.lower())
        if old is not None and await v.guard_removal(session, project, "files", old):
            delete_file(old.file_path)
            if old.thumbnail_path:
                delete_file(old.thumbnail_path)
            await session.delete(old)
        record = ProjectFile(
            project_id=project_id,
            filename=name,
            file_path=path,
            file_size=len(data),
            stored_size=len(packed_data) if packed else None,
            file_type=file_category(name),
            # Keep what the author wrote about the old copy.
            description=old.description if old is not None else None,
            print_settings=old.print_settings if old is not None else None,
            print_qty=old.print_qty if old is not None else None,
            print_colour=old.print_colour if old is not None else None,
            filament=old.filament if old is not None else None,
        )
        session.add(record)
        await session.flush()
        sent.append(SentFile(id=record.id, filename=name, replaced=old is not None))

    if cover_data is not None:
        cover_name = COVER_NAME + cover_ext
        old_cover = await session.scalar(
            select(ProjectImage).where(
                ProjectImage.project_id == project_id,
                ProjectImage.removed_at.is_(None),
                ProjectImage.superseded_at.is_(None),
                ProjectImage.filename.like(COVER_NAME + ".%"),
            )
        )
        path = save_file(cover_data, generate_filename(cover_name, project_id), project_id, "images")
        if path is None:
            raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="The cover picture couldn't be stored.")
        if old_cover is not None:
            sort_order = old_cover.sort_order
            caption = old_cover.caption
            if await v.guard_removal(session, project, "images", old_cover):
                delete_file(old_cover.file_path)
                await session.delete(old_cover)
        else:
            # The first picture is the cover: a project with none gets this
            # one first, and one that has pictures keeps its own cover.
            max_sort = await session.scalar(
                select(func.max(ProjectImage.sort_order)).where(
                    ProjectImage.project_id == project_id, ProjectImage.removed_at.is_(None)
                )
            )
            sort_order = (max_sort + 1) if max_sort is not None else 0
            caption = None
        session.add(
            ProjectImage(
                project_id=project_id, filename=cover_name, file_path=path, sort_order=sort_order, caption=caption
            )
        )

    await session.commit()
    return RobotSent(
        project_id=project.id,
        title=project.title,
        url=_page(project),
        editor_url=f"{EDITOR}{project.id}",
        files=sent,
        cover=cover_data is not None,
    )
