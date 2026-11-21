"""Which suite apps a project was made with ("Made with Snakie / BetterCAD").

Nothing is entered by hand; it is worked out from what the project holds:

* ``snakie``    the project has wiring sent from Snakie (``project_wiring``)
* ``bettercad`` the author imported a BetterCAD model and build steps
                (an *applied* ``project_imports`` row from BetterCAD)

Cards, the gallery filter (``?made_with=``) and the profile's "Builds with"
counts all read from here, so the rule lives in one place.
"""

from __future__ import annotations

from typing import Iterable, Literal

from sqlalchemy import exists, select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import Project, ProjectImport, ProjectWiring

App = Literal["snakie", "bettercad"]
APPS: tuple[App, ...] = ("snakie", "bettercad")


def made_with_clause(app: App):
    """SQL filter: projects made with ``app``."""
    if app == "snakie":
        return exists().where(ProjectWiring.project_id == Project.id)
    return exists().where(
        ProjectImport.project_id == Project.id,
        ProjectImport.source == "bettercad",
        ProjectImport.status == "applied",
    )


async def made_with_map(session: AsyncSession, project_ids: Iterable[int]) -> dict[int, list[str]]:
    """``{project_id: ["snakie", "bettercad"]}`` for each id (two queries)."""
    ids = list(project_ids)
    found: dict[int, list[str]] = {pid: [] for pid in ids}
    if not ids:
        return found
    snakie = set(
        (await session.scalars(
            select(ProjectWiring.project_id).where(ProjectWiring.project_id.in_(ids))
        )).all()
    )
    bettercad = set(
        (await session.scalars(
            select(ProjectImport.project_id).where(
                ProjectImport.project_id.in_(ids),
                ProjectImport.source == "bettercad",
                ProjectImport.status == "applied",
            )
        )).all()
    )
    for pid in ids:
        if pid in snakie:
            found[pid].append("snakie")
        if pid in bettercad:
            found[pid].append("bettercad")
    return found


async def made_with_for(session: AsyncSession, project_id: int) -> list[str]:
    return (await made_with_map(session, [project_id]))[project_id]
