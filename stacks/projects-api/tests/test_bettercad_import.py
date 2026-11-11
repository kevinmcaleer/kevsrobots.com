"""BetterCad build steps sent to a project and imported in the editor."""

from __future__ import annotations

import io
import zipfile

import pytest
from sqlalchemy import select

from projects_api import bettercad_import as bc
from projects_api.models import ProjectFile, ProjectImage
from .conftest import make_auth_header

PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 32

README = """---
title: "Bracket"
steps: 2
generator: BetterCad
---

# How to build Bracket

2 steps in BetterCad. Sizes are in millimetres. Open [Bracket.bcad](Bracket.bcad) in BetterCad to change it.

![Bracket, finished](images/finished.png)

## Parameters

| Name | Value | Comment |
| --- | --- | --- |
| `width` | 40 mm |  |

## Step 1: Sketch1

![Step 1: Sketch1](images/step-01.png)

Draw a rectangle.

## Step 2: Extrude1

![Step 2: Extrude1](images/step-02.png)

Extrude **Sketch1**: 5 mm.
"""


def make_zip(readme: str = README, extra: dict[str, bytes] | None = None) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("README.md", readme)
        z.writestr("Bracket.bcad", '{"bcad": 1}')
        z.writestr("models/Bracket.stl", "solid b\nendsolid b\n")
        for name in ("images/step-01.png", "images/step-02.png", "images/finished.png"):
            z.writestr(name, PNG)
        for name, data in (extra or {}).items():
            z.writestr(name, data)
    return buf.getvalue()


def read(data: bytes) -> bc.Bundle:
    return bc.read_bundle(data, max_image=10_000_000, max_file=10_000_000)


# --- the zip and the text ---


def test_reads_the_bundle() -> None:
    b = read(make_zip())
    assert b.title == "Bracket"
    assert b.steps == 2
    assert list(b.images)[0] == "images/finished.png"
    assert set(b.files) == {"Bracket.bcad", "models/Bracket.stl"}


@pytest.mark.parametrize("name,why", [
    ("../evil.stl", "isn't allowed"),
    ("/etc/passwd.txt", "isn't allowed"),
    ("images/../../x.png", "isn't allowed"),
    ("images/run.exe", "Only PNG and JPEG"),
    ("tool.exe", "can't go in a project"),
    ("other/thing.stl", "doesn't use"),
])
def test_refuses_bad_entries(name, why) -> None:
    with pytest.raises(bc.BundleError, match=why):
        read(make_zip(extra={name: b"x"}))


def test_refuses_a_fake_picture() -> None:
    with pytest.raises(bc.BundleError, match="isn't a PNG"):
        read(make_zip(extra={"images/step-03.png": b"GIF89a"}))


def test_refuses_no_readme_or_not_bettercad() -> None:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("Bracket.bcad", "{}")
    with pytest.raises(bc.BundleError, match="no README"):
        read(buf.getvalue())
    with pytest.raises(bc.BundleError, match="wasn't made by BetterCad"):
        read(make_zip(readme="# Hello\n"))
    with pytest.raises(bc.BundleError, match="isn't a .zip"):
        read(b"not a zip")


def test_section_moves_headings_down_and_links_to_stored_things() -> None:
    urls = {"images/step-01.png": "https://x/1", "Bracket.bcad": "https://x/f"}
    s = bc.section(README, urls.get)
    assert s.startswith(bc.START + "\n## Build steps\n")
    assert s.endswith(bc.END)
    assert "# How to build" not in s
    assert "### Step 1: Sketch1" in s
    assert "### Parameters" in s
    assert "](https://x/1)" in s
    assert "[Bracket.bcad](https://x/f)" in s
    # Unknown links stay as they were.
    assert "](images/step-02.png)" in s
    assert "generator" not in s


def test_splice_appends_then_replaces_only_the_section() -> None:
    first = bc.splice("# My robot\n\nMy words.\n", f"{bc.START}\nold\n{bc.END}")
    assert first == f"# My robot\n\nMy words.\n\n{bc.START}\nold\n{bc.END}\n"
    second = bc.splice(first + "\nMore of mine.\n", f"{bc.START}\nnew\n{bc.END}")
    assert "old" not in second
    assert second.startswith("# My robot\n\nMy words.\n")
    assert second.endswith(f"{bc.START}\nnew\n{bc.END}\n\nMore of mine.\n")
    assert bc.splice(None, "S") == "S\n"


# --- the API ---


@pytest.fixture
async def project_id(client) -> int:
    resp = await client.post(
        "/api/projects", json={"title": "Robot arm build", "status": "draft"},
        headers=make_auth_header(),
    )
    return resp.json()["id"]


async def send(client, project_id, data=None, user="testuser"):
    return await client.post(
        f"/api/projects/{project_id}/imports",
        files={"file": ("Bracket build steps.zip", data or make_zip(), "application/zip")},
        data={"model_url": "https://bettercad.kevsrobots.com/#/m/abc"},
        headers=make_auth_header(user),
    )


@pytest.mark.asyncio
async def test_send_then_import(client, session, project_id) -> None:
    resp = await send(client, project_id)
    assert resp.status_code == 201, resp.text
    sent = resp.json()
    assert sent["status"] == "pending"
    assert (sent["title"], sent["steps"], sent["pictures"], sent["files"]) == ("Bracket", 2, 3, 2)
    assert sent["is_update"] is False

    pending = (await client.get(f"/api/projects/{project_id}/imports/pending", headers=make_auth_header())).json()
    assert pending["id"] == sent["id"]

    resp = await client.post(
        f"/api/projects/{project_id}/imports/{sent['id']}/apply",
        json={"content_md": "# Robot arm\n\nIntro.\n"}, headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    applied = resp.json()
    assert applied["pictures"] == 3 and applied["files"] == 2
    assert applied["import"]["status"] == "applied"
    text = applied["content_md"]
    assert text.startswith("# Robot arm\n\nIntro.\n")
    assert "### Step 2: Extrude1" in text
    assert f"/api/projects/{project_id}/images/" in text
    assert f"/api/projects/{project_id}/files/" in text

    images = (await session.scalars(select(ProjectImage).where(ProjectImage.project_id == project_id))).all()
    assert len(images) == 3
    assert images[0].filename.endswith("finished.png")
    files = (await session.scalars(select(ProjectFile).where(ProjectFile.project_id == project_id))).all()
    # The STL is a part of the project's 3D model too.
    assert {f.filename for f in files} == {"Bracket.bcad", "Bracket.stl", "robot-arm-build.model.json"}
    assert {f.file_type for f in files if f.filename.endswith(".stl")} == {"model"}

    nothing = await client.get(f"/api/projects/{project_id}/imports/pending", headers=make_auth_header())
    assert nothing.json() is None


@pytest.mark.asyncio
async def test_update_replaces_the_last_import(client, session, project_id) -> None:
    first = (await send(client, project_id)).json()
    applied = (await client.post(
        f"/api/projects/{project_id}/imports/{first['id']}/apply",
        json={"content_md": "Mine.\n"}, headers=make_auth_header(),
    )).json()

    second = (await send(client, project_id)).json()
    assert second["is_update"] is True
    resp = await client.post(
        f"/api/projects/{project_id}/imports/{second['id']}/apply",
        json={"content_md": applied["content_md"] + "\nAfter.\n"}, headers=make_auth_header(),
    )
    text = resp.json()["content_md"]
    assert resp.json()["import"]["is_update"] is True
    assert text.count(bc.START) == 1
    assert text.startswith("Mine.\n") and text.endswith("\nAfter.\n")
    images = (await session.scalars(select(ProjectImage).where(ProjectImage.project_id == project_id))).all()
    assert len(images) == 3
    files = (await session.scalars(select(ProjectFile).where(ProjectFile.project_id == project_id))).all()
    assert sorted(f.filename for f in files) == ["Bracket.bcad", "Bracket.stl", "robot-arm-build.model.json"]


@pytest.mark.asyncio
async def test_a_newer_send_replaces_a_pending_one(client, project_id) -> None:
    first = (await send(client, project_id)).json()
    second = (await send(client, project_id)).json()
    pending = (await client.get(f"/api/projects/{project_id}/imports/pending", headers=make_auth_header())).json()
    assert pending["id"] == second["id"]
    resp = await client.post(
        f"/api/projects/{project_id}/imports/{first['id']}/apply", json={}, headers=make_auth_header(),
    )
    assert resp.status_code == 409


@pytest.mark.asyncio
async def test_dismiss(client, project_id) -> None:
    sent = (await send(client, project_id)).json()
    resp = await client.post(f"/api/projects/{project_id}/imports/{sent['id']}/dismiss", headers=make_auth_header())
    assert resp.json()["status"] == "dismissed"
    nothing = await client.get(f"/api/projects/{project_id}/imports/pending", headers=make_auth_header())
    assert nothing.json() is None


@pytest.mark.asyncio
async def test_only_the_author_and_only_good_zips(client, project_id) -> None:
    assert (await send(client, project_id, user="someoneelse")).status_code == 403
    assert (await client.get(f"/api/projects/{project_id}/imports/pending")).status_code == 401
    bad = await send(client, project_id, data=make_zip(extra={"../x.stl": b"x"}))
    assert bad.status_code == 400
    assert "isn't allowed" in bad.json()["detail"]
    assert (await send(client, 99999)).status_code == 404


# --- the project's 3D model ---


def binary_stl(*corners: tuple[float, float, float]) -> bytes:
    """One triangle per three corners."""
    import struct
    n = len(corners) // 3
    out = bytearray(b"\0" * 80) + struct.pack("<I", n)
    for i in range(n):
        out += struct.pack("<3f", 0, 0, 1)
        for c in corners[i * 3:i * 3 + 3]:
            out += struct.pack("<3f", *c)
        out += b"\0\0"
    return bytes(out)


def test_stl_centre_binary_and_ascii() -> None:
    assert bc.stl_centre(binary_stl((0, 0, 0), (10, 0, 0), (0, 20, 4))) == [5, 10, 2]
    ascii_stl = b"solid a\nfacet normal 0 0 1\nouter loop\nvertex -2 0 0\nvertex 2 4 0\nvertex 0 0 6\nendloop\nendfacet\nendsolid a\n"
    assert bc.stl_centre(ascii_stl) == [0, 2, 3]
    assert bc.stl_centre(b"solid b\nendsolid b\n") is None


def test_model_parts_use_the_assembly_then_the_stl() -> None:
    import json
    data = make_zip(extra={
        "models/Bracket-Arm.stl": binary_stl((0, 0, 0), (10, 10, 10), (0, 0, 10)),
        "assembly.json": json.dumps({"generator": "BetterCad", "parts": [
            {"name": "Arm", "file": "models/Bracket-Arm.stl", "position": [1, 2, 30]},
            {"name": "Gone", "file": "models/missing.stl", "position": [0, 0, 0]},
        ]}).encode(),
    })
    b = read(data)
    assert "assembly.json" not in b.files
    parts = bc.model_parts(b, lambda p: f"https://x/{p}")
    by_name = {p["name"]: p for p in parts}
    assert by_name["Arm"]["position"] == [1, 2, 30]
    assert by_name["Arm"]["url"] == "https://x/models/Bracket-Arm.stl"
    assert by_name["Arm"]["file"] == "Bracket-Arm.stl"
    # No assembly entry and an empty STL: at the origin.
    assert by_name["Bracket"]["position"] == [0, 0, 0]
    assert all(p["id"].startswith(bc.PART_PREFIX) for p in parts)
    assert len({p["id"] for p in parts}) == len(parts)


def test_a_bad_assembly_is_left_out() -> None:
    b = read(make_zip(extra={"assembly.json": b"not json"}))
    assert b.assembly.parts == {} and b.assembly.joints == []


def test_merge_keeps_the_authors_parts_and_joins() -> None:
    old = {
        "version": 1, "stamp": 5, "views": [{"id": "v1"}],
        "parts": [
            {"id": "p1", "name": "Servo", "url": "u", "position": [0, 0, 0], "rotation": [0, 0, 0]},
            {"id": "bc-arm", "name": "Arm", "url": "old", "position": [9, 9, 9], "rotation": [0, 0, 90],
             "parent": "p1", "joint": {"normal": [0, 0, 1], "distance": 5}},
            {"id": "bc-old", "name": "Old", "url": "old", "position": [0, 0, 0], "rotation": [0, 0, 0]},
        ],
    }
    new = [
        {"id": "bc-arm", "name": "Arm", "url": "new", "file": "a.stl", "position": [1, 2, 3], "rotation": [0, 0, 0]},
        {"id": "bc-base", "name": "Base", "url": "new2", "file": "b.stl", "position": [4, 5, 6], "rotation": [0, 0, 0]},
    ]
    m = bc.merge_model(old, new, 3)
    assert m["stamp"] == 6
    assert m["views"] == [{"id": "v1"}]
    ids = [p["id"] for p in m["parts"]]
    assert ids == ["bc-arm", "bc-base", "p1"]
    arm = m["parts"][0]
    assert arm["url"] == "new" and arm["parent"] == "p1" and arm["position"] == [9, 9, 9]
    assert bc.merge_model(None, new, 1000)["stamp"] == 1000


def jointed_zip() -> bytes:
    import json
    box = binary_stl((0, 0, 0), (10, 10, 10), (0, 0, 10))
    return make_zip(extra={
        "models/Arm-Base.stl": box,
        "models/Arm-Upper.stl": box,
        "models/Arm-Upper pin.stl": box,
        "models/Arm-Slide.stl": box,
        "assembly.json": json.dumps({"generator": "BetterCad", "parts": [
            {"name": "Base", "file": "models/Arm-Base.stl", "position": [0, 0, 5]},
            {"name": "Upper", "file": "models/Arm-Upper.stl", "position": [0, 0, 40], "link": "l1"},
            {"name": "Upper pin", "file": "models/Arm-Upper pin.stl", "position": [5, 0, 45], "link": "l1"},
            {"name": "Slide", "file": "models/Arm-Slide.stl", "position": [0, 30, 40], "link": "l2"},
        ], "links": [{"id": "l1", "name": "Upper"}, {"id": "l2", "name": "Slide"}], "joints": [
            {"kind": "revolute", "child": "l1", "parent": None, "origin": [0, 0, 20], "axis": [0, 2, 0], "lower": -45, "upper": 400},
            {"kind": "prismatic", "child": "l2", "parent": "l1", "origin": [0, 20, 40], "axis": [0, 1, 0], "lower": 0, "upper": 30},
        ]}).encode(),
    })


def test_joints_come_over_as_the_3d_models_joints() -> None:
    parts = {p["name"]: p for p in bc.model_parts(read(jointed_zip()), lambda p: f"https://x/{p}")}
    base, upper, pin, slide = parts["Base"], parts["Upper"], parts["Upper pin"], parts["Slide"]
    assert "parent" not in base and base["position"] == [0, 0, 5]
    # The hinge: Upper hangs from Base about Y through (0, 0, 20).
    assert upper["parent"] == base["id"]
    assert upper["position"] == [0, 0, 35]
    j = upper["joint"]
    assert j["type"] == "revolute" and (j["lower"], j["upper"]) == (-45, 180)
    assert j["normal"] == [0, 1, 0] and j["childNormal"] == [0, -1, 0]
    assert j["point"] == [0, 0, 15] and j["childPoint"] == [0, 0, -20]
    # Layers puts the child at point - childPoint (no turn at home): where it is.
    assert [j["point"][k] - j["childPoint"][k] for k in range(3)] == upper["position"]
    # The component's other part is bolted to its lead part.
    assert pin["parent"] == upper["id"] and pin["position"] == [5, 0, 5] and "joint" not in pin
    # The slider hangs from the hinge's component.
    assert slide["parent"] == upper["id"] and slide["position"] == [0, 30, 0]
    assert slide["joint"]["type"] == "prismatic" and (slide["joint"]["lower"], slide["joint"]["upper"]) == (0, 30)


def test_a_new_send_keeps_the_authors_explode_but_takes_bettercads_joints() -> None:
    parts = bc.model_parts(read(jointed_zip()), lambda p: f"https://x/{p}")
    first = bc.merge_model(None, parts, 1)
    upper = next(p for p in first["parts"] if p["name"] == "Upper")
    upper["joint"]["distance"] = 99
    upper["joint"]["lower"] = -10
    upper["hidden"] = True
    again = bc.merge_model(first, bc.model_parts(read(jointed_zip()), lambda p: f"https://y/{p}"), 2)
    upper2 = next(p for p in again["parts"] if p["name"] == "Upper")
    assert upper2["joint"]["distance"] == 99 and upper2["joint"]["lower"] == -45
    assert upper2["hidden"] is True and upper2["url"].startswith("https://y/")


@pytest.mark.asyncio
async def test_import_puts_the_parts_in_the_3d_model(client, session, project_id) -> None:
    import json
    from projects_api.storage import read_file
    data = make_zip(extra={
        "models/Bracket-Arm.stl": binary_stl((0, 0, 0), (10, 10, 10), (0, 0, 10)),
        "assembly.json": json.dumps({"parts": [{"name": "Arm", "file": "models/Bracket-Arm.stl", "position": [1, 2, 30]}]}).encode(),
    })
    sent = (await send(client, project_id, data=data)).json()
    resp = await client.post(
        f"/api/projects/{project_id}/imports/{sent['id']}/apply", json={}, headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["parts"] == 2

    async def model_files():
        rows = (await session.scalars(select(ProjectFile).where(ProjectFile.project_id == project_id))).all()
        return [f for f in rows if f.filename.endswith(".model.json") and f.removed_at is None]

    [f] = await model_files()
    assert f.filename == "robot-arm-build.model.json"
    model = json.loads(read_file(f.file_path))
    arm = next(p for p in model["parts"] if p["name"] == "Arm")
    assert arm["position"] == [1, 2, 30]
    assert f"/api/projects/{project_id}/files/" in arm["url"]

    # The author adds a part of their own in a 3D scene; an update keeps it.
    model["parts"].append({"id": "p1", "name": "Servo", "position": [0, 0, 0], "rotation": [0, 0, 0]})
    from projects_api.storage import save_file
    f.file_path = save_file(json.dumps(model).encode(), "robot-arm-build.model.json", project_id, "files")
    await session.commit()

    again = (await send(client, project_id, data=data)).json()
    resp = await client.post(
        f"/api/projects/{project_id}/imports/{again['id']}/apply", json={}, headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    [f2] = await model_files()
    model2 = json.loads(read_file(f2.file_path))
    assert sorted(p["name"] for p in model2["parts"]) == ["Arm", "Bracket", "Servo"]
    assert model2["stamp"] > model["stamp"]
    # The model's parts point at files that are there.
    ids = {r.id for r in (await session.scalars(select(ProjectFile).where(ProjectFile.project_id == project_id))).all()}
    for p in model2["parts"]:
        if p["id"].startswith(bc.PART_PREFIX):
            assert int(p["url"].split("/files/")[1].split("/")[0]) in ids
