"""The Chinese documentation mirrors the English documentation page by page."""

import re
from pathlib import Path

import pytest

DOCS = Path(__file__).resolve().parents[1] / "docs"
SECTIONS = ("user", "developer", "agents")

pytestmark = pytest.mark.skipif(not (DOCS / "zh" / "index.md").is_file(), reason="the documentation is not here")


def pages(root: Path) -> list[str]:
    return ["index.md"] + sorted(f"{s}/{p.name}" for s in SECTIONS for p in (root / s).glob("*.md"))


def structure(path: Path) -> tuple[int, int]:
    """Number of headings (outside code blocks) and of code blocks on a page."""
    headings = fences = 0
    in_code = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if re.match(r"^\s*(```|~~~)", line):
            in_code = not in_code
            fences += in_code
        elif not in_code and re.match(r"^#{1,6}\s", line):
            headings += 1
    return headings, fences


def test_same_pages_in_both_languages():
    assert pages(DOCS / "zh") == pages(DOCS)
    assert len(pages(DOCS)) > 20


@pytest.mark.parametrize("page", pages(DOCS))
def test_same_structure_in_both_languages(page):
    assert structure(DOCS / "zh" / page) == structure(DOCS / page)
