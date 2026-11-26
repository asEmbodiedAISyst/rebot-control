#!/usr/bin/env python
"""Build the documentation site.

Output (default ``docs/_build/html``)::

    index.html                redirect to en/
    en/, zh/                  the two Sphinx sites
    llms.txt, llms-full.txt   page index and full text for agents (English); the Chinese files are in zh/

Usage: ``python docs/build.py [--out DIR] [--base-url URL]``. The base URL (or ``DOCS_BASE_URL``) makes the links
in ``llms.txt`` absolute. A Sphinx warning stops the build.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

DOCS = Path(__file__).resolve().parent
SITES = {"en": DOCS, "zh": DOCS / "zh"}
SECTION_TITLES = {
    "en": {"user": "User guide", "developer": "Developer guide", "agents": "AI agents"},
    "zh": {"user": "用户指南", "developer": "开发者指南", "agents": "AI agent"},
}
SUMMARY = {
    "en": "Cartesian control, configurable tools and self-calibration for the Seeed reBot Arm B601-RS, with a "
          "MuJoCo simulator that runs the same control code.",
    "zh": "面向 Seeed reBot Arm B601-RS 的笛卡尔控制、可配置工具与自标定库，附带运行同一套控制代码的 MuJoCo 仿真器。",
}
SECTIONS = ("user", "developer", "agents")
_TOCTREE = re.compile(r"```\{toctree\}\n(.*?)```", re.S)


def strip_toctrees(text: str) -> str:
    """The page text without its toctree blocks (they are navigation, not content)."""
    return _TOCTREE.sub("", text).rstrip() + "\n"


def pages(root: Path) -> list[tuple[str, str]]:
    """(path, title) of every page: the front page, then each section's index and its pages in toctree order."""
    paths = ["index.md"]
    for section in SECTIONS:
        index = (root / section / "index.md").read_text(encoding="utf-8")
        listed = [line.strip() for block in _TOCTREE.findall(index) for line in block.splitlines()]
        listed = [f"{section}/{name}.md" for name in listed if name and not name.startswith(":")]
        others = sorted(f"{section}/{p.name}" for p in (root / section).glob("*.md"))
        for path in [f"{section}/index.md", *listed, *others]:
            if path not in paths:
                paths.append(path)
    out = []
    for path in paths:
        text = (root / path).read_text(encoding="utf-8")
        title = next((line[2:].strip() for line in text.splitlines() if line.startswith("# ")), path)
        out.append((path, title))
    return out


REDIRECT = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <title>rebot-control</title>
    <meta http-equiv="refresh" content="0; url=en/">
    <link rel="canonical" href="en/">
  </head>
  <body>
    <p><a href="en/">English</a> | <a href="zh/">中文</a></p>
  </body>
</html>
"""


def sphinx(source: Path, out: Path, doctrees: Path) -> None:
    cmd = [sys.executable, "-m", "sphinx", "-b", "html", "-W", "--keep-going", "-q", "-d", str(doctrees),
           str(source), str(out)]
    subprocess.run(cmd, check=True)


def write_llms(language: str, site: Path, site_url: str) -> None:
    """``llms.txt`` (index with links to the Markdown sources) and ``llms-full.txt`` (all pages in one file)."""
    root = SITES[language]
    index = [f"# rebot-control\n\n> {SUMMARY[language]}"]
    for section in SECTIONS:
        index.append(f"\n## {SECTION_TITLES[language][section]}\n")
        for path, title in pages(root):
            if path.startswith(f"{section}/") and not path.endswith("/index.md"):
                index.append(f"- [{title}]({site_url}_sources/{path}.txt)")
    (site / "llms.txt").write_text("\n".join(index) + "\n", encoding="utf-8")

    full = [f"<!-- {path} -->\n\n{strip_toctrees((root / path).read_text(encoding='utf-8'))}" for path, _ in pages(root)]
    (site / "llms-full.txt").write_text("\n".join(full), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=DOCS / "_build" / "html")
    parser.add_argument("--base-url", default=os.environ.get("DOCS_BASE_URL", ""),
                        help="public URL of the site root, for absolute links in llms.txt")
    args = parser.parse_args()
    out: Path = args.out.resolve()
    base = args.base_url.rstrip("/")
    os.environ["DOCS_BASE_URL"] = base  # conf.py reads it for html_baseurl

    for language, source in SITES.items():
        sphinx(source, out / language, out.parent / "doctrees" / language)
        write_llms(language, out / language, f"{base}/{language}/" if base else "")

    # The English llms files are also at the site root, where crawlers look for them.
    for name in ("llms.txt", "llms-full.txt"):
        text = (out / "en" / name).read_text(encoding="utf-8")
        if not base:
            text = text.replace("](_sources/", "](en/_sources/")
        (out / name).write_text(text, encoding="utf-8")
    (out / "index.html").write_text(REDIRECT, encoding="utf-8")
    (out / ".nojekyll").write_text("", encoding="utf-8")
    print(f"documentation built in {out}")


if __name__ == "__main__":
    main()
