"""Sphinx configuration of the English site. ``docs/zh/conf.py`` loads this file and overrides the language."""

from __future__ import annotations

import os
from importlib import metadata

project = "rebot-control"
author = "Jinqi Wei"
copyright = "2026, Jinqi Wei"
try:
    release = metadata.version("rebot-control")
except metadata.PackageNotFoundError:  # the package is not installed: build without a version
    release = ""
version = release
language = "en"

extensions = ["myst_parser", "autodoc2", "sphinx_copybutton"]
source_suffix = {".md": "markdown", ".rst": "restructuredtext"}
root_doc = "index"
# zh/ is a separate Sphinx project. internal/ holds local notes that are never published.
exclude_patterns = ["_build", "_templates", "internal", "zh"]
templates_path = ["_templates"]

myst_heading_anchors = 3

# API reference, generated from the docstrings without importing the package.
autodoc2_packages = [{"path": "../src/rebot_control", "exclude_dirs": ["assets", "configs"]}]
autodoc2_output_dir = "apidocs"
autodoc2_hidden_objects = ["private", "inherited"]
suppress_warnings = ["autodoc2.dup_item"]

# -- HTML output ------------------------------------------------------------------------------------------------

# The docs workflow sets these two variables. A local build needs neither.
_repository = os.environ.get("GITHUB_REPOSITORY", "asEmbodiedAISyst/rebot-control")
_base_url = os.environ.get("DOCS_BASE_URL", "").rstrip("/")

html_theme = "furo"
html_title = f"rebot-control {release}".strip()
html_baseurl = f"{_base_url}/en/" if _base_url else ""
html_copy_source = True  # llms.txt links to the Markdown sources under _sources/
html_theme_options = {
    "source_repository": f"https://github.com/{_repository}",
    "source_branch": "main",
    "source_directory": "docs/",
    "top_of_page_buttons": ["view"],
}
html_sidebars = {
    "**": [
        "sidebar/brand.html",
        "sidebar/search.html",
        "sidebar/scroll-start.html",
        "sidebar/navigation.html",
        "sidebar/language.html",
        "sidebar/scroll-end.html",
    ]
}
# Link to the same page of the other language (see _templates/sidebar/language.html).
html_context = {"other_language_label": "中文", "other_language_root": "../zh/"}
