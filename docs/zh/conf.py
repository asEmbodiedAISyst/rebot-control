"""Sphinx configuration of the Chinese site: the English configuration with a few overrides."""

from __future__ import annotations

import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location("_english_conf", Path(__file__).resolve().parent.parent / "conf.py")
_english = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_english)
globals().update({k: v for k, v in vars(_english).items() if not k.startswith("_")})

language = "zh_CN"
# The API reference exists in English only: the front page links to it.
extensions = [e for e in _english.extensions if e != "autodoc2"]
exclude_patterns = ["_build"]
templates_path = ["../_templates"]

html_baseurl = _english.html_baseurl.replace("/en/", "/zh/")
html_theme_options = {**_english.html_theme_options, "source_directory": "docs/zh/"}
html_context = {"other_language_label": "English", "other_language_root": "../en/"}
