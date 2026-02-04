#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
from __future__ import annotations

from copy import deepcopy
from importlib import resources
import json

_LAST_VIEWER_CONFIG: dict | None = None


def _read_viewer_html() -> str:
    """
    Load the Viewer App HTML from a packaged resource.

    The UI is built by esbuild (npm run build) which bundles src/*.tsx
    into index.html. The MCP tool returns it as a single embedded HTML resource.
    """
    base = resources.files("server").joinpath("ui", "ros2_viewer_app")
    return base.joinpath("index.html").read_text(encoding="utf-8")


def _set_last_viewer_config(config: dict | None) -> None:
    global _LAST_VIEWER_CONFIG
    _LAST_VIEWER_CONFIG = deepcopy(config) if config else None


def get_last_viewer_config() -> dict | None:
    return deepcopy(_LAST_VIEWER_CONFIG) if _LAST_VIEWER_CONFIG else None


def get_viewer_html(config: dict | None = None) -> str:
    if config:
        _set_last_viewer_config(config)
    html = _read_viewer_html()
    if not config:
        return html
    try:
        payload = json.dumps(config, separators=(",", ":"))
    except TypeError:
        return html
    snippet = f"<script>window.__ROS2_VIEWER_CONFIG__={payload};</script>"
    idx = html.find("<script")
    if idx == -1:
        return html + snippet
    return html[:idx] + snippet + html[idx:]

def get_viewer_html_with_last_config() -> str:
    config = get_last_viewer_config()
    return get_viewer_html(config) if config else _read_viewer_html()


ROS2_VIEWER_APP_HTML = _read_viewer_html()
