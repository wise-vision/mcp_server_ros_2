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

from importlib import resources
import json


def _read_viewer_html() -> str:
    """
    Load the Viewer App HTML from a packaged resource.

    The UI is built by esbuild (npm run build) which bundles src/*.tsx
    into index.html. The MCP tool returns it as a single embedded HTML resource.
    """
    base = resources.files("server").joinpath("ui", "ros2_viewer_app")
    return base.joinpath("index.html").read_text(encoding="utf-8")


def get_viewer_html(config: dict | None = None) -> str:
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


ROS2_VIEWER_APP_HTML = _read_viewer_html()
