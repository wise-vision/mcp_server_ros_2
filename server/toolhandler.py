#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
from collections.abc import Sequence
from mcp.types import (
    Tool,
    TextContent,
    ImageContent,
    EmbeddedResource,
)


class ToolHandler:
    def __init__(self, tool_name: str, *, ui_only: bool = False):
        self.name = tool_name
        self.ui_only = ui_only

    def get_tool_description(self) -> Tool:
        raise NotImplementedError()

    def run_tool(
        self, args: dict
    ) -> Sequence[TextContent | ImageContent | EmbeddedResource]:
        raise NotImplementedError()
