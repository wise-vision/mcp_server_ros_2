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
import json

from mcp.types import EmbeddedResource, ImageContent, TextContent, TextResourceContents, Tool

from . import toolhandler
from .ros2_viewer_app import get_viewer_html
from .tools_ros2 import get_ros


class ROS2ViewerApp(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_viewer_app")

    def get_tool_description(self) -> Tool:
        return Tool(
            name=self.name,
            description="Open an interactive ROS 2 viewer app (Image + PointCloud2) for MCP clients that support Apps/HTML resources.",
            _meta={"ui": {"resourceUri": "ui://ros2-viewer/app"}},
            inputSchema={
                "type": "object",
                "properties": {
                    "auto_start": {"type": "boolean", "default": True},
                    "preferred_kind": {"type": "string", "enum": ["image", "pointcloud", "auto"]},
                    "topic_name": {"type": "string"},
                },
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:
        config: dict[str, object] = {}
        if isinstance(args, dict):
            if "auto_start" in args:
                config["autoStart"] = bool(args.get("auto_start"))
            else:
                config["autoStart"] = True
            if "preferred_kind" in args:
                config["preferredKind"] = str(args.get("preferred_kind"))
            if "topic_name" in args:
                config["topicName"] = str(args.get("topic_name"))
        html = get_viewer_html(config if config else None)
        return [
            EmbeddedResource(
                type="resource",
                resource=TextResourceContents(
                    uri="ui://ros2-viewer/app",
                    mimeType="text/html;profile=mcp-app",
                    text=html,
                ),
            ),
            TextContent(
                type="text",
                text=(
                    "ROS 2 Viewer App returned as an embedded HTML resource. "
                    "If your client supports MCP Apps, open the resource to view and stream Image/PointCloud2 topics."
                ),
            ),
        ]


class ROS2StreamStart(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_stream_start", ui_only=True)

    def get_tool_description(self) -> Tool:
        return Tool(
            name=self.name,
            description="Start a keep-latest stream session for a ROS 2 topic (Image/CompressedImage/PointCloud2). Returns a session_id. UI/App-only tool.",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {"type": "string"},
                    "kind": {"type": "string", "enum": ["image", "pointcloud"]},
                    "target_fps": {"type": "number", "description": "Soft cap for processing frames (keep-latest, drops when over).", "default": 20.0},
                    "max_width": {"type": "integer", "default": 960},
                    "max_height": {"type": "integer", "default": 540},
                    "jpeg_quality": {"type": "integer", "default": 80},
                    "max_points": {
                        "type": "integer",
                        "default": 0,
                        "description": "Max points to return; <=0 means no limit.",
                    },
                    "qos_preset": {
                        "type": "string",
                        "enum": ["auto", "sensor_data", "system_default"],
                        "default": "auto",
                        "description": "QoS preset for the stream subscription. Use sensor_data for most camera topics.",
                    },
                },
                "required": ["topic_name", "kind"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent]:
        ros = get_ros()
        topic_name = str(args.get("topic_name"))
        kind = str(args.get("kind"))
        target_fps = args.get("target_fps", None)

        max_width = args.get("max_width", None)
        max_height = args.get("max_height", None)
        jpeg_quality = args.get("jpeg_quality", 80)
        max_points = args.get("max_points", None)
        qos_preset = args.get("qos_preset", "auto")

        resp = ros.start_stream(
            topic_name=topic_name,
            kind=kind,
            target_fps=target_fps,
            max_width=max_width,
            max_height=max_height,
            jpeg_quality=jpeg_quality,
            max_points=max_points,
            qos_preset=qos_preset,
        )
        return [TextContent(type="text", text=json.dumps(resp, indent=2))]


class ROS2StreamNext(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_stream_next", ui_only=True)

    def get_tool_description(self) -> Tool:
        return Tool(
            name=self.name,
            description="Fetch the latest item from a stream session. If after_seq is provided, returns available=false until seq increases. UI/App-only tool.",
            inputSchema={
                "type": "object",
                "properties": {
                    "session_id": {"type": "string"},
                    "after_seq": {"type": "integer"},
                },
                "required": ["session_id"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent]:
        ros = get_ros()
        session_id = str(args.get("session_id"))
        after_seq = args.get("after_seq", None)
        resp = ros.stream_next(session_id=session_id, after_seq=after_seq)
        return [TextContent(type="text", text=json.dumps(resp, indent=2))]


class ROS2StreamNextImage(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_stream_next_image", ui_only=True)

    def get_tool_description(self) -> Tool:
        return Tool(
            name=self.name,
            description="Fetch the latest image from an image stream session and return it as ImageContent (useful when the client cannot render the Viewer App UI). UI/App-only tool.",
            inputSchema={
                "type": "object",
                "properties": {
                    "session_id": {"type": "string"},
                    "after_seq": {"type": "integer"},
                },
                "required": ["session_id"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | ImageContent]:
        ros = get_ros()
        session_id = str(args.get("session_id"))
        after_seq = args.get("after_seq", None)
        resp = ros.stream_next(session_id=session_id, after_seq=after_seq)
        if resp.get("error"):
            return [TextContent(type="text", text=json.dumps(resp, indent=2))]
        if not resp.get("available"):
            return [TextContent(type="text", text=json.dumps(resp, indent=2))]
        if resp.get("mimeType") and resp.get("data"):
            meta = {k: v for k, v in resp.items() if k not in ("data",)}
            return [
                ImageContent(type="image", data=resp["data"], mimeType=resp["mimeType"]),
                TextContent(type="text", text=json.dumps(meta, indent=2)),
            ]
        return [TextContent(type="text", text=json.dumps(resp, indent=2))]


class ROS2StreamStop(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_stream_stop", ui_only=True)

    def get_tool_description(self) -> Tool:
        return Tool(
            name=self.name,
            description="Stop and clean up a stream session previously started with ros2_stream_start. UI/App-only tool.",
            inputSchema={
                "type": "object",
                "properties": {"session_id": {"type": "string"}},
                "required": ["session_id"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent]:
        ros = get_ros()
        session_id = str(args.get("session_id"))
        resp = ros.stop_stream(session_id=session_id)
        return [TextContent(type="text", text=json.dumps(resp, indent=2))]
