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
from typing import Optional
from mcp.types import (
    Tool,
    TextContent,
    EmbeddedResource,
    LoggingLevel,
)

import json
from . import toolhandler
from . import ros2_manager

_ros_instance = None


def get_ros() -> ros2_manager.ROS2Manager:
    global _ros_instance
    if _ros_instance is None or not _ros_instance.node.context.ok():
        import rclpy

        if not rclpy.ok():
            raise RuntimeError(
                "rclpy is not initialized. Make sure rclpy.init() was called in main()."
            )
        _ros_instance = ros2_manager.ROS2Manager()
    return _ros_instance


class ROS2TopicList(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_topic_list")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            inputSchema={"type": "object", "properties": {}},
            description="""Returns a list of available ROS 2 topics and their types.""",
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        ros = get_ros()
        topics_list_with_types = ros.list_topics()

        return [
            TextContent(type="text", text=json.dumps(topics_list_with_types, indent=2))
        ]


class ROS2ServiceList(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_service_list")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            inputSchema={"type": "object", "properties": {}},
            description="""Returns a list of available ROS 2 services and their request fields.""",
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        ros = get_ros()
        service_list_with_types = ros.list_services()

        return [
            TextContent(type="text", text=json.dumps(service_list_with_types, indent=2))
        ]


class ROS2InterfaceList(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_interface_list")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            inputSchema={"type": "object", "properties": {}},
            description="""Returns a list of available ROS 2 interfaces.""",
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        ros = get_ros()
        interfaces_list = ros.list_interfaces()

        return [TextContent(type="text", text=json.dumps(interfaces_list, indent=2))]


class ROS2ServiceCall(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_service_call")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""Call a ROS 2 service by name and type using provided fields.
            Will ask the user to confirm if some fields are missing unless 'force_call' is set to True.
            Before **every** use of this tool, the agent must call 'ros2 service list' and 'ros2 interface list' to ensure the latest interface information is available.""",
            inputSchema={
                "type": "object",
                "properties": {
                    "service_name": {
                        "type": "string",
                        "description": "Name of the service to call",
                    },
                    "service_type": {
                        "type": "string",
                        "description": "Full ROS 2 service type, before pass, check service type using tool ros2_service_list",
                    },
                    "fields": {
                        "type": "object",
                        "description": "Dictionary of fields to send in the service request.",
                    },
                    "force_call": {
                        "type": "boolean",
                        "description": "Whether to call the service even if some fields are missing",
                        "default": False,
                    },
                },
                "required": ["service_name", "service_type", "fields"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        service_name = args.get("service_name")
        service_type = args.get("service_type")
        fields = args.get("fields")
        force_call = args.get("force_call")

        ros = get_ros()
        # Check if service exists
        available_services = [srv["service_name"] for srv in ros.list_services()]
        if service_name not in available_services:
            return {"error": f"Service '{service_name}' is not available."}
        # Get required fields
        required_fields = ros.get_request_fields(service_type)
        if "error" in required_fields:
            return {"error": required_fields["error"]}

        missing_fields = [key for key in required_fields if key not in fields]

        if missing_fields and not force_call:
            message = f"You're missing fields: {missing_fields}. "
            message += "Would you like to call the service anyway (set 'force_call' = true) or add more inputs?"
            return [TextContent(type="text", text=json.dumps(message, indent=2))]
        response = ros.call_service(service_name, service_type, fields)

        return [TextContent(type="text", text=json.dumps(response, indent=2))]


class ROS2TopicSubscribe(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_topic_subscribe")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""Subscribe to a ROS 2 topic by name collecting messages for a given time or count limit.
            Before **every** use of this tool, the agent must call 'ros2_topic_list'
            to ensure it has the latest available topics""",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {
                        "type": "string",
                        "description": "Name of the topic to subscribe to",
                    },
                    "duration": {
                        "type": "number",
                        "description": "If provided, collects messages for this many seconds.",
                    },
                    "message_limit": {
                        "type": "integer",
                        "description": "If provided, stops after receiving this number of messages.",
                    },
                },
                "required": ["topic_name", "message_type"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        topic_name = args.get("topic_name")
        duration = args.get("duration")
        message_limit = args.get("message_limit")
        if duration == "":
            duration = None
        if message_limit == "":
            message_limit = None

        ros = get_ros()
        messages = ros.subscribe_topic(
            topic_name, duration, message_limit
        )

        return [TextContent(type="text", text=json.dumps(messages, indent=2))]


# Legacy support for wisevision_data_black_box
class ROS2GetMessages(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_get_messages")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""Calls the ROS2 '/get_messages' service to retrieve past messages from a topic.""",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {
                        "type": "string",
                        "description": "Name of the topic to retrieve messages from.",
                    },
                    "message_type": {
                        "type": "string",
                        "description": "Full ROS2 message type used for decoding",
                    },
                    "number_of_messages": {
                        "type": "integer",
                        "description": "Number of messages to fetch.",
                        "default": 0,
                    },
                    "time_start": {
                        "type": "string",
                        "description": "ISO8601 timestamp string to filter messages after a point in time.",
                    },
                    "time_end": {
                        "type": "string",
                        "description": "ISO8601 timestamp string to filter messages before a point in time.",
                    },
                },
                "required": ["topic_name", "message_type"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        topic_name = args.get("topic_name")
        message_type = args.get("message_type")
        number_of_messages = args.get("number_of_messages")
        time_start = args.get("time_start")
        time_end = args.get("time_end")

        ros = get_ros()
        response = ros.call_get_messages_service_any(
            {
                "topic_name": topic_name,
                "message_type": message_type,
                "number_of_msgs": number_of_messages,
                "time_start": time_start,
                "time_end": time_end,
            }
        )
        return [TextContent(type="text", text=json.dumps(response, indent=2))]


class ROS2GetMessageFields(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_get_message_fields")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="Returns the fields of a given ROS2 message type.",
            inputSchema={
                "type": "object",
                "properties": {
                    "message_type": {
                        "type": "string",
                        "description": "Full ROS2 message type, e.g., std_msgs/msg/String",
                    },
                },
                "required": ["message_type"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        message_type = args.get("message_type")

        ros = get_ros()
        request_fields = ros.get_request_fields(message_type)
        return [TextContent(type="text", text=json.dumps(request_fields, indent=2))]


class ROS2TopicPublish(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_topic_publish")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""Publish a message to a ROS 2 topic by name and message type using provided field values.
            Before **every** use of this tool, the agent must call 'ros2_topic_list' and 'ros2_interface_list'
            to ensure the latest available topics and message types are known.""",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {
                        "type": "string",
                        "description": "Name of the topic to publish to",
                    },
                    "message_type": {
                        "type": "string",
                        "description": "Full ROS 2 message type, e.g., 'std_msgs/msg/String'",
                    },
                    "data": {
                        "type": "object",
                        "description": "Dictionary containing the message fields and values",
                    },
                },
                "required": ["topic_name", "message_type", "data"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        topic_name = args.get("topic_name")
        message_type = args.get("message_type")
        data = args.get("data")

        ros = get_ros()
        publish_to_topic = ros.publish_to_topic(topic_name, message_type, data)
        return [TextContent(type="text", text=json.dumps(publish_to_topic, indent=2))]
