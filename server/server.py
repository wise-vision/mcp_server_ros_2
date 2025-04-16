#!/usr/bin/env python3
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

from mcp.server.fastmcp import FastMCP
import rclpy
import atexit
from typing import Any, Optional
from server.ros2_manager import ROS2Manager
import os
from starlette.requests import Request
from starlette.routing import Mount, Route
from mcp.server import Server
import uvicorn
from starlette.applications import Starlette
from mcp.server.sse import SseServerTransport

# Use the port from the environment variable if available
port = int(os.getenv("MCP_SERVER_PORT", 3333))

# Initialize MCP server with the dynamic port
mcp = FastMCP("MyServer", port=port)


#Init ROS 2 once globally
rclpy.init()

#Gracefully shutdown when MCP server stops
@atexit.register
def shutdown_ros():
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        print(f"ROS shutdown skipped: {e}")

#Create instance of the service manager
ros = ROS2Manager()

@mcp.tool("ros2_topic_list")
def list_all_topics(input: Optional[dict] = None) -> list:
    """
    Returns a list of available ROS 2 topics and their types.

    Accepts an 'input' dictionary to comply with Studio schema.
    """
    return ros.list_topics()

@mcp.tool("ros2_service_list")
def list_all_services(input: Optional[dict] = None) -> list:
    """
    Returns a list of available ROS 2 services and their request fields.

    Accepts an 'input' dictionary to comply with Studio schema.
    """
    return ros.list_services()

# MCP Tool to call any ROS2 service
@mcp.tool("ros2_service_call")
def call_ros2_service(
    service_name: str,
    service_type: str,
    fields: dict,
    force_call: Optional[bool] = False,
) -> dict:
    """
    Call a ROS 2 service by name and type using provided fields.
    Will ask the user to confirm if some fields are missing unless 'force_call' is set to True.
    """

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
        return {
            "message": (
                f"You're missing fields: {missing_fields}. "
                "Would you like to call the service anyway (set 'force_call' = true) or add more inputs?"
            )
        }

    # Call the actual service
    return ros.call_service(service_name, service_type, fields)

@mcp.tool("ros2-topic-subscribe")
def subscribe_to_topic(
    topic_name: str,
    msg_type: str,
    duration: Optional[float] = None,
    message_limit: Optional[int] = None,
) -> dict:
    """
    Subscribes to a ROS 2 topic and collects messages either for a duration or a message limit.
    """
    # Convert empty strings to None
    if duration == "":
        duration = None
    if message_limit == "":
        message_limit = None

    return ros.subscribe_topic(topic_name, msg_type, duration, message_limit)

@mcp.tool("ros2-get-messages")
def ros2_get_messages(
    topic_name: str,
    message_type: str,
    number_of_msgs: Optional[int] = 0,
    time_start: Optional[str] = None,
    time_end: Optional[str] = None
) -> dict:
    """
    Calls the ROS2 '/get_messages' service to retrieve past messages from a topic.

    Parameters:
    - topic_name (str): equired. Name of the topic to retrieve messages from.
    - message_type (str): Required. Full ROS2 message type used for decoding
      (e.g. 'std_msgs/msg/String', 'custom_msgs/msg/MyMsg').
    - number_of_msgs (int): Optional. Number of messages to fetch. Default is 0 (fetch all available).
      Alias note: Some systems may call this 'count'.
    - time_start (str): Optional. ISO8601 timestamp string to filter messages after a point in time.
    - time_end (str): Optional. ISO8601 timestamp string to filter messages before a point in time.

    Common pitfalls:
    - Make sure to use 'topic_name' (not just 'topic')
    - Use 'number_of_msgs', not 'count' or 'number_of_messages'
    - message_type (str): Required. Full ROS2 message type used to deserialize the data
      (e.g. 'std_msgs/msg/String', 'custom_msgs/msg/MyMsg').
      This must exactly match the type published on the topic.
      
      AI agents or automation systems should **not guess** this field. If unknown, the user must provide it explicitly,
      or the system may fail to deserialize the message.
    - Time filters must be ISO 8601 (e.g. '2024-04-07T12:00:00Z')

    Example usage:
    {
        "topic_name": "/sensor/data",
        "message_type": "my_msgs/msg/SensorData",
        "number_of_msgs": 10,
        "time_start": "2024-04-06T10:00:00Z"
    }
    """

    return ros.call_get_messages_service_any({
        "topic_name": topic_name,
        "message_type": message_type,
        "number_of_msgs": number_of_msgs,
        "time_start": time_start,
        "time_end": time_end
    })


def main():
    mcp.run(transport="stdio")

if __name__ == "__main__":
    main()