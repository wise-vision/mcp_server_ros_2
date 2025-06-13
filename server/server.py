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

import atexit
from typing import Any, Optional, Dict
import os

import rclpy
from pydantic import BaseModel
from mcp.server.fastmcp import FastMCP

from server.ros2_manager import ROS2Manager

# Use the port from the environment variable if available
port = int(os.getenv("MCP_SERVER_PORT", 3333))

# Initialize MCP server with the dynamic port
mcp = FastMCP("MyServer", port=port)


# Init ROS 2 once globally
rclpy.init()


# Gracefully shutdown when MCP server stops
@atexit.register
def shutdown_ros():
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        print(f"ROS shutdown skipped: {e}")


# Create instance of the service manager
ros = ROS2Manager()


@mcp.tool()
def ros2_topic_list() -> list:
    """
    Returns a list of available ROS 2 topics and their types.

    Accepts an 'input' dictionary to comply with Studio schema.
    """
    return ros.list_topics()


@mcp.tool()
def ros2_service_list() -> list:
    """
    Returns a list of available ROS 2 services and their request fields.

    Accepts an 'input' dictionary to comply with Studio schema.
    """
    return ros.list_services()


@mcp.tool()
def ros2_interface_list() -> list:
    """
    Returns a list of available ROS 2 interfaces.

    Accepts an 'input' dictionary to comply with Studio schema.
    """
    return ros.list_services()


# MCP Tool to call any ROS2 service
class ServiceCallInput(BaseModel):
    service_name: str
    service_type: str
    fields: Dict[str, Any]
    force_call: bool = False  # Default to False


@mcp.tool()
def ros2_service_call(input: ServiceCallInput) -> dict:
    """
    Call a ROS 2 service by name and type using provided fields.
    Will ask the user to confirm if some fields are missing unless 'force_call' is set to True.
    ⚠️ Before **every** use of this tool, the agent must call 'ros2 service list' and 'ros2 interface list' to ensure the latest interface information is available.


    Parameters (wrapped inside 'input'):
    - input (ServiceCallInput): A dictionary with the following keys:
        - service_name (str): Required. Name of the service to call.
        - service_type (str): Required. Full ROS 2 service type, e.g., 'example_interfaces/srv/AddTwoInts'.
        - fields (dict): Required. Dictionary of fields to send in the service request.
        - force_call (bool): Optional. If True, will call the service even if some fields are missing. Default is False.

    Note:
    This tool depends on current service and interface definitions. The agent is expected to refresh them
    by calling 'ros2 service list' and 'ros2 interface list' before **every** service call.
    """
    service_name = input.service_name
    service_type = input.service_type
    fields = input.fields
    force_call = input.force_call

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
    response = ros.call_service(service_name, service_type, fields)

    # Ensure the response has the expected structure
    response["Information for agent"] = (
        "To call any service again, run again ros2_service_list and ros2_interface_list, then call ros2_service_call with the service name, type, and fields."
    )
    return response


class TopicSubscribeInput(BaseModel):
    topic_name: str
    msg_type: str
    duration: Optional[float] = None
    message_limit: Optional[int] = None


@mcp.tool()
def ros2_topic_subscribe(input: TopicSubscribeInput) -> dict:
    """
    Subscribe to a ROS 2 topic by name and message type, collecting messages for a given time or count limit.

    ⚠️ Before **every** use of this tool, the agent must call 'ros2_topic_list' and 'ros2_interface_list'
    to ensure it has the latest available topics and message types.

    Parameters:
    - topic_name (str): Required. Name of the topic to subscribe to.
    - msg_type (str): Required. Full ROS 2 message type, e.g., 'std_msgs/msg/String'.
    - duration (float, optional): If provided, collects messages for this many seconds.
    - message_limit (int, optional): If provided, stops after receiving this number of messages.

    Note:
    This tool depends on the current list of topics and message interfaces. The agent is expected to refresh them
    by calling 'ros2_topic_list' and 'ros2_interface_list' before **every** subscription.
    """
    topic_name = input.topic_name
    msg_type = input.msg_type
    duration = input.duration
    message_limit = input.message_limit
    # Convert empty strings to None
    if duration == "":
        duration = None
    if message_limit == "":
        message_limit = None

    return ros.subscribe_topic(topic_name, msg_type, duration, message_limit)


class GetMessagesInput(BaseModel):
    topic_name: str
    message_type: str
    number_of_msgs: Optional[int] = 0
    time_start: Optional[str] = None
    time_end: Optional[str] = None


@mcp.tool()
def ros2_get_messages(input: GetMessagesInput) -> dict:
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

    """

    topic_name = input.topic_name
    message_type = input.message_type
    number_of_msgs = input.number_of_msgs
    time_start = input.time_start
    time_end = input.time_end

    return ros.call_get_messages_service_any(
        {
            "topic_name": topic_name,
            "message_type": message_type,
            "number_of_msgs": number_of_msgs,
            "time_start": time_start,
            "time_end": time_end,
        }
    )


class GetMessageFieldsInput(BaseModel):
    message_type: str


@mcp.tool()
def ros2_get_message_fields(input: GetMessageFieldsInput) -> dict:
    """
    Returns the fields of a given ROS2 message type.

    Parameters:
    - message_type: Full ROS2 message type, e.g., std_msgs/msg/String
    """
    message_type = input.message_type
    return ros.get_request_fields(message_type)


class TopicPublishInput(BaseModel):
    topic_name: str
    message_type: str
    data: Dict[str, Any]


@mcp.tool("ros2_topic_publish")
def ros2_topic_publish(input: TopicPublishInput) -> dict:
    """
    Publish a message to a ROS 2 topic by name and message type using provided field values.

    ⚠️ Before **every** use of this tool, the agent must call 'ros2_topic_list' and 'ros2_interface_list'
    to ensure the latest available topics and message types are known.

    Parameters:
    - topic_name (str): Required. Name of the topic to publish to.
    - message_type (str): Required. Full ROS 2 message type, e.g., 'std_msgs/msg/String'.
    - data (dict): Required. Dictionary containing the message fields and values.

    Note:
    This tool depends on the current list of topics and message interfaces. The agent is expected to refresh them
    by calling 'ros2_topic_list' and 'ros2_interface_list' before **every** publish action.
    """
    topic_name = input.topic_name
    message_type = input.message_type
    data = input.data

    return ros.publish_to_topic(topic_name, message_type, data)


class TopicEchoInput(BaseModel):
    topic_name: str
    msg_type: str
    timeout: Optional[float] = 5.0


@mcp.tool()
def ros2_topic_echo_wait(input: TopicEchoInput) -> dict:
    """
    Wait for a single message from a ROS 2 topic and return its contents.

    ⚠️ Before **every** use of this tool, the agent must call 'ros2_topic_list' and 'ros2_interface_list'
    to ensure it has the latest list of available topics and message types.

    Parameters:
    - topic_name (str): Required. Name of the topic to listen to.
    - msg_type (str): Required. Full ROS 2 message type, e.g., 'std_msgs/msg/String'.
    - timeout (float, optional): Maximum time to wait for a message in seconds (default: 5.0).

    Note:
    This tool depends on current topic and message definitions. The agent is expected to refresh them
    by calling 'ros2_topic_list' and 'ros2_interface_list' before **every** call.
    """
    topic_name = input.topic_name
    msg_type = input.msg_type
    timeout = input.timeout

    if timeout in [None, "", "null", "undefined"]:
        timeout = 5.0
    return ros.echo_topic_once(topic_name, msg_type, timeout)
