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

import rclpy
from rclpy.node import Node
import importlib
from typing import Any, Optional
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import get_interfaces
from rosidl_runtime_py.utilities import get_service, get_message
from dateutil import parser
import numpy as np
import array
import time
from rclpy.task import Future
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor

QOS_DEPTH=1_000


class ServiceNode(Node):
    def __init__(self):
        super().__init__("mcp_service_lister")


class ROS2Manager:
    def __init__(self):
        self.node = ServiceNode()

    def get_qos_profile_for_topic(self, topic_name: str) -> QoSProfile:
        infos = self.node.get_publishers_info_by_topic(topic_name)

        if not infos:
            raise RuntimeError(f"No publisher found on topic '{topic_name}'.")

        pub_info = infos[0]
        qos = pub_info.qos_profile

        history = qos.history if qos.history in (
            HistoryPolicy.KEEP_LAST, HistoryPolicy.KEEP_ALL
        ) else HistoryPolicy.KEEP_LAST

        depth = QOS_DEPTH

        reliability = qos.reliability if qos.reliability in (
            ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT
        ) else ReliabilityPolicy.RELIABLE

        durability = qos.durability if qos.durability in (
            DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL
        ) else DurabilityPolicy.VOLATILE

        return QoSProfile(
            history=history,
            depth=depth,
            reliability=reliability,
            durability=durability
        )

    def list_topics(self):
        topic_names_and_types = self.node.get_topic_names_and_types()
        result = []
        for name, types in topic_names_and_types:
            topic_type = types[0] if types else "unknown"
            request_fields = self.get_request_fields(topic_type)
            result.append(
                {
                    "topic_name": name,
                    "topic_type": topic_type,
                    "request_fields": request_fields,
                }
            )
        return result

    def list_services(self):
        service_list = self.node.get_service_names_and_types()
        result = []
        for name, types in service_list:
            service_type = types[0] if types else "unknown"
            request_fields = self.get_request_fields(service_type)
            result.append(
                {
                    "service_name": name,
                    "service_type": service_type,
                    "request_fields": request_fields,
                }
            )
        return result

    def list_interfaces(self):
        interfaces = get_interfaces()
        result = []
        for pkg_name, iface_list in interfaces.items():
            for iface in iface_list:
                # iface like "msg/String" or "srv/SetBool"
                result.append(f"{pkg_name}/{iface}")

        return result

    def get_request_fields(self, ros_type: str):
        try:
            parts = ros_type.split("/")
            if len(parts) == 3:
                pkg, kind, name = parts
            elif len(parts) == 2:
                pkg, name = parts
                kind = "msg"
            else:
                return {"error": f"Invalid type format: {ros_type}"}

            if kind == "msg":
                module = importlib.import_module(f"{pkg}.msg")
                msg_class = getattr(module, name)
                return msg_class.get_fields_and_field_types()

            elif kind == "srv":
                module = importlib.import_module(f"{pkg}.srv")
                srv_class = getattr(module, name)
                return srv_class.Request.get_fields_and_field_types()

            else:
                return {"error": f"Unsupported ROS type kind: {kind}"}

        except Exception as e:
            return {"error": f"Failed to load {ros_type}: {str(e)}"}

    def call_service(self, service_name: str, service_type: str, fields: dict) -> dict:
        try:
            parts = service_type.split("/")
            if len(parts) == 2:
                pkg, srv = parts
            elif len(parts) == 3:
                pkg, _, srv = parts
            else:
                return {"error": f"Invalid service type format: {service_type}"}
            module = importlib.import_module(f"{pkg}.srv")
            srv_class = getattr(module, srv)
            client = self.node.create_client(srv_class, service_name)

            if not client.wait_for_service(timeout_sec=3.0):
                return {"error": f"Service '{service_name}' not available (timed out)."}

            request = self.fill_ros_message(srv_class.Request, fields)

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None:
                return {"result": str(future.result())}
            else:
                return {"error": "Service call failed."}
        except Exception as e:
            return {"error": str(e)}

    def serialize_msg(self, msg):
        try:
            if isinstance(msg, memoryview):
                try:
                    return list(msg.cast('d'))
                except TypeError:
                    return list(msg)

            elif isinstance(msg, (bytes, bytearray)):
                return list(msg)

            elif isinstance(msg, (int, float, str, bool)) or msg is None:
                return msg

            elif hasattr(msg, "data"):
                return self.serialize_msg(msg.data)

            elif isinstance(msg, (list, tuple)):
                return [self.serialize_msg(item) for item in msg]

            elif hasattr(msg, "__slots__"):
                return {
                    slot: self.serialize_msg(getattr(msg, slot))
                    for slot in msg.__slots__
                }

            elif isinstance(msg, dict):
                return {
                    key: self.serialize_msg(value)
                    for key, value in msg.items()
                }

            else:
                return str(msg)

        except Exception as e:
            return {"error": f"Failed to serialize message: {str(e)}"}

    def subscribe_topic(
        self,
        topic_name: str,
        duration: Optional[float] = None,
        message_limit: Optional[int] = None,
    ) -> dict:
        import importlib
        import time
        from rclpy.task import Future

        available_topics = self.node.get_topic_names_and_types()
        topic_map = {name: types for name, types in available_topics}
        if topic_name not in topic_map:
            return {
                "error": f"Topic '{topic_name}' not found. Available topics: {list(topic_map.keys())}"
            }

        types = topic_map[topic_name]
        if not types:
            return {"error": f"Topic '{topic_name}' has no associated message types."}

        msg_type = types[0]

        # Fallback to avoid infinite wait
        if not duration and not message_limit:
            duration = 5.0  # default duration in seconds

        # Dynamically load message class
        parts = msg_type.split("/")
        if len(parts) == 3:
            pkg, _, msg = parts
        elif len(parts) == 2:
            pkg, msg = parts
        else:
            return {"error": f"Invalid message type format: {msg_type}"}

        try:
            module = importlib.import_module(f"{pkg}.msg")
            msg_class = getattr(module, msg)
        except Exception as e:
            return {"error": f"Failed to import message type: {str(e)}"}

        tmp_node = Node("mcp_subscribe_tmp")
        received = []
        done_future = Future()
        qos = self.get_qos_profile_for_topic(topic_name)

        def callback(msg):
            received.append(msg)
            if message_limit and len(received) >= message_limit:
                done_future.set_result(True)

        tmp_node.create_subscription(msg_class, topic_name, callback, qos)
        executor = SingleThreadedExecutor(context=tmp_node.context)
        executor.add_node(tmp_node)

        start = time.time()
        try:
            while rclpy.ok() and not done_future.done():
                executor.spin_once(timeout_sec=0.1)
                if duration and (time.time() - start) >= duration:
                    break
        finally:
            executor.remove_node(tmp_node)
            executor.shutdown()
            tmp_node.destroy_node

        elapsed = time.time() - start

        return {
            "messages": [self.serialize_msg(msg) for msg in received],
            "count": len(received),
            "duration": round(elapsed, 2),
        }

    def call_get_messages_service_any(self, params: dict) -> dict:
        service_type = get_service("lora_msgs/srv/GetMessages")
        if not service_type:
            raise ImportError("Service type not found for 'GetMessages'")
        client = self.node.create_client(service_type, "/get_messages")
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception(
                    "Interrupted while waiting for the service. ROS shutdown."
                )

        request = service_type.Request()
        topic_name = params.get("topic_name")
        request.topic_name = topic_name
        request.message_type = "any"
        request.number_of_msgs = params.get("number_of_msgs", 0)

        def parse_iso8601_to_fulldatetime(iso8601_str):
            FullDateTime = get_message("lora_msgs/msg/FullDateTime")

            dt = parser.isoparse(iso8601_str)

            full_datetime = FullDateTime()
            full_datetime.year = dt.year
            full_datetime.month = dt.month
            full_datetime.day = dt.day
            full_datetime.hour = dt.hour
            full_datetime.minute = dt.minute
            full_datetime.second = dt.second
            full_datetime.nanosecond = dt.microsecond * 1000

            return full_datetime

        if params.get("time_start"):
            request.time_start = parse_iso8601_to_fulldatetime(params["time_start"])
        if params.get("time_end"):
            request.time_end = parse_iso8601_to_fulldatetime(params["time_end"])

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)
        if future.done():
            print("Service call completed")
        else:
            print("Service call did not complete within the timeout")
        response = future.result()
        if response:
            try:
                MessageType = get_message(params.get("message_type"))
                messages = []
                data = response.data
                offset = 0

                while offset < len(data):
                    message_length = int.from_bytes(
                        data[offset : offset + 4], byteorder="big"
                    )
                    offset += 4

                    message_data = bytes(data[offset : offset + message_length])
                    offset += message_length

                    message = deserialize_message(message_data, MessageType())
                    messages.append(message)

                def serialize_ros_message(msg):
                    result = {}
                    for (
                        field_name,
                        field_type,
                    ) in msg.get_fields_and_field_types().items():
                        value = getattr(msg, field_name)

                        if hasattr(value, "get_fields_and_field_types"):
                            result[field_name] = serialize_ros_message(value)
                        elif isinstance(value, list):
                            serialized_list = []
                            for item in value:
                                if hasattr(item, "get_fields_and_field_types"):
                                    serialized_list.append(serialize_ros_message(item))
                                elif isinstance(item, (array.array, tuple)):
                                    serialized_list.append(list(item))
                                else:
                                    serialized_list.append(item)
                            result[field_name] = serialized_list
                        elif isinstance(value, (array.array, tuple)):
                            result[field_name] = list(value)
                        elif isinstance(value, np.ndarray):
                            result[field_name] = value.tolist()
                        elif isinstance(value, (bytes, bytearray)):
                            result[field_name] = value.decode("utf-8", errors="ignore")
                        elif isinstance(value, (int, float, str, bool)):
                            result[field_name] = value
                        else:
                            print(
                                f"Unsupported type for JSON serialization: {field_name} of type {type(value)}"
                            )
                            result[field_name] = str(value)

                    return result

                serialized_response = {
                    "timestamps": [
                        serialize_ros_message(timestamp)
                        for timestamp in response.timestamps
                    ],
                    "messages": [serialize_ros_message(msg) for msg in messages],
                }

                return serialized_response
            except Exception as e:
                raise Exception(f"Message deserialization error:” {e}")
        else:
            return None

    def fill_ros_message(self, msg_class, data):
        msg = msg_class()
        for key, value in data.items():
            if not hasattr(msg, key):
                raise AttributeError(f"{msg_class} has no field '{key}'")

            attr = getattr(msg, key)
            expected_type = type(attr)

            # Support for nested messages
            if hasattr(expected_type, "__slots__") and isinstance(value, dict):
                setattr(msg, key, self.fill_ros_message(expected_type, value))

            # List handling
            elif isinstance(attr, list) and isinstance(value, list):
                # List of simple type elements
                if not attr:
                    setattr(msg, key, value)
                else:
                    sub_type = type(attr[0])
                    filled_list = []
                    for v in value:
                        if hasattr(sub_type, "__slots__") and isinstance(v, dict):
                            filled_list.append(self.fill_ros_message(sub_type, v))
                        else:
                            filled_list.append(self._coerce_type(v, sub_type))
                    setattr(msg, key, filled_list)

            # Special type: Time / Duration
            elif isinstance(attr, (Time, Duration)) and isinstance(value, dict):
                setattr(
                    msg,
                    key,
                    expected_type(
                        sec=value.get("sec", 0), nanosec=value.get("nanosec", 0)
                    ),
                )

            # Header
            elif isinstance(attr, Header) and isinstance(value, dict):
                header = Header()
                if "frame_id" in value:
                    header.frame_id = value["frame_id"]
                if "stamp" in value:
                    stamp_data = value["stamp"]
                    header.stamp = Time(
                        sec=stamp_data.get("sec", 0),
                        nanosec=stamp_data.get("nanosec", 0),
                    )
                setattr(msg, key, header)

            # Simple types: float, int, bool, str
            else:
                coerced = self._coerce_type(value, expected_type)
                setattr(msg, key, coerced)

        return msg

    def _coerce_type(self, value, expected_type):
        try:
            if expected_type == float and not isinstance(value, float):
                return float(value)
            elif expected_type == int and not isinstance(value, int):
                return int(value)
            elif expected_type == bool and not isinstance(value, bool):
                return bool(value)
            elif expected_type == str and not isinstance(value, str):
                return str(value)
            else:
                return value
        except Exception as e:
            raise ValueError(
                f"Cannot convert value '{value}' to type '{expected_type}': {e}"
            )

    def publish_to_topic(self, topic_name: str, message_type: str, data: dict) -> dict:
        # Validate topic_name
        if not isinstance(topic_name, str) or not topic_name.strip():
            return {"error": "Invalid topic name. It must be a non-empty string."}

        # Validate message_type
        if not isinstance(message_type, str) or "/" not in message_type:
            return {
                "error": "Invalid message type. It must be a valid ROS2 message type string."
            }

        # Validate data
        if not isinstance(data, dict):
            return {"error": "Invalid data. It must be a dictionary."}

        try:
            parts = message_type.split("/")
            if len(parts) == 3:
                pkg, _, msg = parts
            elif len(parts) == 2:
                pkg, msg = parts
            else:
                return {"error": f"Invalid message type format: {message_type}"}

            module = importlib.import_module(f"{pkg}.msg")
            msg_class = getattr(module, msg)
        except Exception as e:
            return {"error": f"Failed to load message type: {str(e)}"}

        try:
            pub = self.node.create_publisher(msg_class, topic_name, 10)
            msg_instance = self.fill_ros_message(msg_class, data)
            pub.publish(msg_instance)

            return {"status": "published", "data": data}
        except Exception as e:
            return {"error": "Failed to publish message due to an internal error."}


    def shutdown(self):
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"ROS shutdown skipped: {e}")
