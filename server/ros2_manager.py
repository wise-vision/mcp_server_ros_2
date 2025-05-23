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
from rosidl_runtime_py.utilities import get_service, get_message
from dateutil import parser
import numpy as np
import array
import time
from rclpy.task import Future


class ServiceNode(Node):
    def __init__(self):
        super().__init__('mcp_service_lister')

class ROS2Manager:
    def __init__(self):
        self.node = ServiceNode()

    def list_topics(self):
        topic_names_and_types = self.node.get_topic_names_and_types()
        result = []
        for name, types in topic_names_and_types:
            topic_type = types[0] if types else "unknown"
            result.append({
                "topic_name": name,
                "topic_type": topic_type
            })
        return result

    def list_services(self):
        service_list = self.node.get_service_names_and_types()
        result = []
        for name, types in service_list:
            service_type = types[0] if types else "unknown"
            request_fields = self.get_request_fields(service_type)
            result.append({
                "service_name": name,
                "service_type": service_type,
                "request_fields": request_fields
            })
        return result

    def get_request_fields(self, ros_type: str):
        try:
            parts = ros_type.split('/')
            if len(parts) == 3:
                pkg, kind, name = parts
            elif len(parts) == 2:
                pkg, name = parts
                kind = "msg"  # domyślnie zakładamy msg
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
            parts = service_type.split('/')
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

            request = srv_class.Request()
            for key, value in fields.items():
                setattr(request, key, value)

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None:
                return {"result": str(future.result())}
            else:
                return {"error": "Service call failed."}
        except Exception as e:
            return {"error": str(e)}
        

    def serialize_msg(msg):
        try:
            # Try to extract fields using slots
            if hasattr(msg, '__slots__'):
                return {slot: getattr(msg, slot) for slot in msg.__slots__}
            # Fallback for simple messages like Int32
            elif hasattr(msg, 'data'):
                return {'data': msg.data}
            else:
                return {'value': str(msg)}
        except Exception as e:
            return {'error': f'Failed to serialize message: {str(e)}'}
            
    def subscribe_topic(
        self,
        topic_name: str,
        msg_type: str,
        duration: Optional[float] = None,
        message_limit: Optional[int] = None
    ) -> dict:
        import importlib
        import time
        from rclpy.task import Future

        available_topics = self.node.get_topic_names_and_types()
        topic_names = [name for name, _ in available_topics]
        if topic_name not in topic_names:
            return {"error": f"Topic '{topic_name}' not found. Available topics: {topic_names}"}

        # Fallback to avoid infinite wait
        if not duration and not message_limit:
            duration = 5.0  # default duration in seconds

        # Dynamically load message class
        parts = msg_type.split('/')
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

        received = []
        done_future = Future()

        def callback(msg):
            received.append(msg)
            if message_limit and len(received) >= message_limit:
                done_future.set_result(True)

        sub = self.node.create_subscription(msg_class, topic_name, callback, 10)

        start = time.time()
        try:
            while rclpy.ok() and not done_future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if duration and (time.time() - start) >= duration:
                    break
        finally:
            sub.destroy()

        elapsed = time.time() - start

        return {
            "messages": [ROS2Manager.serialize_msg(msg) for msg in received],
            "count": len(received),
            "duration": round(elapsed, 2)
        }

    def call_get_messages_service_any(self, params: dict) -> dict:
        service_type = get_service('lora_msgs/srv/GetMessages')
        if not service_type:
            raise ImportError("Service type not found for 'GetMessages'")
        client = self.node.create_client(service_type, '/get_messages')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request()
        topic_name = params.get('topic_name')
        request.topic_name = topic_name
        request.message_type = 'any'
        request.number_of_msgs = params.get('number_of_msgs', 0)

        def parse_iso8601_to_fulldatetime(iso8601_str):
            FullDateTime = get_message('lora_msgs/msg/FullDateTime')

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

        if params.get('time_start'):
            request.time_start = parse_iso8601_to_fulldatetime(params['time_start'])
        if params.get('time_end'):
            request.time_end = parse_iso8601_to_fulldatetime(params['time_end'])

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)
        if future.done():
            print("Service call completed")
        else:
            print("Service call did not complete within the timeout")
        response = future.result()
        if response:
            try:
                MessageType = get_message(params.get('message_type'))
                messages = []
                data = response.data
                offset = 0

                while offset < len(data):
                    message_length = int.from_bytes(data[offset:offset + 4], byteorder='big')
                    offset += 4

                    message_data = bytes(data[offset:offset + message_length])
                    offset += message_length

                    message = deserialize_message(message_data, MessageType())
                    messages.append(message)


                def serialize_ros_message(msg):
                    result = {}
                    for field_name, field_type in msg.get_fields_and_field_types().items():
                        value = getattr(msg, field_name)

                        if hasattr(value, 'get_fields_and_field_types'):
                            result[field_name] = serialize_ros_message(value)
                        elif isinstance(value, list):
                            serialized_list = []
                            for item in value:
                                if hasattr(item, 'get_fields_and_field_types'):
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
                            result[field_name] = value.decode('utf-8', errors='ignore')
                        elif isinstance(value, (int, float, str, bool)):
                            result[field_name] = value
                        else:
                            print(f"Unsupported type for JSON serialization: {field_name} of type {type(value)}")
                            result[field_name] = str(value)

                    return result
                serialized_response = {
                    'timestamps': [serialize_ros_message(timestamp) for timestamp in response.timestamps],
                    'messages': [serialize_ros_message(msg) for msg in messages]
                }

                return serialized_response
            except Exception as e:
                raise Exception(f"Message deserialization error:” {e}")
        else:
            return None

    def publish_to_topic(self, topic_name: str, message_type: str, data: dict) -> dict:
        # Validate topic_name
        if not isinstance(topic_name, str) or not topic_name.strip():
            return {"error": "Invalid topic name. It must be a non-empty string."}

        # Validate message_type
        if not isinstance(message_type, str) or '/' not in message_type:
            return {"error": "Invalid message type. It must be a valid ROS2 message type string."}

        # Validate data
        if not isinstance(data, dict):
            return {"error": "Invalid data. It must be a dictionary."}

        try:
            parts = message_type.split('/')
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
            msg_instance = msg_class()
            for key, value in data.items():
                setattr(msg_instance, key, value)
            pub.publish(msg_instance)

            return {"status": "published", "data": data}
        except Exception as e:
            return {"error": f"Failed to publish: {str(e)}"}
        
    def echo_topic_once(self, topic_name: str, msg_type: str, timeout: float = 5.0) -> dict:
        # Validate topic_name
        if not isinstance(topic_name, str) or not topic_name.strip():
            return {"error": "Invalid topic name. It must be a non-empty string."}

        # Validate msg_type
        if not isinstance(msg_type, str) or '/' not in msg_type:
            return {"error": "Invalid message type. It must be a valid ROS2 message type string."}

        # Validate timeout
        if not isinstance(timeout, (int, float)) or timeout <= 0:
            return {"error": "Invalid timeout. It must be a positive number."}

        available_topics = self.node.get_topic_names_and_types()
        topic_names = [name for name, _ in available_topics]
        if topic_name not in topic_names:
            return {"error": f"Topic '{topic_name}' not found. Available: {topic_names}"}

        parts = msg_type.split('/')
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

        result_future = Future()

        def callback(msg):
            result_future.set_result(msg)

        sub = self.node.create_subscription(msg_class, topic_name, callback, 10)

        start = time.time()
        try:
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if time.time() - start > timeout:
                    break
        finally:
            sub.destroy()

        if result_future.done():
            msg = result_future.result()
            return {
                "message": ROS2Manager.serialize_msg(msg),
                "received": True
            }
        else:
            return {
                "error": f"No message received within {timeout} seconds.",
                "received": False
            }


    def shutdown(self):
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"ROS shutdown skipped: {e}")
