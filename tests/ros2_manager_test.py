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

from unittest.mock import MagicMock, patch
from server.ros2_manager import ROS2Manager
import rclpy

@patch("server.ros2_manager.ServiceNode")
def test_list_topics(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = [
        ("/chatter", ["std_msgs/msg/String"]),
        ("/sensor", ["std_msgs/msg/Float32"])
    ]
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    result = manager.list_topics()

    assert result == [
        {"topic_name": "/chatter", "topic_type": "std_msgs/msg/String"},
        {"topic_name": "/sensor", "topic_type": "std_msgs/msg/Float32"},
    ]

@patch("server.ros2_manager.ServiceNode")
def test_list_topics_empty(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = []
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    result = manager.list_topics()

    assert result == [] 

@patch("server.ros2_manager.ROS2Manager.get_request_fields")
@patch("server.ros2_manager.ServiceNode")
def test_list_services_with_services(mock_node_cls, mock_get_request_fields):
    mock_node = MagicMock()
    mock_node.get_service_names_and_types.return_value = [
        ("/add_two_ints", ["example_interfaces/srv/AddTwoInts"]),
        ("/reset_robot", ["my_msgs/srv/Reset"])
    ]
    mock_node_cls.return_value = mock_node

    mock_get_request_fields.side_effect = [
        {"a": "int64", "b": "int64"},
        {"confirm": "bool"}
    ]

    manager = ROS2Manager()
    result = manager.list_services()

    assert result == [
        {
            "service_name": "/add_two_ints",
            "service_type": "example_interfaces/srv/AddTwoInts",
            "request_fields": {"a": "int64", "b": "int64"}
        },
        {
            "service_name": "/reset_robot",
            "service_type": "my_msgs/srv/Reset",
            "request_fields": {"confirm": "bool"}
        }
    ]

@patch("server.ros2_manager.ServiceNode")
def test_list_services_empty(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_service_names_and_types.return_value = []
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    result = manager.list_services()

    assert result == []

@patch("server.ros2_manager.ServiceNode")
@patch("server.ros2_manager.importlib.import_module")
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
def test_call_service_success(mock_spin, mock_import, mock_node_cls):
    try:
        if not rclpy.ok():
            rclpy.init()

        # Set up the fake node to be injected
        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node  # this replaces ServiceNode()

        manager = ROS2Manager()  # <- teraz to działa, bo node jest podmieniony!

        # Mock service class
        mock_srv_class = MagicMock()
        mock_request_instance = MagicMock()
        mock_srv_class.Request.return_value = mock_request_instance
        mock_srv_class.Request.get_fields_and_field_types.return_value = {"a": "int64", "b": "int64"}

        # Mock import of service module
        mock_module = MagicMock()
        mock_module.AddTwoInts = mock_srv_class
        mock_import.return_value = mock_module

        # Setup mock client + future
        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_future = MagicMock()
        mock_future.result.return_value = MagicMock()
        mock_client.call_async.return_value = mock_future

        # Inject the mock client into the node
        mock_node.create_client.return_value = mock_client

        # Call actual method
        result = manager.call_service(
            service_name="/add_two_ints",
            service_type="example_interfaces/srv/AddTwoInts",
            fields={"a": 5, "b": 3}
        )

        assert "result" in result
        mock_client.wait_for_service.assert_called_once()
        mock_client.call_async.assert_called_once()

    finally:
        if rclpy.ok():
            rclpy.shutdown()

# Test for serialize message
class FakeSlotMsg:
    __slots__ = ["x", "y"]
    def __init__(self):
        self.x = 10
        self.y = 20

def test_serialize_msg_slots():
    msg = FakeSlotMsg()
    result = ROS2Manager.serialize_msg(msg)
    assert result == {"x": 10, "y": 20}

class FakeDataMsg:
    def __init__(self):
        self.data = 42

def test_serialize_msg_data():
    msg = FakeDataMsg()
    result = ROS2Manager.serialize_msg(msg)
    assert result == {"data": 42}

class WeirdMsg:
    def __str__(self):
        return "<weird>"

def test_serialize_msg_fallback():
    msg = WeirdMsg()
    result = ROS2Manager.serialize_msg(msg)
    assert result == {"value": "<weird>"}

# Test for subscribe topic
@patch("server.ros2_manager.ServiceNode")
def test_subscribe_topic_topic_not_found(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = [("/chatter", ["std_msgs/msg/String"])]
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    manager.node = mock_node

    result = manager.subscribe_topic("/not_here", "std_msgs/msg/String")
    assert "error" in result
    assert "not found" in result["error"]

@patch("server.ros2_manager.ServiceNode")
def test_subscribe_topic_invalid_type_format(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = [("/chatter", ["std_msgs/msg/String"])]
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    manager.node = mock_node

    result = manager.subscribe_topic("/chatter", "badtypeformat")
    assert "error" in result
    assert "Invalid message type format" in result["error"]

@patch("server.ros2_manager.importlib.import_module", side_effect=ImportError("Boom"))
@patch("server.ros2_manager.ServiceNode")
def test_subscribe_topic_import_fail(mock_node_cls, mock_import):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = [("/chatter", ["std_msgs/msg/String"])]
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    manager.node = mock_node

    result = manager.subscribe_topic("/chatter", "std_msgs/msg/String")
    assert "error" in result
    assert "Failed to import" in result["error"]

@patch("server.ros2_manager.importlib.import_module")
@patch("server.ros2_manager.ServiceNode")
def test_subscribe_topic_success(mock_node_cls, mock_import):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = [("/chatter", ["std_msgs/msg/String"])]
    mock_node_cls.return_value = mock_node

    # Fake message type
    class FakeMsg:
        __slots__ = ["data"]
        def __init__(self): self.data = 42

    fake_module = MagicMock()
    fake_module.String = MagicMock()
    mock_import.return_value = fake_module
    fake_module.String.return_value = FakeMsg()

    manager = ROS2Manager()
    manager.node = mock_node

    # Patch create_subscription to simulate immediate callback
    def fake_create_subscription(cls, topic, callback, qos):
        callback(FakeMsg())  # simulate receiving a message
        return MagicMock()

    mock_node.create_subscription.side_effect = fake_create_subscription

    result = manager.subscribe_topic("/chatter", "std_msgs/msg/String", message_limit=1)

    assert "messages" in result
    assert result["count"] == 1
    assert result["messages"][0]["data"] == 42

from unittest.mock import patch, MagicMock
from server.ros2_manager import ROS2Manager

@patch("server.ros2_manager.get_service")
@patch("server.ros2_manager.get_message")
@patch("server.ros2_manager.deserialize_message")
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
def test_call_get_messages_service_any_success(mock_spin, mock_deserialize, mock_get_msg, mock_get_srv):
    try:
        rclpy.init()
        # Mock the service class
        mock_request = MagicMock()
        mock_service = MagicMock()
        mock_service.Request.return_value = mock_request
        mock_get_srv.return_value = mock_service

        # Mock FullDateTime
        full_datetime_class = MagicMock()
        mock_get_msg.side_effect = [full_datetime_class, MagicMock()]  # time + message

        # Fake response with binary data
        fake_msg = MagicMock()
        fake_msg.get_fields_and_field_types.return_value = {"temp": "float"}
        mock_deserialize.return_value = fake_msg

        fake_response = MagicMock()
        fake_response.data = b'\x00\x00\x00\x05hello'  # length 5 + "hello"
        fake_response.timestamps = [MagicMock()]
        fake_response.timestamps[0].get_fields_and_field_types.return_value = {"sec": "int32"}
        setattr(fake_response.timestamps[0], "sec", 12345)

        # Setup fake future
        mock_future = MagicMock()
        mock_future.result.return_value = fake_response
        mock_future.done.return_value = True

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_client.call_async.return_value = mock_future

        # Replace node + client
        manager = ROS2Manager()
        manager.node.create_client = MagicMock(return_value=mock_client)

        result = manager.call_get_messages_service_any({
            "topic_name": "sensor_data",
            "message_type": "my_msgs/msg/SensorData",
            "number_of_msgs": 1
        })

        assert "messages" in result
        assert "timestamps" in result
        assert isinstance(result["messages"], list)
        assert isinstance(result["timestamps"], list)
        assert result["messages"][0] == {"temp": {}}
    finally:
        rclpy.shutdown()