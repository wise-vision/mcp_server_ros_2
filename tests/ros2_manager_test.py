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
        {
            "topic_name": "/chatter",
            "topic_type": "std_msgs/msg/String",
            "request_fields": {"data": "string"},
        },
        {
            "topic_name": "/sensor",
            "topic_type": "std_msgs/msg/Float32",
            "request_fields": {"data": "float"},
        },
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

from example_interfaces.srv import AddTwoInts

@patch("server.ros2_manager.ServiceNode") 
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
def test_call_service_success_real_type(mock_spin, mock_node_cls):
    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        manager = ROS2Manager()

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_future = MagicMock()

        resp = AddTwoInts.Response()
        resp.sum = 8
        mock_future.result.return_value = resp
        mock_client.call_async.return_value = mock_future
        mock_node.create_client.return_value = mock_client

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
    rclpy.init()
    try:
        msg = FakeSlotMsg()
        manager = ROS2Manager()
        result = manager.serialize_msg(msg)
        assert result == {"x": 10, "y": 20}
    finally:
        rclpy.shutdown()

class FakeDataMsg:
    def __init__(self):
        self.data = 42

def test_serialize_msg_data():
    rclpy.init()
    try:
        msg = FakeDataMsg()
        manager = ROS2Manager()
        result = manager.serialize_msg(msg)
        print(result)
        assert result == 42
    finally:
        rclpy.shutdown()

class WeirdMsg:
    def __str__(self):
        return "<weird>"

def test_serialize_msg_fallback():
    rclpy.init()
    try:
        msg = WeirdMsg()
        manager = ROS2Manager()
        result = manager.serialize_msg(msg)
        assert result == "<weird>"
    finally:
        rclpy.shutdown()

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

from unittest.mock import patch, MagicMock
from server.ros2_manager import ROS2Manager

from unittest.mock import MagicMock, patch
import rclpy
from server.ros2_manager import ROS2Manager


@patch("server.ros2_manager.get_service")
@patch("server.ros2_manager.get_message")
@patch("server.ros2_manager.deserialize_message")
def test_call_get_messages_service_any_success(
    mock_deserialize, mock_get_msg, mock_get_srv
):
    try:
        rclpy.init()

        mock_request = MagicMock()
        mock_service = MagicMock()
        mock_service.Request.return_value = mock_request
        mock_get_srv.return_value = mock_service

        full_datetime_cls = MagicMock()
        sensor_msg_cls = MagicMock()
        mock_get_msg.side_effect = [full_datetime_cls, sensor_msg_cls]

        fake_msg = MagicMock()
        fake_msg.get_fields_and_field_types.return_value = {"temp": "float"}
        setattr(fake_msg, "temp", 22.5)
        mock_deserialize.return_value = fake_msg

        payload = b"\x01"
        length_bytes = len(payload).to_bytes(4, byteorder="big")
        fake_response = MagicMock()
        fake_response.data = length_bytes + payload

        fake_ts = MagicMock()
        fake_ts.get_fields_and_field_types.return_value = {"sec": "int32"}
        setattr(fake_ts, "sec", 12345)
        fake_response.timestamps = [fake_ts]

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_client.call.return_value = fake_response

        manager = ROS2Manager()
        manager.node.create_client = MagicMock(return_value=mock_client)

        result = manager.call_get_messages_service_any(
            {
                "topic_name": "sensor_data",
                "message_type": "my_msgs/msg/SensorData",
                "number_of_msgs": 1,
            }
        )

        assert "messages" in result
        assert "timestamps" in result
        assert isinstance(result["messages"], list)
        assert isinstance(result["timestamps"], list)
        assert result["messages"][0] == {"temp": 22.5}
        assert result["timestamps"][0]["sec"] == 12345

    finally:
        rclpy.shutdown()

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_string_no_type_mocks(mock_node_cls):
    from std_msgs.msg import String

    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        manager = ROS2Manager()

        mock_publisher = MagicMock()
        mock_node.create_publisher.return_value = mock_publisher

        result = manager.publish_to_topic("/chatter", "std_msgs/msg/String", {"data": "Hello"})

        assert result["status"] == "published"
        mock_node.create_publisher.assert_called_once()
        mock_publisher.publish.assert_called_once()

        published_msg = mock_publisher.publish.call_args[0][0]
        assert isinstance(published_msg, String)
        assert published_msg.data == "Hello"
    finally:
        if rclpy.ok():
            rclpy.shutdown()

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_invalid_topic_name(mock_node_cls):
    manager = ROS2Manager()
    result = manager.publish_to_topic("", "std_msgs/msg/String", {"data": "Hello"})
    assert "error" in result
    assert result["error"] == "Invalid topic name. It must be a non-empty string."

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_invalid_message_type(mock_node_cls):
    manager = ROS2Manager()
    result = manager.publish_to_topic("/chatter", "invalidtype", {"data": "Hello"})
    assert "error" in result
    assert result["error"] == "Invalid message type. It must be a valid ROS2 message type string."

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_invalid_data(mock_node_cls):
    manager = ROS2Manager()
    result = manager.publish_to_topic("/chatter", "std_msgs/msg/String", "invalid_data")
    assert "error" in result
    assert result["error"] == "Invalid data. It must be a dictionary."


@patch("server.ros2_manager.ServiceNode")
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
def test_mavros_waypoint_push_int_to_float_no_type_mocks(mock_spin, mock_node_cls):
    from mavros_msgs.srv import WaypointPush
    from mavros_msgs.msg import Waypoint

    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        manager = ROS2Manager()

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_future = MagicMock()

        resp = WaypointPush.Response()
        resp.success = True
        resp.wp_transfered = 3
        mock_future.result.return_value = resp
        mock_client.call_async.return_value = mock_future
        mock_node.create_client.return_value = mock_client

        fields = {
            "start_index": 0,
            "waypoints": [
                {
                    "frame": 3, "command": 22, "is_current": True, "autocontinue": True,
                    "param1": 0, "param2": 0, "param3": 0, "param4": 0,
                    "x_lat": 0, "y_long": 0, "z_alt": 7
                },
                {
                    "frame": 3, "command": 16, "is_current": False, "autocontinue": True,
                    "param1": 0, "param2": 0, "param3": 0, "param4": 0,
                    "x_lat": 52.43304071995481, "y_long": 20.72276917967427, "z_alt": 7
                },
                {
                    "frame": 3, "command": 20, "is_current": False, "autocontinue": True,
                    "param1": 0, "param2": 0, "param3": 0, "param4": 0,
                    "x_lat": 0, "y_long": 0, "z_alt": 0
                }
            ]
        }

        result = manager.call_service(
            service_name="/mavros/mission/push",
            service_type="mavros_msgs/srv/WaypointPush",
            fields=fields
        )

        assert "result" in result
        mock_client.wait_for_service.assert_called_once()
        mock_client.call_async.assert_called_once()

        sent_req = mock_client.call_async.call_args[0][0]
        assert isinstance(sent_req, WaypointPush.Request)
        assert isinstance(sent_req.start_index, int) and sent_req.start_index == 0
        assert isinstance(sent_req.waypoints, list) and len(sent_req.waypoints) == 3
        assert all(isinstance(w, Waypoint) for w in sent_req.waypoints)

        wp0 = sent_req.waypoints[0]
        assert isinstance(wp0.x_lat, float) and wp0.x_lat == 0.0
        assert isinstance(wp0.y_long, float) and wp0.y_long == 0.0
        assert isinstance(wp0.z_alt, float) and wp0.z_alt == 7.0
        assert isinstance(wp0.param1, float) and wp0.param1 == 0.0

        wp1 = sent_req.waypoints[1]
        assert isinstance(wp1.z_alt, float) and wp1.z_alt == 7.0

    finally:
        if rclpy.ok():
            rclpy.shutdown()


from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import time

def test_get_qos_for_publisher_topic_superset_real_subs():
    rclpy.init()
    try:
        topic = "/qos_integration_test"

        sub_a_node = rclpy.create_node("sub_a_node")
        sub_b_node = rclpy.create_node("sub_b_node")

        qos_a = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_b = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3,
        )

        sub_a = sub_a_node.create_subscription(String, topic, lambda m: None, qos_a)
        sub_b = sub_b_node.create_subscription(String, topic, lambda m: None, qos_b)

        inspector_node = rclpy.create_node("inspector_node")

        from server.ros2_manager import ROS2Manager
        manager = ROS2Manager()
        manager.node = inspector_node

        # Discovery
        exec = SingleThreadedExecutor()
        exec.add_node(sub_a_node)
        exec.add_node(sub_b_node)
        exec.add_node(inspector_node)

        for _ in range(10):
            exec.spin_once(timeout_sec=0.05)
            time.sleep(0.01)

        qos = manager.get_qos_for_publisher_topic(inspector_node, topic)

        assert qos.reliability == QoSReliabilityPolicy.RELIABLE
        assert qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
        assert qos.history == QoSHistoryPolicy.KEEP_LAST
        if qos.depth != 0:
            assert qos.depth >= 10

    finally:
        try:
            sub_a_node.destroy_subscription(sub_a)
            sub_b_node.destroy_subscription(sub_b)
        except Exception:
            pass
        try:
            sub_a_node.destroy_node()
            sub_b_node.destroy_node()
        except Exception:
            pass
        try:
            inspector_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()



from unittest.mock import patch, MagicMock

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_uses_selected_qos(mock_node_cls):
    from std_msgs.msg import String
    from server.ros2_manager import ROS2Manager

    rclpy.init()
    chosen_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )

    mock_node = MagicMock()
    mock_publisher = MagicMock()
    mock_node.create_publisher.return_value = mock_publisher
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    manager.get_qos_for_publisher_topic = MagicMock(return_value=chosen_qos)

    result = manager.publish_to_topic("/chatter", "std_msgs/msg/String", {"data": "hello"})

    assert result["status"] == "published"
    args, kwargs = mock_node.create_publisher.call_args
    assert args[0] is String
    assert args[1] == "/chatter"
    passed_qos = args[2]
    assert isinstance(passed_qos, QoSProfile)
    assert passed_qos.reliability == QoSReliabilityPolicy.RELIABLE
    assert passed_qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
    assert passed_qos.history == QoSHistoryPolicy.KEEP_LAST
    assert passed_qos.depth == 5