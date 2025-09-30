#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
from . import prompthandler

class DroneMissionWithMAVROS2Prompt(prompthandler.BasePromptHandler):
    def __init__(self) -> None:
        super().__init__(
            name="drone-mavros2-mission",
            description=(
                "User only provides dest_x, dest_y, dest_z (meters relative to HOME, MAV_FRAME=3) "
                "and the flags return_to_launch and land_at_launch. "
                "The prompt hides the full MAVROS2 mission sequence."
            ),
            args=[
                prompthandler.ArgSpec(
                    "dest_x", "Target X (meters relative to HOME, MAV_FRAME=3)", True, "number"
                ),
                prompthandler.ArgSpec(
                    "dest_y", "Target Y (meters relative to HOME, MAV_FRAME=3)", True, "number"
                ),
                prompthandler.ArgSpec(
                    "dest_z", "Target Z (meters relative to HOME, MAV_FRAME=3)", True, "number"
                ),
                prompthandler.ArgSpec(
                    "return_to_launch",
                    "Whether the drone should return to HOME before landing (Pass true or false)",
                    True,
                    "boolean",
                    enum=[True, False],
                    default=True,
                ),
                prompthandler.ArgSpec(
                    "land_at_launch",
                    "Whether the drone should land at HOME(Pass true or false)",
                    True,
                    "boolean",
                    enum=[True, False],
                    default=True,
                ),
            ],
            messages_template=[
                ("assistant",
                 "You are an MCP agent controlling a drone via ROS 2 + MAVROS2. "
                 "You must construct and upload a mission using MAV_FRAME_GLOBAL_RELATIVE_ALT (3) "
                 "for all items. Do not reveal technical steps to the user; only call tools. "
                 "Sequence to perform:\n"
                 "1) PREPARE FOR TAKEOFF:\n"
                 "   - Arm the vehicle via MAVROS2 arming service.\n"
                 "2) BUILD MISSION (do not reset existing waypoints after upload; do not add instructions to start from first WP):\n"
                 "   - Item #1: MAV_CMD_NAV_TAKEOFF (22), frame=3, is_current=true, altitude=10 meters.\n"
                 "   - Item #2: MAV_CMD_NAV_WAYPOINT (16), frame=3 to coordinates (x={dest_x}, y={dest_y}, z={dest_z}) relative to HOME.\n"
                 "   - If return_to_launch is true: add a NAV_WAYPOINT to HOME with frame=3 and (x=0, y=0, z=7).\n"
                 "   - If land_at_launch is true: add MAV_CMD_NAV_LAND (21) at HOME with frame=3 and (x=0, y=0, z=0).\n"
                 "   Notes:\n"
                 "   - is_current must be true ONLY for the first TAKEOFF item.\n"
                 "   - Use frame=3 for all items.\n"
                 "   - Do not add any command to reset/clear waypoints after upload.\n"
                 "   - Do not add any instruction to start from the first waypoint; it is already implied.\n"
                 "3) UPLOAD the mission.\n"
                 "4) Switch mode to AUTO so the mission begins.\n"
                 "\n"
                 "TOOLS to use (examples, adapt to your exact ROS2/MAVROS2 topics/services):\n"
                 "- Use ROS2ServiceCall for arming (e.g., /mavros/cmd/arming std_srvs/srv/SetBool {{ data: true }}).\n"
                 "- Use ROS2ServiceCall or a mission upload service (e.g., /mavros/mission/push mavros_msgs/srv/WaypointPush) "
                 "  with a list of waypoints (cmd, frame=3, is_current flags, x/y/z fields as appropriate).\n"
                 "- Use ROS2ServiceCall to set mode AUTO (e.g., /mavros/set_mode mavros_msgs/srv/SetMode {{ base_mode: 0, custom_mode: \"AUTO\" }}).\n"
                 "\n"
                 "Critically: build a proper mission payload with TAKEOFF, WAYPOINT (dest), optional HOME WP, optional LAND, "
                 "upload it once, do not clear/reset afterwards, and finally set AUTO."
                ),
                ("assistant",
                 "Acknowledged. I will arm, build and upload the mission (frame=3), and set AUTO."),
                ("user",
                 "Destination request: x={dest_x}, y={dest_y}, z={dest_z}; "
                 "return_to_launch={return_to_launch}; land_at_launch={land_at_launch}")
            ],
        )

class Nav2NavigateToPosePrompt(prompthandler.BasePromptHandler):
    def __init__(self) -> None:
        super().__init__(
            name="nav2-navigate-to-pose",
            description=(
                "User provides target pose in the map frame. "
                "The prompt hides Nav2 details and only calls tools to send the NavigateToPose action, "
                "optionally waits for result, streams feedback, and cancels on timeout."
            ),
            args=[
                prompthandler.ArgSpec(
                    "target_x",
                    "Target X in meters (map frame)",
                    True,
                    "number",
                ),
                prompthandler.ArgSpec(
                    "target_y",
                    "Target Y in meters (map frame)",
                    True,
                    "number",
                ),
                prompthandler.ArgSpec(
                    "target_yaw_deg",
                    "Target yaw (degrees, counter-clockwise, map frame)",
                    True,
                    "number",
                ),
                prompthandler.ArgSpec(
                    "frame_id",
                    "Frame id for the pose (usually 'map' or 'odom')",
                    True,
                    "string",
                    enum=["map", "odom"],
                    default="map",
                ),
                prompthandler.ArgSpec(
                    "wait_for_result",
                    "Whether to wait for the NavigateToPose result",
                    True,
                    "boolean",
                    enum=[True, False],
                    default=True,
                ),
                prompthandler.ArgSpec(
                    "timeout_sec",
                    "Timeout in seconds for the action result (used when wait_for_result=true)",
                    True,
                    "number",
                    default=120.0,
                ),
                prompthandler.ArgSpec(
                    "cancel_on_timeout",
                    "If true and timeout occurs, cancel the goal",
                    True,
                    "boolean",
                    enum=[True, False],
                    default=True,
                ),
                prompthandler.ArgSpec(
                    "subscribe_feedback",
                    "If true, subscribe to feedback during execution (non-blocking stream window)",
                    True,
                    "boolean",
                    enum=[True, False],
                    default=True,
                ),
                prompthandler.ArgSpec(
                    "feedback_window_sec",
                    "How many seconds to collect feedback messages when subscribe_feedback=true",
                    True,
                    "number",
                    default=5.0,
                ),
            ],
            messages_template=[
                (
                    "assistant",
                    # System-style instruction to the agent:
                    "You are an MCP agent controlling Nav2 via ROS 2 actions. "
                    "Your job is to navigate the robot to a target pose using the action "
                    "`nav2_msgs/action/NavigateToPose` on `/navigate_to_pose`.\n"
                    "\n"
                    "Do NOT reveal technical steps to the user; only call tools. "
                    "Follow this sequence strictly:\n"
                    "1) Ensure action availability (call 'ros2_list_actions' and verify that "
                    "`/navigate_to_pose` with type `nav2_msgs/action/NavigateToPose` is present). "
                    "If not present, return a concise error.\n"
                    "2) Construct the goal payload:\n"
                    "   - frame_id = {frame_id}\n"
                    "   - pose.position.x = {target_x}\n"
                    "   - pose.position.y = {target_y}\n"
                    "   - pose.position.z = 0.0\n"
                    "   - Convert yaw (degrees) to quaternion around Z only:\n"
                    "       yaw_rad = {target_yaw_deg} * pi / 180\n"
                    "       qz = sin(yaw_rad/2), qw = cos(yaw_rad/2)\n"
                    "     Set pose.orientation.z = qz, pose.orientation.w = qw; x=y=0.\n"
                    "3) Send goal using tool 'ros2_send_action_goal' with:\n"
                    "   - action_name='/navigate_to_pose'\n"
                    "   - action_type='nav2_msgs/action/NavigateToPose'\n"
                    "   - goal_fields as constructed above\n"
                    "   - wait_for_result={wait_for_result}\n"
                    "   - timeout_sec={timeout_sec}\n"
                    "4) If wait_for_result=false and subscribe_feedback=true:\n"
                    "   - Call 'ros2_action_subscribe_feedback' (action_name='/navigate_to_pose', "
                    "     action_type='nav2_msgs/action/NavigateToPose', goal_id_hex=<from send_goal>, "
                    "     duration_sec={feedback_window_sec}, max_messages=100) to stream feedback window.\n"
                    "5) If wait_for_result=true and result status is TIMEOUT (or no final result received):\n"
                    "   - If {cancel_on_timeout} is true, call 'ros2_cancel_action_goal' with goal_id_hex to cancel the goal. "
                    "   - Return a concise timeout message.\n"
                    "6) If wait_for_result=true and result is received: return the final status and result payload.\n"
                    "\n"
                    "TOOLS available (call exactly as needed; do not explain them to the user):\n"
                    "- ros2_list_actions\n"
                    "- ros2_send_action_goal\n"
                    "- ros2_action_subscribe_feedback\n"
                    "- ros2_action_request_result\n"
                    "- ros2_cancel_action_goal\n"
                    "\n"
                    "Important notes:\n"
                    "- Use only Z-axis yaw; do not set orientation x/y.\n"
                    "- Always set frame_id correctly (default 'map').\n"
                    "- Never disclose internal calculations or tool call details to the user; "
                    "respond with the high-level outcome only."
                ),
                (
                    "assistant",
                    "Acknowledged. I will send a NavigateToPose goal (frame={frame_id}), "
                    "optionally wait for the result, stream feedback if requested, and cancel on timeout when configured."
                ),
                (
                    "user",
                    "Go to: x={target_x}, y={target_y}, yaw_deg={target_yaw_deg}, "
                    "frame_id={frame_id}, wait_for_result={wait_for_result}, "
                    "timeout_sec={timeout_sec}, subscribe_feedback={subscribe_feedback}, "
                    "feedback_window_sec={feedback_window_sec}, cancel_on_timeout={cancel_on_timeout}"
                ),
            ],
        )