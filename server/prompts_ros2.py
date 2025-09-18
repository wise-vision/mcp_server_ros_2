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