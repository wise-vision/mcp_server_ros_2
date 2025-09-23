
# WiseVision ROS2 MCP Server

[![Discord](https://img.shields.io/badge/Discord-Join%20Us-5865F2?logo=discord)](https://discord.gg/9aSw6HbUaw)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple)
[![Pro Version](https://img.shields.io/badge/Pro%20Version-Upgrade-gold)](https://app.easy.tools/ec/wisevision/wisevision-mcp-ros-2-pro)

![Flow graph](docs/assets/flow-graph.gif)

**Python** server implementing **Model Context Protocol (MCP)** for **ROS2**.

🚀 **Enjoying this project?**  
You’re welcome to try the **Pro version** with extra features and priority support.  
👉 [Get Pro here](https://app.easy.tools/ec/wisevision/wisevision-mcp-ros-2-pro)


# Real-world examples:
![Demo](docs/assets/mcp-ros2-server.gif)

# Features
- List available topics 
- List available services 
- Call service
- Get messages from [WiseVision Data Black Box](https://github.com/wise-vision/wisevision_data_black_box) ([influxDB](https://www.influxdata.com) alternative to [Rosbag2](https://github.com/ros2/rosbag2))
- Subscribe topic to get messages
- Publish message on topic
- Echo message on topic
- Get fields from message type

# 🤖 Available Prompts

## ✈️ drone-mavros2-mission

Control a drone with MAVROS2 using just target coordinates (dest_x, dest_y, dest_z) and simple flags (return_to_launch, land_at_launch).

➡️ The prompt builds the full MAVLink mission (TAKEOFF, WAYPOINT, RTL, LAND) and switches to AUTO.

## 🗺️ nav2-navigate-to-pose

Navigate a ground robot with Nav2 by providing only x, y, and yaw in the map frame.

➡️ The prompt sends a NavigateToPose goal, handles result/timeout, streams feedback, and cancels if needed.

### 💡 Don’t know what prompts are? [See the MCP spec here](https://modelcontextprotocol.io/specification/2025-06-18/server/prompts#user-interaction-model).

**Note:** To call service with custom service source it before start server.


### 🔧 ROS 2 Tools

#### 📋 **Topics**
| Tool | Description | Inputs | Outputs |
|------|-------------|--------|---------|
| **`ros2_topic_list`** | Returns list of available topics | – | `topic_name` (string): Topic name <br> `topic_type` (string): Message type |
| **`ros2_topic_subscribe`** | Subscribes to a ROS 2 topic and collects messages for a duration or message limit | `topic_name` (string) <br> `duration` (float) <br> `message_limit` (int) <br> *(defaults: first msg, 5s)* | `messages` <br> `count` <br> `duration` |
| **`ros2_get_messages`** | Retrieves past messages from a topic (data black box) | `topic_name` (string) <br> `message_type` (string) <br> `number_of_msg` (int) <br> `time_start` (str) <br> `time_end` (str) | `timestamps` <br> `messages` |
| **`ros2_get_message_fields`** | Gets field names and types for a message type | `message_type` (string) | Field names + types |
| **`ros2_topic_publish`** | Publishes message to a topic | `topic_name` (string) <br> `message_type` (string) <br> `data` (dict) | `status` |

---

#### 🛠 **Services**
| Tool | Description | Inputs | Outputs |
|------|-------------|--------|---------|
| **`ros2_service_list`** | Returns list of available services | – | `service_name` (string) <br> `service_type` (string) <br> `request_fields` (array) |
| **`ros2_service_call`** | Calls a ROS 2 service | `service_name` (string) <br> `service_type` (string) <br> `fields` (array) <br> `force_call` (bool, default: false) | `result` (string) <br> `error` (string, if any) |


## Usage

### MCP Server Configuration

**Docker run**

Set MCP setting to mcp.json.
```json
"mcp_server_ros_2": {
    "command": "docker",
    "args": [
        "run",
        "-i",
        "--rm",
        "wisevision/mcp_server_ros_2:<humble/jazzy>"
    ],
    }

```

### Build docker image locally
```bash
git clone https://github.com/wise-vision/mcp_server_ros_2.git
cd mcp_server_ros_2
docker build -t mcp_server_ros_2:<humble/jazzy>  --build-arg ROS_DISTRO=<humble/jazzy> .
```


Add  this to AI Agent prompt:
```txt
You are an AI assistant that uses external tools via an MCP server.
Before calling any tool, always check your memory to see if the list of available tools is known.
	•	If you don’t have the current tool list in memory, your first action should be to call the list-tools tool.
	•	Never guess tool names or parameters.
	•	If a user requests something that may require a tool and you don’t have the right tool info, ask them or call list-tools first.
Once the tool list is loaded, you may call tools directly using their documented names and schemas.
```

# Debugging

Since MCP servers run over stdio, debugging can be challenging. For the best debugging
experience, we strongly recommend using the [MCP Inspector](https://github.com/modelcontextprotocol/inspector).

You can launch the MCP Inspector via [ `npm` ](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) with this command:

```bash
npx @modelcontextprotocol/inspector uv --directory /path/to/mcp_server_ros2 run mcp_ros_2_server
```

Upon launching, the Inspector will display a URL that you can access in your browser to begin debugging.
