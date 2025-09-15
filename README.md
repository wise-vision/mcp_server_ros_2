
# WiseVision ROS2 MCP Server

[![Discord](https://img.shields.io/badge/Discord-Join%20Us-5865F2?logo=discord)](https://discord.gg/9aSw6HbUaw)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple)
[![Pro Version](https://img.shields.io/badge/Pro%20Version-Upgrade-gold)](https://app.easy.tools/ec/wisevision/wisevision-mcp-ros-2-pro)

![Flow graph](docs/assets/flow-graph.gif)

A **Python** implementation of the **Model Context Protocol (MCP)** for **ROS2**. This server enables AI tooling to connect with **ROS2** nodes, topics, and services using the **MCP** standard over **stdio**. It is designed to be **the easiest** **ROS2** server to configure **in the world**.
## 🎯 Why Choose This MCP Server?

**Save hours of development time** with native AI integration for your ROS2 projects:

- **⚡ 1-minute setup** - World's easiest ROS2 MCP configuration
- **💯 99% ROS2 compatibility** - Works with almost all ROS2 commands and message types  
- **🤖 AI-powered debugging** - Let AI help you troubleshoot ROS2 issues in real-time
- **📊 Smart data analysis** - Query your robot's sensor data using natural language
- **🚀 Boost productivity** - Control robots, analyze logs, and debug issues through AI chat
- **💡 No ROS2 expertise required** - AI translates your requests into proper ROS2 commands

**Perfect for:** Robotics developers, researchers, students, and anyone working with ROS2 who wants to leverage AI for faster development and debugging.

🚀 **Enjoying this project?**  
You’re welcome to try the **Pro version** with extra features and priority support.  
👉 [Get Pro here](https://app.easy.tools/ec/wisevision/wisevision-mcp-ros-2-pro)


# 🌍 Real-world examples:
![Demo](docs/assets/mcp-ros2-server.gif)

# ✨ Features
- List available topics 
- List available services 
- Call service
- Get messages from [WiseVision Data Black Box](https://github.com/wise-vision/wisevision_data_black_box) ([influxDB](https://www.influxdata.com) alternative to [Rosbag2](https://github.com/ros2/rosbag2))
- Subscribe topic to get messages
- Publish message on topic
- Echo message on topic
- Get fields from message type


**Note:** To call service with custom service source it before start server.


# ⚙️ Installation

Follow the [installation guide](docs/setup.md) for step-by-step instructions:
- [🧩 Install in Visual Studio Code Copilot](docs/setup.md#configure-visual-studio-code-copilot)
- [🤖 Install in Claude Desktop](docs/setup.md#configure-claude-desktop)
- [💻 Install in Warp](docs/setup.md#configure-warp)
- [🐳 Build Docker Image locally](docs/setup.md#build-docker-image-locally)



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


# 🐞 Debugging

Since MCP servers run over stdio, debugging can be challenging. For the best debugging
experience, we strongly recommend using the [MCP Inspector](https://github.com/modelcontextprotocol/inspector).

You can launch the MCP Inspector via [ `npm` ](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) with this command:

```bash
npx @modelcontextprotocol/inspector uv --directory /path/to/mcp_server_ros2 run mcp_ros_2_server
```

Upon launching, the Inspector will display a URL that you can access in your browser to begin debugging.
