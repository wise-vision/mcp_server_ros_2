# Installation guide

## Configure Visual Studio Code Copilot

This tutorial will guide you through the process of configuring Visual Studio Code Copilot, an AI-powered code completion tool for developers. Copilot integrates seamlessly with VS Code to provide intelligent code suggestions, accelerate development, and improve productivity across multiple programming languages.

### Prerequisites

Before you begin, ensure you have the following:
- ⁠ROS 2 installed on your system (humble or later). [Get ROS 2](https://docs.ros.org/en/humble/Installation.html)
- ⁠Docker installed and running. [Get Docker](https://docs.docker.com/get-docker/)
- ⁠Visual Studio Code installed. [Get Visual Studio Code](https://code.visualstudio.com/Download)

### Step 1: Install WiseVision MCP ROS 2

1. Click on the `tools` icon
2. Click on the `MCP` icon
![Go to settings](../docs/assets/vscode_add_mcp_1.png)

3. Choose type of the MCP server as `Docker Image`

![Choose docker](../docs/assets/vscode_add_mcp_2.png)

4. Enter the name of the image `mcp/ros2`

![Enter image name](../docs/assets/vscode_add_mcp_3.png)

## Configure Claude Desktop

This tutorial will guide you through the process of configuring Claude Desktop, a modern AI assistant available on Windows and macOS. Claude provides natural language interaction, context-aware reasoning, and productivity tools designed to streamline your workflow and enhance creativity directly from your desktop.

### Prerequisites

Before you begin, ensure you have the following:
- ⁠ROS 2 installed on your system (humble or later). [Get ROS 2](https://docs.ros.org/en/humble/Installation.html)
- ⁠Docker installed and running. [Get Docker](https://docs.docker.com/get-docker/)
- Claude Desktop installed. [Get Claude Desktop](https://claude.ai/download)

### Step 1: Install WiseVision MCP ROS 2
1. ⁠Open `Claude` and navigate to the settings.
2. ⁠In the settings menu, select ⁠ `Developer ⁠ from the sidebar.
3. Click on the `Edit config` in `local MCP servers`.

![Enter image name](../docs/assets/claude_add_mcp.png)
4. Paste it into config:
```json
{
  "mcp_server_ros_2": {
    "command": "docker",
    "args": [
      "run",
      "-i",
      "--rm",
      "wisevision/mcp_server_ros_2:<humble/jazzy>"
    ],
    "env": {},
    "working_directory": null,
    "start_on_launch": true
  }
}
```
To use custom messages [create folder](#add-custom-messages) and paste it into config:
```json
{
  "mcp_server_ros_2": {
    "command": "docker",
    "args": [
      "run",
      "-i",
      "--rm",
      "-v", "~/mcp_custom_messages:/app/custom_msgs"
      "wisevision/mcp_server_ros_2:<humble/jazzy>"
    ],
    "env": {},
    "working_directory": null,
    "start_on_launch": true
  }
}
```
### Step 2: Save and Enjoy

After saving you should see indicator that the MCP server is running.

## Configure WARP

This tutorial will guide you through the process of configuring WARP, a modern terminal emulator for Windows, macOS, and Linux. WARP offers a sleek interface and powerful features that enhance your command-line experience including MCP support.

### Prerequisites

Before you begin, ensure you have the following:
- ⁠ROS 2 installed on your system (humble or later). [Get ROS 2](https://docs.ros.org/en/humble/Installation.html)
- ⁠Docker installed and running. [Get Docker](https://docs.docker.com/get-docker/)
- ⁠WARP installed. [Get WARP](https://www.warp.dev/)

### Step 1: Install WiseVision MCP ROS 2

Install by cloning the repository and building the package.

```bash
git clone https://github.com/wise-vision/mcp_server_ros_2.git
cd mcp_server_ros_2
docker build -t wisevision/mcp_server_ros_2 .
```


### Step 2: Configure WARP

1. ⁠Open WARP and navigate to the settings by clicking on the gear icon in the bottom left corner.
2. ⁠In the settings menu, select ⁠ AI ⁠ from the sidebar.
3. ⁠Scroll down to the ⁠ MCP Servers ⁠ section.
4. ⁠Click on ⁠ Add ⁠ to create a new server configuration.

![Add MCP Server](../docs/assets/warp_add_mcp.png)


### Step 3: Add MCP Server

```json
{
  "mcp_server_ros_2": {
    "command": "docker",
    "args": [
      "run",
      "-i",
      "--rm",
      "wisevision/mcp_server_ros_2:<humble/jazzy>"
    ],
    "env": {},
    "working_directory": null,
    "start_on_launch": true
  }
}
```
To use custom messages [create folder](#add-custom-messages) and paste it into config:
```json
{
  "mcp_server_ros_2": {
    "command": "docker",
    "args": [
      "run",
      "-i",
      "--rm",
      "-v", "~/mcp_custom_messages:/app/custom_msgs"
      "wisevision/mcp_server_ros_2:<humble/jazzy>"
    ],
    "env": {},
    "working_directory": null,
    "start_on_launch": true
  }
}
```

### Step 4: Save and Enjoy

After saving you should see indicator that the MCP server is running. You can now test the setup by using the AI features in WARP.

# Build docker image locally:
```bash
git clone https://github.com/wise-vision/mcp_server_ros_2.git
cd mcp_server_ros_2
docker build -t mcp_server_ros_2:<humble/jazzy>  --build-arg ROS_DISTRO=<humble/jazzy> .
```

---

### Thanks for using our MCP server for ROS 2! Good luck with your projects!
---

## Add custom messages
### Create folder for custom messages
```bash
mkdir -p ~/mcp_custom_messages/src
```
### Add messages package to `src` in `mcp_custom_messages`
```bash
cd ~/mcp_custom_messages/src
git clone https://github.com/ros/ros_tutorials.git #or your custom meesage pack
```
### Build
```bash
cd ~/mcp_custom_messages
colcon build
```