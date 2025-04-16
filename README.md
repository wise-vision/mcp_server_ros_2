
# WiseVision ROS2 MCP Server

Python server implementing Model Context Protocol (MCP) for ROS2.

# Features
- List available topics 
- List available services 
- Call service

**Note:** To call service with custom service source it before start server.

## API

### Tools

- **ros2_topic_list**
    - Retrun list of available topics
    - Output:
        - `topic_name` (string): Topic name
        - `topic_type` (string): Message topic type

- **ros2_service_list**
    - Retruns list available services
    - Output:
        - `service_name` (string): Service name
        - `service_type` (string): Service type
        - `request_fields` (string array): Fields in service

- **ros2_service_call**
    - Call ros2 service
    - Inputs:
        - `service_name` (string): Service name
        - `service_type` (string): Service type
        - `fields` (string array): Fields in service request filled with user data
        - `force_call` (bool): Force service call without every field in service field up, Deafult set to false
    - Output
        - `result` (string): Return result of the service call
        - `error` (string): Return error in case of error
    - Features:
        - Check if service exists
        - Check if every field in service is provide


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
        "wisevision/mcp_server_ros_2"
    ],
    }

```

### Build docker image locally
```bash
git clone https://github.com/wise-vision/mcp_server_ros_2.git
cd mcp_server_ros_2
docker build -t mcp_server_ros_2 .
```
**Docker run**

Set MCP setting to mcp.json.
```json
"mcp_server_ros_2": {
    "command": "docker",
    "args": [
        "run",
        "-i",
        "--rm",
        "mcp_server_ros_2"
    ],
    }

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