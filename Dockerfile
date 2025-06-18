FROM wisevision/ros_with_wisevision_msgs_and_wisevision_core:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    ca-certificates \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-action-msgs \
    ros-humble-diagnostic-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-visualization-msgs \
    ros-humble-example-interfaces \
    ros-humble-rclpy \
    ros-humble-ros2cli \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip install uv

WORKDIR /app
COPY . /app

RUN uv venv

RUN uv sync

ENTRYPOINT []
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /root/wisevision_ws/install/setup.bash && source .venv/bin/activate && uv run mcp_ros_2_server"]