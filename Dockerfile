FROM wisevision/ros_with_wisevision_msgs_and_wisevision_core:humble

RUN apt-get update && apt-get install -y python3-pip build-essential ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN pip install uv

WORKDIR /app
COPY . /app

RUN uv venv

RUN uv sync

ENTRYPOINT []
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /root/wisevision_ws/install/setup.bash && source .venv/bin/activate && python3 -m server.server"]