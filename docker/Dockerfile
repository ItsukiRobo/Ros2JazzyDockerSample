FROM ros:jazzy

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-venv python3-pip \
    ros-dev-tools \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-rosbag2 \
 && rm -rf /var/lib/apt/lists/*

# venv に rosbags
RUN python3 -m venv /opt/venv \
 && /opt/venv/bin/python -m pip install --upgrade pip \
 && /opt/venv/bin/pip install --no-cache-dir rosbags catkin_pkg

ENV PATH="/opt/venv/bin:${PATH}"

WORKDIR /ros2_ws
COPY ros2_ws /ros2_ws

RUN rm -rf /ros2_ws/build /ros2_ws/install /ros2_ws/log

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]