FROM ros:jazzy

# Install dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    && pip install --break-system-packages opencv-python numpy


# Set up workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copy source code
COPY src/ ${ROS_WS}/src/

# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build

# Source environment and set entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
