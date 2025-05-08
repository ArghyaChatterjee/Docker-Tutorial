FROM ros:humble

# Install dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    && pip install opencv-python numpy


# Set up workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copy source code
COPY src/ ${ROS_WS}/src/

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Source environment and set entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
