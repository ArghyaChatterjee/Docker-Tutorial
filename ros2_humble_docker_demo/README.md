## ROS2 Docker Demo

## Requirements
This test was performed on Ubuntu 24.04, ROS2 Jazzy and Python 3.12 while the docker had Ubuntu 22.04, ROS2 Humble and Python 3.10.

## Objective: 

Run `talker` (publisher) on the host using ROS2 Jazzy, and `listener` (subscriber) inside Docker using ROS 2 Humble.

## Step-by-Step Instructions

### On Host System (ROS 2 Jazzy installed natively):

1. **Source your ROS environment:**

```bash
source /opt/ros/jazzy/setup.bash
```

2. **Run the talker node:**

```bash
ros2 run demo_nodes_cpp talker
```

This node publishes to `/chatter`.

---

### In a **Docker Container** (with ROS 2 Humble):
1. **Build the docker**

```bash
docker build -t ros2_humble_demo .
# Build Docker image with logs 
# docker build --no-cache --progress=plain -f Dockerfile -t ros2_humble_demo . 2>&1 | tee docker_build.log
```

2. **Run the container:**

```bash
docker run -it --rm \
  --env ROS_DOMAIN_ID=166 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --volume /dev/shm:/dev/shm \
  --name ros2_listener \
  ros2_humble_demo \
  bash
```

> `--net=host` allows the container to use the host's network stack, so they share DDS multicast/broadcast for ROS 2 communication.

3. **Inside the container shell:**

```bash
source /opt/ros/humble/setup.bash
apt update
apt install ros-humble-demo-nodes-cpp
ros2 run demo_nodes_cpp listener
```

---

### See logs like:

```
[INFO] [listener]: I heard: "Hello World: 42"
```

This confirms that:
- The talker on your host is broadcasting.
- The listener inside the container is subscribing successfully.
