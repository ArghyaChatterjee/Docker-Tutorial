#!/bin/bash

DOCKER_DIR=$(cd "$(dirname \"$0\")" && pwd)
PROJECT_DIR=$(cd "$DOCKER_DIR/.." && pwd)

# Build the Docker image
echo "Building Docker image 'm3t_image:latest'..."
docker build -t m3t_image:latest "$DOCKER_DIR" || { echo "Docker build failed"; exit 1; }

# Build Docker image with logs without any cache 
# docker build --no-cache --progress=plain -f Dockerfile.realsense -t m3t_image:0.0.2 . 2>&1 | tee docker_build.log

echo "Docker image built successfully!"

# Allow GUI connections
xhost +

# Run the Docker container interactively and automatically build the M3T project
echo "Starting Docker container 'm3t_container' and building the M3T project..."
docker run --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --network=host \
    --name m3t_container \
    --cap-add=SYS_PTRACE \
    --security-opt seccomp=unconfined \
    -v "$PROJECT_DIR":"$PROJECT_DIR" \
    -v /home:/home \
    -v /mnt:/mnt \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp:/tmp \
    --ipc=host \
    -e DISPLAY=${DISPLAY} \
    -e GIT_INDEX_FILE \
    m3t_image:latest bash -c "cd $PROJECT_DIR && mkdir -p build && cd build && cmake .. && make -j\$(nproc) && exec bash"

echo "Docker container started and M3T project built successfully!"

