#!/bin/bash

DIR=$(cd "$(dirname "$0")/.." && pwd)

xhost + && docker run --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --network=host \
    --name m3t_container \
    --cap-add=SYS_PTRACE \
    --security-opt seccomp=unconfined \
    -v "$DIR":"$DIR" \
    -v /home:/home \
    -v /mnt:/mnt \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp:/tmp \
    --ipc=host \
    -e DISPLAY=${DISPLAY} \
    -e GIT_INDEX_FILE \
    m3t_hand_pose:latest bash -c "cd $DIR && bash"
