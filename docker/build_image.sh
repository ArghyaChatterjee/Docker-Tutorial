#!/bin/bash

# Build the Docker image
echo "Building Docker image 'm3t_image:latest'..."
docker build -t m3t_image:latest . || { echo "Docker build failed"; exit 1; }

echo "Docker image built successfully!"

# Make sure the run_container.sh script is executable
# chmod +x run_container.sh

# Run the container
# ./run_container.sh << EOF

# Inside the container: navigate and build
# cd M3T
# mkdir -p build && cd build
# cmake ..
# make -j\$(nproc)

# EOF

# echo "Container started. You are now inside your Docker container."

