# Docker Tutorial
This repository is all about docker tutorial and cheat sheets.

## Install Docker
- Set up Docker's apt repository:

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
- Install the Docker packages:

To install the latest version, run:
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
Add current user to the docker group:
```
sudo groupadd docker      
sudo usermod -aG docker $USER
newgrp docker
```
- Verify Installation

Verify that the installation is successful by running the hello-world image:
```bash
docker run hello-world
```
## Install Nvidia Container Toolkit
### Configuring Docker
Configure the container runtime by using the `nvidia-ctk` command:
```
sudo nvidia-ctk runtime configure --runtime=docker
```
The nvidia-ctk command modifies the `/etc/docker/daemon.json` file on the host. The file is updated so that Docker can use the NVIDIA Container Runtime.

Restart the Docker daemon:
```
sudo systemctl restart docker
```
### Rootless mode
To configure the container runtime for Docker running in Rootless mode, follow these steps:

Configure the container runtime by using the nvidia-ctk command:
```
nvidia-ctk runtime configure --runtime=docker --config=$HOME/.config/docker/daemon.json
```
Restart the Rootless Docker daemon:
```
systemctl --user restart docker
```
Configure `/etc/nvidia-container-runtime/config.toml` by using the `sudo nvidia-ctk` command:
```
sudo nvidia-ctk config --set nvidia-container-cli.no-cgroups --in-place
```
## Running Docker 
### Build Image and Run Container
In this repo, there is a sample Dockerfile. Build it this way:
```
cd docker
chmod +x build_image.sh
./build_image.sh
```
Then, run a container from the image that you built:
```
chmod +x run_container.sh
./run_container.sh
```
### Build and Run Image and Attach Container
There is a another sample Dockerfile. Build image and run container in one go:
```
cd docker
chmod +x build_image_and_run_container.sh
./build_image_and_run_container.sh
```
Then, enter into the already existing container:
```
chmod +x attach_container.sh
./attach_container.sh
```
## Docker Cheat Sheet
### If your system's memory is full due to docker:
```
docker system prune
```
### Ways to see docker images
```
docker image ls
```
### Ways to see docker container
```
docker ps
## Or
docker container ls
```
### Detach from the container
```
You can detach from it without stopping the container by pressing Ctrl + P followed by Ctrl + Q if you are attached to the container's shell. This key sequence signals Docker to detach from the container, but keeps it running in the background.
```
### Exit the shell of the container
```
Type exit or press Ctrl + D. This will stop the shell, which, if it's the main process of the container, will stop the container as well.
```
### Run the docker container
```
docker run -d --name <container_name> <image_name> tail -f /dev/null
```
### Start the docker container
```
docker exec -it synthetic_data_container bash
```
### Stop the docker container
```
docker stop <container-name-or-id>
```
### Remove the container
```
docker rm <container_name>
```
### Remove the image
```
docker rmi <image_name>
```
### Force remove the image
```
docker rmi -f synthetic-data-gen
```
### Docker directory in system
```
Docker stores its images, containers, volumes, and other data in /var/lib/docker/
```
### Stop and Remove All Running Containers
```
docker container stop $(docker container ls -aq)
docker container rm $(docker container ls -aq)
```
### Remove All Images
```
docker rmi $(docker image ls -aq)
```
### Remove Build Cache
```
docker builder prune -f
```
### Remove All Volumes
```
docker volume rm $(docker volume ls -q)
```
### Remove All Networks
```
docker network prune -f
```
### Remove Everything at Once
If you want to remove everything (containers, images, volumes, and networks) in one go, you can use:
```
docker system prune -a --volumes -f
```
