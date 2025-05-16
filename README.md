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
### Install
Configure the production repository:
```
ARCH=$(dpkg --print-architecture) && \
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && \
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed "s#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g" | \
  sed "s#\\\$(ARCH)#${ARCH}#g" | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
```
Optionally, configure the repository to use experimental packages:
```
sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
Update the packages list from the repository:
```
sudo apt-get update
```
Install the NVIDIA Container Toolkit packages:
```
sudo apt-get install -y nvidia-container-toolkit
```
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
### Build the docker image
Build the docker image with cache without filename and logs:
```
docker build -t <docker_name>:0.0.2 .
```
Build the docker image without filename, cache and logs:
```
docker build --no-cache -t <docker_name>:0.0.2 .
```
Build the docker with file names without cache and logs:
```
docker build --no-cache -f Dockerfile.x -t <docker_name>:0.0.2 .
```
Build the docker with file names and logs without cache:
```
docker build --no-cache --progress=plain -f Dockerfile.realsense -t foundationpose-ros2:0.0.2 . 2>&1 | tee docker_build.log
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
## Access Docker Container to and from Host
After cloning the current repo, you have to set the `$ROS_DOMAIN_ID` correctly to the host and docker. Set it to the host terminal and inside the `.bashrc` file:
```
export ROS_DOMAIN_ID=166
```
This assumes the docker is being run as `root` and the host is being run as `$USER`. Now, build the docker container:
```
cd ros2_jazzy_docker_demo
docker build -t ros2_jazzy_demo .
```
Now, run the docker container:
```
cd ros2_docker_demo
docker run -it --rm   --env ROS_DOMAIN_ID=166   --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp   --volume /dev/shm:/dev/shm   --name ros2_listener   ros2_jazzy_demo   bash
apt update
apt install ros-jazzy-demo-nodes-cpp
```
Then, try the first demo:
```
ros2 run demo_nodes_cpp talker
```
You will see:
```
root@arghya:/ros2_ws# ros2 run demo_nodes_cpp talker
[INFO] [1746566639.785472203] [talker]: Publishing: 'Hello World: 1'
[INFO] [1746566640.785383631] [talker]: Publishing: 'Hello World: 2'
[INFO] [1746566641.785467609] [talker]: Publishing: 'Hello World: 3'
[INFO] [1746566642.785453459] [talker]: Publishing: 'Hello World: 4'
```
On the host machine, run this (assumes ros2 is installed):
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run demo_nodes_cpp listener
```
You will see:
```
arghya@arghya:~$ ros2 run demo_nodes_cpp listener
[INFO] [1746566944.554639897] [listener]: I heard: [Hello World: 1]
[INFO] [1746566945.554418037] [listener]: I heard: [Hello World: 2]
[INFO] [1746566946.554225905] [listener]: I heard: [Hello World: 3]
[INFO] [1746566947.554156548] [listener]: I heard: [Hello World: 4]
```
If you want to use `--net=host`, then you have to do the following:
```
docker run -it --rm   --net=host   --ipc=host   --env ROS_DOMAIN_ID=166   --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp   --volume /dev/shm:/dev/shm   --name ros2_listener   ros2_jazzy_demo   bash
```
If it's not working, the problem is due to the SHM transport. When you launch a Docker with --net=host and --ipc=host configuration, the publisher and subscriber detect that they are on the same host and try to use SHM. This is due to how the calculation of SHM usage was done until now. In addition, SHM uses the `/dev/shm` directory, which in this case is being attempted to be shared by users who have different permissions: `root` in the case of Docker and `$USER` in the case of the host. This causes communication to fail.

To fix this you have several quick solutions:

- Have the same user on Docker as on the host. For example, docker and host both will have `$USER` or they both will have `root`. 
- Launch the ROS 2 application as root (equivalent to the above but without changing the Docker configuration).
- Use UDP instead of SHM. This is as simple as running ```- export FASTDDS_BUILTIN_TRANSPORTS=UDPv4``` before launching the ROS 2 application in the Docker.
- Do not set --ipc=host --net=host when running the Docker container.

You can also disable the `shm`. These are the following ways:

- Clean SHM segments: Run this to clear leftover shared memory files:
```
sudo find /dev/shm/ -name 'fastrtps_*' -exec rm -f {} \;
```
- Disable SHM transport (as a workaround): You can disable Fast DDS shared memory transport (uses UDP instead). Add this to your environment:
```
export FASTRTPS_DEFAULT_PROFILES_FILE=""
```
Or create a profile that disables SHM:
```
<!-- ~/.ros/disable_shm.xml -->
<profiles>
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
</profiles>
```
And set:

```
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/disable_shm.xml
```
- Check Docker Shared Memory (if running in container): If you're running the ros2 node in Docker, make sure the container has access to enough shared memory:

```
docker run --shm-size=512m ...  
```
A detailed description is described [here](https://github.com/eProsima/Fast-DDS/issues/5396).

## Change Permission for Docker Generated Files
If you run a process inside the docker container, it may generate some files. Here is how you want to change the access permission to those files /folder:
```
sudo chown -R $USER:$USER ./<folder name>
```
## Sources
- https://github.com/ArghyaChatterjee/SubT-Docker-Setup
- https://github.com/NVlabs/BundleSDF/tree/master/docker
- https://github.com/leggedrobotics/rsl_panoptic
- https://github.com/dfki-ric/docker_image_development
