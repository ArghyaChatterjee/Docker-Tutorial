# All-About-Docker
This repository is all about docker cheat sheets.

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
