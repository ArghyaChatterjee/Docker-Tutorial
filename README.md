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
docker container ls
```
### Detach from the container
```
You can detach from it without stopping the container by pressing Ctrl + P followed by Ctrl + Q if you are attached to the container's shell. This key sequence signals Docker to detach from the container, but keeps it running in the background.
```
### Exit the shell & Container
```
Type exit or press Ctrl + D. This will stop the shell, which, if it's the main process of the container, will stop the container as well.
```
### Remove the container
```
docker rm <container_name>
```
### Remove the image
```
docker rmi <image_name>
```
### Docker directory in system
```
Docker stores its images, containers, volumes, and other data in /var/lib/docker/
```
