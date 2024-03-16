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
### Detach from the container: 
```
If you are attached to the container's shell, you can detach from it without stopping the container by pressing Ctrl + P followed by Ctrl + Q. This key sequence signals Docker to detach from the container, but keeps it running in the background.
```
### Exit the shell: 
```
If you are inside the container's shell (like bash), you can simply type exit or press Ctrl + D. This will stop the shell, which, if it's the main process of the container, will stop the container as well.
```
