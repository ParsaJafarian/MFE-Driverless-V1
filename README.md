# MFE25-Driverless-V0
This repository contains bash scripts to setup a ROS2 environment in a Docker container. The scripts are tested on Ubuntu 20.04 and ROS2 Jazzy.

## How to get started
1. When at the root of the repo, build the Docker image by running the following command:
```bash
docker build -t <image_name> .
```
2. Create a new container by running the image:
```bash
bash run_image.sh <image_name>
```
3. The Docker container is now running. You can now run ROS2 commands in the container.
4. To restart the container, run the following command:
```bash
docker start -i {container_id or container_name}
```
