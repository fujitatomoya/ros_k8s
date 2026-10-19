# Build Docker Images

Based on the official ROS docker images, we will build full ROS / ROS 2 docker images and upload those images to [dockerhub](https://hub.docker.com/).
The reason to build full docker images for ROS and ROS 2 is basic container images do not include useful samples or examples packages, docker images with full packages can be very useful for container application on Kubernetes cluster system.

| Image | Base Image | Ubuntu | Note |
| --- | --- | --- | --- |
| `ros:noetic` | [ros:noetic](https://hub.docker.com/_/ros) | 20.04 | ROS 1, **EOL May 2025** |
| `ros:jazzy` | [ros:jazzy](https://hub.docker.com/_/ros) | 24.04 | ROS 2 LTS, EOL May 2029 |
| `ros:rolling` | [ros:rolling](https://hub.docker.com/_/ros) | 26.04 | ROS 2 development distribution |
| `ros:jazzy-gazebo` | [ros:jazzy](https://hub.docker.com/_/ros) | 24.04 | ROS 2 Jazzy with Gazebo Harmonic (`ros-jazzy-ros-gz`) for [Gazebo Simulation](./ROS2_Gazebo_Simulation.md) |

To build and upload all ROS container images, you can use [build_docker_images.sh](../scripts/build_docker_images.sh).

```bash
### Build and upload the images for the host architecture
> export DOCKERHUB_USERNAME="<your dockerhub username>"
> ./scripts/build_docker_images.sh

### Build and upload the images, and update the multi-arch manifest
> ./scripts/build_docker_images.sh -m

### Build and upload multi-arch (amd64/arm64) images at once with buildx
> ./scripts/build_docker_images.sh -x
```

## Setup Prerequisite

### [DockerHub](https://hub.docker.com/)

The following procedure requires [dockerhub](https://hub.docker.com/) access to download and upload built images.
Kubernetes can pull the container images from [dockerhub](https://hub.docker.com/) in default, but it also can be specified with private registry if necessary.
We must have account [DockerHub](https://hub.docker.com/), if you do not have one, create account.
Then, make sure [docker login](https://docs.docker.com/reference/cli/docker/login/) works okay as following.

```bash
> docker login
...<snip>
Login Succeeded
```

see https://docs.docker.com/reference/cli/docker/login/ for detail.

### [Docker buildx](https://docs.docker.com/build/concepts/overview/) (Multi-Arch Build)

`docker buildx` is included in Docker Engine as `docker-buildx-plugin` package, see [Install Kubernetes Packages](./Install_Kubernetes_Packages.md#docker--containerd).
To build multi-arch container image for amd64 and arm64 on the single host system, QEMU user-mode emulation and a `docker-container` builder are required.

```bash
> docker buildx version
github.com/docker/buildx v0.2x.x xxxxxxx

### Install QEMU emulators for the foreign architectures
> docker run --privileged --rm tonistiigi/binfmt --install all

### Create multi-arch builder
> docker buildx create --name multiarch --driver docker-container --use
> docker buildx inspect --bootstrap
...<snip>
Platforms: linux/amd64, linux/arm64, ...
```

Building the foreign architecture image with QEMU is much slower than the native build, building the images on each native platform and creating the manifest (described below) is the alternative.

## ROS Noetic

> [!WARNING]
> ROS Noetic reached end of life in May 2025, `ros:noetic` base image is no longer updated.

The following operation needs to be done for each platform architecture, for example amd64 and arm64.

- Make sure that we can pull docker ros:noetic image.

```bash
> docker pull ros:noetic
noetic: Pulling from library/ros
Digest: sha256:ab5d0ba8771862f65925511a61f212c7623e8b020d05fd391611b6071e0e43c2
Status: Image is up to date for ros:noetic
docker.io/library/ros:noetic
```

- Build desktop image

```bash
> export DOCKERHUB_USERNAME="tomoyafujita"
> export ARCH=$(dpkg --print-architecture)
> echo $ARCH
amd64
> cd ros_k8s/
> docker build --rm -f ./docker/Dockerfile.noetic -t $DOCKERHUB_USERNAME/ros:noetic-$ARCH .
...<snip>
```

- Upload desktop image

```bash
> docker push $DOCKERHUB_USERNAME/ros:noetic-$ARCH
```

- Create and push manifest

  This operation needs to be done just once to support multiple architecture image.
  Here it describes how to support multiple architecture amd64 and arm64 platform.

```bash
> docker manifest create $DOCKERHUB_USERNAME/ros:noetic $DOCKERHUB_USERNAME/ros:noetic-amd64 --amend $DOCKERHUB_USERNAME/ros:noetic-arm64
> docker manifest inspect $DOCKERHUB_USERNAME/ros:noetic
> docker manifest push $DOCKERHUB_USERNAME/ros:noetic
```

- Pull desktop image

  If multiple architecture images are successfully uploaded, the following pull command will download the appropriate architecture image based on your machine.

```bash
> docker pull $DOCKERHUB_USERNAME/ros:noetic
```

## ROS Jazzy

ROS 2 Jazzy Jalisco is the LTS distribution based on Ubuntu 24.04, which is the same Ubuntu release with the host system in this tutorial.
The procedure is the same with ROS Rolling below, just replace `rolling` with `jazzy`.

```bash
> docker pull ros:jazzy
> docker build --rm -f ./docker/Dockerfile.jazzy -t $DOCKERHUB_USERNAME/ros:jazzy-$ARCH .
> docker push $DOCKERHUB_USERNAME/ros:jazzy-$ARCH
> docker manifest create $DOCKERHUB_USERNAME/ros:jazzy $DOCKERHUB_USERNAME/ros:jazzy-amd64 --amend $DOCKERHUB_USERNAME/ros:jazzy-arm64
> docker manifest push $DOCKERHUB_USERNAME/ros:jazzy
```

## ROS Jazzy with Gazebo

[Dockerfile.jazzy-gazebo](./../docker/Dockerfile.jazzy-gazebo) adds [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/) simulator and [ros_gz](https://github.com/gazebosim/ros_gz) integration (`ros-jazzy-ros-gz`) to the ROS Jazzy full image, that is used by [ROS 2 Gazebo Simulation](./ROS2_Gazebo_Simulation.md).
The image is around 7GB since it includes the simulator, GUI and rendering libraries.

```bash
> docker build --rm -f ./docker/Dockerfile.jazzy-gazebo -t $DOCKERHUB_USERNAME/ros:jazzy-gazebo-$ARCH .
> docker push $DOCKERHUB_USERNAME/ros:jazzy-gazebo-$ARCH
> docker manifest create $DOCKERHUB_USERNAME/ros:jazzy-gazebo $DOCKERHUB_USERNAME/ros:jazzy-gazebo-amd64 --amend $DOCKERHUB_USERNAME/ros:jazzy-gazebo-arm64
> docker manifest push $DOCKERHUB_USERNAME/ros:jazzy-gazebo

### Check Gazebo and ros_gz in the image
> docker run -it --rm $DOCKERHUB_USERNAME/ros:jazzy-gazebo bash -c 'source /opt/ros/jazzy/setup.bash && gz sim --version && ros2 pkg list | grep ros_gz'
Gazebo Sim, version 8.15.0
...<snip>
ros_gz_bridge
ros_gz_sim
...<snip>
```

## ROS Rolling

The following operation needs to be done for each platform architecture, for example amd64 and arm64.

- Make sure that we can pull docker ros:rolling image.

```bash
> docker pull ros:rolling
rolling: Pulling from library/ros
Digest: sha256:...
Status: Image is up to date for ros:rolling
docker.io/library/ros:rolling
```

- Build desktop image

```bash
> export DOCKERHUB_USERNAME="tomoyafujita"
> export ARCH=$(dpkg --print-architecture)
> echo $ARCH
amd64
> cd ros_k8s/
> docker build --rm -f ./docker/Dockerfile.rolling -t $DOCKERHUB_USERNAME/ros:rolling-$ARCH .
...<snip>
```

- Upload desktop image

```bash
> docker push $DOCKERHUB_USERNAME/ros:rolling-$ARCH
```

- Create and push manifest

  This operation needs to be done just once to support multiple architecture image.
  Here it describes how to support multiple architecture amd64 and arm64 platform.

```bash
> docker manifest create $DOCKERHUB_USERNAME/ros:rolling $DOCKERHUB_USERNAME/ros:rolling-amd64 --amend $DOCKERHUB_USERNAME/ros:rolling-arm64
> docker manifest inspect $DOCKERHUB_USERNAME/ros:rolling
> docker manifest push $DOCKERHUB_USERNAME/ros:rolling
```

- Pull desktop image

  If multiple architecture images are successfully uploaded, the following pull command will download the appropriate architecture image based on your machine.

```bash
> docker pull $DOCKERHUB_USERNAME/ros:rolling
```

## Multi-Arch Build with buildx

Instead of building the image on each platform and creating the manifest manually, `docker buildx` can build and push the multi-arch image at once.

```bash
> docker buildx build --platform linux/amd64,linux/arm64 -f ./docker/Dockerfile.rolling -t $DOCKERHUB_USERNAME/ros:rolling --push .
> docker buildx imagetools inspect $DOCKERHUB_USERNAME/ros:rolling
Name:      docker.io/tomoyafujita/ros:rolling
MediaType: application/vnd.oci.image.index.v1+json
...<snip>
  Platform:  linux/amd64
...<snip>
  Platform:  linux/arm64
```

## Verify Images

Before deploying container images via Kubernetes, it would be nice to check if the container images can be running on the platform.

- ROS talker and listener

```bash
> export DOCKERHUB_USERNAME="tomoyafujita"
> docker run -it --rm $DOCKERHUB_USERNAME/ros:noetic /bin/bash
root@ffedcfd31e7c:/# source /opt/ros/noetic/setup.bash
root@ffedcfd31e7c:/# roscore &
...<snip>
started core service [/rosout]

root@ffedcfd31e7c:/# rosrun roscpp_tutorials talker & rosrun rospy_tutorials listener.py
[2] 292
[ INFO] [1671522107.034150075]: hello world 0
[ INFO] [1671522107.134188598]: hello world 1
[ INFO] [1671522107.234184871]: hello world 2
[ INFO] [1671522107.334181908]: hello world 3
[ INFO] [1671522107.434191743]: hello world 4
[ INFO] [1671522107.534171889]: hello world 5
[INFO] [1671522107.534725]: /listener_293_1671522107280I heard hello world 5
[ INFO] [1671522107.634173819]: hello world 6
[INFO] [1671522107.634479]: /listener_293_1671522107280I heard hello world 6
[ INFO] [1671522107.734172475]: hello world 7
[INFO] [1671522107.734669]: /listener_293_1671522107280I heard hello world 7
...<snip>
```

- ROS 2 talker and listener

```bash
> export DOCKERHUB_USERNAME="tomoyafujita"
> docker run -it --rm $DOCKERHUB_USERNAME/ros:rolling /bin/bash
root@266162be906b:/# source /opt/ros/rolling/setup.bash
root@266162be906b:/# ros2 run demo_nodes_cpp talker & ros2 run demo_nodes_py listener
[1] 88
[INFO] [1671521236.301471647] [talker]: Publishing: 'Hello World: 1'
[INFO] [1671521236.329742222] [listener]: I heard: [Hello World: 1]
[INFO] [1671521237.301410324] [talker]: Publishing: 'Hello World: 2'
[INFO] [1671521237.302500635] [listener]: I heard: [Hello World: 2]
[INFO] [1671521238.301440713] [talker]: Publishing: 'Hello World: 3'
[INFO] [1671521238.303480752] [listener]: I heard: [Hello World: 3]
[INFO] [1671521239.301415859] [talker]: Publishing: 'Hello World: 4'
[INFO] [1671521239.302524654] [listener]: I heard: [Hello World: 4]
^C[INFO] [1671521240.301487315] [talker]: Publishing: 'Hello World: 5'
...<snip>
```
