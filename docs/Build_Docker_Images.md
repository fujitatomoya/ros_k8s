# Build Docker Images

Based on `ros:noetic` (ROS 1) and `ros:rolling` (ROS 2), we will build full ROS / ROS 2 docker images and upload those images to [dockerhub](https://hub.docker.com/).
The reason to build full docker images for ROS and ROS 2 is basic container images do not include useful samples or examples packages, docker images with full packages can be very useful for container application on Kubernetes cluster system.

This build procedure basically needs to be done only once.

## Setup Prerequisite

### [DockerHub](https://hub.docker.com/)

The following procedure requires [dockerhub](https://hub.docker.com/) access to download and upload built images.
Kubernetes can pull the container images from [dockerhub](https://hub.docker.com/) in default, but it also can be specified with private registry if necessary.
We must have account [DockerHub](https://hub.docker.com/), if you do not have one, create account.
Then, make sure [docker login](https://docs.docker.com/engine/reference/commandline/login/) works okay as following.

```bash
tomoyafujita@~/DVT >docker login
Authenticating with existing credentials...
WARNING! Your password will be stored unencrypted in /home/tomoyafujita/.docker/config.json.
Configure a credential helper to remove this warning. See
https://docs.docker.com/engine/reference/commandline/login/#credentials-store

Login Succeeded
```

see https://docs.docker.com/engine/reference/commandline/login/ for detail.

### ~~Install [Docker buildx](https://docs.docker.com/build/install-buildx/)~~

**Since `docker buildx` is under development and still experimental phase, this is not recommended.**

To build multi-arch container image for amd64 and arm64, `docker buildx` needs to be enabled.
On Linux platform such as Ubuntu, `docker buildx` is under experimental so it needs to be installed manually.
See https://github.com/docker/buildx#linux-packages to download the executable binary statically linked.

```bash

tomoyafujita@~ >mkdir ~/.docker/cli-plugins
tomoyafujita@~ >mv Downloads/buildx-v0.10.1.linux-amd64 ~/.docker/cli-plugins
tomoyafujita@~ >chmod +x ~/.docker/cli-plugins/buildx-v0.10.1.linux-amd64
tomoyafujita@~ >ls -lt ~/.docker/cli-plugins
total 48M
-rwxrwxr-x 1 tomoyafujita tomoyafujita 47M Jan 26 22:30 buildx-v0.10.1.linux-amd64*
tomoyafujita@~ >mv ~/.docker/cli-plugins/buildx-v0.10.1.linux-amd64 ~/.docker/cli-plugins/docker-buildx
tomoyafujita@~ >docker buildx

Usage:  docker buildx [OPTIONS] COMMAND

Extended build capabilities with BuildKit

Options:
      --builder string   Override the configured builder instance

Management Commands:
  imagetools  Commands to work on images in registry

Commands:
  bake        Build from a file
  build       Start a build
  create      Create a new builder instance
  du          Disk usage
  inspect     Inspect current builder instance
  ls          List builder instances
  prune       Remove build cache
  rm          Remove a builder instance
  stop        Stop builder instance
  use         Set the current builder instance
  version     Show buildx version information

Run 'docker buildx COMMAND --help' for more information on a command.
```

## ROS Noetic

The following operation needs to be done for each platform architecture, for example amd64 and arm64.

- Make sure that we can pull docker ros:noetic image.

```bash
tomoyafujita@~/DVT >docker pull ros:noetic
noetic: Pulling from library/ros
Digest: sha256:ab5d0ba8771862f65925511a61f212c7623e8b020d05fd391611b6071e0e43c2
Status: Image is up to date for ros:noetic
docker.io/library/ros:noetic
```

- Build desktop image

```bash
tomoyafujita@~/DVT/github.com/fujitatomoya >export DOCKERHUB_USERNAME="tomoyafujita"
tomoyafujita@~/DVT/github.com/fujitatomoya >cd ros_k8s/docker/
tomoyafujita@~/DVT/github.com/fujitatomoya/ros_k8s/docker >docker build --rm -f Dockerfile.noetic -t $DOCKERHUB_USERNAME/ros-noetic .
...<snip>
```

- Upload / Pull desktop image

```bash

tomoyafujita@~/DVT/github.com/fujitatomoya >export DOCKERHUB_USERNAME="tomoyafujita"
tomoyafujita@~/DVT >docker images | grep ros-noetic
tomoyafujita/ros-noetic    latest         6cda625eb50c   35 minutes ago   3.44GB
tomoyafujita@~/DVT/github.com/fujitatomoya/ros_k8s/docker >docker push $DOCKERHUB_USERNAME/ros-noetic
...<snip>
tomoyafujita@~/DVT >docker pull $DOCKERHUB_USERNAME/ros-noetic
Using default tag: latest
latest: Pulling from tomoyafujita/ros-noetic
Digest: sha256:b0b504895041e8f9893c3053211d3e497dc814189b05efcc6e3600064a7a4833
Status: Image is up to date for tomoyafujita/ros-noetic:latest
docker.io/tomoyafujita/ros-noetic:latest
```

## ROS Rolling

The following operation needs to be done for each platform architecture, for example amd64 and arm64.

- Make sure that we can pull docker ros:rolling image.

```bash
tomoyafujita@~/DVT >docker pull ros:rolling
noetic: Pulling from library/ros
Digest: sha256:ab5d0ba8771862f65925511a61f212c7623e8b020d05fd391611b6071e0e43c2
Status: Image is up to date for ros:noetic
docker.io/library/ros:noetic
```

- Build desktop image

```bash
tomoyafujita@~/DVT/github.com/fujitatomoya >export DOCKERHUB_USERNAME="tomoyafujita"
tomoyafujita@~/DVT/github.com/fujitatomoya >cd ros_k8s/docker/
tomoyafujita@~/DVT/github.com/fujitatomoya/ros_k8s/docker >docker build --rm -f Dockerfile.rolling -t $DOCKERHUB_USERNAME/ros-rolling .
...<snip>
```

- Upload / Pull desktop image

```bash
tomoyafujita@~/DVT/github.com/fujitatomoya >export DOCKERHUB_USERNAME="tomoyafujita"
tomoyafujita@~/DVT/github.com/fujitatomoya/ros_k8s/docker >docker images | grep ros-rolling
tomoyafujita/ros-rolling   latest         b5d288a52b37   29 minutes ago   3.57GB
tomoyafujita@~/DVT/github.com/fujitatomoya/ros_k8s/docker >docker push $DOCKERHUB_USERNAME/ros-rolling
...<snip>
tomoyafujita@~/DVT/github.com/fujitatomoya/ros_k8s/docker >docker pull $DOCKERHUB_USERNAME/ros-rolling
Using default tag: latest
latest: Pulling from tomoyafujita/ros-rolling
Digest: sha256:258c9a4daa46bdb5054c2858590539da124fd3543067c366ed92e745681ba9e3
Status: Image is up to date for tomoyafujita/ros-rolling:latest
docker.io/tomoyafujita/ros-rolling:latest
```

## Verify Images

- ROS talker and listener

```bash
tomoyafujita@~/DVT >export DOCKERHUB_USERNAME="tomoyafujita"
tomoyafujita@~/DVT >docker run -it --rm $DOCKERHUB_USERNAME/ros-noetic /bin/bash
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
tomoyafujita@~/DVT >export DOCKERHUB_USERNAME="tomoyafujita"
tomoyafujita@~/DVT >docker run -it --rm $DOCKERHUB_USERNAME/ros-rolling /bin/bash
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
