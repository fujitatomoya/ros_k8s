#!/bin/bash

### enable debug
#set -exvfC
set -e

###################################################
# This script builds and uploads ROS docker images.
###################################################

################
# User Setting #
################

DOCKERHUB_USERNAME="${DOCKERHUB_USERNAME:-tomoyafujita}"
ARCH=$(dpkg --print-architecture)

ros_distros=(
    "noetic"
    "jazzy"
    "rolling"
    "jazzy-gazebo"
)

build_manifest=false
build_multiarch=false

### Functions Implementation

function print_usage() {
    echo "Usage: $0 [-m] [-x]"
    echo "Options(default):"
    echo "  -m : build and upload manifest along with container images for multi-arch support. (default: false)"
    echo "  -x : build and upload multi-arch (amd64/arm64) images at once with docker buildx, requires QEMU. (default: false)"
    exit 1
}

function exit_trap() {
    if [ $? != 0 ]; then
        echo "Command [$BASH_COMMAND] is failed"
        exit 1
    fi
}

function check_dockerhub_setting () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: checking dockerhub setting and configuration."
    if [ -z "$DOCKERHUB_USERNAME" ]; then
        echo "DOCKERHUB_USERNAME is not set."
        exit 1
    fi
    # check if docker login succeeds
    docker login
}

function build_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: building ROS docker container images."
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro image building for $ARCH"
        docker build --rm -f ./docker/Dockerfile.$distro -t $DOCKERHUB_USERNAME/ros:$distro-$ARCH .
    done
    echo "----- all images successfully generated!!! -----"
}

function build_manifest_for_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: building multi-arch manifest ROS docker container images."
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro manifest"
        docker manifest create $DOCKERHUB_USERNAME/ros:$distro $DOCKERHUB_USERNAME/ros:$distro-amd64 --amend $DOCKERHUB_USERNAME/ros:$distro-arm64
        docker manifest inspect $DOCKERHUB_USERNAME/ros:$distro
        docker manifest push $DOCKERHUB_USERNAME/ros:$distro
    done
    echo "----- all manifest successfully updated!!! -----"
}

function upload_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: uploading ROS docker container images."
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro image uploading for $ARCH"
        docker push $DOCKERHUB_USERNAME/ros:$distro-$ARCH
    done
    echo "----- all images successfully uploaded!!! -----"
}

function build_multiarch_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: building and uploading multi-arch ROS docker container images with buildx."
    # see docs/Build_Docker_Images.md for QEMU and builder setup
    if ! docker buildx inspect multiarch &> /dev/null; then
        docker buildx create --name multiarch --driver docker-container --use
        docker buildx inspect --bootstrap
    fi
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro multi-arch image building and uploading"
        docker buildx build --builder multiarch --platform linux/amd64,linux/arm64 \
            -f ./docker/Dockerfile.$distro -t $DOCKERHUB_USERNAME/ros:$distro --push .
        docker buildx imagetools inspect $DOCKERHUB_USERNAME/ros:$distro
    done
    echo "----- all multi-arch images successfully uploaded!!! -----"
}

### Main

# set the trap on error
trap exit_trap ERR

# parse command line options
while getopts ":mx" opt; do
    case $opt in
        m)
            build_manifest=true
            ;;
        x)
            build_multiarch=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG"
            print_usage
            ;;
    esac
done
shift $((OPTIND-1))

# always run from the repository root
cd "$(dirname "$0")/.."

echo "Check Docker Environment ----------"
check_dockerhub_setting

if [ "$build_multiarch" == true ]; then
    echo "Building Multi-Arch Container Images ----------"
    build_multiarch_images
    exit 0
fi

echo "Building Container Images ----------"
build_images

echo "Uploading Container Images ----------"
upload_images

if [ "$build_manifest" == true ]; then
    echo "Building Container Manifest ----------"
    build_manifest_for_images
fi

exit 0
