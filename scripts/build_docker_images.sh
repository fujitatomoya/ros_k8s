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
    "rolling"
)

build_manifest=false

### Functions Implementation

function print_usage() {
    echo "Usage: $0 [-m]"
    echo "Options(default):"
    echo "  -m : build and upload manifest along with container images for multi-arch support. (default: false)"
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

### Main

# set the trap on error
trap exit_trap ERR

# parse command line options
while getopts ":m" opt; do
    case $opt in
        m)
            build_manifest=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG"
            print_usage
            ;;
    esac
done
shift $((OPTIND-1))

echo "Check Docker Environment ----------"
check_dockerhub_setting

echo "Buiding Container Images ----------"
build_images

if [ "$build_manifest" == true ]; then
    echo "Building Container Manifest ----------"
    build_manifest_for_images
fi

echo "Uploading Container Images ----------"
upload_images

exit 0
