[![build-docker-images](https://github.com/fujitatomoya/ros_k8s/actions/workflows/build-docker-images.yml/badge.svg)](https://github.com/fujitatomoya/ros_k8s/actions/workflows/build-docker-images.yml)

# ROS Kubernetes / KubeEdge

## Abstract

This repository provides tutorials how to use ROS and ROS 2 with Kubernetes and KubeEdge Cluster System.
User might need to have knowledge about Kubernetes to understand what is really going on the cluster system.

## Motivation

The primary goal for this repository is that everyone can try ROS and ROS 2 with Kubernetes and KubeEdge Cluster.
Using container images and container orchestration allows application developer to be agnostic from system platform but only application logic.
ROS and ROS 2 provides good isolation between nodes, so that we can take the most advantage of application runtime framework and container orchestration.

To share the information and experience for this can bring more use cases and requirements, so that we can develop more robotics and robot requirements for container orchestration.
The concept to bring `Cloud Native Container Orchestration` to edge IoT devices are already topic and discussed in Kubernetes Edge IoT WG.
But key point here is robot and robotics, which can be much more complicated than usual edge IoT devices such as complicated and special devices, hardware acceleration and so on.
To break down the requirements for robot and robotics would be very much useful for ROS and ROS 2 user, also Kubernetes container orchestration for edge IoT use cases.

## Supported ROS / ROS 2 Distribution

Since Kubernetes uses container images and agnostic from it, any distribution should be supported.
But the following is primary supported distribution of ROS and ROS 2.

- [ROS Noetic](http://wiki.ros.org/noetic)
- [ROS Rolling](https://docs.ros.org/en/rolling/)

## Cluster System Configuration

We can use either `Physical Host System` or `Virtualized Instance`.

- Host System

This is practical environment for robot and robotics in general.
This will set up Kubernetes cluster on host system as followings.

| Hostname | IP Address | OS | architecture |
| --- | --- | --- | --- |
| tomoyafujita | 192.168.1.248 | Ubuntu 22.04 (Ubuntu 20.04) | amd64 |
| ubuntu | 192.168.1.79 | Ubuntu 20.04 | aarch64 (Raspberry Pi4) |

- Virtualized Instance

This is virtualized Kubernetes cluster environment which can be established on single physical host.
We can use this environment if container requires pure computation resource but hardware or physical sensor devices.
This environment is very much useful to try or test your container images or services to make sure if those work as expected, and easy to break down whole Kubernetes cluster with one command.

## Tutorials

- [Install Kubernetes Packages](./docs/Install_Kubernetes_Packages.md)
- [Build ROS / ROS 2 Full Docker Multi-Arch Images](./docs/Build_Docker_Images.md)
  - [ROS Noetic Container Build](./docs//Build_Docker_Images.md#ros-noetic)
  - [ROS Rolling Container Build](./docs//Build_Docker_Images.md#ros-rolling)
- [Setup Kuberenetes Cluster](./docs/Setup_Kubernetes_Cluster.md)
  - [Container Network Interface (CNI)](./docs/Setup_Kubernetes_Cluster.md#container-network-interface-cni)
  - [Setup Kubernetes API Server](./docs/Setup_Kubernetes_Cluster.md#setup-kubernetes-api-server)
  - [Join Cluster from Worker Node](./docs/Setup_Kubernetes_Cluster.md#join-the-cluster)
  - [Deploy Container Network Interface](./docs/Setup_Kubernetes_Cluster.md#deploy-cni-plugin)
  - [Deploy Kubernetes Dashboard](./docs/Setup_Kubernetes_Cluster.md#kubernetes-dashboard)
- [Setup KubeEdge Cloud/Edge Node](./docs/Setup_KubeEdge.md)
  - [Setup Kubernetes API server](./docs/Setup_KubeEdge.md#setup-kubernetes-api-server)
  - [Setup KubeEdge Cloudcore and Edgecore](./docs/Setup_KubeEdge.md#setup-kubeedge)
- [Setup Virtualized Kuberenetes Cluster](./docs/Setup_Virtualized_Cluster.md)
  - [What is Kubernetes IN Docker](./docs/Setup_Virtualized_Cluster.md#kubernetes-in-docker)
  - [Single Node Cluster Setup](./docs/Setup_Virtualized_Cluster.md#single-node-cluster-setup)
  - [Multiple Nodes Cluster Setup](./docs/Setup_Virtualized_Cluster.md#multiple-node-cluster-setup)
- [ROS Deployment Demonstration](./docs/ROS_Deployment_Demonstration.md)
  - [ROS DaemonSet Deployment](./docs/ROS_Deployment_Demonstration.md#ros-daemonset-deployment-with-cni)
  - [ROS Multi-Node Deployment](./docs/ROS_Deployment_Demonstration.md#ros-multi-node-deployment-with-cni)
  - [ROS Multi-Node Deployment with Host Network](./docs/ROS_Deployment_Demonstration.md#ros-multi-node-deployment-with-host-network)
- [ROS 2 Deployment Demonstration](./docs/ROS2_Deployment_Demonstration.md)
  - [ROS 2 Simple Distributed System](./docs/ROS2_Deployment_Demonstration.md#ros-2-simple-distributed-system)
  - [ROS 2 Localhost Only](./docs/ROS2_Deployment_Demonstration.md#ros-2-localhost-only)
  - [ROS 2 Logical Partition / Multiple RMW Implementation](./docs/ROS2_Deployment_Demonstration.md#ros-2-logical-partition--multiple-rmw-implementation)
- [ROS 2 Deployment Intermediate](./docs/ROS2_Deployment_Intermediate.md)
  - [ROS 2 GUI Display Access](./docs/ROS2_Deployment_Intermediate.md#ros-2-gui-display-access)
  - [ROS 2 Zero Copy Data Sharing](./docs/ROS2_Deployment_Intermediate.md#ros-2-zero-copy-data-sharing)
  - [ROS 2 Fast-DDS Discovery Server](./docs/ROS2_Deployment_Intermediate.md#ros-2-fast-dds-discovery-server)

## Slides / Talks

- [ROS with Kubernetes Introduction in ROS TSC Meeting](https://www.slideshare.net/FujitaTomoya/rostscrosk8s20230309pdf)
- [ROSCon 2023 ROS / ROS 2 with Kubernetes and KubeEdge](https://roscon.ros.org/2023/talks/ROS_with_KubernetesKubeEdge.pdf) / [Talk](https://vimeo.com/879001688/33b2495a49)
- [ROS By the Bay Presentation Slide Deck](https://raw.githack.com/fujitatomoya/ros_k8s/master/presentation/ROS-By-the-Bay_20231214.html)

[![Watch the video](https://img.youtube.com/vi/Amxsy5A2NWE/maxresdefault.jpg)](https://www.youtube.com/watch?v=Amxsy5A2NWE)

## Reference

- [Kubernetes Official Documentation](https://kubernetes.io/docs/home/)
- [KubeEdge Official Documentation](https://kubeedge.io/docs/welcome/getting-started)
- [Kubernetes IN Docker](https://kind.sigs.k8s.io/)
- [ROS Noetic](http://wiki.ros.org/noetic)
- [ROS Rolling](https://docs.ros.org/en/rolling/)

## Author

- Tomoya Fujita <tomoya.fujita825@gmail.com>
