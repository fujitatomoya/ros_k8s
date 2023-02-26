# ROS Kubernetes

## Abstract

This repository provides tutorials how to use ROS and ROS 2 with Kubernetes Cluster System.
User might need to have knowledge about Kubernetes to understand what is really going on the cluster system.

## Motivation

The primary goal for this repository is that everyone can try ROS and ROS 2 with Kubernetes Cluster.
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
| tomoyafujita-HP-Compaq-Elite-8300-SFF | 192.168.1.248 | Ubuntu 20.04.5 LTS | x86_64 |
| ubuntu | 192.168.1.79 | Ubuntu 20.04.5 LTS | aarch64 |

- Virtualized Instance

This is virtualized Kubernetes cluster environment which can be established on single physical host.
We can use this environment if container requires pure computation resource but hardware or physical sensor devices.
This environment is very much useful to try or test your container images or services to make sure if those work as expected, and easy to break down whole Kubernetes cluster with one command.

## Tutorials

- [Install Kubernetes Packages](./docs/Install_Kubernetes_Packages.md)
- [Build ROS / ROS 2 Full Docker Images](./docs/Build_Docker_Images.md)
- [Setup Kuberenetes Cluster](./docs/Setup_Kubernetes_Cluster.md)
- [Setup Virtualized Kuberenetes Cluster](./docs/Setup_Virtualized_Cluster.md)
- [ROS Deployment Demonstration](./docs/ROS_Deployment_Demonstration.md)
- [ROS 2 Deployment Demonstration](./docs/ROS2_Deployment_Demonstration.md)

## Reference

- [kubernetes official documentation](https://kubernetes.io/docs/home/)
- [Kubernetes IN Docker](https://kind.sigs.k8s.io/)
- [ROS Noetic](http://wiki.ros.org/noetic)
- [ROS Rolling](https://docs.ros.org/en/rolling/)

## Author

- Tomoya Fujita <tomoya.fujita825@gmail.com>
