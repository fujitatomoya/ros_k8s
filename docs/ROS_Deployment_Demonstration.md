# ROS Deployment Demonstration

This section provides several example deployment, starting with basic deployment to complicated configuration with diagrams.

## ROS Noetic Demonstration

For ROS Noetic, we can use any CNI implementation (Flannel/Weave/Cilium).
This is because that ROS Noetic network is based on TCP/UDP IP network w/o multicast.

### ROS DaemonSet Deployment via CNI

This example deploys all application containers for each cluster node.
That is said rosmaster will be starting on each cluster node and other ROS application container will be connecting each other in the localhost system as following.

multiple rosmaster/roscore cannot be running in the same LAN, so using CNI will isolate the IP addresses assigned by Kubernetes.

***INSERT PICTURE***

```bash
### Command Here
```

### ROS Multi-Node Deployment via CNI

This example deploys distributed system with ROS, application containers will be deployed corresponding or targeted cluster node.
In this case, rosmaster will be running on one of the cluster node, and other ROS application container will be connecting to that rosmaster as distributed application.

***INSERT PICTURE***

```bash
### Command Here
```

***Explanation about Headless Service***

### ROS Multi-Node Deployment via Host Network

This example deploys distributed system with ROS, application containers will be deployed corresponding or targeted cluster node.
In this case, rosmaster will be running on one of the cluster node, and other ROS application container will be connecting to that rosmaster as distributed application.

***INSERT PICTURE***

```bash
### Command Here
```

We can access the ROS network via host system using docker container.

```bash
### Command Here
```

