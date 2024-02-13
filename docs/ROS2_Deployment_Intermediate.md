# ROS 2 Deployment Intermediate

This section provides intermediate examples such as GUI display access, security enclaves and hardware acceleration.

## ROS Rolling Demonstration

***<span style="color: red"> Please Use WeaveNet CNI Plugin </span>***

see [Why we need to use WeaveNet](./ROS2_Deployment_Demonstration.md#ros-rolling-demonstration)

### ROS 2 GUI Display Access

ROS provides many useful tools via GUI such as rqt, rviz and GUI application like turtlesim.
This example deploys ROS 2 turtlesim application container bound to the node which has display, and access the xserver from the container, application container can display the turtlesim on the display.
At the same time, it will bring up rqt to monitor the topic on the same node which has display.
To control the capability such as `which node has display`, it will use node labels to describe the capability and at the deployment we can select this label as application container requirements.

**To run this example, at least one physical node must have display xserver running.**

**see deployment description [ROS 2 turtlesim and rqt](./../yaml/ros2-turtlesim.yaml)**

![ROS 2 Turtlesim and Rqt Overview](./../images/ros2_turtlesim_overview.png)

- Disable xhost access control

```bash
> xhost +local:root
non-network local connections being added to access control list
```

- Labeling the display capability.

  In this example, `edgeserver` is the server runs GUI capability, but `edgedevice`.

```bash
> kubectl label nodes tomoyafujita-hp-compaq-elite-8300-sff nodetype=edgeserver
> kubectl label nodes ubuntu nodetype=edgedevice
> kubectl get nodes --show-labels
NAME                                    STATUS   ROLES           AGE   VERSION   LABELS
tomoyafujita-hp-compaq-elite-8300-sff   Ready    control-plane   63m   v1.25.5   beta.kubernetes.io/arch=amd64,beta.kubernetes.io/os=linux,kubernetes.io/arch=amd64,kubernetes.io/hostname=tomoyafujita-hp-compaq-elite-8300-sff,kubernetes.io/os=linux,node-role.kubernetes.io/control-plane=,node.kubernetes.io/exclude-from-external-load-balancers=,nodetype=edgeserver
ubuntu                                  Ready    <none>          45m   v1.25.5   beta.kubernetes.io/arch=arm64,beta.kubernetes.io/os=linux,kubernetes.io/arch=arm64,kubernetes.io/hostname=ubuntu,kubernetes.io/os=linux,nodetype=edgedevice
```

- Start turtlesim and rqt on `edgeserver`, and start bash process node in `edgedevice`.

```bash
> kubectl apply -f ros2-turtlesim.yaml
deployment.apps/ros2-turtlesim-gui created
deployment.apps/rqt created
deployment.apps/ros2-teleop-key created
```

- Use `turtle_teleop_key` to control the turtlesim.

  process is already running in the cluster system on the host `edgedevice` labeled, so we can jump in the container `turtle_teleop_key` via kubernetes command as below.

```bash
> kubectl get pods
NAME                                  READY   STATUS    RESTARTS   AGE
ros2-teleop-key-75f7c5475c-s6hwj      1/1     Running   0          33s
ros2-turtlesim-gui-849df7c66b-gzmt9   1/1     Running   0          33s
rqt-5886589dfc-v8q92                  1/1     Running   0          33s
> kubectl exec --stdin --tty ros2-teleop-key-75f7c5475c-s6hwj -- /bin/bash

root@ros2-teleop-key-75f7c5475c-s6hwj:/# source /opt/ros/rolling/setup.bash
root@ros2-teleop-key-75f7c5475c-s6hwj:/# ros2 run turtlesim turtle_teleop_key
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
'Q' to quit.
...
```

<img src="./../images/ros2_turtlesim_output.png" width="800">

### ROS 2 Zero Copy Data Sharing

ROS 2 provides [LoanedMessage](https://design.ros2.org/articles/zero_copy.html) RMW interface to allow user application to borrow the memory from the underlying middleware such as `rmw_fastrtps`.
This enables user application to write and publish the message without any copy from user space to the middleware, which is really efficient for edge IoT devices.
On the subscription side, underlying middleware automatically resolve if the memory can be loaned to subscription as well, that said subscription can also borrow the memory from the middleware to read the message without any extra copy.

Besides, [rmw_fastrtps](https://github.com/ros2/rmw_fastrtps) RMW implementation middleware also supports zero copy data sharing feature in [Fast-DDS](https://github.com/eProsima/Fast-DDS).
[Fast-DDS Data Sharing Delivery](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/datasharing.html) can provide `True Zero Copy Data Sharing` to the ROS 2 user application without any copy.
It uses POSIX shared memory file `/dev/shm` in default including [Shared Memory Transport](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html#shared-memory-transport), that said we need to bind `/dev/shm` tmpfs for each container in the same pod with same IP address assignment by Kubernetes.

**see deployment description [ROS 2 Zero Copy Data Sharing](./../yaml/ros2-data-sharing.yaml)**

***<span style="color: red"> [Fast-DDS](https://github.com/eProsima/Fast-DDS) can achieve zero copy feature without any daemon process running, that makes it easier to use this feature in the container. </span>***

![ROS 2 Data Sharing](./../images/ros2_data_sharing.png)

- Start DaemonSet

```bash
> kubectl apply -f ros2-data-sharing.yaml
daemonset.apps/ros2-data-sharing created
```

- Stop DaemonSet

```bash
> kubectl delete -f ros2-data-sharing.yaml
daemonset.apps "ros2-data-sharing" deleted
```

### ROS 2 Fast-DDS Discovery Server

ROS 2 provides [Fast-DDS discovery server](https://fast-dds.docs.eprosima.com/en/v2.1.0/fastdds/discovery/discovery_server.html#discovery-server), that is a feature to support centralized dynamic discovery mechanism.
Centralized discovery is way more efficient for discovery process compared to distributed discovery protocol, this improves significant discovery performance when there are many ROS 2 context (a.k.a DDS participant) in the network.

But this centralized design also brings back the problem for the static configuration for the ROS 2 application, which requires to set the discovery server IP addresses when the application starts.
Taking advantage of DNS and Headless service provided by Kubernetes, discovery server IP addresses are resolved dynamically by DNS in the cluster network.

This example shows how to start the [Fast-DDS discovery server](https://fast-dds.docs.eprosima.com/en/v2.1.0/fastdds/discovery/discovery_server.html#discovery-server) with headless service, and how application resolves the addresses by DNS.

**see deployment description [ROS 2 Fast-DDS Discovery Server](./../yaml/ros2-fastdds-discovery-server.yaml) and [Application Deployment](./../yaml/ros2-fastdds-discovery-server-apps.yaml)**

![ROS 2 Fast-DDS Discovery Server](./../images/ros2_fastdds_discovery_server.png)

- Fast-DDS Discovery Server Primary and Secondary

  [ROS 2 Fast-DDS Discovery Server](./../yaml/ros2-fastdds-discovery-server.yaml) starts running primary and secondary discovery server with specific port number with any available network interfaces in the pods.
  These discovery server containers will be deployed to the `control-plane` in this example, but this can be deployed anywhere else.
  It is highly recommended that primary and secondary discovery server should be deployed to different physical host system, so that it can tolerate the situation.

```bash
>kubectl apply -f ./yaml/ros2-fastdds-discovery-server.yaml
deployment.apps/discovery-server-primary created
service/primary-discovery-server created
deployment.apps/discovery-server-secondary created
service/secondary-discovery-server created
```

- ROS 2 Talker and Listener Application Deployment

  Once discovery servers are deployed, we can start the application container anywhere we want in the cluster system.
  In the [Application Deployment](./../yaml/ros2-fastdds-discovery-server-apps.yaml), it binds the `ROS_DISCOVERY_SERVER` environmental variable with DNS name `primary-discovery-server:11811;secondary-discovery-server:11888`.
  These URLs are resolved by DNS in the cluster when the application starts, and they will be replaced into the cluster IP addresses where discovery server containers are running.
  After all, all ROS 2 application containers are able to access discovery servers in the cluster system to connect endpoints to start communication.

```bash
> kubectl apply -f ./yaml/ros2-fastdds-discovery-server-apps.yaml
deployment.apps/ros2-talker-1 created
deployment.apps/ros2-talker-2 created
deployment.apps/ros2-talker-3 created
deployment.apps/ros2-listener-1 created
deployment.apps/ros2-listener-2 created
deployment.apps/ros2-listener-3 created
```
