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
