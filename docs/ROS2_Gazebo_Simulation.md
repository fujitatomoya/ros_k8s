# ROS 2 Gazebo Simulation

This section describes how to run [Gazebo](https://gazebosim.org/) simulation with ROS 2 application containers on the Kubernetes cluster, from the local container development to the cluster deployment with the GUI on the display node.

## Reference

- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/) (Gazebo distribution paired with ROS 2 Jazzy)
- [ros_gz](https://github.com/gazebosim/ros_gz) (ROS 2 integration, `ros_gz_bridge`)
- [Gazebo Transport Environment Variables](https://gazebosim.org/api/transport/14/envvars.html)
- [Gazebo Server and GUI on different machines](https://gazebosim.org/api/sim/8/distributedsimulation.html)

## Development Cycle with Gazebo Simulation

The typical development cycle with the simulation looks like the following.

1. Application engineer develops and tests the ROS 2 application with Gazebo simulation in the container on the local host system (**no Kubernetes**).
2. Application container images are pushed to the registry, and deployed by Kubernetes.
3. For the showcase / demonstration / exhibition, Kubernetes brings up the Gazebo simulation and the application containers in the cluster, Gazebo GUI pops up on the specified node that has the display device.
4. Physics (and sensor rendering) computation of the simulation can be offloaded to the server node, GUI client runs on the display node, and the application containers run anywhere in the cluster.

Gazebo (gz-sim) is designed with the client / server architecture, `gz sim -s` runs the simulation server (physics, sensors, plugins) without GUI, and `gz sim -g` runs the GUI client that connects to the server via [Gazebo Transport](https://gazebosim.org/api/transport/14/index.html).
ROS 2 application containers communicate with the simulation via [ros_gz_bridge](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge) that bridges the Gazebo Transport topics to ROS 2 topics.

```
   display node (nodetype=edgeserver)        server node (nodetype=simserver)              any node (nodetype=edgedevice)
+-------------------------------+   gz-transport  +-------------------------------+   ROS 2 / DDS   +---------------------------+
| gz sim -g (GUI) + X11 display | <-------------> | gz sim -s (physics, sensors)  |                 |                           |
+-------------------------------+ 239.255.0.7     | ros_gz_bridge (same pod)      | <-------------> | ROS 2 application         |
                                                  +-------------------------------+   239.255.0.1   +---------------------------+
```

## Container Image with Gazebo

[Dockerfile.jazzy-gazebo](./../docker/Dockerfile.jazzy-gazebo) builds `tomoyafujita/ros:jazzy-gazebo` image, which adds `ros-jazzy-ros-gz` (Gazebo Harmonic and ros_gz) to the [ROS Jazzy](./Build_Docker_Images.md#ros-jazzy) full image.
See [Build Docker Images](./Build_Docker_Images.md#ros-jazzy-with-gazebo) for the details, and [Load local container images](./Setup_Virtualized_Cluster.md#multiple-node-cluster-setup) to use the locally built image in the kind cluster.

```bash
> docker build --rm -f ./docker/Dockerfile.jazzy-gazebo -t tomoyafujita/ros:jazzy-gazebo .
> kind load docker-image tomoyafujita/ros:jazzy-gazebo
```

## Local Development with Container

Before deploying to the cluster, the simulation and the application can be developed and verified in the container on the local host system.
The following runs the headless simulation server with [diff_drive.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive.sdf) world, bridges the topics and controls the robot via ROS 2 in the single container.

```bash
> docker run -it --rm tomoyafujita/ros:jazzy-gazebo /bin/bash
root@6f1d2c3a4b5e:/# source /opt/ros/jazzy/setup.bash
root@6f1d2c3a4b5e:/# gz sim --version
Gazebo Sim, version 8.15.0

### Start the simulation server (headless) in background, -r runs the simulation at start
root@6f1d2c3a4b5e:/# gz sim -s -r -v 1 diff_drive.sdf &

### Gazebo Transport topics from the simulation
root@6f1d2c3a4b5e:/# gz topic -l
/clock
/model/vehicle_blue/cmd_vel
/model/vehicle_blue/odometry
/model/vehicle_blue/tf
/model/vehicle_green/cmd_vel
/model/vehicle_green/odometry
/stats
/world/diff_drive/clock
...<snip>

### Bridge Gazebo Transport topics to ROS 2 ([ : Gazebo -> ROS 2, ] : ROS 2 -> Gazebo)
root@6f1d2c3a4b5e:/# ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist /model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry &
[INFO] [1792432323.461685773] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/model/vehicle_blue/odometry (gz.msgs.Odometry) -> /model/vehicle_blue/odometry (nav_msgs/msg/Odometry)] (Lazy 0)
[INFO] [1792432323.462681040] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)] (Lazy 0)
...<snip>

### ROS 2 application side
root@6f1d2c3a4b5e:/# ros2 topic list
/clock
/model/vehicle_blue/cmd_vel
/model/vehicle_blue/odometry
/parameter_events
/rosout
root@6f1d2c3a4b5e:/# ros2 topic echo /model/vehicle_blue/odometry --once --field pose.pose.position
x: 6.200616963166462e-18
y: 3.241813234658251e-37
z: 0.0
---
root@6f1d2c3a4b5e:/# ros2 topic pub -t 10 -r 10 /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
root@6f1d2c3a4b5e:/# ros2 topic echo /model/vehicle_blue/odometry --once --field pose.pose.position
x: 0.9767500000003548
y: -5.467528879876727e-16
z: 0.0
---
```

To use the Gazebo GUI on the local host system, bind the X11 socket and `DISPLAY` to the container in the same way with [ROS 2 GUI Display Access](./ROS2_Deployment_Intermediate.md#ros-2-gui-display-access) (`xhost +local:root` and `docker run -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...`), and start `gz sim -g` in the container.

## Gazebo Transport in the Cluster

Gazebo Transport has its own discovery protocol, that uses **UDP multicast `239.255.0.7:10317`** in default (`GZ_DISCOVERY_MULTICAST_IP` / `GZ_DISCOVERY_MSG_PORT`), which is different from the ROS 2 / DDS multicast group `239.255.0.1`.
To make the Gazebo server, GUI and bridge discover each other across the cluster nodes, add the multicast group to Cilium, see [Enable Multicast](./Setup_Kubernetes_Cluster.md#enable-multicast).

```bash
> cilium multicast add --group-ip 239.255.0.7
Added 2 nodes (kind-control-plane, kind-worker2) to multicast group 239.255.0.7 in kind-worker
Added 2 nodes (kind-worker2, kind-worker) to multicast group 239.255.0.7 in kind-control-plane
Added 2 nodes (kind-control-plane, kind-worker) to multicast group 239.255.0.7 in kind-worker2

> cilium multicast list group
Node                 Group
kind-control-plane   239.255.0.7
                     239.255.0.1
kind-worker2         239.255.0.1
                     239.255.0.7
kind-worker          239.255.0.1
                     239.255.0.7
```

Besides that, Gazebo Transport nodes only communicate within the same **partition** (`GZ_PARTITION`), whose default value is `<hostname>:<username>`.
Since each pod has the different hostname, `GZ_PARTITION` must be set to the same value for all Gazebo Transport containers in the deployment descriptions.
If multicast is not available in your environment, `GZ_RELAY` environmental variable can be used to send the discovery messages to the specific IP addresses via unicast.

## Deploy Simulation Server and Application

**see deployment description [Gazebo Simulation Server and ROS 2 Application](./../yaml/ros2-gazebo.yaml)**

[ros2-gazebo.yaml](./../yaml/ros2-gazebo.yaml) deploys the following.

- `gz-sim-server`: pod with 2 containers on the node labeled `nodetype=simserver`, Gazebo server (`gz sim -s -r diff_drive.sdf`) and `ros_gz_bridge` that bridges `/clock`, `/model/vehicle_blue/cmd_vel` and `/model/vehicle_blue/odometry`.
- `ros2-gz-teleop`: ROS 2 application container on the node labeled `nodetype=edgedevice`, that subscribes the odometry from the simulation via ROS 2 topic.

- Label the nodes

```bash
### physical cluster: kubectl label nodes <server-node> nodetype=simserver, kubectl label nodes <device-node> nodetype=edgedevice
> kubectl label nodes kind-worker2 nodetype=simserver
> kubectl label nodes kind-worker nodetype=edgedevice
```

- Start deployment

```bash
> kubectl apply -f ./yaml/ros2-gazebo.yaml
deployment.apps/gz-sim-server created
deployment.apps/ros2-gz-teleop created

> kubectl get pods -o wide
NAME                              READY   STATUS    RESTARTS   AGE   IP             NODE           NOMINATED NODE   READINESS GATES
gz-sim-server-6bbf7459d7-hmxvk    2/2     Running   0          21s   10.244.2.118   kind-worker2   <none>           <none>
ros2-gz-teleop-55f557987d-fmtf7   1/1     Running   0          21s   10.244.1.237   kind-worker    <none>           <none>

> kubectl logs deployment/gz-sim-server -c ros-gz-bridge
[INFO] [1792432799.599448642] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)] (Lazy 0)
[INFO] [1792432799.600155979] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/model/vehicle_blue/cmd_vel (geometry_msgs/msg/Twist) -> /model/vehicle_blue/cmd_vel (gz.msgs.Twist)] (Lazy 0)
[INFO] [1792432799.601173968] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/model/vehicle_blue/odometry (gz.msgs.Odometry) -> /model/vehicle_blue/odometry (nav_msgs/msg/Odometry)] (Lazy 0)
```

- Control the robot from the application container

  Jump in the application container on `kind-worker`, Gazebo Transport topics from the server on `kind-worker2` are discovered via the multicast group `239.255.0.7`, and ROS 2 topics from the bridge via `239.255.0.1`.
  Publishing `cmd_vel` moves the robot in the simulation, and the odometry comes back via the bridge.

```bash
> kubectl exec --stdin --tty deployment/ros2-gz-teleop -- /bin/bash
root@ros2-gz-teleop-55f557987d-fmtf7:/# source /opt/ros/jazzy/setup.bash
root@ros2-gz-teleop-55f557987d-fmtf7:/# echo $GZ_PARTITION
ros_k8s

### Gazebo Transport topics from the simulation server on the other node
root@ros2-gz-teleop-55f557987d-fmtf7:/# gz topic -l
/clock
/gazebo/resource_paths
/model/vehicle_blue/cmd_vel
/model/vehicle_blue/odometry
/model/vehicle_blue/tf
/model/vehicle_green/odometry
/model/vehicle_green/tf
/stats
...<snip>

### ROS 2 topics from ros_gz_bridge
root@ros2-gz-teleop-55f557987d-fmtf7:/# ros2 topic list
/clock
/model/vehicle_blue/cmd_vel
/model/vehicle_blue/odometry
/parameter_events
/rosout
root@ros2-gz-teleop-55f557987d-fmtf7:/# ros2 topic echo /clock --once
clock:
  sec: 26
  nanosec: 799000000
---

### Move the robot
root@ros2-gz-teleop-55f557987d-fmtf7:/# ros2 topic echo /model/vehicle_blue/odometry --once --field pose.pose.position
x: 7.36023718120501e-18
y: -1.4667433755733231e-36
z: 0.0
---
root@ros2-gz-teleop-55f557987d-fmtf7:/# ros2 topic pub -t 20 -r 10 /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
root@ros2-gz-teleop-55f557987d-fmtf7:/# ros2 topic echo /model/vehicle_blue/odometry --once --field pose.pose.position
x: 2.2472147503070676
y: 1.6362103208758465
z: 0.0
---

### Simulation statistics via Gazebo Transport
root@ros2-gz-teleop-55f557987d-fmtf7:/# gz topic -e -t /stats -n 1
sim_time {
  sec: 39
  nsec: 25000000
}
real_time {
  sec: 40
  nsec: 100520416
}
iterations: 39025
real_time_factor: 0.99984...
...<snip>
```

  The application container log also shows the odometry via ROS 2 topic.

```bash
> kubectl logs deployment/ros2-gz-teleop --tail=4
x: 2.371434221743204
y: 2.119674176712727
z: 0.0
---
```

## Gazebo GUI on the Display Node

**see deployment description [Gazebo GUI](./../yaml/ros2-gazebo-gui.yaml)**

Gazebo GUI client (`gz sim -g`) connects to the running simulation server via Gazebo Transport, so that it can be deployed to the node that has the display device, in the same way with [ROS 2 GUI Display Access](./ROS2_Deployment_Intermediate.md#ros-2-gui-display-access).
The GUI pod needs the X11 socket (`/tmp/.X11-unix`) and `DISPLAY` from the display node, and the same `GZ_PARTITION` with the server.

> [!NOTE]
> This requires the physical node with the display (X server or Xwayland), it cannot be verified on the kind cluster.
> Gazebo GUI uses OpenGL for the rendering, if the container cannot access the GPU on the display node, set `LIBGL_ALWAYS_SOFTWARE=1` in the deployment description to fall back to the software rendering.

```bash
### On the display node
> xhost +local:root
> echo $DISPLAY
:1

### Label the display node and start the GUI
> kubectl label nodes <display-node> nodetype=edgeserver
> kubectl apply -f ./yaml/ros2-gazebo-gui.yaml
deployment.apps/gz-sim-gui created
```

## Stop deployment

```bash
> kubectl delete -f ./yaml/ros2-gazebo-gui.yaml
> kubectl delete -f ./yaml/ros2-gazebo.yaml
> kubectl label nodes kind-worker2 nodetype-
> kubectl label nodes kind-worker nodetype-
> cilium multicast delete --group-ip 239.255.0.7
```
