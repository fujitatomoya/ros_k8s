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

![ROS 2 Turtlesim and Rqt Example](./../images/ros2_turtlesim_sample.png)

- Labeling the display capability.

```bash
xxx
```

- Start turtlesim and rqt on the node with display.

```bash
xxx
```

- Use `turtle_teleop_key` to control the turtlesim.

```bash
xxx
```