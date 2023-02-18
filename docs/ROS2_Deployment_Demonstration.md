# ROS 2 Deployment Demonstration

This section provides several example deployments with ROS 2, starting with basic deployment to complicated configuration with diagrams.

## ROS Rolling Demonstration

***<span style="color: red"> Please Use WeaveNet CNI Plugin </span>***

For ROS Rolling, we can only use any CNI implementation Weave but Flannel/Cilium, since ROS 2 uses DDS underneath as RMW implementation for transport framework.
DDS uses multicast (depends on DDS implementation), but some CNI implementation cannot support multicast packets, in that case ROS 2 discovery process cannot work as expected.

If we can be sure that RMW implementation does not rely on UDP multicast, we can of course use other CNI plugins.
Most likely this case applied to use case ofr discovery server and so on, using TCP/UDP unicast packets.

### ROS 2 Simple Distributed System

This example deploys ROS 2 application pods to any available nodes in the cluster, since we do not set the `nodeselector`, Kubernetes decides where to deploy the application container based on statistics from the system.
Kubernetes by default will not deploy any application pods to control-plane(master) node, this is because Kubernetes tries to conserve the resource on control-plane to manage the all fleet in the cluster and query from user.
In this deployment description, it uses `tolerations` for the application deployment so that Kubernetes can deploy these application containers even to master node.
That is said that Kubernetes is responsible to keep the deployment even if anything happens to container crashes, it will be re-instantiated by Kubernetes.
This is actually controlled by `restartPolicy` description in the yaml file.

**This example shows that use case that, we do not even need to know where to deploy but application should be available somewhere in the ROS 2 system.**

Using WeaveNet can provide layer 2 emulation, that means all containers are connected via virtual physical local area network even with multicast capability.
So all discovery protocol just works out of the box like physical network.

**see deployment description [ROS 2 Simple Distributed System](./../yaml/ros2-sample.yaml)**

![ROS noetic Multiple Node Deployment](./../images/ros2_simple_sample.png)

- Start deployment and check availability.

```bash
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:/ros_k8s/yaml# kubectl apply -f ros2-sample.yaml
deployment.apps/ros2-talker-1 created
deployment.apps/ros2-talker-2 created
deployment.apps/ros2-talker-3 created
deployment.apps/ros2-listener-1 created
deployment.apps/ros2-listener-2 created
deployment.apps/ros2-listener-3 created

root@tomoyafujita-HP-Compaq-Elite-8300-SFF:/ros_k8s/yaml# kubectl get pods -o wide
NAME                               READY   STATUS    RESTARTS      AGE   IP          NODE                                    NOMINATED NODE   READINESS GATES
ros2-listener-1-5c87cc854d-c4q78   1/1     Running   0             15m   10.32.0.4   tomoyafujita-hp-compaq-elite-8300-sff   <none>           <none>
ros2-listener-2-7c8f99f79-lg8dp    1/1     Running   1 (15m ago)   15m   10.44.0.5   ubuntu                                  <none>           <none>
ros2-listener-3-5778bb66c4-j9pp8   1/1     Running   0             15m   10.32.0.5   tomoyafujita-hp-compaq-elite-8300-sff   <none>           <none>
ros2-talker-1-54f5fb9dcd-2dlv6     1/1     Running   0             15m   10.44.0.4   ubuntu                                  <none>           <none>
ros2-talker-2-774d9b5846-bwm2s     1/1     Running   0             15m   10.44.0.3   ubuntu                                  <none>           <none>
ros2-talker-3-67c88d57d-z52l4      1/1     Running   0             15m   10.44.0.6   ubuntu                                  <none>           <none>
```

- Jump in one of container to see the activity.

```bash
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubectl exec --stdin --tty ros2-talker-1-54f5fb9dcd-2dlv6 -- /bin/bash

root@ros2-talker-1-54f5fb9dcd-2dlv6:/# source /opt/ros/rolling/setup.bash

root@ros2-talker-1-54f5fb9dcd-2dlv6:/# ros2 topic list
/chatter1
/chatter2
/chatter3
/parameter_events
/rosout
root@ros2-talker-1-54f5fb9dcd-2dlv6:/# ros2 topic list -v
Published topics:
 * /chatter1 [std_msgs/msg/String] 1 publisher
 * /chatter2 [std_msgs/msg/String] 1 publisher
 * /chatter3 [std_msgs/msg/String] 1 publisher
 * /parameter_events [rcl_interfaces/msg/ParameterEvent] 10 publishers
 * /rosout [rcl_interfaces/msg/Log] 10 publishers

Subscribed topics:
 * /chatter1 [std_msgs/msg/String] 1 subscriber
 * /chatter2 [std_msgs/msg/String] 1 subscriber
 * /chatter3 [std_msgs/msg/String] 1 subscriber

root@ros2-talker-1-54f5fb9dcd-2dlv6:/# ros2 topic echo /chatter1
data: Hello, I am talker-1
---
data: Hello, I am talker-1
---
^Croot@ros2-talker-1-54f5fb9dcd-2dlv6:/# ros2 topic echo /chatter2
data: Hello, I am talker-2
---
data: Hello, I am talker-2
---
^Croot@ros2-talker-1-54f5fb9dcd-2dlv6:/# ros2 topic echo /chatter3
data: Hello, I am talker-3
---
data: Hello, I am talker-3
---
...
```

### XXX

### XXX

### XXX
