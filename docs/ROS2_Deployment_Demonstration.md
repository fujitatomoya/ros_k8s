# ROS 2 Deployment Demonstration

This section provides several example deployments with ROS 2, starting with basic deployment to complicated configuration with diagrams.

## ROS Rolling Demonstration

***<span style="color: red"> Please Use Cilium CNI Plugin with Multicast Enabled </span>***

ROS 2 uses DDS underneath as RMW implementation for transport framework.
DDS uses multicast for the participant discovery in default (depends on DDS implementation), but most of CNI implementations such as Flannel and Calico cannot support multicast packets, in that case ROS 2 discovery process cannot work as expected across the cluster nodes.

In this tutorial, [Cilium](https://cilium.io/) with multicast (beta) enabled is used as CNI plugin, see [Enable Multicast](./Setup_Kubernetes_Cluster.md#enable-multicast) for more details.
[WeaveNet](https://github.com/weaveworks/weave), which was the recommended CNI plugin for multicast in this tutorial before, has reached end of life and is no longer maintained.

If we can be sure that RMW implementation does not rely on UDP multicast, we can of course use other CNI plugins.
Most likely this case applied to use case of [Fast DDS Discovery Server](./ROS2_Deployment_Intermediate.md#ros-2-fast-dds-discovery-server) or static peers (`ROS_STATIC_PEERS`) and so on, using TCP/UDP unicast packets.

> [!NOTE]
> `tomoyafujita/ros:rolling` container image is built from the official `ros:rolling` image, which is currently Ubuntu 26.04 based.
> Container images are agnostic from the host system, so that it can be deployed on Ubuntu 24.04 host system without any problems.
> If you prefer the Ubuntu 24.04 based LTS distribution, build and use `ros:jazzy` instead, see [Build Docker Images](./Build_Docker_Images.md).
> The following examples are verified with the images built from [Dockerfile.rolling](./../docker/Dockerfile.rolling) and [Dockerfile.jazzy](./../docker/Dockerfile.jazzy) on the kind cluster, see [Load local container images](./Setup_Virtualized_Cluster.md#multiple-node-cluster-setup) to use the locally built image in the kind cluster.

### ROS 2 Simple Distributed System

This example deploys ROS 2 application pods to any available nodes in the cluster, since we do not set the `nodeselector`, Kubernetes decides where to deploy the application container based on statistics from the system.
Kubernetes by default will not deploy any application pods to control-plane node, this is because Kubernetes tries to conserve the resource on control-plane to manage the all fleet in the cluster and query from user.
In this deployment description, it uses `tolerations` for the application deployment so that Kubernetes can deploy these application containers even to control-plane node.
That is said that Kubernetes is responsible to keep the deployment even if anything happens to container crashes, it will be re-instantiated by Kubernetes.
This is actually controlled by `restartPolicy` description in the yaml file.

**This example shows that use case that, we do not even need to know where to deploy but application should be available somewhere in the ROS 2 system.**

Using Cilium with multicast enabled can deliver the DDS discovery multicast packets to all cluster nodes, that means all containers are connected via virtual local area network even with multicast capability.
So all discovery protocol just works out of the box like physical network.

**see deployment description [ROS 2 Simple Distributed System](./../yaml/ros2-sample.yaml)**

![ROS 2 Simple Distributed System](./../images/ros2_simple_sample.png)

- Start deployment and check availability.

```bash
> kubectl apply -f ./yaml/ros2-sample.yaml
deployment.apps/ros2-talker-1 created
deployment.apps/ros2-talker-2 created
deployment.apps/ros2-talker-3 created
deployment.apps/ros2-listener-1 created
deployment.apps/ros2-listener-2 created
deployment.apps/ros2-listener-3 created

> kubectl get pods -o wide
NAME                               READY   STATUS    RESTARTS   AGE   IP             NODE           NOMINATED NODE   READINESS GATES
ros2-listener-1-55d8fc9f47-thhzl   1/1     Running   0          3s    10.244.2.31    kind-worker2   <none>           <none>
ros2-listener-2-7b64bfb8dd-pdwsh   1/1     Running   0          3s    10.244.2.27    kind-worker2   <none>           <none>
ros2-listener-3-59fc44bfdf-mglbq   1/1     Running   0          3s    10.244.1.64    kind-worker    <none>           <none>
ros2-talker-1-57cf57d5bf-psb4x     1/1     Running   0          3s    10.244.2.97    kind-worker2   <none>           <none>
ros2-talker-2-6544ddddf7-4gdhm     1/1     Running   0          3s    10.244.2.164   kind-worker2   <none>           <none>
ros2-talker-3-5774bb7c9c-2vv4k     1/1     Running   0          3s    10.244.2.11    kind-worker2   <none>           <none>

### talker-3 (kind-worker2) and listener-3 (kind-worker) are discovered via multicast across the cluster nodes
> kubectl logs deployment/ros2-listener-3 --tail=2
data: Hello, I am talker-3
---
```

- Jump in one of container to see the activity.

```bash
> kubectl exec --stdin --tty ros2-talker-1-57cf57d5bf-psb4x -- /bin/bash

root@ros2-talker-1-57cf57d5bf-psb4x:/# source /opt/ros/rolling/setup.bash

root@ros2-talker-1-57cf57d5bf-psb4x:/# ros2 topic list
/chatter1
/chatter2
/chatter3
/parameter_events
/rosout
root@ros2-talker-1-57cf57d5bf-psb4x:/# ros2 topic list -v
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

root@ros2-talker-1-57cf57d5bf-psb4x:/# ros2 topic echo /chatter1 std_msgs/String
data: Hello, I am talker-1
---
data: Hello, I am talker-1
---
^Croot@ros2-talker-1-57cf57d5bf-psb4x:/# ros2 topic echo /chatter2 std_msgs/String
data: Hello, I am talker-2
---
data: Hello, I am talker-2
---
^Croot@ros2-talker-1-57cf57d5bf-psb4x:/# ros2 topic echo /chatter3 std_msgs/String
data: Hello, I am talker-3
---
data: Hello, I am talker-3
---
...
```

- Stop deployment

```bash
> kubectl delete -f ./yaml/ros2-sample.yaml
```

### ROS 2 Localhost Only

This sample description starts deployment multiple application pods for each cluster node.
Application containers in each pod will only start communication in localhost only with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` environmental variable which is bound to application container runtime.
(`ROS_LOCALHOST_ONLY=1` is deprecated since ROS 2 Iron, `ROS_AUTOMATIC_DISCOVERY_RANGE` is the replacement.)

The following diagram gives you an good example and overview what is going on this deployment.

Taking advantage virtual network interface from CNI plugin, we can even have multiple localhost network in single cluster node.
This is useful to multiple users in single cluster system, it just appears to be for users to have dedicated network interfaces so that it will not affect any other ROS 2 communication.
This example works with any CNI plugin, since there is no communication across the pods.

**see deployment description [ROS 2 Localhost Only](./../yaml/ros2-localhost.yaml)**

![ROS 2 Localhost Only](./../images/ros2_localhost_only.png)

-  Start deployment and check availability

```bash
> kubectl apply -f ./yaml/ros2-localhost.yaml
daemonset.apps/ros2-deamonset-1 created
daemonset.apps/ros2-deamonset-2 created

> kubectl get pods -o wide
NAME                     READY   STATUS    RESTARTS   AGE   IP             NODE                 NOMINATED NODE   READINESS GATES
ros2-deamonset-1-hcjf8   2/2     Running   0          3s    10.244.0.237   kind-control-plane   <none>           <none>
ros2-deamonset-1-sqm6g   2/2     Running   0          3s    10.244.1.73    kind-worker          <none>           <none>
ros2-deamonset-1-wlfh4   2/2     Running   0          3s    10.244.2.218   kind-worker2         <none>           <none>
ros2-deamonset-2-m7znk   2/2     Running   0          3s    10.244.1.36    kind-worker          <none>           <none>
ros2-deamonset-2-mdd88   2/2     Running   0          3s    10.244.2.30    kind-worker2         <none>           <none>
ros2-deamonset-2-mfqd2   2/2     Running   0          3s    10.244.0.205   kind-control-plane   <none>           <none>
```

- Check each pod to see if localhost communication actually takes place.

```bash
> kubectl exec --stdin --tty ros2-deamonset-1-hcjf8 -- /bin/bash
Defaulted container "ros2-talker-1" out of: ros2-talker-1, ros2-listener-1
root@ros2-deamonset-1-hcjf8:/# echo $ROS_AUTOMATIC_DISCOVERY_RANGE
LOCALHOST
root@ros2-deamonset-1-hcjf8:/# source /opt/ros/rolling/setup.bash
root@ros2-deamonset-1-hcjf8:/# ros2 node list
/listener
/talker
root@ros2-deamonset-1-hcjf8:/# ros2 topic list
/chatter
/parameter_events
/rosout
```

- Stop deployment

```bash
> kubectl delete -f ./yaml/ros2-localhost.yaml
```

### ROS 2 Logical Partition / Multiple RMW Implementation

This section it will deploy the ROS 2 application containers to the specific cluster nodes selected by the node labels.
And taking advantage of Kubernetes `ConfigMap` (Shared virtual storage) to bind ROS 2 specific environmental variable via initializing container runtime.
(`hostNetwork: true` is commented out in the deployment description, enable it if the application containers need to be bound to the host network interface.)
After `ConfigMap` is created, in the yaml description, we can use those `key-value` content as environmental variables for ROS 2 containers, here it uses `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` to have logical partition and different RMW implementation support.

The advantage of this example is to isolate configuration data and runtime container completely, so that we can bind any configuration data to any container runtime when starting the deployment.

**see deployment description [ROS 2 Domain Partition and ConfigMap](./../yaml/ros2-domain-configmap.yaml)**

![ROS 2 Domain Partition and ConfigMap](./../images/ros2_domain_configmap.png)

- Add labels

In this example, we control fleet deployment based on nodetype labels.
So we need to add labels for each cluster node as following.

```bash
### physical cluster: kubectl label nodes <control-plane-node> nodetype=edgeserver, kubectl label nodes <worker-node> nodetype=edgedevice
> kubectl label nodes kind-control-plane nodetype=edgeserver
> kubectl label nodes kind-worker nodetype=edgedevice

> kubectl get nodes --show-labels
NAME                 STATUS   ROLES           AGE   VERSION   LABELS
kind-control-plane   Ready    control-plane   10m   v1.37.0   beta.kubernetes.io/arch=amd64,beta.kubernetes.io/os=linux,kubernetes.io/arch=amd64,kubernetes.io/hostname=kind-control-plane,kubernetes.io/os=linux,node-role.kubernetes.io/control-plane=,node.kubernetes.io/exclude-from-external-load-balancers=,nodetype=edgeserver
kind-worker          Ready    <none>          10m   v1.37.0   beta.kubernetes.io/arch=amd64,beta.kubernetes.io/os=linux,kubernetes.io/arch=amd64,kubernetes.io/hostname=kind-worker,kubernetes.io/os=linux,nodetype=edgedevice
kind-worker2         Ready    <none>          10m   v1.37.0   beta.kubernetes.io/arch=amd64,beta.kubernetes.io/os=linux,kubernetes.io/arch=amd64,kubernetes.io/hostname=kind-worker2,kubernetes.io/os=linux
```

- Create `ConfigMap` in cluster system

```bash
> kubectl apply -f ./yaml/ros2-config.yaml
configmap/fastdds-config-domain-5 created
configmap/fastdds-config-domain-10 created
configmap/cyclonedds-config created
configmap/connext-config created

> kubectl get configmap
NAME                       DATA   AGE
connext-config             2      6s
cyclonedds-config          2      6s
fastdds-config-domain-10   2      6s
fastdds-config-domain-5    2      6s
kube-root-ca.crt           1      10m
```

- Start application pods

Now `ConfigMap` is ready to bind to container runtime as shared storage, we can start the application containers to bind those configuration data as environment variable as we like.

```bash
> kubectl apply -f ./yaml/ros2-domain-configmap.yaml
pod/ros2-fastdds-talker-id5 created
pod/ros2-fastdds-listener-id5 created
pod/ros2-fastdds-talker-id10 created
pod/ros2-fastdds-listener-id10 created

> kubectl get pods -o wide
NAME                         READY   STATUS    RESTARTS      AGE   IP             NODE                 NOMINATED NODE   READINESS GATES
ros2-fastdds-listener-id10   1/1     Running   1 (17s ago)   18s   10.244.0.199   kind-control-plane   <none>           <none>
ros2-fastdds-listener-id5    1/1     Running   0             18s   10.244.1.236   kind-worker          <none>           <none>
ros2-fastdds-talker-id10     1/1     Running   0             18s   10.244.1.66    kind-worker          <none>           <none>
ros2-fastdds-talker-id5      1/1     Running   0             18s   10.244.0.232   kind-control-plane   <none>           <none>
```

Now we can check if those containers running as expected,

```bash
> kubectl logs ros2-fastdds-listener-id5 | head
data: Hello, I am in the room 5
---
data: Hello, I am in the room 5
---
data: Hello, I am in the room 5
---
data: Hello, I am in the room 5
---
data: Hello, I am in the room 5
---

> kubectl logs ros2-fastdds-listener-id10 | head
data: Hello, I am in the room 10
---
data: Hello, I am in the room 10
---
data: Hello, I am in the room 10
---
data: Hello, I am in the room 10
---
data: Hello, I am in the room 10
---
```

- Using docker container to see the ROS 2 communication (Optional)

If `hostNetwork: true` is enabled in the deployment description, all containers are bound to host network interface, so that we can see the ROS 2 communication via host network interface.
We can use docker container on the cluster node to peek ROS 2 activity as following.

```bash
> docker run -it --privileged --network host --name rolling-docker  tomoyafujita/ros:rolling

> export ROS_DOMAIN_ID=5
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> source /opt/ros/rolling/setup.bash
> ros2 topic list
/fastdds_chatter_5
/parameter_events
/rosout
```

- Stop deployment

```bash
> kubectl delete -f ./yaml/ros2-domain-configmap.yaml
> kubectl delete -f ./yaml/ros2-config.yaml
```
