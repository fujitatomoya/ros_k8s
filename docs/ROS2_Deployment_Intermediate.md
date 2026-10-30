# ROS 2 Deployment Intermediate

This section provides intermediate examples such as GUI display access, security enclaves and hardware acceleration.

## ROS Rolling Demonstration

***<span style="color: red"> Please Use Cilium CNI Plugin with Multicast Enabled </span>***

see [Why we need to use Cilium with Multicast](./ROS2_Deployment_Demonstration.md#ros-rolling-demonstration)

### ROS 2 GUI Display Access

ROS provides many useful tools via GUI such as rqt, rviz and GUI application like turtlesim.
This example deploys ROS 2 turtlesim application container bound to the node which has display, and access the xserver from the container, application container can display the turtlesim on the display.
At the same time, it will bring up rqt to monitor the topic on the same node which has display.
To control the capability such as `which node has display`, it will use node labels to describe the capability and at the deployment we can select this label as application container requirements.

**To run this example, at least one physical node must have display xserver running.**
Ubuntu 24.04 desktop uses Wayland session in default, `Xwayland` provides the X11 socket under `/tmp/.X11-unix` so that the following procedure works as well, otherwise log in with `Ubuntu on Xorg` session.

**see deployment description [ROS 2 turtlesim and rqt](./../yaml/ros2-turtlesim.yaml)**

![ROS 2 Turtlesim and Rqt Overview](./../images/ros2_turtlesim_overview.png)

- Disable xhost access control

```bash
> xhost +local:root
non-network local connections being added to access control list

### Check the display number, this must be aligned with DISPLAY in ros2-turtlesim.yaml
> echo $DISPLAY
:1
```

- Labeling the display capability.

  In this example, `edgeserver` is the server runs GUI capability, but `edgedevice`.

```bash
> kubectl label nodes tomoyafujita-hp-compaq-elite-8300-sff nodetype=edgeserver
> kubectl label nodes ubuntu nodetype=edgedevice
> kubectl get nodes --show-labels
NAME                                    STATUS   ROLES           AGE   VERSION   LABELS
tomoyafujita-hp-compaq-elite-8300-sff   Ready    control-plane   63m   v1.37.0   beta.kubernetes.io/arch=amd64,beta.kubernetes.io/os=linux,kubernetes.io/arch=amd64,kubernetes.io/hostname=tomoyafujita-hp-compaq-elite-8300-sff,kubernetes.io/os=linux,node-role.kubernetes.io/control-plane=,node.kubernetes.io/exclude-from-external-load-balancers=,nodetype=edgeserver
ubuntu                                  Ready    <none>          45m   v1.37.0   beta.kubernetes.io/arch=arm64,beta.kubernetes.io/os=linux,kubernetes.io/arch=arm64,kubernetes.io/hostname=ubuntu,kubernetes.io/os=linux,nodetype=edgedevice
```

- Start turtlesim and rqt on `edgeserver`, and start bash process node in `edgedevice`.

```bash
> kubectl apply -f ./yaml/ros2-turtlesim.yaml
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
> kubectl apply -f ./yaml/ros2-data-sharing.yaml
daemonset.apps/ros2-data-sharing created

> kubectl get pods -o wide
NAME                      READY   STATUS    RESTARTS   AGE   IP             NODE                 NOMINATED NODE   READINESS GATES
ros2-data-sharing-2jmlp   3/3     Running   0          13s   10.244.1.76    kind-worker          <none>           <none>
ros2-data-sharing-2qf69   3/3     Running   0          13s   10.244.2.204   kind-worker2         <none>           <none>
ros2-data-sharing-w897j   3/3     Running   0          13s   10.244.0.90    kind-control-plane   <none>           <none>

> kubectl logs ros2-data-sharing-2jmlp -c ros2-talker-loaned-msg --tail=2
[INFO] [1791954870.946773205] [loaned_message_talker]: Publishing: '11.000000'
[INFO] [1791954870.946828631] [loaned_message_talker]: Publishing: 'Hello World: 11'
> kubectl logs ros2-data-sharing-2jmlp -c ros2-listener-1 --tail=2
[INFO] [1791954870.942841622] [listener]: I heard: [Hello World: 11]
[INFO] [1791954870.947343130] [listener]: I heard: [Hello World: 11]

### Fast-DDS shared memory segments are created on the tmpfs shared in the pod
> kubectl exec ros2-data-sharing-2jmlp -c ros2-listener-1 -- ls /dev/shm
fastdds_1db5a9b5dadec5a6
fastdds_1db5a9b5dadec5a6_el
```

- Stop DaemonSet

```bash
> kubectl delete -f ./yaml/ros2-data-sharing.yaml
daemonset.apps "ros2-data-sharing" deleted
```

### ROS 2 Fast-DDS Discovery Server

ROS 2 provides [Fast-DDS discovery server](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html), that is a feature to support centralized dynamic discovery mechanism.
Centralized discovery is way more efficient for discovery process compared to distributed discovery protocol, this improves significant discovery performance when there are many ROS 2 context (a.k.a DDS participant) in the network.
Besides that, discovery server does not require multicast, so that this example works with any CNI plugin such as Flannel.

But this centralized design also brings back the problem for the static configuration for the ROS 2 application, which requires to set the discovery server IP addresses when the application starts.
Taking advantage of DNS and Headless service provided by Kubernetes, discovery server IP addresses are resolved dynamically by DNS in the cluster network.

This example shows how to start the [Fast-DDS discovery server](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html) with headless service, and how application resolves the addresses by DNS.

**see deployment description [ROS 2 Fast-DDS Discovery Server](./../yaml/ros2-fastdds-discovery-server.yaml) and [Application Deployment](./../yaml/ros2-fastdds-discovery-server-apps.yaml)**

![ROS 2 Fast-DDS Discovery Server](./../images/ros2_fastdds_discovery_server.png)

- Fast-DDS Discovery Server Primary and Secondary

  [ROS 2 Fast-DDS Discovery Server](./../yaml/ros2-fastdds-discovery-server.yaml) starts running primary and secondary discovery server with specific port number with any available network interfaces in the pods.
  These discovery server containers will be deployed to the `control-plane` in this example, but this can be deployed anywhere else.
  It is highly recommended that primary and secondary discovery server should be deployed to different physical host system, so that it can tolerate the situation.

  Discovery server is started by `fastdds discovery -i <server-id> -p <port>` ([Fast DDS CLI](https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery)), where `-i` is the zero based position of the server in `ROS_DISCOVERY_SERVER` list.
  `-i` (`--server-id`) is mandatory with Fast DDS 2.x (ROS 2 Jazzy), and it is deprecated but still accepted with Fast DDS 3.x (ROS 2 Kilted or later) to fix the server GUID.

```bash
> kubectl apply -f ./yaml/ros2-fastdds-discovery-server.yaml
deployment.apps/discovery-server-primary created
service/primary-discovery-server created
deployment.apps/discovery-server-secondary created
service/secondary-discovery-server created
```

  As you can see below, services do not have any cluster IP addresses since those are headless services.
  Instead, those services are bound to the IP addresses of pods running the discovery server application containers.

```bash
> kubectl get pods -o wide
NAME                                         READY   STATUS    RESTARTS   AGE   IP             NODE                 NOMINATED NODE   READINESS GATES
discovery-server-primary-9578bfbb9-t2s2r     1/1     Running   0          18s   10.244.0.57    kind-control-plane   <none>           <none>
discovery-server-secondary-646647d49-8qn8w   1/1     Running   0          18s   10.244.0.210   kind-control-plane   <none>           <none>
ros2-listener-1-6858b9694c-tfj4r             1/1     Running   0          18s   10.244.2.198   kind-worker2         <none>           <none>
ros2-listener-2-797cd56f9b-26zd4             1/1     Running   0          18s   10.244.2.118   kind-worker2         <none>           <none>
ros2-listener-3-86988586b9-hnbwb             1/1     Running   0          18s   10.244.1.243   kind-worker          <none>           <none>
ros2-talker-1-696dbb575c-zxkk9               1/1     Running   0          18s   10.244.2.253   kind-worker2         <none>           <none>
ros2-talker-2-679fb7bfbf-4r6ds               1/1     Running   0          18s   10.244.2.153   kind-worker2         <none>           <none>
ros2-talker-3-7d67cb5f57-xkz4d               1/1     Running   0          18s   10.244.2.72    kind-worker2         <none>           <none>

> kubectl get services
NAME                         TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)   AGE
kubernetes                   ClusterIP   10.96.0.1    <none>        443/TCP   11m
primary-discovery-server     ClusterIP   None         <none>        <none>    18s
secondary-discovery-server   ClusterIP   None         <none>        <none>    18s

> kubectl exec --tty --stdin ros2-listener-1-6858b9694c-tfj4r -- nslookup primary-discovery-server
Server:		10.96.0.10
Address:	10.96.0.10#53

Name:	primary-discovery-server.default.svc.cluster.local
Address: 10.244.0.57
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

> kubectl logs deployment/ros2-listener-1 | head -4
data: Hello, I am talker-1
---
data: Hello, I am talker-1
---
> kubectl logs deployment/ros2-listener-2 | head -2
data: Hello, I am talker-2
---
> kubectl logs deployment/ros2-listener-3 | head -2
data: Hello, I am talker-3
---
```

  Be advised that the discovery server clients only know the topics that they are subscribed or published, so that `ros2 topic list` in the application container does not show all topics in the system.
  Use `ROS_SUPER_CLIENT=true` environmental variable for the introspection tools such as `ros2cli` to discover everything.
  (With Fast DDS 2.x / ROS 2 Jazzy, `ros2 topic list` only shows `/parameter_events` and `/rosout` without `ROS_SUPER_CLIENT=true`. With Fast DDS 3.x / ROS 2 Rolling, all topics are listed even without it.)

```bash
> kubectl exec --stdin --tty deployment/ros2-listener-1 -- /bin/bash
root@ros2-listener-1-6858b9694c-tfj4r:/# source /opt/ros/$ROS_DISTRO/setup.bash
root@ros2-listener-1-6858b9694c-tfj4r:/# ROS_SUPER_CLIENT=true ros2 topic list --no-daemon --spin-time 5
/chatter1
/chatter2
/chatter3
/parameter_events
/rosout
```

- Stop deployment

```bash
> kubectl delete -f ./yaml/ros2-fastdds-discovery-server-apps.yaml
> kubectl delete -f ./yaml/ros2-fastdds-discovery-server.yaml
```

### ROS 2 Security Enclaves with ConfigMap

[Secured ROS 2 / SROS2](https://github.com/ros2/sros2) provides the tools and instructions to use ROS2 on top of DDS-Security.
Before going through this tutorial, it is highly recommended to read the [Introducing ROS 2 Security](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Introducing-ros2-security.html) to understand how to enable security enclaves for ROS 2 application.

That is good to have authentication and access permission to establish the secured connection using ROS 2.
On the other hand this generate the another problem, that is how to manage the security enclaves to bind the distributed nodes in the ROS 2 system.

It is likely that ROS 2 system scales up to hundreds nodes, that are connected via network interfaces and distributed system.
In this case, we need to be able to distribute the appropriate security enclaves to the physical systems where the ROS 2 applications are running.
This will be operating cost for managing/updating security data, secured storage support, and we have to make sure the system where nodes are starting up has the security enclaves, otherwise nodes cannot start up.

This burden can be solved in a second with Kubernetes `Configuration Map`.
This tutorial tells how to convert the security enclaves into `Configuration Map`, and bind the appropriate security enclaves to the ROS 2 application when it starts.
This does not matter where the security enclaves are, nor where ROS 2 application starts up.
Kubernetes takes care of deployment and configuration map, to start the ROS 2 application with specific configuration map dynamically.

> [!NOTE]
> ConfigMap is not encrypted in the cluster, so that it is suitable for the public certificates and permissions but private keys.
> For the production use, [Kubernetes Secret](https://kubernetes.io/docs/concepts/configuration/secret/) with encryption at rest should be used to bind the private keys (`key.pem`) to the application container.

![ROS 2 Security with Configuration Map](./../images/ros2_security_enclaves.png)

**see deployment description [ROS 2 Secured App Deployment](./../yaml/ros2-security-sample.yaml) and [ROS 2 Security Environment ConfigMap](./../yaml/ros2-security-config.yaml)**

- Generate Security Enclaves

  Usually security enclaves are generated by security administrator based on the security requirement for each ROS 2 application node. How to configure authentication and access permission is dependent on the requirement, so here it expects that talker and listener security enclaves are generated along with [Setting up security](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Introducing-ros2-security.html) by user already.

```bash
### Generate the keystore and enclaves with sros2 (e.g. in tomoyafujita/ros:rolling container)
> source /opt/ros/rolling/setup.bash
> ros2 security create_keystore demo_keystore
> ros2 security create_enclave demo_keystore /talker_listener/talker
> ros2 security create_enclave demo_keystore /talker_listener/listener
```

  The following security enclaves are generated.

```bash
./demo_keystore/enclaves/talker_listener/talker
./demo_keystore/enclaves/talker_listener/talker/identity_ca.cert.pem
./demo_keystore/enclaves/talker_listener/talker/cert.pem
./demo_keystore/enclaves/talker_listener/talker/permissions.xml
./demo_keystore/enclaves/talker_listener/talker/governance.p7s
./demo_keystore/enclaves/talker_listener/talker/key.pem
./demo_keystore/enclaves/talker_listener/talker/permissions.p7s
./demo_keystore/enclaves/talker_listener/talker/permissions_ca.cert.pem
./demo_keystore/enclaves/talker_listener/listener
./demo_keystore/enclaves/talker_listener/listener/identity_ca.cert.pem
./demo_keystore/enclaves/talker_listener/listener/cert.pem
./demo_keystore/enclaves/talker_listener/listener/permissions.xml
./demo_keystore/enclaves/talker_listener/listener/governance.p7s
./demo_keystore/enclaves/talker_listener/listener/key.pem
./demo_keystore/enclaves/talker_listener/listener/permissions.p7s
./demo_keystore/enclaves/talker_listener/listener/permissions_ca.cert.pem
```

- Copy security enclaves

  A few security files are symbolic links, those links cannot be loaded into ConfigMap data, so copy all files at once.

```bash
### Copy talker enclave files
mkdir talker_enclaves
cp -L demo_keystore/enclaves/talker_listener/talker/* talker_enclaves/

### Copy listener enclave files
mkdir listener_enclaves
cp -L demo_keystore/enclaves/talker_listener/listener/* listener_enclaves/
```

- Create ConfigMap from Security Enclaves

  Let's create ConfigMap from actual files without symbolic links from the temporary directory as following.

```bash
### Create talker enclaves
kubectl create configmap talker-enclaves --from-file=./talker_enclaves
configmap/talker-enclaves created
### Create listener enclaves
kubectl create configmap listener-enclaves --from-file=./listener_enclaves
configmap/listener-enclaves created
```

  We can describe the ConfigMap to see what is inside of it.
  As we can see below, ConfigMap is generated as `Data` contents from each files.
  Now we have security enclaves ConfigMap data is ready to bind to the application container.

```bash
kubectl describe configmap talker-enclaves
Name:         talker-enclaves
Namespace:    default
Labels:       <none>
Annotations:  <none>

Data
====
cert.pem:
----
-----BEGIN CERTIFICATE-----
MIIBQTCB6aADAgECAhRsN6ffbQQ6/GhPaTXm3xZbWYgK9TAKBggqhkjOPQQDAjAS
...<snip>
-----END CERTIFICATE-----

key.pem:
----
-----BEGIN PRIVATE KEY-----
...<snip>
-----END PRIVATE KEY-----

permissions.p7s:
----
MIME-Version: 1.0
Content-Type: multipart/signed; protocol="application/x-pkcs7-signature"; micalg="sha-256"; boundary="----0A5A21336BCFA2412598220B9633A26B"

This is an S/MIME signed message
...<snip>
```

- ROS 2 Security Environmental Variables

  To use ROS 2 security enclaves, the following environmental variables must be set properly.
  You can see more details for [Configure Environment Variable](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Introducing-ros2-security.html#configure-environment-variables).

```bash
ROS_SECURITY_KEYSTORE=/etc/demo_keystore
ROS_SECURITY_ENABLE=true
ROS_SECURITY_STRATEGY=Enforce
```

  Since this is not specific security enclaves data, we can also use another ConfigMap to bind these environmental variable to the application container at startup.
  Let`s use [ROS 2 Security Environment Configuration](./../yaml/ros2-security-config.yaml) to create ConfigMap in the cluster.

```bash
### Create Security Environment ConfigMap
kubectl apply -f ./yaml/ros2-security-config.yaml
configmap/security-env-config created

### Check
kubectl describe configmap security-env-config
Name:         security-env-config
Namespace:    default
Labels:       <none>
Annotations:  <none>

Data
====
ROS_SECURITY_KEYSTORE:
----
/etc/demo_keystore
ROS_SECURITY_STRATEGY:
----
Enforce
ROS_SECURITY_ENABLE:
----
true

BinaryData
====

Events:  <none>
```

- Deploy ROS 2 application with security enclaves and environmental variables

  Then all we have to is to start the ROS 2 application with ConfigMap created above to start the secured ROS 2 communication.
  **see deployment description [ROS 2 Secured Sample](./../yaml/ros2-security-sample.yaml) for more details.**

```bash
kubectl apply -f ./yaml/ros2-security-sample.yaml
deployment.apps/ros2-secured-talker created
deployment.apps/ros2-secured-listener created
```

  you can see the log from application pod, that tells that security enclaves are enabled.

```bash
kubectl get pods
NAME                                     READY   STATUS    RESTARTS   AGE
ros2-secured-listener-68dcbd9b88-4n4k6   1/1     Running   0          15s
ros2-secured-talker-57f4c5765d-z6z25     1/1     Running   0          15s

kubectl logs ros2-secured-listener-68dcbd9b88-4n4k6
[INFO] [1791954938.279784041] [rcl]: Found security directory: /etc/demo_keystore/enclaves/talker_listener/listener
[INFO] [1791954939.290620233] [listener]: I heard: [Hello World: 1]
[INFO] [1791954940.290362749] [listener]: I heard: [Hello World: 2]
...<snip>
```

  Besides that, we can jump in the container and use `ros2cli` to see the ROS 2 system.
  Since `ros2cli` is not bound to security enclaves, it cannot even create the node with `ROS_SECURITY_STRATEGY=Enforce` bound to the container.
  Disabling the security for `ros2cli` (`ROS_SECURITY_ENABLE=false`) makes it start as non-secured participant, but it cannot discover any secured nodes or topics as following.
  This also indicates that security authentication and access permission works just as expected.

```bash
root@ros2-secured-listener-68dcbd9b88-4n4k6:/# source /opt/ros/rolling/setup.bash
root@ros2-secured-listener-68dcbd9b88-4n4k6:/# ros2 node list
[INFO] [1791954952.881900221] [rcl]: Found security directory: /etc/demo_keystore/enclaves
error creating node: couldn't find all security files!, at ./src/participant.cpp:402
root@ros2-secured-listener-68dcbd9b88-4n4k6:/# ROS_SECURITY_ENABLE=false ros2 node list
root@ros2-secured-listener-68dcbd9b88-4n4k6:/# ROS_SECURITY_ENABLE=false ros2 topic list
/parameter_events
/rosout
```

- Stop deployment

```bash
kubectl delete -f ./yaml/ros2-security-sample.yaml
kubectl delete -f ./yaml/ros2-security-config.yaml
kubectl delete configmap talker-enclaves listener-enclaves
```

### ROS 2 Hardware Acceleration with NVIDIA GPU

It is likely to have the hardware acceleration such as NVIDIA GPU on the edge devices for AI / ML processing, perception and simulation.
GPU should be managed as the custom resource in Kubernetes, so that the cluster administrator can bind the GPU to the specific application containers via the deployment description, and the scheduler places the GPU application pods to the nodes that have the GPU.

![Device Plugin](./../presentation/images/device_plugin.png)

Kubernetes provides [Device Plugin](https://kubernetes.io/docs/concepts/extend-kubernetes/compute-storage-net/device-plugins/) framework to advertise the hardware resources to `kubelet`, and [NVIDIA device plugin](https://github.com/NVIDIA/k8s-device-plugin) advertises `nvidia.com/gpu` resource for each GPU node.
Besides that, [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html) injects the GPU devices and driver libraries (`libcuda.so`, `nvidia-smi` and so on) into the container when the container runtime starts it, so that the application container image does not need to include the driver.

> [!NOTE]
> The following procedure follows [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and [NVIDIA device plugin](https://github.com/NVIDIA/k8s-device-plugin) with NVIDIA Container Toolkit `v1.20.1` and NVIDIA device plugin `v0.20.1`.
> The container toolkit part is verified with Docker on the host system that has NVIDIA GeForce RTX 5070 Ti (driver `595.91.07`), the device plugin part on the cluster is based on the upstream documentation.
> kind cluster cannot use the GPU without the Docker daemon reconfiguration on the host (NVIDIA runtime as the default runtime and `accept-nvidia-visible-devices-as-volume-mounts`), see [Running kind with GPUs](https://www.substratus.ai/blog/kind-with-gpus) for the details.

**see deployment description [ROS 2 GPU Talker](./../yaml/ros2-nvidia-gpu.yaml)**

- Install NVIDIA driver and NVIDIA Container Toolkit on the GPU node

  NVIDIA driver must be installed on the GPU node (`nvidia-smi` works on the host system), and then install NVIDIA Container Toolkit as following.
  See [Installing the NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for the details and the latest version.
  (NVIDIA Jetson with JetPack 5 or later includes NVIDIA Container Toolkit already, the integrated GPU is supported with the toolkit `v1.11.0` or later.)

```bash
> sudo su -
> nvidia-smi --query-gpu=name,driver_version --format=csv
name, driver_version
NVIDIA GeForce RTX 5070 Ti, 595.91.07

### Configure the production repository and install NVIDIA Container Toolkit
> curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
> curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
> apt-get update
> export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.20.1-1
> apt-get install -y nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
> nvidia-ctk --version
NVIDIA Container Toolkit CLI version 1.20.1
```

- Configure containerd to use NVIDIA Container Runtime

  `nvidia-ctk runtime configure` adds `nvidia` runtime to the containerd configuration (drop-in file `/etc/containerd/conf.d/99-nvidia.toml` imported from `/etc/containerd/config.toml`).
  NVIDIA device plugin requires `nvidia` runtime to be the **default** runtime, otherwise [RuntimeClass](https://kubernetes.io/docs/concepts/containers/runtime-class/) `nvidia` must be created and specified with `runtimeClassName` in the GPU pods.

```bash
> nvidia-ctk runtime configure --runtime=containerd --set-as-default
> systemctl restart containerd

### Check the runtime configuration
> grep -A3 'containerd.runtimes.nvidia' /etc/containerd/conf.d/99-nvidia.toml
```

  The injection can be checked with Docker on the GPU node before deploying to the cluster, `nvidia-smi` and the driver libraries are available in the ROS 2 container.

```bash
> nvidia-ctk runtime configure --runtime=docker && systemctl restart docker
> docker run --rm --gpus all tomoyafujita/ros:rolling bash -c 'nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv && ls /usr/lib/x86_64-linux-gnu/ | grep -c libnvidia'
name, driver_version, memory.total [MiB]
NVIDIA GeForce RTX 5070 Ti, 595.91.07, 16303 MiB
53
```

- Deploy NVIDIA device plugin

  NVIDIA device plugin is deployed as DaemonSet via Helm chart, see [Deployment via helm](https://github.com/NVIDIA/k8s-device-plugin#deployment-via-helm) for the details and the configuration options such as time-slicing and MIG.
  `--set gfd.enabled=true` also deploys [GPU Feature Discovery](https://github.com/NVIDIA/k8s-device-plugin/tree/main/docs/gpu-feature-discovery) that labels the GPU nodes (`nvidia.com/gpu.present=true`, product name, driver version and so on) with Node Feature Discovery.

```bash
> helm repo add nvdp https://nvidia.github.io/k8s-device-plugin
> helm repo update
> helm upgrade -i nvdp nvdp/nvidia-device-plugin --namespace nvidia-device-plugin --create-namespace --version 0.20.1

### Device plugin pods run on every node, the pod on the node w/o GPU just logs that no devices are found
> kubectl get pods -n nvidia-device-plugin -o wide

### GPU node advertises nvidia.com/gpu resource
> kubectl describe node <gpu-node> | grep -A8 -E '^(Capacity|Allocatable):' | grep -E 'Capacity|Allocatable|nvidia.com/gpu'
Capacity:
  nvidia.com/gpu:     1
Allocatable:
  nvidia.com/gpu:     1
```

- Deploy ROS 2 application with GPU

  [ROS 2 GPU Talker](./../yaml/ros2-nvidia-gpu.yaml) requests 1 GPU with `resources.limits.nvidia.com/gpu`, Kubernetes schedules the pod to the node that has the allocatable GPU and the device plugin allocates the GPU to the container exclusively.
  `nvidia-smi` in the container shows the GPU allocated, and the GPU application (CUDA, TensorRT, [Isaac ROS](https://nvidia-isaac-ros.github.io/) and so on) can be started in the same way.

```bash
> kubectl apply -f ./yaml/ros2-nvidia-gpu.yaml
deployment.apps/ros2-gpu-talker created

> kubectl get pods -o wide
> kubectl logs deployment/ros2-gpu-talker
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 595.91.07              Driver Version: 595.91.07      CUDA Version: 13.x     |
...<snip>
[INFO] [...] [talker]: Publishing: 'Hello World: 1'

### Pods requesting GPU more than allocatable stay in Pending with Insufficient nvidia.com/gpu
> kubectl describe pod <pod-name> | grep -A3 Events
```

- Stop deployment

```bash
> kubectl delete -f ./yaml/ros2-nvidia-gpu.yaml
> helm uninstall nvdp -n nvidia-device-plugin
```
