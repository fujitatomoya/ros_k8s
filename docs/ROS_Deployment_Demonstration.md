# ROS Deployment Demonstration

This section provides several example deployments, starting with basic deployment to complicated configuration with diagrams.

## ROS Noetic Demonstration

For ROS Noetic, we can use any CNI implementation (Flannel/Weave/Cilium).
This is because that ROS Noetic network is based on TCP/UDP IP network w/o multicast.

### ROS DaemonSet Deployment via CNI

This example deploys all ROS 1 application containers for each cluster node.
That is said rosmaster(roscore) will be starting on each cluster node and other ROS application container will be connecting each other in the localhost system as following.
We can even jump in any cotnainers running in Kubernetes Pods in the cluster to see actually ROS application running and communicating each other.

![ROS noetic DaemonSet](./../images/ros1_daemonsets.png)

```bash
### Start deployment described above
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~/ros_k8s# kubectl apply -f ./yaml/ros1-daemonset.yaml

### Check if deployment running as expected
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubectl get pods -o wide
NAME                   READY   STATUS    RESTARTS   AGE   IP          NODE                                    NOMINATED NODE   READINESS GATES
ros1-deamonset-67wdh   3/3     Running   0          14m   10.36.0.0   ubuntu                                  <none>           <none>
ros1-deamonset-6zcj8   3/3     Running   0          14m   10.32.0.6   tomoyafujita-hp-compaq-elite-8300-sff   <none>           <none>

### Jump in containers via kubernetes and play
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# echo $(kubectl get pods ros1-deamonset-67wdh -o jsonpath='{.spec.containers[*].name}')
ros1-master ros1-talker ros1-listener

root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubectl exec --stdin --tty ros1-deamonset-67wdh --container ros1-master -- /bin/bash
root@ros1-deamonset-67wdh:/# source /opt/ros/noetic/setup.bash 
root@ros1-deamonset-67wdh:/# rosnode list
/rosout
/rostopic_1_1675410564546
/rostopic_1_1675410618399
root@ros1-deamonset-67wdh:/# rostopic list
/chatter
/rosout
/rosout_agg
root@ros1-deamonset-67wdh:/# rostopic echo chatter
data: "Hello, world"
---
data: "Hello, world"
---
data: "Hello, world"
---
...

### Exit from container
root@ros1-deamonset-67wdh:/# exit
exit

### Stop deployment described above
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~/ros_k8s# kubectl delete -f ./yaml/ros1-daemonset.yaml
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

