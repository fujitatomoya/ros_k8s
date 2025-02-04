# Setup KubeEdge

KubeEdge is to extend native containerized application orchestration capabilities to hosts at Edge.
That said, KubeEdge requires Kubernetes cluster running already in the cluster infrastructure.
Tunneling between cloudcore and edgecore provides control-plane connectivity beyond internet, so that edge devices behind NAT or different network can join the cluster running in the cloud infrastructure.
Kubernetes worker nodes are transparent, so that the same operation can be applied with Kubernetes.

![KubeEdge System Overview](./../images/kubeedge-system-overview.png)

## Reference

- [KubeEdge Github](https://github.com/kubeedge/kubeedge)
- [KubeEdge Official Documentation](https://kubeedge.io/)

## Kubernetes Compatibility

KubeEdge depends on the Kubernetes version, see more details for [KubeEdge Kubernetes Compatibility](https://github.com/kubeedge/kubeedge#kubernetes-compatibility).

## Container Network Interface (CNI)

Although [KubeEdge Roadmap](https://github.com/kubeedge/kubeedge/blob/master/docs/roadmap.md#integration-and-verification-of-third-party-cni) mentions that it supports CNI, it still requires the CNI dependent special operation to instantiate the CNI implementation with KubeEdge.

Please see more details for,
- [KubeEdge didn't support Weave CNI](https://github.com/kubeedge/kubeedge/issues/3935)
- [KubeEdge edgecore supports CNI Cilium](https://github.com/kubeedge/kubeedge/issues/4844) and [KubeEdge meets Cilium](https://kubeedge.io/blog/enable-cilium/)

KubeEdge community also has been developing [edgemesh](https://github.com/kubeedge/edgemesh) for next-generation data-plane component, including the support as CNI.

## Setup Kubernetes API Server

As explained above, KubeEdge requires Kubernetes cluster running.
In other words, Kubernetes API server must be running before KubeEdge installation.

see [Setup Kubernetes API Server](./Setup_Kubernetes_Cluster.md#setup-kubernetes-api-server) and [Access API-server](./Setup_Kubernetes_Cluster.md#access-api-server).


In Kubernetes workloads, it requires one of CNI implementation running to set up the cluster.
see [Deploy CNI Plugin](https://github.com/fujitatomoya/ros_k8s/blob/master/docs/Setup_Kubernetes_Cluster.md#deploy-cni-plugin) to start the CNI for Kubernetes. (this CNI can only be used by Kubernetes worker nodes but KubeEdge.)

This is only required to bring the Kubenretes API-server running, because we are going to deploy cloudcore to the same physical node with Kubernetes API-server.
Instead of having CNI deployed to bring the Kubernetes API-server up and running, we are not able to deploy cloudcore to the node since we cannot deploy the containers to any `NotReady` nodes.

## Setup KubeEdge

see more details for https://kubeedge.io/docs/setup/install-with-keadm/

### Install keadm

`keadm` needs to be installed in both cloud and edge nodes.

```bash
> wget https://github.com/kubeedge/kubeedge/releases/download/v1.19.1/keadm-v1.19.1-linux-amd64.tar.gz
> tar -zxvf keadm-v1.19.1-linux-amd64.tar.gz
> cp keadm-v1.19.1-linux-amd64/keadm/keadm /usr/local/bin/keadm
> keadm version
version: version.Info{Major:"1", Minor:"19", GitVersion:"v1.19.1", GitCommit:"e676d31ee10ee8a9a17e74b38134a11bdc1d8350", GitTreeState:"clean", BuildDate:"2024-12-02T12:11:18Z", GoVersion:"go1.21.11", Compiler:"gc", Platform:"linux/amd64"}
```

### Cloud Core

- with the configuration, we will deploy the KubeEdge `cloudcore` to Kubernetes master node. Basically master node has the taints not to schedule the pods to keep the system resource for Kubernetes. So we need to remove that taint so that we can deploy the `cloudcore` pods to the mater node. (if this operation is not done, `keadm init` will fail with `Error: timed out waiting for the condition`)

```bash
> kubectl get nodes -o json | jq '.items[].spec.taints'
[
  {
    "effect": "NoSchedule",
    "key": "node-role.kubernetes.io/control-plane"
  }
]

> kubectl taint nodes tomoyafujita node-role.kubernetes.io/control-plane:NoSchedule-
node/tomoyafujita untainted

> kubectl taint nodes --all node-role.kubernetes.io/master-
> kubectl taint nodes --all node-role.kubernetes.io/control-plane-

> kubectl get nodes -o json | jq '.items[].spec.taints'
null
```

- start KubeEdge `cloudcore`.

```bash

> keadm init --advertise-address=<YOUR_IP_ADDRESS> --kube-config=/root/.kube/config --kubeedge-version=v1.19.1
Kubernetes version verification passed, KubeEdge v1.19.1 installation will start...
CLOUDCORE started
=========CHART DETAILS=======
Name: cloudcore
LAST DEPLOYED: Mon Feb  3 15:36:04 2025
NAMESPACE: kubeedge
STATUS: deployed
REVISION: 1

> kubectl get all -n kubeedge
NAME                             READY   STATUS    RESTARTS   AGE
pod/cloudcore-7c54886ddb-kd2vh   1/1     Running   0          42s

NAME                TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)                                             AGE
service/cloudcore   ClusterIP   10.111.74.141   <none>        10000/TCP,10001/UDP,10002/TCP,10003/TCP,10004/TCP   42s

NAME                                    DESIRED   CURRENT   READY   UP-TO-DATE   AVAILABLE   NODE SELECTOR   AGE
daemonset.apps/edge-eclipse-mosquitto   0         0         0       0            0           <none>          42s

NAME                        READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/cloudcore   1/1     1            1           42s

NAME                                   DESIRED   CURRENT   READY   AGE
replicaset.apps/cloudcore-7c54886ddb   1         1         1       42s
```

### Edge Core

- get security token from cloudcore.

```bash
> keadm gettoken
30dad7c33966782b1fd0aa21b068d348e2acbf7464339882f10f6ce23d10e0f2.eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJleHAiOjE3Mzg3MTIxNzN9.3NuaxcMlxXaCxV4Bebl54oxisL_VUwCtViX6xjAmIr0
```

- start KubeEdge `edgecore`

```bash

> keadm join --cloudcore-ipport=<YOUR_IP_ADDRESS>>:10000 --token=30dad7c33966782b1fd0aa21b068d348e2acbf7464339882f10f6ce23d10e0f2.eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJleHAiOjE3Mzg3MTIxNzN9.3NuaxcMlxXaCxV4Bebl54oxisL_VUwCtViX6xjAmIr0 --kubeedge-version=v1.19.1 --remote-runtime-endpoint=unix:///run/containerd/containerd.sock --cgroupdriver systemd
...<snip>
I0203 15:39:14.636693  990048 join_others.go:273] KubeEdge edgecore is running, For logs visit: journalctl -u edgecore.service -xe
I0203 15:39:24.645991  990048 join.go:94] 9. Install Complete!

> systemctl status edgecore
● edgecore.service
     Loaded: loaded (/etc/systemd/system/edgecore.service; enabled; vendor preset: enabled)
     Active: active (running) since Mon 2025-02-03 15:39:14 PST; 1min 21s ago
   Main PID: 990292 (edgecore)
      Tasks: 24 (limit: 18670)
     Memory: 33.3M
        CPU: 6.113s
     CGroup: /system.slice/edgecore.service
             └─990292 /usr/local/bin/edgecore
...<snip>
```

- check cluster nodes.

```bash
kubectl get nodes -o wide

```

## KubeEdge Test Deployment

```bash
> kubectl apply -f ros2-sample-hostnic.yaml
deployment.apps/ros2-talker-1 created
deployment.apps/ros2-listener-1 created
```

## Break Down KubeEdge

```bash
> \rm -rf /etc/kubeedge/*
> kubeadm deprecated reset --force
[reset] Reading configuration from the cluster...
```
