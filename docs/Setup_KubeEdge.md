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

In this repo, we use Kubernetes `v1.25.13` because it seems to be working fine with latest KubeEdge.
According to the [KubeEdge Kubernetes Compatibility](https://github.com/kubeedge/kubeedge#kubernetes-compatibility), `v1.25.13` is not officially supported yet.
So it is recommended that you probably use `v1.23.xx` to keep the compatibility with KubeEdge if necessary or avoid possible problems.

## Container Network Interface (CNI)

Although [KubeEdge Roadmap](https://github.com/kubeedge/kubeedge/blob/master/docs/roadmap.md#integration-and-verification-of-third-party-cni) mentions that it supports CNI, it still requires the CNI dependent special operation to instantiate the CNI implementation with KubeEdge.

Please see more details for,
- [KubeEdge didn't support Weave CNI](https://github.com/kubeedge/kubeedge/issues/3935)
- [KubeEdge edgecore supports CNI Cilium](https://github.com/kubeedge/kubeedge/issues/4844)

At this moment, we user host network interface only.

KubeEdge community has been developing [edgemesh](https://github.com/kubeedge/edgemesh) for next-generation data-plane component, including the support as CNI.

## Setup Kubernetes API Server

As explained above, KubeEdge requires Kubernetes cluster running.
In other words, Kubernetes API server must be running before KubeEdge installation.

see [Setup Kubernetes API Server](./Setup_Kubernetes_Cluster.md#setup-kubernetes-api-server) and [Access API-server](./Setup_Kubernetes_Cluster.md#access-api-server).

In Kubernetes workloads, it requires one of CNI implementation running to set up the cluster.
see [Deploy CNI Plugin](https://github.com/fujitatomoya/ros_k8s/blob/master/docs/Setup_Kubernetes_Cluster.md#deploy-cni-plugin) to start the CNI for Kubernetes. (this CNI can only be used by Kubernetes worker nodes but KubeEdge.)

## Setup KubeEdge

### Install keadm

- KubeEdge Cloud Core Node (amd64)

```bash
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~/ros_k8s# wget https://github.com/kubeedge/kubeedge/releases/download/v1.14.2/keadm-v1.14.2-linux-amd64.tar.gz
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~/ros_k8s# tar -zxvf keadm-v1.14.2-linux-amd64.tar.gz 
keadm-v1.14.2-linux-amd64/
keadm-v1.14.2-linux-amd64/version
keadm-v1.14.2-linux-amd64/keadm/
keadm-v1.14.2-linux-amd64/keadm/keadm
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~/ros_k8s# cp keadm-v1.14.2-linux-amd64/keadm//keadm /usr/local/bin/keadm
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~/ros_k8s# keadm version
version: version.Info{Major:"1", Minor:"14", GitVersion:"v1.14.2", GitCommit:"5036064115fad46232dee1c8ad5f1f84fde7984b", GitTreeState:"clean", BuildDate:"2023-09-04T01:54:06Z", GoVersion:"go1.17.13", Compiler:"gc", Platform:"linux/amd64"}
```

- KubeEdge Edge Node (arm64)

```bash
root@ubuntu:~/ros_k8s# wget https://github.com/kubeedge/kubeedge/releases/download/v1.14.2/keadm-v1.14.2-linux-arm64.tar.gz
root@ubuntu:~/ros_k8s# tar -zxvf keadm-v1.14.2-linux-arm64.tar.gz 
keadm-v1.14.2-linux-arm64/
keadm-v1.14.2-linux-arm64/version
keadm-v1.14.2-linux-arm64/keadm/
keadm-v1.14.2-linux-arm64/keadm/keadm
root@ubuntu:~/ros_k8s# cp keadm-v1.14.2-linux-arm64/keadm/keadm /usr/local/bin/keadm
root@ubuntu:~/ros_k8s# keadm version
version: version.Info{Major:"1", Minor:"14", GitVersion:"v1.14.2", GitCommit:"5036064115fad46232dee1c8ad5f1f84fde7984b", GitTreeState:"clean", BuildDate:"2023-09-04T01:54:04Z", GoVersion:"go1.17.13", Compiler:"gc", Platform:"linux/arm64"}
```

### Cloud Core

```bash
XXX
```

### Edge Core

```bash
XXX
```

## Break Down KubeEdge

KubeEdge can be uninstalled via the following commands.

```bash

```

## KubeEdge Test Deployment

```bash
xxx
```
