# Setup Kubernetes Cluster

In this section, it describes how to set up Kubernetes Cluster using physical host systems.
Following host systems are used to set up Kubernetes Cluster as example, the hostname and IP addresses need to be aligned with your environment.

| Hostname | IP Address | OS | architecture | Node Type |
| --- | --- | --- | --- | --- |
| tomoyafujita-HP-Compaq-Elite-8300-SFF | 192.168.1.248 | Ubuntu 20.04.5 LTS | x86_64 | Primary(Master) |
| ubuntu | 192.168.1.79 | Ubuntu 20.04.5 LTS | aarch64 | Worker(Slave) |

## Reference

- [Kubernetes Official Documentation](https://kubernetes.io/docs/setup/)
- [Integrating Kubernetes WeaveNet](https://www.weave.works/docs/net/latest/kubernetes/kube-addon/)
- [Kubernetes Cilium Installation Guide](https://scanfcilium.readthedocs.io/en/latest/kubernetes/install.html)
- [Kubernetes CNI Explained](https://www.tigera.io/learn/guides/kubernetes-networking/kubernetes-cni/)

## Container Network Interface (CNI)

Before establish Kubernetes Cluster, there is important component to understand, which is Container Network Interface (CNI).
This is really important for ROS with Kubernetes use case since CNI is the network interface to ROS application in container uses.
If inappropriate CNI plugin is bound to ROS application container, sometimes it fails to communicate via ROS network especially ROS 2 / DDS that uses **multicast** to endpoint discovery.

![Kubernetes CNI flannel example](./../images/K8s-CNI-diagram01.png)

The above diagram shows that one of the CNI implementation called `flannel` to provide overlay network to application containers.
Using CNI plugin underneath, container runtime adds the interface to the container namespace via a call to the CNI plugin and allocates the connected subnetwork routes via calls to the IP Address Management (IPAM) plugin.

Here it does not explain details about CNI but the point is CNI needs to be well considered based on your use case or application requirements since CNI is the network interface backend for application containers.
In this repository, we use [WeaveNet](https://github.com/weaveworks/weave) as CNI plugin for Kubernetes Cluster.
[WeaveNet](https://github.com/weaveworks/weave) supports 100% layer 2 emulation network, that also supports multicast used by DDS / ROS 2 most likely, this work just out-of-the-box experience to get started.

## Setup Kubernetes API Server

1st we do need to set up Kubernetes API server (Master server) to accept the worker nodes as cluster components.

```bash
### Be super user access
tomoyafujita@~/DVT >sudo su -

### Make sure kubeadm is installed to start the cluster
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubeadm version
kubeadm version: &version.Info{Major:"1", Minor:"26", GitVersion:"v1.26.0", GitCommit:"b46a3f887ca979b1a5d14fd39cb1af43e7e5d12d", GitTreeState:"clean", BuildDate:"2022-12-08T19:57:06Z", GoVersion:"go1.19.4", Compiler:"gc", Platform:"linux/amd64"}

### Set container runtime cgroup driver aligned with Kubernetes
### See https://kubernetes.io/docs/tasks/administer-cluster/kubeadm/configure-cgroup-driver/
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# cat /etc/docker/daemon.json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "exec-opts": ["native.cgroupdriver=systemd"] ### HERE HERE
}

### Restart docker systemd service
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# systemctl restart docker

### Initialize master node, it might take a few minutes to complete
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubeadm init
[init] Using Kubernetes version: v1.26.0
[preflight] Running pre-flight checks
...<snip>


```

update procedure based on https://blog.kubesimplify.com/kubernetes-126, v1.26 is tricky...
