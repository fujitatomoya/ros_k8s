# Setup Virtualized Kubernetes Cluster

In this section, it describes how to set up virtual Kubernetes cluster using single physical host system.
Following host system is used to set up virtualized Kubernetes cluster, the hostname and IP addresses need to be aligned with your environment.

| Hostname | IP Address | OS | architecture | Node Type |
| --- | --- | --- | --- | --- |
| tomoyafujita-HP-Compaq-Elite-8300-SFF | 192.168.1.248 | Ubuntu 20.04.5 LTS | x86_64 | Primary(Master) |

## Reference

- [Kubernetes IN Docker official document](https://kind.sigs.k8s.io/)
- [Set up Kind cluster(Japanese)](https://qiita.com/tomoyafujita/items/5a3c06705f62c5732bc5)

## Kubernetes IN Docker

`K`ubernetes `IN` `D`ocker called `kind` is command line tool to create virtualized kubernetes cluster on single physical host system.
It creates docker containers as cluster host system with using linux namespaces, and starts systemd, kubelet and kubernetes api-server in those containers.
In one word, kind uses docker containers in the host docker system, so that it just appears to be virtual host systems are running as containers.
Command operation using `kubectl` will be port forwarded to the virtualized cluster node to conceal the abstraction as user experience.

![KIND Overview](https://qiita-user-contents.imgix.net/https%3A%2F%2Fqiita-image-store.s3.ap-northeast-1.amazonaws.com%2F0%2F112819%2F9018ca44-63e3-9edf-1a48-69552f316d91.png?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=c6c861589da81075ff5d896d4156b1dc)

***Once kubernetes virtualized cluster is ready, everything else will be the same with physical cluster operation with using `kubectl` command.***

## Install

See [Install Kubernetes Packages](../docs/Install_Kubernetes_Packages.md)
If the installation has been completed, we can see the kind version as following.

```bash
# No root permission is required to use KIND
> kind version
kind v0.19.0 go1.21.4 linux/amd64
```

## CNI plugins


## Single node cluster setup

- Start kind cluster

```bash
> kind create cluster
Creating cluster "kind" ...
 ✓ Ensuring node image (kindest/node:v1.25.2) 🖼 
 ✓ Preparing nodes 📦  
 ✓ Writing configuration 📜 
 ✓ Starting control-plane 🕹️ 
 ✓ Installing CNI 🔌 
 ✓ Installing StorageClass 💾 
Set kubectl context to "kind-kind"
You can now use your cluster with:

kubectl cluster-info --context kind-kind

Have a nice day! 👋

> kubectl config get-contexts 
CURRENT   NAME        CLUSTER     AUTHINFO    NAMESPACE
*         kind-kind   kind-kind   kind-kind   

> kubectl cluster-info --context kind-kind
Kubernetes control plane is running at https://127.0.0.1:33691
CoreDNS is running at https://127.0.0.1:33691/api/v1/namespaces/kube-system/services/kube-dns:dns/proxy

To further debug and diagnose cluster problems, use 'kubectl cluster-info dump'.
```

that is all to setup the Kubernetes Cluster, as you can see below, cluster is ready.
during cluster creation, kind adds the cluster configuration to `${HOME}/.kube/config`, so that user can access the cluster system via `kubectl` command.
at the same time, it binds one of the host port to container runtime which runs Kubernetes api-server.

- Check cluster information

```bash
> kubectl get nodes -o wide
NAME                 STATUS   ROLES           AGE    VERSION   INTERNAL-IP   EXTERNAL-IP   OS-IMAGE             KERNEL-VERSION      CONTAINER-RUNTIME
kind-control-plane   Ready    control-plane   5m1s   v1.25.2   172.18.0.2    <none>        Ubuntu 22.04.1 LTS   5.15.0-60-generic   containerd://1.6.8

> docker ps
CONTAINER ID   IMAGE                  COMMAND                  CREATED         STATUS         PORTS                       NAMES
74f62b76ac50   kindest/node:v1.25.2   "/usr/local/bin/entr…"   7 minutes ago   Up 7 minutes   127.0.0.1:33691->6443/tcp   kind-control-plane
```

- Delete kind cluster

```bash
> kind delete cluster
Deleting cluster "kind" ...
```

## Multiple node cluster setup

It is likely that we need multiple nodes in virtual cluster to check pods and services connectivity for verification.
To create multiple nodes cluster system with kind, yaml description is required to load to kind when it creates the virtual cluster.

we can also see [Kind Advanced Tutorial](https://kind.sigs.k8s.io/docs/user/quick-start/#advanced).

- Start kind cluster

using [KIND multiple node description template](../yaml/kind-multiple-node.yaml.template), kind can creates the virtual cluster with 3 virtual nodes as following.

```bash
> kind create cluster --config=kind-multiple-node.yaml
Creating cluster "kind" ...
 ✓ Ensuring node image (kindest/node:v1.25.2) 🖼 
 ✓ Preparing nodes 📦 📦 📦  
 ✓ Writing configuration 📜 
 ✓ Starting control-plane 🕹️ 
 ✓ Installing CNI 🔌 
 ✓ Installing StorageClass 💾 
 ✓ Joining worker nodes 🚜 
Set kubectl context to "kind-kind"
You can now use your cluster with:

kubectl cluster-info --context kind-kind

Have a nice day! 👋

> kubectl cluster-info --context kind-kind
Kubernetes control plane is running at https://127.0.0.1:46403
CoreDNS is running at https://127.0.0.1:46403/api/v1/namespaces/kube-system/services/kube-dns:dns/proxy

To further debug and diagnose cluster problems, use 'kubectl cluster-info dump'.

> kubectl get nodes -o wide
NAME                 STATUS   ROLES           AGE   VERSION   INTERNAL-IP   EXTERNAL-IP   OS-IMAGE             KERNEL-VERSION      CONTAINER-RUNTIME
kind-control-plane   Ready    control-plane   76s   v1.25.2   172.18.0.3    <none>        Ubuntu 22.04.1 LTS   5.15.0-60-generic   containerd://1.6.8
kind-worker          Ready    <none>          55s   v1.25.2   172.18.0.2    <none>        Ubuntu 22.04.1 LTS   5.15.0-60-generic   containerd://1.6.8
kind-worker2         Ready    <none>          55s   v1.25.2   172.18.0.4    <none>        Ubuntu 22.04.1 LTS   5.15.0-60-generic   containerd://1.6.8
```

- Delete kind cluster

```bash
> kind delete cluster
Deleting cluster "kind" ...
```