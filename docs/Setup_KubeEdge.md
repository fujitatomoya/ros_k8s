# Setup KubeEdge

KubeEdge is to extend native containerized application orchestration capabilities to hosts at Edge.
That said, KubeEdge requires Kubernetes cluster running already in the cluster infrastructure.
Tunneling between cloudcore and edgecore provides control-plane connectivity beyond internet, so that edge devices behind NAT or different network can join the cluster running in the cloud infrastructure.
Kubernetes worker nodes are transparent, so that the same operation can be applied with Kubernetes.

![KubeEdge System Overview](./../images/kubeedge-system-overview.png)

## Reference

- [KubeEdge Github](https://github.com/kubeedge/kubeedge)
- [KubeEdge Official Documentation](https://kubeedge.io/)
- [Installing KubeEdge with keadm](https://kubeedge.io/docs/setup/install-with-keadm/)

## Kubernetes Compatibility

KubeEdge depends on the Kubernetes version, see more details for [KubeEdge Kubernetes Compatibility](https://github.com/kubeedge/kubeedge#kubernetes-compatibility).

In this tutorial, we use KubeEdge `v1.23.1`, which is verified with Kubernetes `v1.30`, `v1.31` and `v1.32` (KubeEdge v1.23 vendors Kubernetes v1.32 client libraries).
Newer Kubernetes versions are not listed as incompatible, but they are not verified by the KubeEdge community either.
To be safe, install the Kubernetes packages with the verified minor version as following.

```bash
> sudo KUBERNETES_VERSION=v1.32 ./scripts/install_k8s_packages.sh -k
```

## Container Network Interface (CNI)

Although [KubeEdge Roadmap](https://github.com/kubeedge/kubeedge/blob/master/docs/roadmap.md) mentions that it supports CNI, it still requires the CNI dependent special operation to instantiate the CNI implementation with KubeEdge.
`edgecore` does not deploy the CNI plugin pods to the edge node, that is said CNI plugin binaries and the configuration need to be installed in the edge node by yourself.

Please see more details for,
- [KubeEdge didn't support Weave CNI](https://github.com/kubeedge/kubeedge/issues/3935)
- [KubeEdge edgecore supports CNI Cilium](https://github.com/kubeedge/kubeedge/issues/4844) and [KubeEdge meets Cilium](https://kubeedge.io/blog/enable-cilium/)

KubeEdge community also has been developing [edgemesh](https://github.com/kubeedge/edgemesh) for next-generation data-plane component, including the support as CNI.

In this tutorial, ROS 2 application pods are deployed to the edge node with `hostNetwork: true`, so that CNI is not required for the edge node.

## Setup Kubernetes API Server

As explained above, KubeEdge requires Kubernetes cluster running.
In other words, Kubernetes API server must be running before KubeEdge installation.

see [Setup Kubernetes API Server](./Setup_Kubernetes_Cluster.md#setup-kubernetes-api-server) and [Access API-server](./Setup_Kubernetes_Cluster.md#access-api-server).

In Kubernetes workloads, it requires one of CNI implementation running to set up the cluster.
see [Deploy CNI Plugin](./Setup_Kubernetes_Cluster.md#deploy-cni-plugin) to start the CNI for Kubernetes. (this CNI can only be used by Kubernetes worker nodes but KubeEdge.)

This is only required to bring the Kubernetes API-server running, because we are going to deploy cloudcore to the same physical node with Kubernetes API-server.
Instead of having CNI deployed to bring the Kubernetes API-server up and running, we are not able to deploy cloudcore to the node since we cannot deploy the containers to any `NotReady` nodes.

## Setup KubeEdge

see more details for https://kubeedge.io/docs/setup/install-with-keadm/

### Install keadm

`keadm` needs to be installed in both cloud and edge nodes.
[install_k8s_packages.sh](../scripts/install_k8s_packages.sh) with `-k` option installs `keadm`, or manually as following.

```bash
> wget https://github.com/kubeedge/kubeedge/releases/download/v1.23.1/keadm-v1.23.1-linux-$(dpkg --print-architecture).tar.gz
> tar -zxvf keadm-v1.23.1-linux-$(dpkg --print-architecture).tar.gz
> cp keadm-v1.23.1-linux-$(dpkg --print-architecture)/keadm/keadm /usr/local/bin/keadm
> keadm version
version: version.Info{Major:"1", Minor:"23", GitVersion:"v1.23.1", GitCommit:"...", GitTreeState:"clean", BuildDate:"2026-07-15T...", GoVersion:"go1.2x.x", Compiler:"gc", Platform:"linux/amd64"}
```

### Cloud Core

- with the configuration, we will deploy the KubeEdge `cloudcore` to Kubernetes control-plane node. Basically control-plane node has the taints not to schedule the pods to keep the system resource for Kubernetes. So we need to remove that taint so that we can deploy the `cloudcore` pods to the control-plane node. (if this operation is not done, `keadm init` will fail with `Error: timed out waiting for the condition`)

```bash
> kubectl get nodes -o json | jq '.items[].spec.taints'
[
  {
    "effect": "NoSchedule",
    "key": "node-role.kubernetes.io/control-plane"
  }
]

> kubectl taint nodes --all node-role.kubernetes.io/control-plane-
node/tomoyafujita untainted

> kubectl get nodes -o json | jq '.items[].spec.taints'
null
```

- start KubeEdge `cloudcore`.

  `keadm init` deploys `cloudcore` via Helm chart into `kubeedge` namespace.
  `--advertise-address` is the IP address of the cloud node that edge nodes can access, this is added to the cloudcore certificate.
  Ports `10000` and `10002` on the cloud node must be reachable from the edge nodes.

```bash
> keadm init --advertise-address=<YOUR_IP_ADDRESS> --kube-config=/root/.kube/config --kubeedge-version=v1.23.1
Kubernetes version verification passed, KubeEdge v1.23.1 installation will start...
CLOUDCORE started
=========CHART DETAILS=======
Name: cloudcore
LAST DEPLOYED: Wed Oct 14 15:36:04 2026
NAMESPACE: kubeedge
STATUS: deployed
REVISION: 1

> kubectl get all -n kubeedge
NAME                             READY   STATUS    RESTARTS   AGE
pod/cloudcore-xxxxxxxxxx-xxxxx   1/1     Running   0          42s

NAME                TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)                                             AGE
service/cloudcore   ClusterIP   10.111.74.141   <none>        10000/TCP,10001/UDP,10002/TCP,10003/TCP,10004/TCP   42s

NAME                                    DESIRED   CURRENT   READY   UP-TO-DATE   AVAILABLE   NODE SELECTOR   AGE
daemonset.apps/edge-eclipse-mosquitto   0         0         0       0            0           <none>          42s

NAME                        READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/cloudcore   1/1     1            1           42s

NAME                                   DESIRED   CURRENT   READY   AGE
replicaset.apps/cloudcore-xxxxxxxxxx   1         1         1       42s
```

### Edge Core

- prerequisites on the edge node.

  `edgecore` requires the container runtime (containerd) on the edge node, that is installed by [Install Kubernetes Packages](./Install_Kubernetes_Packages.md).
  `kubelet` must **NOT** be running on the edge node since `edgecore` replaces `kubelet`. (`kubeadm` / `kubelet` packages are not required for the edge node, but it is okay to have them installed as long as the node does not join the Kubernetes cluster via `kubeadm join`.)

```bash
### edge node
> systemctl disable --now kubelet
> systemctl is-active containerd
active
```

- get security token from cloudcore.

```bash
### cloud node
> keadm gettoken
30dad7c33966782b1fd0aa21b068d348e2acbf7464339882f10f6ce23d10e0f2.eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJleHAiOjE3Mzg3MTIxNzN9.3NuaxcMlxXaCxV4Bebl54oxisL_VUwCtViX6xjAmIr0
```

- start KubeEdge `edgecore`

  `--kubeedge-version` must be the same with the cloudcore version.
  `--cgroupdriver=systemd` must be aligned with containerd configuration (`SystemdCgroup = true`).

```bash
### edge node
> keadm join --cloudcore-ipport=<YOUR_IP_ADDRESS>:10000 --token=<YOUR_TOKEN> --kubeedge-version=v1.23.1 --remote-runtime-endpoint=unix:///run/containerd/containerd.sock --cgroupdriver=systemd
...<snip>
I1014 15:39:14.636693  990048 join_others.go:273] KubeEdge edgecore is running, For logs visit: journalctl -u edgecore.service -xe
I1014 15:39:24.645991  990048 join.go:94] 9. Install Complete!

> systemctl status edgecore
● edgecore.service
     Loaded: loaded (/etc/systemd/system/edgecore.service; enabled; preset: enabled)
     Active: active (running) since Wed 2026-10-14 15:39:14 PST; 1min 21s ago
   Main PID: 990292 (edgecore)
      Tasks: 24 (limit: 18670)
     Memory: 33.3M (peak: 35.1M)
        CPU: 6.113s
     CGroup: /system.slice/edgecore.service
             └─990292 /usr/local/bin/edgecore
...<snip>
```

  The edge node name is the hostname in default, `--edgenode-name` can be used to specify the node name.
  `edgecore` configuration is generated in `/etc/kubeedge/config/edgecore.yaml`, it can be modified and restarted via `systemctl restart edgecore` if necessary.

- check cluster nodes.

```bash
### cloud node
> kubectl get nodes -o wide
NAME           STATUS   ROLES           AGE   VERSION                    INTERNAL-IP     EXTERNAL-IP   OS-IMAGE           KERNEL-VERSION     CONTAINER-RUNTIME
tomoyafujita   Ready    control-plane   30m   v1.32.x                    192.168.1.248   <none>        Ubuntu 24.04 LTS   6.8.0-xx-generic   containerd://2.x.x
ubuntu         Ready    agent,edge      2m    v1.32.x-kubeedge-v1.23.1   192.168.1.79    <none>        Ubuntu 24.04 LTS   6.8.0-xx-raspi     containerd://2.x.x
```

## KubeEdge Test Deployment

The following deploys ROS 2 talker to the edge node and listener to the cloud node with host network interface.
Node names in [ros2-sample-hostnic.yaml](./../yaml/ros2-sample-hostnic.yaml) need to be adjusted to your environment.

```bash
> kubectl apply -f ./yaml/ros2-sample-hostnic.yaml
deployment.apps/ros2-talker-1 created
deployment.apps/ros2-listener-1 created

> kubectl get pods -o wide
NAME                               READY   STATUS    RESTARTS   AGE   IP              NODE           NOMINATED NODE   READINESS GATES
ros2-listener-1-xxxxxxxxxx-xxxxx   1/1     Running   0          1m    192.168.1.248   tomoyafujita   <none>           <none>
ros2-talker-1-xxxxxxxxxx-xxxxx     1/1     Running   0          1m    192.168.1.79    ubuntu         <none>           <none>

> kubectl delete -f ./yaml/ros2-sample-hostnic.yaml
```

> [!NOTE]
> `kubectl logs` and `kubectl exec` to the pods on the edge node require the `cloudStream` / `edgeStream` tunnel to be enabled.
> see [Enable Kubectl logs/exec/attach/port-forward](https://kubeedge.io/docs/advanced/debug/) for more details.

## Break Down KubeEdge

```bash
### edge node
> keadm reset
> rm -rf /etc/kubeedge /var/lib/kubeedge

### cloud node, this deletes kubeedge namespace and resources
> keadm reset --kube-config=/root/.kube/config
> rm -rf /etc/kubeedge
```
