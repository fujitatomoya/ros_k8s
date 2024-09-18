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

**<span style="color: red;">TODO: CNI needs to be uninstalled</span>**

This is only required to bring the Kubenretes API-server running, because we are going to deploy cloudcore to the same physical node with Kubernetes API-server.
Instead of having CNI deployed to bring the Kubernetes API-server up and running, we are not able to deploy cloudcore to the node since we cannot deploy the containers to any `NotReady` nodes.

## Setup KubeEdge

see more details for https://kubeedge.io/docs/setup/install-with-keadm/

### Install keadm

`keadm` needs to be installed in both cloud and edge nodes.

```bash
> wget https://github.com/kubeedge/kubeedge/releases/download/v1.16.3/keadm-v1.16.3-linux-amd64.tar.gz
> tar -zxvf keadm-v1.16.3-linux-amd64.tar.gz
> cp keadm-v1.16.3-linux-amd64/keadm//keadm /usr/local/bin/keadm
> keadm version
version: version.Info{Major:"1", Minor:"16", GitVersion:"v1.16.3", GitCommit:"4f1da43e6807c127e549e5d4859ed1b66c6f5806", GitTreeState:"clean", BuildDate:"2024-07-12T03:02:51Z", GoVersion:"go1.20.10", Compiler:"gc", Platform:"linux/amd64"}
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

> kubectl get nodes -o json | jq '.items[].spec.taints'
null
```

- start KubeEdge `cloudcore`.

```bash
> keadm init --advertise-address=192.168.1.248 --kube-config=/root/.kube/config --kubeedge-version=v1.16.3
Kubernetes version verification passed, KubeEdge installation will start...
CLOUDCORE started
=========CHART DETAILS=======
NAME: cloudcore
LAST DEPLOYED: Wed Sep 18 14:56:13 2024
NAMESPACE: kubeedge
STATUS: deployed
REVISION: 1

> kubectl get all -n kubeedge
NAME                             READY   STATUS    RESTARTS   AGE
pod/cloudcore-77b5dfdd57-btmlp   1/1     Running   0          24s

NAME                        READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/cloudcore   1/1     1            1           24s

NAME                                   DESIRED   CURRENT   READY   AGE
replicaset.apps/cloudcore-77b5dfdd57   1         1         1       24s
```

### Edge Core

- get security token from cloudcore.

```bash
> keadm gettoken
40d9bb8bb2c3818728da3d46f1a78b58f4b9fba8665cc392ded4698d1eb5cab1.eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJleHAiOjE2OTQ4NDA5ODN9.niGZHHdR7s89K4-919fCNKEVTyudb8DtTmE9p5PFzKg
```

- start KubeEdge `edgecore`

```bash
> keadm join --cloudcore-ipport=192.168.1.248:10000 --token=40d9bb8bb2c3818728da3d46f1a78b58f4b9fba8665cc392ded4698d1eb5cab1.eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJleHAiOjE2OTQ4NDA5ODN9.niGZHHdR7s89K4-919fCNKEVTyudb8DtTmE9p5PFzKg --kubeedge-version=v1.16.3 --remote-runtime-endpoint=unix:///run/containerd/containerd.sock --cgroupdriver systemd
I0915 06:28:19.544584   10424 command.go:845] 1. Check KubeEdge edgecore process status
I0915 06:28:19.578838   10424 command.go:845] 2. Check if the management directory is clean
I0915 06:28:19.579238   10424 join.go:107] 3. Create the necessary directories
I0915 06:28:19.580874   10424 join.go:184] 4. Pull Images
Pulling kubeedge/installation-package:v1.12.1 ...
Pulling kubeedge/pause:3.6 ...
Pulling eclipse-mosquitto:1.6.15 ...
I0915 06:28:19.590357   10424 join.go:184] 5. Copy resources from the image to the management directory
I0915 06:28:27.255374   10424 join.go:184] 6. Start the default mqtt service
I0915 06:28:27.256005   10424 join.go:107] 7. Generate systemd service file
I0915 06:28:27.256873   10424 join.go:107] 8. Generate EdgeCore default configuration
I0915 06:28:27.256988   10424 join.go:270] The configuration does not exist or the parsing fails, and the default configuration is generated
W0915 06:28:27.263577   10424 validation.go:71] NodeIP is empty , use default ip which can connect to cloud.
I0915 06:28:27.270836   10424 join.go:107] 9. Run EdgeCore daemon
I0915 06:28:34.350843   10424 join.go:435]
I0915 06:28:34.350933   10424 join.go:436] KubeEdge edgecore is running, For logs visit: journalctl -u edgecore.service -xe

> systemctl status edgecore
● edgecore.service
     Loaded: loaded (/etc/systemd/system/edgecore.service; enabled; vendor preset: enabled)
     Active: active (running) since Fri 2023-09-15 06:28:34 UTC; 1min 9s ago
   Main PID: 10603 (edgecore)
      Tasks: 16 (limit: 9190)
     Memory: 31.4M
        CPU: 11.565s
     CGroup: /system.slice/edgecore.service
             └─10603 /usr/local/bin/edgecore
...<snip>
```

- check cluster nodes.

```bash
> kubectl get nodes -o wide
NAME                                    STATUS   ROLES                  AGE    VERSION                    INTERNAL-IP     EXTERNAL-IP   OS-IMAGE             KERNEL-VERSION      CONTAINER-RUNTIME
tomoyafujita-hp-compaq-elite-8300-sff   Ready    control-plane,master   104m   v1.23.17                   192.168.1.248   <none>        Ubuntu 20.04.6 LTS   5.15.0-83-generic   docker://24.0.5
ubuntu                                  Ready    agent,edge             97s    v1.22.6-kubeedge-v1.12.1   192.168.1.238   <none>        Ubuntu 22.04.3 LTS   5.15.0-1034-raspi   docker://24.0.5
```

## KubeEdge Test Deployment

```bash
> kubectl apply -f ros2-sample-hostnic.yaml
deployment.apps/ros2-talker-1 created
deployment.apps/ros2-listener-1 created

> kubectl get pods -o wide
NAME                               READY   STATUS    RESTARTS   AGE   IP              NODE                                    NOMINATED NODE   READINESS GATES
ros2-listener-1-64c4c996b4-scm9k   1/1     Running   0          14m   192.168.1.248   tomoyafujita-hp-compaq-elite-8300-sff   <none>           <none>
ros2-talker-1-bdd899d8d-drt2f      1/1     Running   0          14m   192.168.1.238   ubuntu                                  <none>           <none>
```

## Break Down KubeEdge

```bash
> \rm -rf /etc/kubeedge/*
> kubeadm deprecated reset --force
[reset] Reading configuration from the cluster...
```
