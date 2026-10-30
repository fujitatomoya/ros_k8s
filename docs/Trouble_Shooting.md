# Trouble Shooting

## Kubernetes

### How to check `kubelet` journal log

Sometimes we need to check the `kubelet` journal log to debug why application pods are failing.

The following is the example that CNI plugin (cilium) fails to initialize.
In this case, application pods are not instantiated yet, that is said we are not able to see any log via `kubectl describe pods xxx`.

```bash
> kubectl get pods -A -w
NAMESPACE     NAME                                                            READY   STATUS              RESTARTS      AGE
kube-system   cilium-54g6h                                                    0/1     Init:Error          2 (7s ago)    46s
kube-system   cilium-bjjf8                                                    1/1     Running             0             17m
kube-system   cilium-envoy-xxxxx                                              1/1     Running             0             17m
kube-system   cilium-operator-xxxxxxxxxx-xxxxx                                1/1     Running             0             17m
kube-system   coredns-xxxxxxxxxx-d9r7g                                        1/1     Running             0             18m
kube-system   coredns-xxxxxxxxxx-zbpgd                                        1/1     Running             0             18m
kube-system   etcd-tomoyafujita-hp-compaq-elite-8300-sff                      1/1     Running             0             18m
kube-system   kube-apiserver-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running             0             18m
kube-system   kube-controller-manager-tomoyafujita-hp-compaq-elite-8300-sff   1/1     Running             0             18m
kube-system   kube-proxy-2stdt                                                1/1     Running             0             18m
kube-system   kube-proxy-tk479                                                1/1     Running             0             44s
kube-system   kube-scheduler-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running             0             18m
kube-system   cilium-54g6h                                                    0/1     Init:CrashLoopBackOff   3 (4s ago)   60s
...<snip>
```

Against this situation, we should check why container runtime `kubelet` fails to start the pods.
Be advised that the following command must be issued on the physical host that `kubelet` is running.

```bash
> journalctl -e -u kubelet
```

### coredns pods does not become `Ready`

- coredns stays in `Pending` state.

```bash
> kubectl get pods -A
NAMESPACE     NAME                                                            READY   STATUS    RESTARTS   AGE
kube-system   coredns-xxxxxxxxxx-jvwvq                                        0/1     Pending   0          42s
kube-system   coredns-xxxxxxxxxx-k99r6                                        0/1     Pending   0          42s
kube-system   etcd-tomoyafujita-hp-compaq-elite-8300-sff                      1/1     Running   0          57s
kube-system   kube-apiserver-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running   0          59s
kube-system   kube-controller-manager-tomoyafujita-hp-compaq-elite-8300-sff   1/1     Running   0          57s
kube-system   kube-proxy-qdx2p                                                1/1     Running   0          43s
kube-system   kube-scheduler-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running   0          57s
```

When we start the cluster, this is expected state for coredns pods.
Since CNI implementation is NOT deployed yet, coredns cannot connect and share the network information.
This is expected result, and it meant to be `Pending` state until CNI is deployed.

see [Deploy CNI Plugin](./Setup_Kubernetes_Cluster.md#deploy-cni-plugin).

- coredns stays in `ContainerCreating`

```bash
> kubectl get pods -A
NAMESPACE     NAME                                                            READY   STATUS              RESTARTS   AGE
kube-system   cilium-hhl6k                                                    1/1     Running             0          4s
kube-system   coredns-xxxxxxxxxx-7zlmq                                        0/1     ContainerCreating   0          50s
kube-system   coredns-xxxxxxxxxx-9rzsf                                        0/1     ContainerCreating   0          50s
kube-system   etcd-tomoyafujita-hp-compaq-elite-8300-sff                      1/1     Running             0          66s
kube-system   kube-apiserver-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running             0          64s
kube-system   kube-controller-manager-tomoyafujita-hp-compaq-elite-8300-sff   1/1     Running             0          64s
kube-system   kube-proxy-r8jt6                                                1/1     Running             0          50s
kube-system   kube-scheduler-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running             0          63s
```

This could be a problem, check the pods via `kubectl describe pods <pod name> -n kube-system`.

```bash
Events:
  Type     Reason                  Age   From               Message
  ----     ------                  ----  ----               -------
  Normal   Scheduled               65s   default-scheduler  Successfully assigned kube-system/coredns-xxxxxxxxxx-7zlmq to tomoyafujita-hp-compaq-elite-8300-sff
  Warning  FailedCreatePodSandBox  5s    kubelet            Failed to create pod sandbox: rpc error: code = Unknown desc = failed to setup network for sandbox "92af7c41c8a6980712f58ab5944752f266f3c74d7d3eb354555e75fc751b14c8": plugin type="cilium-cni" name="cilium" failed (add): unable to connect to Cilium daemon: failed to create cilium agent client after 30.000000 seconds timeout: Get "http:///var/run/cilium/cilium.sock/v1/config": dial unix /var/run/cilium/cilium.sock: connect: no such file or directory
Is the agent running?
  Normal  SandboxChanged  4s  kubelet  Pod sandbox changed, it will be killed and re-created.
```

If you see above error, probably cached CNI information is not successfully removed that leads to unexpected error during loading CNI plugin.
We need to reset the cluster and clear the files under `/etc/cni/net.d`.

```bash
> rm /etc/cni/net.d/*
```

### Nameserver Limits Exceeded

if you see the following error for kubelet service, that is the [DNS resolver know issue](https://kubernetes.io/docs/tasks/administer-cluster/dns-debugging-resolution/#known-issues).

```bash
> systemctl status kubelet
...
Warning: The unit file, source configuration file or drop-ins of kubelet.service changed on disk. Run 'systemctl daemon-reload' t>
○ kubelet.service
     Loaded: masked (Reason: Unit kubelet.service is masked.)
    Drop-In: /etc/systemd/system/kubelet.service.d
             └─10-kubeadm.conf
     Active: inactive (dead) since Fri 2024-02-09 11:26:41 PST; 2 months 30 days ago
   Main PID: 20882 (code=exited, status=0/SUCCESS)
        CPU: 41.031s

Feb 09 11:25:02 edgemaster kubelet[20882]: E0209 11:25:02.547981   20882 dns.go:156] "Nameserver limits exceeded" err="Nameserver>
Feb 09 11:25:35 edgemaster kubelet[20882]: E0209 11:25:35.548358   20882 dns.go:156] "Nameserver limits exceeded" err="Nameserver>
Feb 09 11:25:54 edgemaster kubelet[20882]: E0209 11:25:54.549236   20882 dns.go:156] "Nameserver limits exceeded" err="Nameserver>
Feb 09 11:25:58 edgemaster kubelet[20882]: E0209 11:25:58.547797   20882 dns.go:156] "Nameserver limits exceeded" err="Nameserver>
Feb 09 11:26:04 edgemaster kubelet[20882]: E0209 11:26:04.547791   20882 dns.go:156] "Nameserver limits exceeded" err="Nameserver>
Feb 09 11:26:23 edgemaster kubelet[20882]: E0209 11:26:23.548518   20882 dns.go:156] "Nameserver limits exceeded" err="Nameserver>
Feb 09 11:26:41 edgemaster systemd[1]: Stopping kubelet: The Kubernetes Node Agent...
Feb 09 11:26:41 edgemaster systemd[1]: kubelet.service: Deactivated successfully.
Feb 09 11:26:41 edgemaster systemd[1]: Stopped kubelet: The Kubernetes Node Agent.
Feb 09 11:26:41 edgemaster systemd[1]: kubelet.service: Consumed 41.031s CPU time.
```

we can work-around this add `--resolv-conf` for kubelet.service with less than equal 3 DNS servers.

```bash
### unmask if it is masked already
> systemctl unmask kubelet
Removed /etc/systemd/system/kubelet.service.

### create static resolver configuration for stability
> cat /etc/resolv-static.conf
nameserver xx.xx.xx.xx
nameserver yy:yy:yy:yy:yy:yy:
nameserver zz.zz.zz.zz

### configure kubelet to use static resolver file
> cat /etc/default/kubelet
KUBELET_EXTRA_ARGS=--resolv-conf=/etc/resolv-static.conf

> systemctl daemon-reload
> systemctl restart kubelet
```

### Container runtime network not ready

if you see the following error when initializing kubernetes nodes, that could be related to [Container runtime network not ready: cni config uninitialized](https://stackoverflow.com/questions/49112336/container-runtime-network-not-ready-cni-config-uninitialized).

```bash
Container runtime network not ready: NetworkReady=false reason:NetworkPluginNotReady message:docker: network plugin is not ready: cni config uninitialized
```

please restart the `containerd.service` and retry.

```bash
systemctl restart containerd.service
```

### kubelet fails to start with `container runtime is not running` or `CRI v1 runtime API is not implemented`

`containerd.io` package from Docker apt repository disables the CRI plugin in default (`disabled_plugins = ["cri"]` in `/etc/containerd/config.toml`).
kubelet cannot register the node if the container runtime does not support CRI v1.
Regenerate the default containerd configuration with `SystemdCgroup = true`, see [Docker / Containerd](./Install_Kubernetes_Packages.md#docker--containerd).

```bash
> containerd config default > /etc/containerd/config.toml
> sed -i 's/SystemdCgroup \= false/SystemdCgroup \= true/g' /etc/containerd/config.toml
> systemctl restart containerd
```

### kubelet fails to start with `swap is enabled`

kubelet fails to start if swap memory is enabled on the node in default.
Disable swap on the node and also in `/etc/fstab` so that it stays disabled after reboot, see [System Configuration](./Install_Kubernetes_Packages.md#system-configuration).

```bash
> swapoff -a
> sed -i '/\sswap\s/s/^\(.*\)$/#\1/g' /etc/fstab
```

## Container Runtime

### Inspect containers on the node with `crictl`

Usually application containers can be inspected with `kubectl logs` and `kubectl exec` via Kubernetes API server.
But sometimes these commands cannot be used, because the containers are not instantiated successfully or the API server is not reachable from the node.
This is likely to happen when deploying system containers such as CNI plugin, CDI (Container Device Interface) and Device Plugin, those are the infrastructure for the application pods.
Besides that, Kubernetes removed `dockershim` since v1.24, so that `docker ps` does not show the containers managed by `kubelet` via containerd any more.

In those cases, we can inspect the pods and containers on the node directly via [CRI (Container Runtime Interface)](https://kubernetes.io/docs/concepts/architecture/cri/) with [`crictl`](https://kubernetes.io/docs/tasks/debug/debug-cluster/crictl/) command.
`crictl` is provided by [cri-tools](https://github.com/kubernetes-sigs/cri-tools), which is installed as the dependency of `kubeadm` package by [Install Kubernetes Packages](./Install_Kubernetes_Packages.md).

```bash
### crictl is installed with kubeadm (cri-tools dependency)
> apt-cache depends kubeadm | grep cri-tools
  Depends: cri-tools
> crictl --version
crictl version v1.37.x
```

> [!NOTE]
> kind node images also have `crictl` and `ctr` installed, `docker exec -it kind-worker bash` to log in the kind node and issue the following commands.

- Configure CRI endpoint

  `crictl` requires the CRI runtime endpoint, otherwise it tries the deprecated default endpoints with warning messages as following.
  (`crictl` needs root permission to access the containerd socket.)

```bash
> crictl pods
WARN[0000] runtime connect using default endpoints: [unix:///var/run/dockershim.sock unix:///run/containerd/containerd.sock unix:///run/crio/crio.sock unix:///var/run/cri-dockerd.sock]. As the default settings are now deprecated, you should set the endpoint instead.
```

  Create `/etc/crictl.yaml` to connect the containerd CRI socket, this is the same socket specified with `--cri-socket` for `kubeadm init` / `kubeadm join`.

```bash
> cat <<EOF > /etc/crictl.yaml
runtime-endpoint: unix:///run/containerd/containerd.sock
image-endpoint: unix:///run/containerd/containerd.sock
timeout: 10
debug: false
EOF

### or use crictl config command
> crictl config --set runtime-endpoint=unix:///run/containerd/containerd.sock --set image-endpoint=unix:///run/containerd/containerd.sock

### check the connection to the runtime
> crictl version
Version:  0.1.0
RuntimeName:  containerd
RuntimeVersion:  v2.3.4
RuntimeApiVersion:  v1
```

- List pods and containers on the node

  `crictl` shows the pod sandboxes and the containers running on **this node only**, including the containers that `kubectl` cannot show such as `Exited` init containers.
  The following example is from the kind worker node with Cilium deployed.

```bash
### Pod sandboxes on the node
> crictl pods
POD ID              CREATED             STATE               NAME                                      NAMESPACE            ATTEMPT             RUNTIME
c6f4c4770432a       2 minutes ago       Ready               coredns-559f6c778d-w69hh                  kube-system          0                   (default)
89d279f9fb135       2 minutes ago       Ready               cilium-bgvwb                              kube-system          0                   (default)
5c78aaacab860       2 minutes ago       Ready               cilium-operator-649867d84-skwmf           kube-system          0                   (default)
203a08f9b8c3e       2 minutes ago       Ready               cilium-envoy-8ndxq                        kube-system          0                   (default)
0e38dd7e44529       2 minutes ago       Ready               kube-proxy-mj7hq                          kube-system          0                   (default)
...<snip>

### All containers including exited ones (init containers)
> crictl ps -a
CONTAINER           IMAGE               CREATED             STATE               NAME                      ATTEMPT             POD ID              POD                                       NAMESPACE
9b55856fcfa4c       520212b8b0fcd       2 minutes ago       Running             coredns                   0                   c6f4c4770432a       coredns-559f6c778d-w69hh                  kube-system
e43ad675be1f5       e8a7b89d30c07       2 minutes ago       Running             cilium-agent              0                   89d279f9fb135       cilium-bgvwb                              kube-system
07d8b18464731       18f3aea3c0c65       2 minutes ago       Running             cilium-envoy              0                   203a08f9b8c3e       cilium-envoy-8ndxq                        kube-system
c761a9b631817       e8a7b89d30c07       2 minutes ago       Exited              install-cni-binaries      0                   89d279f9fb135       cilium-bgvwb                              kube-system
7a9df2f19fcec       e8a7b89d30c07       2 minutes ago       Exited              clean-cilium-state        0                   89d279f9fb135       cilium-bgvwb                              kube-system
a34792f616f23       e8a7b89d30c07       2 minutes ago       Exited              mount-bpf-fs              0                   89d279f9fb135       cilium-bgvwb                              kube-system
...<snip>

### Filter by pod name or container name
> crictl ps -a --name cilium-agent
> crictl pods --name cilium
```

- Logs, exec and inspect

```bash
### Container logs (works even if API server is not reachable), -f to follow
> crictl logs e43ad675be1f5 | tail -3
### Logs of the exited init container to see why it failed
> crictl logs c761a9b631817

### Execute command in the running container
> crictl exec -it e43ad675be1f5 cilium-dbg status --brief
OK

### Inspect the container and pod sandbox (JSON), e.g. mounts, environment variables and runtime spec
> crictl inspect e43ad675be1f5 | less
> crictl inspectp 89d279f9fb135 | less

### Resource usage
> crictl stats
```

- Images on the node

```bash
> crictl images
IMAGE                                           TAG                  IMAGE ID            SIZE
registry.k8s.io/coredns/coredns                 v1.14.6              520212b8b0fcd       23.8MB
registry.k8s.io/kube-proxy                      v1.37.0              d6a28daf3e6b0       82.8MB
registry.k8s.io/pause                           3.10                 873ed75102791       320kB
quay.io/cilium/cilium                           <none>               e8a7b89d30c07       258MB
docker.io/tomoyafujita/ros                      rolling              31533583b2f5e       7.24GB
...<snip>

### Pull the image to the node beforehand
> crictl pull docker.io/tomoyafujita/ros:rolling
### Remove unused images to free the disk space
> crictl rmi --prune
```

see [Debugging Kubernetes nodes with crictl](https://kubernetes.io/docs/tasks/debug/debug-cluster/crictl/) for more details.

### Inspect containerd directly with `ctr`

`ctr` is the low-level command line client of containerd, it is installed with `containerd.io` package.
`ctr` does not know anything about Kubernetes pods, it shows the containerd objects such as containers, tasks and images in the containerd namespace.
Kubernetes (CRI plugin) uses `k8s.io` namespace, so that `-n k8s.io` must be specified to see the containers managed by `kubelet`.

```bash
> ctr --version
ctr github.com/containerd/containerd/v2 v2.3.4

### containerd namespaces, kubernetes uses k8s.io
> ctr namespaces ls
NAME   LABELS
k8s.io

### Containers and running tasks (processes) in k8s.io namespace
> ctr -n k8s.io containers ls
> ctr -n k8s.io tasks ls

### Images in k8s.io namespace, the same with crictl images
> ctr -n k8s.io images ls -q
```

`ctr` is useful to check the containerd internal state (e.g. snapshots, content store and leases) when `crictl` does not work, but `crictl` is recommended for the daily debugging since it understands pods and containers in Kubernetes.
see [ctr reference](https://labs.iximiuz.com/courses/containerd-cli/ctr/image-management) for more details.

### Debug the node without login

If `ssh` to the node is not available but `kubectl` works, [`kubectl debug node`](https://kubernetes.io/docs/tasks/debug/debug-cluster/kubectl-node-debug/) creates the privileged debugging pod on the node with the host filesystem mounted on `/host`.

```bash
> kubectl debug node/kind-worker -it --image=ubuntu:24.04 --profile=sysadmin
root@kind-worker:/# chroot /host
root@kind-worker:/# crictl ps
root@kind-worker:/# journalctl -u kubelet -e
```

## Cilium

### ROS 2 nodes on different cluster nodes cannot discover each other

ROS 2 / DDS uses multicast for the participant discovery in default, Cilium multicast is disabled in default and multicast group needs to be configured.
see [Enable Multicast](./Setup_Kubernetes_Cluster.md#enable-multicast).

```bash
> cilium config view | grep multicast
multicast-enabled                                 true
> cilium multicast list subscriber --all
```

If multicast is not available in your environment, use [ROS 2 Fast-DDS Discovery Server](./ROS2_Deployment_Intermediate.md#ros-2-fast-dds-discovery-server) or `ROS_STATIC_PEERS` to use unicast discovery.

### ROS 2 nodes with `ROS_DOMAIN_ID` cannot discover each other

DDS well-known multicast group address `239.255.0.1` is shared by all domain IDs, the domain ID only changes the UDP port number (`7400 + 250 * ROS_DOMAIN_ID` for the discovery).
Cilium multicast forwards the packets based on the group address regardless of the port, so that no additional multicast group is required for `ROS_DOMAIN_ID`.
Check that the application actually sends the discovery packets to `239.255.0.1` with [Hubble](./Setup_Kubernetes_Cluster.md#enable-hubble-observability) in the `cilium-agent` container on the node, and add the multicast group if the application uses the other address (e.g. user-defined multicast locator in the DDS XML profile).
see [Multicast Group Address, UDP Port and `ROS_DOMAIN_ID`](./Setup_Kubernetes_Cluster.md#enable-multicast) for more details.

```bash
> CILIUM_POD=$(kubectl -n kube-system get pods -l k8s-app=cilium --field-selector spec.nodeName=<node-name> -o name)
> kubectl -n kube-system exec $CILIUM_POD -c cilium-agent -- hubble observe --protocol UDP --to-ip 239.255.0.1 --last 20
```

## KIND

### CNI plugins for KIND

KIND node docker image contains the standard CNI plugin binaries under `/opt/cni/bin` for the default network interface which is called `kindnet`.
If using kind `v0.19.0` or earlier, or if your CNI plugin requires other CNI plugin binaries, it would be required to install CNI plugins in the host system and bind them into kind nodes.
The following operation to build the all CNI plugins in the host system to bind the docker container which is actually KIND node instance.

```bash
git clone https://github.com/containernetworking/plugins.git
cd plugins
./build_linux.sh
export OPT_CNI_BIN_PATH=$(realpath -s bin)
```

and add `extraMounts` to each node in [kind-multiple-node.yaml](./../yaml/kind-multiple-node.yaml).

```yaml
nodes:
- role: control-plane
  extraMounts:
  - hostPath: /path/to/plugins/bin  # OPT_CNI_BIN_PATH
    containerPath: /opt/cni/bin
```

### Too many open files

kind cluster with multiple nodes may fail to start the pods with `too many open files` error, since the host system inotify limits are too low.
see [Pod errors due to "too many open files"](https://kind.sigs.k8s.io/docs/user/known-issues/#pod-errors-due-to-too-many-open-files).

```bash
> cat <<EOF > /etc/sysctl.d/kind.conf
fs.inotify.max_user_watches = 524288
fs.inotify.max_user_instances = 512
EOF
> sysctl --system
```
