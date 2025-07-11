# Trouble Shooting

## Kubernetes

### How to check `kubelet` journal log

Sometimes we need to check the `kubelet` journal log to debug why application pods are failing.

The following is the example that weavenet fails to initialize.
In this case, application pods are not instantiated yet, that is said we are not able to see any log via `kubectl describe pods xxx`.

```bash
> kubectl get pods -A -w
NAMESPACE              NAME                                                            READY   STATUS              RESTARTS      AGE
kube-system            coredns-565d847f94-d9r7g                                        1/1     Running             0             18m
kube-system            coredns-565d847f94-zbpgd                                        1/1     Running             0             18m
kube-system            etcd-tomoyafujita-hp-compaq-elite-8300-sff                      1/1     Running             34            18m
kube-system            kube-apiserver-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running             0             18m
kube-system            kube-controller-manager-tomoyafujita-hp-compaq-elite-8300-sff   1/1     Running             0             18m
kube-system            kube-proxy-2stdt                                                1/1     Running             0             18m
kube-system            kube-proxy-tk479                                                1/1     Running             0             44s
kube-system            kube-scheduler-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running             0             18m
kube-system            weave-net-54g6h                                                 1/2     Error               0             44s
kube-system            weave-net-bjjf8                                                 2/2     Running             1 (17m ago)   17m
kubernetes-dashboard   dashboard-metrics-scraper-64bcc67c9c-hgjsm                      0/1     ContainerCreating   0             14m
kubernetes-dashboard   kubernetes-dashboard-5c8bd6b59-zjjph                            0/1     ContainerCreating   0             14m
kube-system            weave-net-54g6h                                                 0/2     Error               1 (14s ago)   44s
kube-system            weave-net-54g6h                                                 1/2     Error               2 (7s ago)    46s
kube-system            weave-net-54g6h                                                 0/2     CrashLoopBackOff    2 (4s ago)    47s
kube-system            weave-net-54g6h                                                 0/2     CrashLoopBackOff    2 (3s ago)    48s
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
kube-system   coredns-565d847f94-jvwvq                                        0/1     Pending   0          42s
kube-system   coredns-565d847f94-k99r6                                        0/1     Pending   0          42s
kube-system   etcd-tomoyafujita-hp-compaq-elite-8300-sff                      1/1     Running   9          57s
kube-system   kube-apiserver-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running   6          59s
kube-system   kube-controller-manager-tomoyafujita-hp-compaq-elite-8300-sff   1/1     Running   6          57s
kube-system   kube-proxy-qdx2p                                                1/1     Running   0          43s
kube-system   kube-scheduler-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running   6          57s
```

When we start the cluster, this is expected state for coredns pods.
Since CNI implementation is NOT deployed yet, coredns cannot connect and share the network information.
This is expected result, and it meant to be `Pending` state until CNI is deployed.

see [Deploy CNI Plugin](./Setup_Kubernetes_Cluster.md#deploy-cni-plugin).

- coredns stays in `ContainerCreating`

```bash
> kubectl get pods -A
NAMESPACE     NAME                                                            READY   STATUS              RESTARTS   AGE
kube-system   coredns-565d847f94-7zlmq                                        0/1     ContainerCreating   0          50s
kube-system   coredns-565d847f94-9rzsf                                        0/1     ContainerCreating   0          50s
kube-system   etcd-tomoyafujita-hp-compaq-elite-8300-sff                      1/1     Running             8          66s
kube-system   kube-apiserver-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running             5          64s
kube-system   kube-controller-manager-tomoyafujita-hp-compaq-elite-8300-sff   1/1     Running             5          64s
kube-system   kube-proxy-r8jt6                                                1/1     Running             0          50s
kube-system   kube-scheduler-tomoyafujita-hp-compaq-elite-8300-sff            1/1     Running             5          63s
kube-system   weave-net-hhl6k                                                 1/2     Running             0          4s
```

This could be a problem, check the pods via `kubectl describe pods <pod name> -n kube-system`.

```bash
Events:
  Type     Reason                  Age   From               Message
  ----     ------                  ----  ----               -------
  Normal   Scheduled               65s   default-scheduler  Successfully assigned kube-system/coredns-565d847f94-7zlmq to tomoyafujita-hp-compaq-elite-8300-sff
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

### CNI plugins for KIND

if using kind `v0.19.0` or earlier, it would be required to install CNI plugins in the host system and bind them into kind images.

KIND node docker image does not contain some CNI plugin binaries under `/opt/cni/bin`, this is because KIND provides default network interface which is called `kindnet`.
User can disable default `kindnet` but using specific CNI plugin such as flannel requires other CNI plugins under the `/opt/cni/bin`.
The following operation to build the all CNI plugins in the host system to bind the docker container which is actually KIND node instance.

```bash
git clone https://github.com/containernetworking/plugins.git
cd plugins
./build_linux.sh
export OPT_CNI_BIN_PATH=$(realpath -s bin)
cd <ros_k8s>/yaml
sed 's/OPT_CNI_BIN_PATH/${OPT_CNI_BIN_PATH}/' kind-multiple-node.yaml.template | envsubst > kind-multiple-node.yaml
```
