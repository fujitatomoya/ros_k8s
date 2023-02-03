# Trouble Shooting

## Kubernetes

### coredns pods does not become `Ready`

- coredns stays in `Pending` state.

```bash
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubectl get pods -A
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
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubectl get pods -A
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
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# rm /etc/cni/net.d/*
```
