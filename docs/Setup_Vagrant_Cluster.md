# Setup Vagrant Virtual Machine Cluster

In this section, it describes how to set up the Kubernetes cluster on the virtual machines with [Vagrant](https://developer.hashicorp.com/vagrant) and [VirtualBox](https://www.virtualbox.org/) on the single physical host system.

[Kubernetes IN Docker](./Setup_Virtualized_Cluster.md) is the quickest way to try the deployment samples, but the kind nodes are the containers that share the host kernel and the pre-built node image, so that the physical host procedures such as [Install Kubernetes Packages](./Install_Kubernetes_Packages.md), [Setup Kubernetes Cluster](./Setup_Kubernetes_Cluster.md) with `kubeadm` and [Setup KubeEdge](./Setup_KubeEdge.md) cannot be verified with kind.
The Vagrant virtual machines are the full Ubuntu 24.04 systems, the same procedures with the physical host systems can be verified without any physical devices.

## Reference

- [Vagrant Documentation](https://developer.hashicorp.com/vagrant/docs)
- [Vagrant VirtualBox Provider](https://developer.hashicorp.com/vagrant/docs/providers/virtualbox)
- [bento/ubuntu-24.04](https://portal.cloud.hashicorp.com/vagrant/discover/bento/ubuntu-24.04) Vagrant box

## Verified Environment

| Component | Version |
| --- | --- |
| Host | Ubuntu 22.04 LTS, 16 CPUs, 32GB memory |
| Vagrant | 2.4.9 |
| VirtualBox | 7.2.22 |
| Vagrant box | `bento/ubuntu-24.04` v202510.26.0 (Ubuntu 24.04.3, kernel 6.8) |

## Virtual Machines

[Vagrantfile](./../vagrant/Vagrantfile) creates the following virtual machines on the VirtualBox host-only network `192.168.56.0/24`, the repository root is mounted to `/vagrant` in the virtual machines.

| Name | IP Address | CPUs | Memory | Role |
| --- | --- | --- | --- | --- |
| `vm-control-plane` | 192.168.56.10 | 2 | 4096MB | Kubernetes control-plane (KubeEdge cloudcore) |
| `vm-worker` | 192.168.56.11 | 2 | 3072MB | Kubernetes worker node |
| `vm-worker2` | 192.168.56.12 | 2 | 3072MB | Kubernetes worker node (or KubeEdge edge node) |

The provisioner executes [install_k8s_packages.sh](./../scripts/install_k8s_packages.sh) with `-k -y` (containerd/docker, kubelet/kubeadm/kubectl, kind and keadm) in each virtual machine, and configures the following that are specific to the virtual machines.

- `/etc/hosts` entries for all virtual machines.
- `KUBELET_EXTRA_ARGS=--node-ip=<host-only IP address>` in `/etc/default/kubelet`, since the VirtualBox NAT interface (`eth0`, `10.0.2.15`) has the same IP address on every virtual machine. Without this, all nodes advertise `10.0.2.15` and the pods on the other nodes cannot be reached.
- Static route for the Kubernetes service CIDR (`10.96.0.0/12` in default) to the host-only interface (`/etc/netplan/60-ros-k8s-service-cidr.yaml`).
  The default route of the virtual machine is the NAT interface, so that the connection from the host network namespace to the ClusterIP (e.g. `cilium-agent` init container `config` to `10.96.0.1:443`) selects `10.0.2.15` as the source address, kube-proxy DNAT sends it to the control-plane via the host-only interface, and the reply never comes back.
  Without this route, `cilium-agent` on the worker node can stay in `Init:Error` with `Unable to contact k8s api-server ... dial tcp 10.96.0.1:443: i/o timeout` after the restart (e.g. `cilium config set multicast-enabled true`), and the node is tainted with `node.cilium.io/agent-not-ready:NoSchedule`.
- `/etc/crictl.yaml` to connect `crictl` to the containerd CRI socket, see [Trouble Shooting](./Trouble_Shooting.md#inspect-containers-on-the-node-with-crictl).

The number of the worker nodes, CPUs, memory and the Kubernetes version can be changed via the environment variables, see the comments in [Vagrantfile](./../vagrant/Vagrantfile).

> [!WARNING]
> Be careful about the resource of the host system, the virtual machines above require 6 CPUs and 10GB memory in total.
> ROS 2 container images are 4GB - 5GB for each, and the images are pulled to every node.
> Do not run the kind cluster and the virtual machines at the same time on the small host system.

## Start Virtual Machines

```bash
> cd vagrant
> vagrant up
Bringing machine 'vm-control-plane' up with 'virtualbox' provider...
Bringing machine 'vm-worker' up with 'virtualbox' provider...
Bringing machine 'vm-worker2' up with 'virtualbox' provider...
==> vm-control-plane: Importing base box 'bento/ubuntu-24.04'...
...<snip>
    vm-control-plane: Installation completed, enjoy your cluster !!!
    vm-control-plane: provisioned vm-control-plane 192.168.56.10
...<snip>

> vagrant status
Current machine states:

vm-control-plane          running (virtualbox)
vm-worker                 running (virtualbox)
vm-worker2                running (virtualbox)

### Log in to the virtual machine, vagrant user has sudo permission without password
> vagrant ssh vm-control-plane
vagrant@vm-control-plane:~$ kubeadm version
vagrant@vm-control-plane:~$ ls /vagrant
```

To use the other Kubernetes version (e.g. `v1.32` for [KubeEdge](./Setup_KubeEdge.md#kubernetes-compatibility)), pass the version to the provisioner.
The provisioner can be executed again with `vagrant provision` to re-install the packages with the different version.

```bash
> KUBERNETES_VERSION=v1.32 vagrant up
### or re-provision the running virtual machines
> KUBERNETES_VERSION=v1.32 vagrant provision
```

## Setup Kubernetes Cluster

The procedure is the same with [Setup Kubernetes Cluster](./Setup_Kubernetes_Cluster.md), except that `kubeadm init` needs `--apiserver-advertise-address` with the host-only IP address of the control-plane since the virtual machine has the NAT interface as the default route.

```bash
> vagrant ssh vm-control-plane
vagrant@vm-control-plane:~$ sudo su -
root@vm-control-plane:~# kubeadm init --apiserver-advertise-address=192.168.56.10 --pod-network-cidr=10.244.0.0/16 --cri-socket unix:///run/containerd/containerd.sock
...<snip>
Your Kubernetes control-plane has initialized successfully!
...<snip>
kubeadm join 192.168.56.10:6443 --token <YOUR_TOKEN> --discovery-token-ca-cert-hash <YOUR_HASH>

root@vm-control-plane:~# mkdir -p $HOME/.kube && cp -i /etc/kubernetes/admin.conf $HOME/.kube/config
root@vm-control-plane:~# kubectl get nodes -o wide
NAME               STATUS     ROLES           AGE   VERSION   INTERNAL-IP     EXTERNAL-IP   OS-IMAGE           KERNEL-VERSION     CONTAINER-RUNTIME
vm-control-plane   NotReady   control-plane   30s   v1.37.2   192.168.56.10   <none>        Ubuntu 24.04.3 LTS   6.8.0-86-generic   containerd://2.4.2
```

Join the worker nodes with the command printed by `kubeadm init`.

```bash
> vagrant ssh vm-worker
vagrant@vm-worker:~$ sudo kubeadm join 192.168.56.10:6443 --token <YOUR_TOKEN> --discovery-token-ca-cert-hash <YOUR_HASH> --cri-socket unix:///run/containerd/containerd.sock
```

Then deploy Cilium with multicast enabled on the control-plane, see [Deploy CNI plugin](./Setup_Kubernetes_Cluster.md#deploy-cni-plugin) and [Enable Multicast](./Setup_Kubernetes_Cluster.md#enable-multicast).

```bash
root@vm-control-plane:~# /vagrant/scripts/install_cilium_cli.sh
root@vm-control-plane:~# cilium install --version 1.20.2
root@vm-control-plane:~# cilium status --wait
root@vm-control-plane:~# cilium config set multicast-enabled true
root@vm-control-plane:~# cilium multicast add --group-ip 239.255.0.1
root@vm-control-plane:~# kubectl get nodes -o wide
NAME               STATUS   ROLES           AGE     VERSION   INTERNAL-IP     EXTERNAL-IP   OS-IMAGE             KERNEL-VERSION     CONTAINER-RUNTIME
vm-control-plane   Ready    control-plane   5m      v1.37.2   192.168.56.10   <none>        Ubuntu 24.04.3 LTS   6.8.0-86-generic   containerd://2.4.2
vm-worker          Ready    <none>          4m      v1.37.2   192.168.56.11   <none>        Ubuntu 24.04.3 LTS   6.8.0-86-generic   containerd://2.4.2
vm-worker2         Ready    <none>          4m      v1.37.2   192.168.56.12   <none>        Ubuntu 24.04.3 LTS   6.8.0-86-generic   containerd://2.4.2
```

Now the deployment samples in this repository can be applied from `/vagrant/yaml` on the control-plane in the same way with the physical cluster.

If [Flannel](./Setup_Kubernetes_Cluster.md#flannel) is used instead of Cilium, `--iface=eth1` must be added to `kube-flannel` container arguments since the default route of the virtual machine is the NAT interface `eth0`.

```bash
root@vm-control-plane:~# kubectl apply -f https://github.com/flannel-io/flannel/releases/latest/download/kube-flannel.yml
root@vm-control-plane:~# kubectl -n kube-flannel patch ds kube-flannel-ds --type json -p '[{"op":"add","path":"/spec/template/spec/containers/0/args/-","value":"--iface=eth1"}]'
```

```bash
root@vm-control-plane:~# kubectl apply -f /vagrant/yaml/ros2-sample.yaml
```

## Setup KubeEdge

`vm-worker2` can be used as the KubeEdge edge node instead of the Kubernetes worker node, do not `kubeadm join` (or `kubeadm reset` if joined already) on `vm-worker2` and follow [Setup KubeEdge](./Setup_KubeEdge.md).
`keadm` is installed by the provisioner in all virtual machines.
The following is the summary verified with Kubernetes `v1.32` (`KUBERNETES_VERSION=v1.32 vagrant provision`) and KubeEdge `v1.23.1`.

```bash
### vm-control-plane: Kubernetes control-plane with Cilium, then cloudcore
root@vm-control-plane:~# kubectl taint nodes --all node-role.kubernetes.io/control-plane-
root@vm-control-plane:~# keadm init --advertise-address=192.168.56.10 --kube-config=/root/.kube/config --kubeedge-version=v1.23.1
root@vm-control-plane:~# keadm gettoken

### vm-worker2: edge node
root@vm-worker2:~# systemctl disable --now kubelet
root@vm-worker2:~# keadm join --cloudcore-ipport=192.168.56.10:10000 --token=<TOKEN> --kubeedge-version=v1.23.1 --remote-runtime-endpoint=unix:///run/containerd/containerd.sock --cgroupdriver=systemd

### vm-control-plane
root@vm-control-plane:~# kubectl get nodes -o wide
NAME               STATUS   ROLES           AGE   VERSION                     INTERNAL-IP     EXTERNAL-IP   OS-IMAGE             KERNEL-VERSION     CONTAINER-RUNTIME
vm-control-plane   Ready    control-plane   17m   v1.32.13                    192.168.56.10   <none>        Ubuntu 24.04.3 LTS   6.8.0-86-generic   containerd://2.4.2
vm-worker          Ready    <none>          15m   v1.32.13                    192.168.56.11   <none>        Ubuntu 24.04.3 LTS   6.8.0-86-generic   containerd://2.4.2
vm-worker2         Ready    agent,edge      15m   v1.32.10-kubeedge-v1.23.1   192.168.56.12   <none>        Ubuntu 24.04.3 LTS   6.8.0-86-generic   containerd://2.4.2
```

The edge node requires the CNI configuration to become `Ready`, the cloud DaemonSets must be kept away from the edge node, and the Cilium taint must be removed from the edge node, see [Edge Node Readiness](./Setup_KubeEdge.md#edge-node-readiness).
Then [KubeEdge Test Deployment](./Setup_KubeEdge.md#kubeedge-test-deployment) with `vm-worker2` (talker) and `vm-control-plane` (listener) node names works on the virtual machines.

## Break Down

```bash
### stop the virtual machines, they can be started again with vagrant up
> vagrant halt

### delete the virtual machines
> vagrant destroy -f
```
