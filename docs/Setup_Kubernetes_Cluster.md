# Setup Kubernetes Cluster

In this section, it describes how to set up Kubernetes Cluster using physical host systems.

## Reference

- [Kubernetes Official Documentation](https://kubernetes.io/docs/setup/)
- [Integrating Kubernetes WeaveNet](https://www.weave.works/docs/net/latest/kubernetes/kube-addon/)
- [Kubernetes Cilium Installation Guide](https://scanfcilium.readthedocs.io/en/latest/kubernetes/install.html)
- [Kubernetes CNI Explained](https://www.tigera.io/learn/guides/kubernetes-networking/kubernetes-cni/)

## Container Network Interface (CNI)

Before establish Kubernetes Cluster, there is important component to understand, which is Container Network Interface (CNI).
This is really important for ROS with Kubernetes use case since CNI is the network interface to ROS application container uses.
If inappropriate CNI plugin is bound to ROS application container, sometimes it fails to communicate via ROS network especially ROS 2 / DDS that uses **multicast** to endpoint discovery.

![Kubernetes CNI flannel example](./../images/K8s-CNI-diagram01.png)

The above diagram shows that one of the CNI implementation called `flannel` to provide overlay network to application containers.
Using CNI plugin underneath, container runtime adds the interface to the container namespace via a call to the CNI plugin and allocates the connected subnetwork routes via calls to the IP Address Management (IPAM) plugin.

Here it does not explain details about CNI but CNI needs to be well considered based on your use case or application requirements since CNI is the network interface backend for application containers.

## Setup Kubernetes API Server

1st we do need to set up Kubernetes API server (Master server) to accept the worker nodes as cluster components.

```bash
### Be super user access
> sudo su -

### Make sure that swap is diabled
>  swapoff -a

### Cgroup for docker is set into systemd
> docker info | grep Cgroup
 Cgroup Driver: systemd
 Cgroup Version: 2

### Make sure kubeadm is installed to start the cluster
> kubeadm version
kubeadm version: &version.Info{Major:"1", Minor:"29", GitVersion:"v1.29.13", GitCommit:"9a58e9398d4aa69d7ad40f40407e54b96025e0c5", GitTreeState:"clean", BuildDate:"2025-01-15T14:41:15Z", GoVersion:"go1.22.10", Compiler:"gc", Platform:"linux/amd64"}

### Set container runtime cgroup driver aligned with Kubernetes
### See https://kubernetes.io/docs/tasks/administer-cluster/kubeadm/configure-cgroup-driver/
> cat /etc/docker/daemon.json
{
    "exec-opts": ["native.cgroupdriver=systemd"]
}

### Restart docker systemd service
> systemctl restart docker

### Initialize master node, it might take a few minutes to complete
> kubeadm init --pod-network-cidr=10.244.0.0/16 --cri-socket unix:///var/run/containerd/containerd.sock
init] Using Kubernetes version: v1.29.13
[preflight] Running pre-flight checks
[preflight] Pulling images required for setting up a Kubernetes cluster
[preflight] This might take a minute or two, depending on the speed of your internet connection
[preflight] You can also perform this action in beforehand using 'kubeadm config images pull'
...<snip>
Your Kubernetes control-plane has initialized successfully!

To start using your cluster, you need to run the following as a regular user:

  mkdir -p $HOME/.kube
  sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config
  sudo chown $(id -u):$(id -g) $HOME/.kube/config

Alternatively, if you are the root user, you can run:

  export KUBECONFIG=/etc/kubernetes/admin.conf

You should now deploy a pod network to the cluster.
Run "kubectl apply -f [podnetwork].yaml" with one of the options listed at:
  https://kubernetes.io/docs/concepts/cluster-administration/addons/

Then you can join any number of worker nodes by running the following on each as root:

kubeadm join <YOUR_IP_ADDRESS>:6443 --token <YOUR_TOKEN> --discovery-token-ca-cert-hash <YOUR_HASH> 
```

***make sure you save the last line that shows how to join the cluster from worker nodes with tokens and CA hash.***

## Access API-server

Make sure that we can access Kubernetes API server w/o any problems.

```bash
### save the configuration
> mkdir -p $HOME/.kube
> cp -i /etc/kubernetes/admin.conf $HOME/.kube/config
> chown $(id -u):$(id -g) $HOME/.kube/config

### check node status
> kubectl get nodes -o wide

### check pods status
> kubectl get pods -A
NAMESPACE     NAME                                   READY   STATUS    RESTARTS   AGE
kube-system   coredns-76f75df574-75r5m               0/1     Pending   0          4m3s
kube-system   coredns-76f75df574-hcqgr               0/1     Pending   0          4m3s
kube-system   etcd-tomoyafujita                      1/1     Running   4          4m16s
kube-system   kube-apiserver-tomoyafujita            1/1     Running   0          4m16s
kube-system   kube-controller-manager-tomoyafujita   1/1     Running   0          4m17s
kube-system   kube-proxy-6bd64                       1/1     Running   0          4m3s
kube-system   kube-scheduler-tomoyafujita            1/1     Running   0          4m16s
```

## Worker Node Join

### Join the cluster

We need to do this procedure for all worker nodes to join the cluster system.

```bash
> swapoff -a

> docker info | grep Cgroup
 Cgroup Driver: systemd
 Cgroup Version: 2

> kubeadm version
kubeadm version: &version.Info{Major:"1", Minor:"29", GitVersion:"v1.29.13", GitCommit:"9a58e9398d4aa69d7ad40f40407e54b96025e0c5", GitTreeState:"clean", BuildDate:"2025-01-15T14:41:15Z", GoVersion:"go1.22.10", Compiler:"gc", Platform:"linux/amd64"}

> cat /etc/docker/daemon.json
{
    "exec-opts": ["native.cgroupdriver=systemd"]
}

> systemctl restart docker
> kubeadm join <YOUR_IP_ADDRESS>:6443 --token <YOUR_TOKEN> --discovery-token-ca-cert-hash <YOUR_HASH>  --cri-socket unix:///var/run/containerd/containerd.sock
[preflight] Running pre-flight checks
[preflight] Reading configuration from the cluster...
[preflight] FYI: You can look at this config file with 'kubectl -n kube-system get cm kubeadm-config -o yaml'
[kubelet-start] Writing kubelet configuration to file "/var/lib/kubelet/config.yaml"
[kubelet-start] Writing kubelet environment file with flags to file "/var/lib/kubelet/kubeadm-flags.env"
[kubelet-start] Starting the kubelet
[kubelet-start] Waiting for the kubelet to perform the TLS Bootstrap...

This node has joined the cluster:
* Certificate signing request was sent to apiserver and a response was received.
* The Kubelet was informed of the new secure connection details.

Run 'kubectl get nodes' on the control-plane to see this node join the cluster.
```

Check the node availability on master node.

```bash
### check node status
> kubectl get nodes -o wide
```

## Deploy CNI plugin

To get cluster nodes ready, we need to deploy CNI plugin as pods.
Here it describes install / uninstall CNI plugins via `kubectl`, CNI plugins are managed under kubernetes pods as well.

### [Cilium](https://cilium.io/)

You can refer to [Quick Installation](https://docs.cilium.io/en/stable/gettingstarted/k8s-install-default/#k8s-install-quick) for Kubernetes if needed.

#### Install Cilium CLI

This operation is only requires once for master node.
The Cilium CLI can be used to install Cilium, inspect the state of a Cilium installation, and enable/disable various features.

```bash
> sudo su -
[sudo] password for tomoyafujita:
> cd <repo>/scripts
> ./install_cilium_cli.sh
...<snip>
Cilium CLI is installed on /usr/local/bin/cilium, for uninstallation you can just delete the executable.
```

#### Install Cilium CNI to the Kubernetes Cluster

```bash
> cilium version
cilium-cli: v0.16.24 compiled with go1.23.4 on linux/amd64
cilium image (default): v1.16.6
cilium image (stable): v1.16.6
cilium image (running): unknown. Unable to obtain cilium version. Reason: release: not found

> cilium install
  Using Cilium version 1.16.6
🔮 Auto-detected cluster name: kubernetes
🔮 Auto-detected kube-proxy has been installed
```

#### Enable Security Encryption

Cilium provides security encryption feature either IPsec or Wireguard statically during initialization.
Packets are not encrypted when they are destined to the same node from which they were sent. This behavior is intended.
Encryption would provide no benefits in that case, given that the raw traffic can be observed on the node anyway.
Encryption requires key, the following procedure uses Kubernetes Secret to bind the key for cilium encryption.

```bash
### If cilium is already deployed, uninstall from cluster system
> cilium uninstall

### Create Kubernetes Secret that is used for encryption
> PSK=($(dd if=/dev/urandom count=20 bs=1 2> /dev/null | xxd -p -c 64))

> echo $PSK
47b57a0241e4c5df0a196ccfbcc7ee1aff204100

> kubectl create -n kube-system secret generic cilium-ipsec-keys --from-literal=keys="3 rfc4106(gcm(aes)) $PSK 128"
secret/cilium-ipsec-keys created

> kubectl describe secret -n kube-system cilium-ipsec-keys
Name:         cilium-ipsec-keys
Namespace:    kube-system
Labels:       <none>
Annotations:  <none>

Type:  Opaque

Data
====
keys:  64 bytes

> SECRET="$(kubectl get secrets cilium-ipsec-keys -o jsonpath='{.data}' -n kube-system | jq -r ".keys")"
> echo $SECRET | base64 --decode
3 rfc4106(gcm(aes)) 47b57a0241e4c5df0a196ccfbcc7ee1aff204100 128

### Start deployment with enabling encryption
> cilium install --encryption ipsec

### Check IPsec is enabled
> cilium config view | grep ipsec
enable-ipsec                               true
ipsec-key-file                             /etc/ipsec/keys
```

#### Enable [Hubble Observability](https://github.com/cilium/hubble/)

Cilium also provides the observability component for gaining deep insights into the network and security behavior of your applications.
This also works with ROS and ROS 2 application containers.
Hubble collects network flow data using eBPF and processes it to generate valuable insights, this data can be visualized and analyzed through the Hubble UI or using the Hubble CLI.

We can just follow [Service Map & Hubble UI](https://docs.cilium.io/en/latest/gettingstarted/hubble/#hubble-ui) procedure to enable Hubble and the following graphical interface will come up.

![Hubble GUI with ROS 2 application](./../images/ros2_hubble_observability.png)

#### Enable Multicast

> [!NOTE]
> Currently multicast feature is only available in [cilium:main](https://github.com/cilium/cilium/tree/main) branch, still under development.

The current mainstream container network plug-ins (such as Calico and Cilium) do not natively support multicast, in addition to this, WeaveNet supports multicast, but entire open source project has gone end of life.

Cilium is one of the most advanced container network plug-in supports multicast (some features e.g IPsec are under progress.) based on eBPF focuses on solving the problem of efficient multicast transmission in the container network and provides support for multiple multicast protocols.

In default Cilium multicast feature is disabled, so we need to walk through the following configuration.

- Enable multicast feature

  After Cilium is deployed in the cluster, we can enable the multicast feature.
  This will restart the cilium-agent pods in the cluster.

  ```bash
  ### Check vxlan mode is enabled
  > kubectl get configmap cilium-config -n kube-system -o yaml | grep vxlan
    tunnel-protocol: vxlan

  ### Enable multicast feature
  > cilium config set multicast-enabled true
  ✨ Patching ConfigMap cilium-config with multicast-enabled=true...
  ♻️  Restarted Cilium pods
  ```

- Configure Multicast Group IP addresses and Subscribers (Manually)

  To use multicast with Cilium, we need to configure multicast group and subscriber IP addresses based on the application requirement.
  ROS 2 (DDS) well-known multicast group IP address is `239.255.0.1`.

  **For each cilium pod**, we need to configure multicast groups.
  The following commands need to be issued in `cilium` pods, there should be `cilium-dbg` command to access `cilium-agent`.

  ```bash
  > cilium-dbg bpf multicast group add 239.255.0.1
  > cilium-dbg bpf multicast group list
  Group Address
  239.255.0.1
  #cilium-dbg bpf multicast group delete
  ```

  Then we need to configure group subscriber IP addresses to that group from `CILIUMINTERNALIP` below.

  ```bash
  > kubectl get ciliumnodes.cilium.io
  NAME                 CILIUMINTERNALIP   INTERNALIP   AGE
  kind-control-plane   10.244.0.72        172.19.0.2   16m
  kind-worker          10.244.1.86        172.19.0.3   16m
  ```

  **For each cilium pod**, we need to configure subscriber IP addresses to group.
  The following commands need to be issued in `cilium` pods, there should be `cilium-dbg` command to access `cilium-agent`.

  ```bash
  # On kind-control-plane
  cilium-dbg bpf multicast subscriber add 239.255.0.1 10.244.1.86
  #cilium-dbg bpf multicast subscriber delete 239.255.0.1 10.244.1.86
  cilium-dbg bpf multicast subscriber list all
  Group           Subscriber      Type
  239.255.0.1     10.244.1.86     Remote Node

  # On kind-worker
  cilium-dbg bpf multicast subscriber add 239.255.0.1 10.244.0.72
  #cilium-dbg bpf multicast subscriber delete 239.255.0.1 10.244.0.72
  cilium-dbg bpf multicast subscriber list all
  ```

  These `cilium-dbg` commands are available in the cilium version 1.16.0 or later.

- Simplified configuration about Multicast Groups and Subscribers using Cilium CLI

  If it's a cluster-wide operation, there's no need to perform operations on each cilium pod individually.
  This feature is available in the cilium-cli version `0.16.14` or later.

  To make all nodes join a specified multicast group, use the `cilium multicast` command. 
  ```bash
  # make all nodes join the specified multicast group
  > cilium multicast add --group-ip 239.255.0.1
  #cilium multicast delete --group-ip 239.255.0.1
  ```

  You can also get information about multicast groups and subscribers cluster-wide.
  ```bash
  # confirm the multicast groups and subscribers
  > cilium multicast list subscriber --all
  Node                 Group         Subscriber     Type
  kind-worker          239.255.0.1   10.244.0.196   Remote Node
  kind-control-plane   239.255.0.1   10.244.1.122   Remote Node
  ```

- Related PRs
  - Issues
    - https://github.com/cilium/cilium/issues/13239, https://github.com/cilium/cilium/issues/28750
    - https://github.com/cilium/cilium/issues/29471 (Multicast IPSec Support, tracking issue)
  - Design CFP
    - https://github.com/cilium/design-cfps/pull/20
  - Pull Requests
    - https://github.com/cilium/cilium/pull/29469
    - https://github.com/cilium/cilium/pull/31355

### [Flannel](https://github.com/flannel-io/flannel)

```bash
### Install
> kubectl apply -f https://raw.githubusercontent.com/flannel-io/flannel/master/Documentation/kube-flannel.yml
namespace/kube-flannel created
clusterrole.rbac.authorization.k8s.io/flannel created
clusterrolebinding.rbac.authorization.k8s.io/flannel created
serviceaccount/flannel created
configmap/kube-flannel-cfg created
daemonset.apps/kube-flannel-ds created

### Uninstall
> kubectl delete -f https://raw.githubusercontent.com/flannel-io/flannel/master/Documentation/kube-flannel.yml
```

### [WeaveNet](https://github.com/weaveworks/weave) **(E.O.L)**

#### Start/Stop WeaveNet DaemonSets

```bash
### Install
> kubectl apply -f https://github.com/weaveworks/weave/releases/download/v2.8.1/weave-daemonset-k8s-1.11.yaml
### Uninstall
> kubectl delete -f https://github.com/weaveworks/weave/releases/download/v2.8.1/weave-daemonset-k8s-1.11.yaml
```

Originally latest images should be applied for weave deployment, but latest image tag does not support arm64 multi-arch.
This problem is issued on https://github.com/weaveworks/weave/issues/3976, and after this problem (docker image multi-arch support for arm64) has been addressed, we should use the latest deployment file.

```bash
### Install
> kubectl apply -f https://github.com/weaveworks/weave/releases/download/v2.8.1/weave-daemonset-k8s.yaml
### Uninstall
> kubectl delete -f https://github.com/weaveworks/weave/releases/download/v2.8.1/weave-daemonset-k8s.yaml
```

#### [WeaveScope](https://github.com/weaveworks/scope) **(E.O.L)**

Weave also provides a powerful visualization and monitoring tool specifically designed for Docker and Kubernetes environments called scope.

```bash
### Install
> kubectl apply -f https://github.com/weaveworks/scope/releases/download/v1.13.2/k8s-scope.yaml
### Uninstall
> kubectl delete -f https://github.com/weaveworks/scope/releases/download/v1.13.2/k8s-scope.yaml
```

By default, weavescope uses Cluster IP address, so we need to `port-forward` it to the node IP address and port.

```bash
> kubectl port-forward svc/weave-scope-app -n weave 4040:80 --address YOUR_NODE_IP
```

and then, you should see the following UI via `http://<YOUR_NODE_IP>:4040/`

![WeaveScope Web UI](./../images/weavescope.png)

#### Security Encryption

WeaveNet provides authentication and encryption based on password, which is used for create session keys for the communication between peers.
WeaveNet uses fast datapath via Kernel OpenVswitch, and IPsec ESP can be enabled statically before any application pods are created.

```bash
> curl -L git.io/weave -o /usr/local/bin/weave
> chmod a+x /usr/local/bin/weave

### Weave CLI installs docker container to connect weavenet daemon to execute the command
> weave status

        Version: 2.8.1 (failed to check latest version - see logs; next check at 2023/03/05 01:37:01)

        Service: router
      Protocol: weave 1..2
          Name: 16:8b:92:14:73:c4(tomoyafujita-hp-compaq-elite-8300-sff)
    Encryption: disabled
  PeerDiscovery: enabled
        Targets: 1
    Connections: 1 (1 established)
          Peers: 2 (with 2 established connections)
TrustedSubnets: none

        Service: ipam
        Status: ready
          Range: 10.32.0.0/12
  DefaultSubnet: 10.32.0.0/12
```

As we can see above, WeaveNet does not enable encryption by default.
Adding environmental variable with your own password can enable WeaveNet encryption, the following shows how to do so via Kubernetes Secret. (which can be bound to application pods at initialization.)

```bash
> openssl rand -hex 128 > weave-passwd

> kubectl create secret -n kube-system generic weave-passwd --from-file=./weave-passwd
secret/weave-passwd created

> kubectl describe secret -n kube-system weave-passwd
Name:         weave-passwd
Namespace:    kube-system
Labels:       <none>
Annotations:  <none>

Type:  Opaque

Data
====
weave-passwd:  257 bytes

> kubectl edit --namespace=kube-system daemonset weave-net
daemonset.apps/weave-net edited

### add the following under `spec.template.spec.containers`
env:
- name: WEAVE_PASSWORD
  valueFrom:
    secretKeyRef:
      name: weave-passwd
      key: weave-passwd

> weave status | grep Encryption
    Encryption: enabled
```

## Kubernetes Dashboard

Kubernetes officially provides default dashboard to monitor the cluster information.
There are many monitoring workload from other vendors, but default dashboard does good enough to see the entire cluster activity even used to manage deployment via GUI.

See details for [Deploy and Access the Kubernetes Dashboard](https://kubernetes.io/docs/tasks/access-application-cluster/web-ui-dashboard/).

```bash
### Start dashboard
> kubectl apply -f https://raw.githubusercontent.com/kubernetes/dashboard/v2.7.0/aio/deploy/recommended.yaml
namespace/kubernetes-dashboard created
serviceaccount/kubernetes-dashboard created
service/kubernetes-dashboard created
secret/kubernetes-dashboard-certs created
secret/kubernetes-dashboard-csrf created
secret/kubernetes-dashboard-key-holder created
configmap/kubernetes-dashboard-settings created
role.rbac.authorization.k8s.io/kubernetes-dashboard created
clusterrole.rbac.authorization.k8s.io/kubernetes-dashboard created
rolebinding.rbac.authorization.k8s.io/kubernetes-dashboard created
clusterrolebinding.rbac.authorization.k8s.io/kubernetes-dashboard created
deployment.apps/kubernetes-dashboard created
service/dashboard-metrics-scraper created
deployment.apps/dashboard-metrics-scraper created

### Check dashboard pods and service are running
> kubectl get pods -n kubernetes-dashboard
NAME                                         READY   STATUS    RESTARTS   AGE
dashboard-metrics-scraper-64bcc67c9c-zmv7t   1/1     Running   0          55s
kubernetes-dashboard-5c8bd6b59-ppmpz         1/1     Running   0          55s
> kubectl get service -n kubernetes-dashboard
NAME                        TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)    AGE
dashboard-metrics-scraper   ClusterIP   10.111.141.248   <none>        8000/TCP   61s
kubernetes-dashboard        ClusterIP   10.96.239.105    <none>        443/TCP    61s

### Generate token to bind ServiceAccount
> kubectl apply -f dashboard-adminuser.yaml
serviceaccount/admin-user created
. kubectl apply -f dashboard-rolebind.yaml
clusterrolebinding.rbac.authorization.k8s.io/admin-user created
> kubectl -n kubernetes-dashboard create token admin-user --duration=8760h
eyJhbGciOiJSUzI1NiIsImtpZCI6ImVScUFkeS1oUkdJTkF3eGxMTkNfZ3Bxc3RjZndET3ZwNEJ2REJCbVAyU0EifQ.eyJhdWQiOlsiaHR0cHM6Ly9rdWJlcm5ldGVzLmRlZmF1bHQuc3ZjLmNsdXN0ZXIubG9jYWwiXSwiZXhwIjoxNjc2Njg3Nzg5LCJpYXQiOjE2NzY2ODQxODksImlzcyI6Imh0dHBzOi8va3ViZXJuZXRlcy5kZWZhdWx0LnN2Yy5jbHVzdGVyLmxvY2FsIiwia3ViZXJuZXRlcy5pbyI6eyJuYW1lc3BhY2UiOiJrdWJlcm5ldGVzLWRhc2hib2FyZCIsInNlcnZpY2VhY2NvdW50Ijp7Im5hbWUiOiJhZG1pbi11c2VyIiwidWlkIjoiMDA2NjhiOWQtYWUwNS00NDdmLWJkYjUtOGI4MGVjMTgzZTc1In19LCJuYmYiOjE2NzY2ODQxODksInN1YiI6InN5c3RlbTpzZXJ2aWNlYWNjb3VudDprdWJlcm5ldGVzLWRhc2hib2FyZDphZG1pbi11c2VyIn0.J19f30ptmJ_PIOmhf9hUMDUc-ajr6F7VSkUQyKaM10MIvXp99mO036kHnKQgJRj1P9K9IJwOTwHdikvrE2iupDLzwilIIbzDNzDhKTQzOgFYkwKL2xQlLv1pksA9UDm9eOJA3fXVhcJ22imyRSxh-IMB4jz7IVQVObaXoZGp04J3A0vzJoQPdPWfcJ1ezZvghlZsRrNkkuTrrH3Yek2yrB2keh6oBjWZtWG7zNHd8MXhO5K0NEP_lWWyTSDX2TI9gdgYHP12gUiD6t14gcbZCObRpV8-m3qtUTbMPR_3DBo-LwrGFL6pc-i6mysjZ8qq2ssD5sZRG4mcKuVoDsjVeg

### Copy the above token

### Start kube-proxy, this is required to proxy cluster network and host network.
> kubectl proxy
Starting to serve on 127.0.0.1:8001
```

Until here, Kubernetes dashboard is running in cluster and proxy bridges to localhost as shows above.

Access http://localhost:8001/api/v1/namespaces/kubernetes-dashboard/services/https:kubernetes-dashboard:/proxy/#/login, to check if you can see the following page,.

![Kubernetes Dashboard Frontpage](./../images/kubernetes_dashboard_frontpage.png)

To login the dashboard, input the token which has been generated above procedure.
Now you should be able to see the following Kubernetes cluster dashboard.

![Kubernetes Dashboard Overview](./../images/kubernetes_dashboard_overview.png)

## Break down the cluster

the following command needs to be issues on each node in the cluster system.

```bash
### Reset the node
kubeadm reset -f --cri-socket unix:///var/run/containerd/containerd.sock
rm -rf $HOME/.kube/config

### CNI conflist will be re-created by CNI installation.
rm -rf /etc/cni/net.d

### If WeaveNet CNI was deployed
rm -rf /var/lib/weave
```
