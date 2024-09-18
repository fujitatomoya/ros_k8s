# Install Kubernetes Packages

These packages are required to both cluster on host and virtualized environment.
And all cluster nodes must install the required packages to join the cluster, that is said we need to install these packages on every single host system.
Even virtualized environment, it will issue Kubernetes API from host system to Kuberentes API server running in the virtualized environment, so that it can provide the consistent user experience.

To install all related packages to host system, you can use [install_k8s_packages.sh](../scripts/install_k8s_packages.sh).

**The following commands requires `sudo` access permission to install packages.**

```bash
> cd ros_k8s/scripts
> ./install_k8s_packages.sh
```

The following are manual steps to install packages that you can refer to understand what packages are actually installed.

## Host Setting

### Enable Cgroup Raspberry Pi

```bash
### Add cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1
> cat /boot/firmware/cmdline.txt
net.ifnames=0 dwc_otg.lpm_enable=0 console=serial0,115200 console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1

> reboot
```

### Load VxLAN kernel module on Raspberry Pi

**see more details for https://github.com/fujitatomoya/ros_k8s/issues/21, this operation is only required Raspi4 Ubuntu 21.10 or later.**

```bash
> apt install -y linux-modules-extra-raspi
> reboot
```

## Docker / Containerd

**Containerd v1.6.0 (Required for Kubernetes v1.26 or later)**

CRI v1alpha2 removed - kubelet will not register the node if the container runtime doesn't support CRI v1. So to work with Kubernetes 1.26 or later, containerd 1.6.0 is required.

```bash
> apt remove -y containerd docker.io
> apt install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common gnupg2
> curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
> add-apt-repository -y "deb [arch=$(dpkg-architecture -q DEB_BUILD_ARCH)] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
> apt update -y
> apt install -y docker-ce docker-ce-cli containerd.io
> mkdir -p /etc/containerd
> containerd config default | tee /etc/containerd/config.toml
> sed -i 's/SystemdCgroup \= false/SystemdCgroup \= true/g' /etc/containerd/config.toml
> systemctl restart containerd
> systemctl enable containerd
> systemctl restart docker
> systemctl enable docker
> docker version
```

## Kubernetes

In this tutorial, we use Kuberentes `v1.26.9-00`.

### Kubernetes Install

```bash
> apt install -y apt-transport-https curl
> curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add
> apt-add-repository -y "deb http://apt.kubernetes.io/ kubernetes-xenial main"
> apt install -y --allow-downgrades kubeadm=1.26.9-00 kubelet=1.26.9-00 kubectl=1.26.9-00
> kubeadm version

### kubectl version will try to access server that leads to error to exit this script
> kubectl version --client
```

## Golang

```bash
> add-apt-repository -y ppa:longsleep/golang-backports
> apt update

### golang-1.21 or later should be fine, just installing
> apt install golang-1.21 golang-go
> go version
```

## Kind

```bash
> go install sigs.k8s.io/kind@v0.19.0

## kind needs to be in path
> $HOME/go/bin/kind version
```
