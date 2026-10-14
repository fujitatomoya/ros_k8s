# Install Kubernetes Packages

These packages are required to both cluster on host and virtualized environment.
And all cluster nodes must install the required packages to join the cluster, that is said we need to install these packages on every single host system.
Even virtualized environment, it will issue Kubernetes API from host system to Kubernetes API server running in the virtualized environment, so that it can provide the consistent user experience.

## Verified Environment

| Component | Version | Note |
| --- | --- | --- |
| Ubuntu | 24.04 LTS (Noble) | amd64 / arm64 (Raspberry Pi 4/5) |
| Docker Engine / containerd | latest stable from [Docker apt repository](https://docs.docker.com/engine/install/ubuntu/) | containerd 2.x |
| Kubernetes | v1.37 | [pkgs.k8s.io](https://kubernetes.io/blog/2023/08/15/pkgs-k8s-io-introduction/) |
| kind | v0.33.0 | default node image `kindest/node:v1.37.0` |
| KubeEdge | v1.23.1 | verified with Kubernetes v1.30 - v1.32, see [Kubernetes Compatibility](https://github.com/kubeedge/kubeedge#kubernetes-compatibility) |
| Cilium | v1.20.x | CNI plugin |

> [!NOTE]
> If you plan to use KubeEdge, it is recommended to install the Kubernetes version that is verified with the KubeEdge release.
> You can specify the Kubernetes minor version via `KUBERNETES_VERSION` environment variable to the installation script, e.g. `KUBERNETES_VERSION=v1.32`.

To install all related packages to host system, you can use [install_k8s_packages.sh](../scripts/install_k8s_packages.sh).

**The following commands requires `sudo` access permission to install packages.**

```bash
### Install containerd/docker, kubelet/kubeadm/kubectl and kind
> sudo ./scripts/install_k8s_packages.sh

### Install KubeEdge keadm as well
> sudo ./scripts/install_k8s_packages.sh -k

### Override versions, e.g. Kubernetes v1.32 for KubeEdge v1.23
> sudo KUBERNETES_VERSION=v1.32 ./scripts/install_k8s_packages.sh -k
```

The following are manual steps to install packages that you can refer to understand what packages are actually installed.
All commands below are expected to be executed as `root` user.

## Raspberry Pi Setting

### Enable Cgroup Raspberry Pi

```bash
### Add cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1
> cat /boot/firmware/cmdline.txt
net.ifnames=0 dwc_otg.lpm_enable=0 console=serial0,115200 console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1

> reboot
```

### Load VxLAN kernel module on Raspberry Pi

**see more details for https://github.com/fujitatomoya/ros_k8s/issues/21, this operation is required for Raspberry Pi Ubuntu 21.10 or later.**

```bash
> apt install -y linux-modules-extra-raspi
> reboot
```

## System Configuration

`kubelet` fails to start if swap memory is enabled on the node, and IPv4 forwarding is required for the Kubernetes networking.

```bash
### Disable swap now and after reboot
> swapoff -a
> sed -i '/\sswap\s/s/^\(.*\)$/#\1/g' /etc/fstab

### Enable IPv4 forwarding
> cat <<EOF > /etc/sysctl.d/k8s.conf
net.ipv4.ip_forward = 1
EOF
> sysctl --system
> sysctl net.ipv4.ip_forward
net.ipv4.ip_forward = 1
```

## Docker / Containerd

Kubernetes only requires the container runtime that supports Container Runtime Interface (CRI) v1, which is `containerd` here.
Docker Engine is installed as well for building and testing container images on the host system, and `kind` requires Docker to run the virtualized cluster.

Both packages are installed from [Docker apt repository](https://docs.docker.com/engine/install/ubuntu/), which provides containerd 2.x.

```bash
### Remove Ubuntu distribution packages if installed
> apt remove -y containerd docker.io docker-compose docker-doc podman-docker runc

### Add Docker apt repository
> apt update
> apt install -y ca-certificates curl gnupg
> install -m 0755 -d /etc/apt/keyrings
> curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
> chmod a+r /etc/apt/keyrings/docker.asc
> cat <<EOF > /etc/apt/sources.list.d/docker.sources
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
> apt update

### Install Docker Engine and containerd
> apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

`containerd.io` package disables CRI plugin by default (`disabled_plugins = ["cri"]` in `/etc/containerd/config.toml`), so the default configuration must be regenerated.
Besides that, `kubelet` uses `systemd` cgroup driver by default, the container runtime must use the same cgroup driver.
see [Configuring the systemd cgroup driver](https://kubernetes.io/docs/setup/production-environment/container-runtimes/#containerd-systemd) for more details.

```bash
> mkdir -p /etc/containerd
> containerd config default > /etc/containerd/config.toml
> sed -i 's/SystemdCgroup \= false/SystemdCgroup \= true/g' /etc/containerd/config.toml

### containerd 2.x configuration should look like below
> grep -A3 'runc.options' /etc/containerd/config.toml
    [plugins.'io.containerd.cri.v1.runtime'.containerd.runtimes.runc.options]
      ...
      SystemdCgroup = true

> systemctl restart containerd
> systemctl enable containerd
> systemctl restart docker
> systemctl enable docker

### Allow non-root user to use docker (log out and log in again required)
> usermod -aG docker <YOUR_USER_NAME>

> containerd --version
> docker version
> docker info | grep -i cgroup
 Cgroup Driver: systemd
 Cgroup Version: 2
```

## Kubernetes

In this tutorial, we use Kubernetes `v1.37.x` in default.
Kubernetes apt repository is hosted per minor version on [pkgs.k8s.io](https://kubernetes.io/blog/2023/08/15/pkgs-k8s-io-introduction/), the minor version in the URL needs to be changed to install other version.

### Kubernetes Install

```bash
### uninstall if installed already
> apt remove -y --allow-change-held-packages kubelet kubectl kubeadm

### install kubernetes
> apt install -y apt-transport-https ca-certificates curl gpg
> install -m 0755 -d /etc/apt/keyrings
> curl -fsSL https://pkgs.k8s.io/core:/stable:/v1.37/deb/Release.key | gpg --dearmor -o /etc/apt/keyrings/kubernetes-apt-keyring.gpg
> chmod 644 /etc/apt/keyrings/kubernetes-apt-keyring.gpg
> echo 'deb [signed-by=/etc/apt/keyrings/kubernetes-apt-keyring.gpg] https://pkgs.k8s.io/core:/stable:/v1.37/deb/ /' > /etc/apt/sources.list.d/kubernetes.list
> chmod 644 /etc/apt/sources.list.d/kubernetes.list
> apt update
> apt install -y kubelet kubeadm kubectl
> apt-mark hold kubelet kubeadm kubectl
> systemctl enable --now kubelet

### kubectl version will try to access server, use --client
> kubeadm version
kubeadm version: &version.Info{Major:"1", Minor:"37", GitVersion:"v1.37.0", ...}
> kubectl version --client
Client Version: v1.37.0
Kustomize Version: v5.7.1
```

## Kind

[kind](https://kind.sigs.k8s.io/) is a statically linked binary, so we can just download the release binary.
(`go install sigs.k8s.io/kind@v0.33.0` can also be used if golang is installed.)

```bash
> curl -Lo /usr/local/bin/kind https://kind.sigs.k8s.io/dl/v0.33.0/kind-linux-$(dpkg --print-architecture)
> chmod +x /usr/local/bin/kind
> kind version
kind v0.33.0 go1.25.0 linux/amd64
```

## KubeEdge keadm (Optional)

`keadm` is the installation tool for KubeEdge, this is only required if you set up KubeEdge cloud / edge nodes.
see [Setup KubeEdge](./Setup_KubeEdge.md) for more details.

```bash
> wget https://github.com/kubeedge/kubeedge/releases/download/v1.23.1/keadm-v1.23.1-linux-$(dpkg --print-architecture).tar.gz
> tar -zxvf keadm-v1.23.1-linux-$(dpkg --print-architecture).tar.gz
> cp keadm-v1.23.1-linux-$(dpkg --print-architecture)/keadm/keadm /usr/local/bin/keadm
> keadm version
version: version.Info{Major:"1", Minor:"23", GitVersion:"v1.23.1", ...}
```

## Golang (Optional)

golang is not required for this tutorial anymore, but it is useful to build `kind` or CNI plugins from source.
Ubuntu 24.04 provides golang packages, or the latest release can be installed from [go.dev](https://go.dev/doc/install).

```bash
> curl -LO https://go.dev/dl/go1.27.2.linux-$(dpkg --print-architecture).tar.gz
> rm -rf /usr/local/go && tar -C /usr/local -xzf go1.27.2.linux-$(dpkg --print-architecture).tar.gz
> export PATH=$PATH:/usr/local/go/bin
> go version
go version go1.27.2 linux/amd64
```
