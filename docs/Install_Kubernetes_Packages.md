# Install Kubernetes Packages

These packages are required to both cluster on host and virtualized environment.
And all cluster nodes must install the required packages to join the cluster, that is said we need to install these packages on every single host system.
Even virtualized environment, it will issue Kubernetes API from host system to Kuberentes API server running in the virtualized environment, so that it can provide the consistent user experience.

**The following commands requires `sudo` access permission to install packages.**

## Docker / Containerd

**Containerd v1.6.0 (Required for Kubernetes v1.26 or later)**

CRI v1alpha2 removed - kubelet will not register the node if the container runtime doesn't support CRI v1. So to work with Kubernetes 1.26, containerd 1.6.0 is required.

```bash
apt remove -y containerd docker.io
apt install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common vim
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
add-apt-repository -y "deb [arch=$(dpkg-architecture -q DEB_BUILD_ARCH)] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
apt update -y
apt install -y docker-ce docker-ce-cli containerd.io
mkdir -p /etc/containerd
containerd config default | tee /etc/containerd/config.toml
systemctl restart containerd
systemctl enable containerd
systemctl restart docker
systemctl enable docker
docker version
```

## Kubernetes

In this tutorial, we use Kuberentes version v1.25.5 instead of v1.26 that removes CRI v1alpha2, which could lead us complication and unexpected problems.

### Kubernetes Install

```bash
apt install -y apt-transport-https curl
curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add
apt-add-repository -y "deb http://apt.kubernetes.io/ kubernetes-xenial main"
apt install -y kubeadm=1.25.5-00 kubelet=1.25.5-00 kubectl=1.25.5-00
kubeadm version
### kubectl version will try to access server that leads to error to exit this script
kubectl version --client
```

## Golang

```bash
add-apt-repository -y ppa:longsleep/golang-backports
apt update
### golang-1.19 or later should be fine, just installing
apt install golang-1.19 golang-go
go version
```

## Kind

```bash
go install sigs.k8s.io/kind@v0.16.0
## kind needs to be in path
$HOME/go/bin/kind version
```
