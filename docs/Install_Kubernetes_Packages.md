# Install Kubernetes Packages

These packages are required to both cluster on host and virtualized environment.
And all cluster nodes must install the required packages to join the cluster, that is said we need to install these packages on every single host system.
Even virtualized environment, it will issue Kubernetes API from host system to Kuberentes API server running in the virtualized environment, so that it can provide the consistent user experience.

**The following commands requires `sudo` access permission to install packages.**

## Docker

```bash
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# apt install docker.io

root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# docker version
Client:
 Version:           20.10.12
 API version:       1.41
 Go version:        go1.16.2
 Git commit:        20.10.12-0ubuntu2~20.04.1
 Built:             Wed Apr  6 02:14:38 2022
 OS/Arch:           linux/amd64
 Context:           default
 Experimental:      true

Server:
 Engine:
  Version:          20.10.12
  API version:      1.41 (minimum version 1.12)
  Go version:       go1.16.2
  Git commit:       20.10.12-0ubuntu2~20.04.1
  Built:            Thu Feb 10 15:03:35 2022
  OS/Arch:          linux/amd64
  Experimental:     false
 containerd:
  Version:          1.5.9-0ubuntu1~20.04.6
  GitCommit:        
 runc:
  Version:          1.1.0-0ubuntu1~20.04.2
  GitCommit:        
 docker-init:
  Version:          0.19.0
  GitCommit:        

```

## Kubernetes

In this tutorial, we use Kuberentes version v1.25.5 instead of v1.26 that removes CRI v1alpha2, which could lead us complication and specific packages.

### Kubernetes Install

```bash
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# apt install apt-transport-https curl
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# apt-add-repository "deb http://apt.kubernetes.io/ kubernetes-xenial main"
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# apt install kubeadm=1.25.5-00 kubelet=1.25.5-00 kubectl=1.25.5-00

root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubeadm version
kubeadm version: &version.Info{Major:"1", Minor:"25", GitVersion:"v1.25.5", GitCommit:"804d6167111f6858541cef440ccc53887fbbc96a", GitTreeState:"clean", BuildDate:"2022-12-08T10:13:29Z", GoVersion:"go1.19.4", Compiler:"gc", Platform:"linux/amd64"}

root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# kubectl version
WARNING: This version information is deprecated and will be replaced with the output from kubectl version --short.  Use --output=yaml|json to get the full version.
Client Version: version.Info{Major:"1", Minor:"25", GitVersion:"v1.25.5", GitCommit:"804d6167111f6858541cef440ccc53887fbbc96a", GitTreeState:"clean", BuildDate:"2022-12-08T10:15:02Z", GoVersion:"go1.19.4", Compiler:"gc", Platform:"linux/amd64"}
Kustomize Version: v4.5.7
The connection to the server localhost:8080 was refused - did you specify the right host or port?
```

### Containerd v1.6.0 (Required for v1.26 or later)

CRI v1alpha2 removed - kubelet will not register the node if the container runtime doesn't support CRI v1. So to work with Kubernetes 1.26, containerd 1.6.0 is required.
This is required if using Kubernetes version 1.26 or later, see details https://blog.kubesimplify.com/kubernetes-126

```bash
root@ubuntu:~# curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
### amd64
root@ubuntu:~# add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
### aarch64
root@ubuntu:~# add-apt-repository "deb [arch=arm64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
root@ubuntu:~# apt update -y
root@ubuntu:~# apt install -y containerd.io
root@ubuntu:~# mkdir -p /etc/containerd
root@ubuntu:~# containerd config default | sudo tee /etc/containerd/config.toml
root@ubuntu:~# systemctl restart containerd
root@ubuntu:~# systemctl enable containerd
```

## Golang

```bash
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# add-apt-repository ppa:longsleep/golang-backports
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# apt update
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# apt install golang-1.19 golang-go
root@tomoyafujita-HP-Compaq-Elite-8300-SFF:~# go version
go version go1.19.5 linux/amd64
```

## Kind

```bash
tomoyafujita@~/DVT >go install sigs.k8s.io/kind@v0.16.0
tomoyafujita@~/DVT >kind version
kind v0.16.0 go1.19.5 linux/amd64
```