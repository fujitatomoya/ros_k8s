# Install Kubernetes Packages

These packages are required to both cluster on host and virtualized environment.
And all cluster nodes must install the required packages to join the cluster, that is said we need to install these packages on every single host system.
Even virtualized environment, it will issue Kubernetes API from host system to Kuberentes API server running in the virtualized environment, so that it can provide the consistent user experience.

**The following commands requires `sudo` access permission to install packages.**

## Docker

```bash
root@ubuntu:~# apt install docker.io
root@ubuntu:~# docker version
Client:
 Version:           20.10.12
 API version:       1.41
 Go version:        go1.16.2
 Git commit:        20.10.12-0ubuntu2~20.04.1
 Built:             Wed Apr  6 02:16:12 2022
 OS/Arch:           linux/arm64
 Context:           default
 Experimental:      true

Server:
 Engine:
  Version:          20.10.12
  API version:      1.41 (minimum version 1.12)
  Go version:       go1.16.2
  Git commit:       20.10.12-0ubuntu2~20.04.1
  Built:            Thu Feb 10 15:03:35 2022
  OS/Arch:          linux/arm64
  Experimental:     false
 containerd:
  Version:          1.5.9-0ubuntu1~20.04.4
  GitCommit:        
 runc:
  Version:          1.1.0-0ubuntu1~20.04.1
  GitCommit:        
 docker-init:
  Version:          0.19.0
  GitCommit:        
```

## Kubernetes

```bash
root@ubuntu:~# apt install apt-transport-https curl
root@ubuntu:~# apt install apt-transport-https curl
root@ubuntu:~# curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add
root@ubuntu:~# apt-add-repository "deb http://apt.kubernetes.io/ kubernetes-xenial main"
root@ubuntu:~# apt install kubeadm kubelet kubectl kubernetes-cni
root@ubuntu:~# kubeadm version
kubeadm version: &version.Info{Major:"1", Minor:"25", GitVersion:"v1.25.3", GitCommit:"434bfd82814af038ad94d62ebe59b133fcb50506", GitTreeState:"clean", BuildDate:"2022-10-12T10:55:36Z", GoVersion:"go1.19.2", Compiler:"gc", Platform:"linux/arm64"}
```

## Golang

```bash
root@ubuntu:~# add-apt-repository ppa:longsleep/golang-backports
root@ubuntu:~# apt update
root@ubuntu:~# apt install golang-1.19 golang-go
root@ubuntu:~# go version
go version go1.19.2 linux/arm64
```

## Kind

```bash
ubuntu@ubuntu:~$ go install sigs.k8s.io/kind@v0.16.0
ubuntu@ubuntu:~$ kind version
kind v0.16.0 go1.19.2 linux/arm64
```