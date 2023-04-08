#!/bin/bash

### enable debug
#set -exvfC
set -e

### Confirmation to install all dependent packages

echo "-------------------------------------------------------------------"
echo "This will reinstall the all depedent packages, including removing installed pacakges."
echo "golang, containerd, docker and kubernetes related packages will be reinstalled with specific version."
echo "*** Basically this needs to be done only once for each system ***"
echo "-------------------------------------------------------------------"

while true; do
    read -p "Do you want to proceed? (yes/no) " yn
    case $yn in 
	    yes ) echo OK, we will proceed;
			break;;
	    no ) echo exiting...;
    		exit;;
	    * ) echo invalid response;
    esac
done

### Functions Implementation

install_container_runtime () {
	apt remove -y containerd docker.io
	apt install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common dpkg-dev
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
}

install_kubernetes () {
	apt install -y apt-transport-https curl
	curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add
	apt-add-repository -y "deb http://apt.kubernetes.io/ kubernetes-xenial main"
	apt install -y kubeadm=1.25.5-00 kubelet=1.25.5-00 kubectl=1.25.5-00
	kubeadm version
	### kubectl version will try to access server that leads to error to exit this script
	kubectl version --client
}

install_golang () {
	add-apt-repository -y ppa:longsleep/golang-backports
	apt update
	### golang-1.19 or later should be fine, just installing
	apt install -y golang-1.19 golang-go
	go version
}

install_kind () {
	go install sigs.k8s.io/kind@v0.16.0
	### kind needs to be in path
	$HOME/go/bin/kind version
}

### Main

echo "Install Container Runtime ----------"
install_container_runtime

echo "Install Kubernetes ----------"
install_kubernetes

echo "Install Golang ----------"
install_golang

echo "Install KIND ----------"
install_kind

echo "Installation completed, enjoy your cluster !!!"
