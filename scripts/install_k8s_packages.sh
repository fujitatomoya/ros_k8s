#!/bin/bash

### enable debug
#set -exvfC
set -e

### User Setting
KUBERNETES_VERSION=v1.26

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

function exit_trap() {
    if [ $? != 0 ]; then
        echo "Command [$BASH_COMMAND] is failed"
        exit 1
    fi
}

function check_sudo() {
	if [ "$EUID" -ne 0 ]; then
		echo "Please run this script with sudo."
		exit 1
	fi
}

function install_container_runtime () {
	trap exit_trap ERR
	apt remove -y containerd docker.io
	apt install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common dpkg-dev gnupg2
	curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
	add-apt-repository -y "deb [arch=$(dpkg-architecture -q DEB_BUILD_ARCH)] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
	apt update -y
	apt install -y docker-ce docker-ce-cli containerd.io
	mkdir -p /etc/containerd
	containerd config default | tee /etc/containerd/config.toml
	sed -i 's/SystemdCgroup \= false/SystemdCgroup \= true/g' /etc/containerd/config.toml
	systemctl restart containerd
	systemctl enable containerd
	systemctl restart docker
	systemctl enable docker
	docker version
}

function install_kubernetes () {
	trap exit_trap ERR
	apt remove -y --allow-change-held-packages kubelet kubectl kubeadm
	apt install -y apt-transport-https ca-certificates curl
	curl -fsSL https://pkgs.k8s.io/core:/stable:/$KUBERNETES_VERSION/deb/Release.key | gpg --yes --dearmor -o /etc/apt/keyrings/kubernetes-apt-keyring.gpg
	echo "deb [signed-by=/etc/apt/keyrings/kubernetes-apt-keyring.gpg] https://pkgs.k8s.io/core:/stable:/$KUBERNETES_VERSION/deb/ /" | tee /etc/apt/sources.list.d/kubernetes.list
	apt update -y
	apt install -y kubelet kubeadm kubectl
	apt-mark hold kubelet kubeadm kubectl
	kubeadm version
	### kubectl version will try to access server that leads to error to exit this script
	kubectl version --client
}

function install_golang () {
	trap exit_trap ERR
	add-apt-repository -y ppa:longsleep/golang-backports
	apt update
	### golang-1.21 or later should be fine, just installing
	apt install -y golang-1.21 golang-go
	go version
}

function install_kind () {
	trap exit_trap ERR
	go install sigs.k8s.io/kind@v0.19.0
	### kind needs to be in path
	$HOME/go/bin/kind version
}

### Main

# set the trap on error
trap exit_trap ERR

check_sudo

echo "Install Container Runtime ----------"
install_container_runtime

echo "Install Kubernetes ----------"
install_kubernetes

echo "Install Golang ----------"
install_golang

echo "Install KIND ----------"
install_kind

echo "Installation completed, enjoy your cluster !!!"

exit 0
