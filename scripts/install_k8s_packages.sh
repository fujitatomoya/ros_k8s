#!/bin/bash

### enable debug
#set -exvfC
set -e

#######################################################################
# Install container runtime, Kubernetes, kind and (optionally) keadm.
#
# Verified host: Ubuntu 24.04 LTS (Noble) amd64 / arm64 (Raspberry Pi)
#
# Usage:
#   sudo ./install_k8s_packages.sh [-k] [-y]
#     -k : also install KubeEdge keadm (default: no)
#     -y : do not ask for confirmation (default: ask)
#
# Versions can be overridden via environment variables, e.g.
#   sudo KUBERNETES_VERSION=v1.32 KUBEEDGE_VERSION=v1.23.1 ./install_k8s_packages.sh -k
#######################################################################

################
# User Setting #
################

# Kubernetes minor version used for the apt repository (pkgs.k8s.io).
# NOTE: KubeEdge v1.23 is verified with Kubernetes v1.30 - v1.32,
#       see https://github.com/kubeedge/kubeedge#kubernetes-compatibility.
KUBERNETES_VERSION="${KUBERNETES_VERSION:-v1.37}"

# kind (Kubernetes IN Docker) release, see https://github.com/kubernetes-sigs/kind/releases
KIND_VERSION="${KIND_VERSION:-v0.33.0}"

# KubeEdge keadm release, see https://github.com/kubeedge/kubeedge/releases
KUBEEDGE_VERSION="${KUBEEDGE_VERSION:-v1.23.1}"

install_keadm=false
assume_yes=false

### Functions Implementation

function print_usage() {
    echo "Usage: sudo $0 [-k] [-y]"
    echo "Options(default):"
    echo "  -k : also install KubeEdge keadm ${KUBEEDGE_VERSION}. (default: false)"
    echo "  -y : do not ask for confirmation. (default: false)"
    exit 1
}

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

function confirm() {
    echo "-------------------------------------------------------------------"
    echo "This will reinstall the all dependent packages, including removing installed packages."
    echo "containerd, docker, kubernetes related packages and kind will be reinstalled with specific version."
    echo "  Kubernetes : ${KUBERNETES_VERSION}"
    echo "  kind       : ${KIND_VERSION}"
    if [ "$install_keadm" == true ]; then
        echo "  KubeEdge   : ${KUBEEDGE_VERSION}"
    fi
    echo "*** Basically this needs to be done only once for each system ***"
    echo "-------------------------------------------------------------------"
    if [ "$assume_yes" == true ]; then
        return
    fi
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
}

function check_platform() {
    trap exit_trap ERR
    . /etc/os-release
    if [ "$ID" != "ubuntu" ]; then
        echo "This script only supports Ubuntu, detected: $ID"
        exit 1
    fi
    echo "Ubuntu ${VERSION_ID} (${VERSION_CODENAME}) $(dpkg --print-architecture) detected."
    if [ "$VERSION_CODENAME" != "noble" ]; then
        echo "WARNING: this script is verified on Ubuntu 24.04 (noble) only."
    fi
}

function configure_system() {
    trap exit_trap ERR
    ### kubelet fails to start if swap is enabled (unless failSwapOn=false)
    swapoff -a
    sed -i.bak '/\sswap\s/s/^\(.*\)$/#\1/g' /etc/fstab
    ### IPv4 forwarding is required for Kubernetes networking
    cat <<EOF > /etc/sysctl.d/k8s.conf
net.ipv4.ip_forward = 1
EOF
    sysctl --system > /dev/null
    sysctl net.ipv4.ip_forward
}

function install_container_runtime () {
    trap exit_trap ERR
    ### remove Ubuntu distribution packages if installed, Docker apt repository is used instead
    apt remove -y containerd docker.io docker-compose docker-doc podman-docker runc || true
    apt update -y
    apt install -y ca-certificates curl gnupg
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    chmod a+r /etc/apt/keyrings/docker.asc
    ### deb822 format, see https://docs.docker.com/engine/install/ubuntu/
    rm -f /etc/apt/sources.list.d/docker.list
    cat <<EOF > /etc/apt/sources.list.d/docker.sources
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
    apt update -y
    apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ### containerd.io package disables CRI plugin by default, regenerate the default configuration.
    ### SystemdCgroup must be enabled since kubelet uses systemd cgroup driver by default.
    mkdir -p /etc/containerd
    containerd config default > /etc/containerd/config.toml
    sed -i 's/SystemdCgroup \= false/SystemdCgroup \= true/g' /etc/containerd/config.toml
    systemctl restart containerd
    systemctl enable containerd
    systemctl restart docker
    systemctl enable docker
    ### allow the user who invoked sudo to use docker without root
    if [ -n "$SUDO_USER" ]; then
        usermod -aG docker "$SUDO_USER"
    fi
    containerd --version
    docker version
}

function install_kubernetes () {
    trap exit_trap ERR
    apt install -y apt-transport-https ca-certificates curl gpg
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://pkgs.k8s.io/core:/stable:/$KUBERNETES_VERSION/deb/Release.key | gpg --yes --dearmor -o /etc/apt/keyrings/kubernetes-apt-keyring.gpg
    chmod 644 /etc/apt/keyrings/kubernetes-apt-keyring.gpg
    echo "deb [signed-by=/etc/apt/keyrings/kubernetes-apt-keyring.gpg] https://pkgs.k8s.io/core:/stable:/$KUBERNETES_VERSION/deb/ /" > /etc/apt/sources.list.d/kubernetes.list
    chmod 644 /etc/apt/sources.list.d/kubernetes.list
    apt update -y
    apt remove -y --allow-change-held-packages kubelet kubectl kubeadm || true
    apt install -y kubelet kubeadm kubectl
    apt-mark hold kubelet kubeadm kubectl
    systemctl enable --now kubelet
    kubeadm version
    ### kubectl version will try to access server that leads to error to exit this script
    kubectl version --client
}

function install_kind () {
    trap exit_trap ERR
    ### kind is a statically linked binary, no golang required.
    ### see https://kind.sigs.k8s.io/docs/user/quick-start/#installation
    KIND_ARCH=$(dpkg --print-architecture)
    curl -Lo /usr/local/bin/kind https://kind.sigs.k8s.io/dl/${KIND_VERSION}/kind-linux-${KIND_ARCH}
    chmod +x /usr/local/bin/kind
    kind version
}

function install_kubeedge_keadm () {
    trap exit_trap ERR
    ### see https://kubeedge.io/docs/setup/install-with-keadm/
    KEADM_ARCH=$(dpkg --print-architecture)
    TMPDIR=$(mktemp -d)
    curl -Lo ${TMPDIR}/keadm.tar.gz https://github.com/kubeedge/kubeedge/releases/download/${KUBEEDGE_VERSION}/keadm-${KUBEEDGE_VERSION}-linux-${KEADM_ARCH}.tar.gz
    tar -zxf ${TMPDIR}/keadm.tar.gz -C ${TMPDIR}
    install -m 0755 ${TMPDIR}/keadm-${KUBEEDGE_VERSION}-linux-${KEADM_ARCH}/keadm/keadm /usr/local/bin/keadm
    rm -rf ${TMPDIR}
    keadm version
}

### Main

# set the trap on error
trap exit_trap ERR

# parse command line options
while getopts ":ky" opt; do
    case $opt in
        k)
            install_keadm=true
            ;;
        y)
            assume_yes=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG"
            print_usage
            ;;
    esac
done
shift $((OPTIND-1))

check_sudo
check_platform
confirm

echo "Configure System ----------"
configure_system

echo "Install Container Runtime ----------"
install_container_runtime

echo "Install Kubernetes ----------"
install_kubernetes

echo "Install KIND ----------"
install_kind

if [ "$install_keadm" == true ]; then
    echo "Install KubeEdge keadm ----------"
    install_kubeedge_keadm
fi

echo "Installation completed, enjoy your cluster !!!"
echo "NOTE: log out and log in again to use docker without sudo."

exit 0
