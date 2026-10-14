#!/bin/bash

### enable debug
#set -xzvfC
set -e

### see https://docs.cilium.io/en/stable/gettingstarted/k8s-install-default/#install-the-cilium-cli

### Get Cilium CLI version
if ! command -v curl &> /dev/null
then
    echo "curl could not be found, please install it and retry."
    exit 1
fi
CILIUM_CLI_VERSION="${CILIUM_CLI_VERSION:-$(curl -s https://raw.githubusercontent.com/cilium/cilium-cli/main/stable.txt)}"
echo "Cilium CLI version is $CILIUM_CLI_VERSION"

### CPU Arch
CLI_ARCH=amd64 #default
if [ "$(uname -m)" = "aarch64" ]; then CLI_ARCH=arm64; fi
echo "CPU architecture is $CLI_ARCH"

### Install Cilium CLI
TMPDIR=$(mktemp -d)
pushd ${TMPDIR} > /dev/null
curl -L --fail --remote-name-all https://github.com/cilium/cilium-cli/releases/download/${CILIUM_CLI_VERSION}/cilium-linux-${CLI_ARCH}.tar.gz{,.sha256sum}
sha256sum --check cilium-linux-${CLI_ARCH}.tar.gz.sha256sum
tar xzvfC cilium-linux-${CLI_ARCH}.tar.gz /usr/local/bin
popd > /dev/null
rm -rf ${TMPDIR}
echo "Cilium CLI is installed on /usr/local/bin/cilium, for uninstallation you can just delete the executable."
/usr/local/bin/cilium version --client
