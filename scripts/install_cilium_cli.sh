#!/bin/bash

### enable debug
#set -xzvfC

### Get Cilium CLI version
if ! command -v curl &> /dev/null
then
    echo "curl could not be found, please install it and retry."
    exit
fi
CILIUM_CLI_VERSION=$(curl -s https://raw.githubusercontent.com/cilium/cilium-cli/master/stable.txt)
echo "Cilium CLI version is $CILIUM_CLI_VERSION"

### CPU Arch
CLI_ARCH=amd64 #default
if [ "$(uname -m)" = "aarch64" ]; then CLI_ARCH=arm64; fi
echo "CPU architecture is $CLI_ARCH"

### Install Cilium CLI
curl -L --fail --remote-name-all https://github.com/cilium/cilium-cli/releases/download/${CILIUM_CLI_VERSION}/cilium-linux-${CLI_ARCH}.tar.gz{,.sha256sum}
sha256sum --check cilium-linux-${CLI_ARCH}.tar.gz.sha256sum
tar xzvfC cilium-linux-${CLI_ARCH}.tar.gz /usr/local/bin
rm cilium-linux-${CLI_ARCH}.tar.gz{,.sha256sum}
echo "Cilium CLI is installed on /usr/local/bin/cilium, for uninstallation you can just delete the executable."
