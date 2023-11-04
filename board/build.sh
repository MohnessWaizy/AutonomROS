#!/bin/bash

set -e
set -x

sudo mkdir build/
sudo tar xzfp ./requirements/ubuntu-base-20.04.4-base-arm64.tar.gz -C ./build/
sudo cp /usr/bin/qemu-aarch64-static ./build/usr/bin/
sudo cp /run/systemd/resolve/stub-resolv.conf ./build/etc/resolv.conf
sudo cp ./install.sh ./build/
sudo chmod u+x ./build/install.sh
