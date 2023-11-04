#!/bin/bash

set -e
set -x

useradd -G sudo -m -s /bin/bash xilinx
apt-get update
apt-get -y upgrade
apt-get -y install locales
apt-get -y install dialog
apt-get -y install sudo ifupdown net-tools udev wireless-tools iputils-ping resolvconf apt-utils wpasupplicant
apt-get -y install kmod openssh-server
apt-get -y install vim
apt-get -y install linux-firmware linux-headers-generic

echo 'Set password with passwd xilinx'
