#!/bin/bash
# Runs before rosdep install in the Docker container for cross compilation

set -e

apt-get update
apt-get install -y unzip

mkdir -p /home/AstraSDK/
cd /home/AstraSDK/

echo "67b7a5430a95ff779e1b548bc6babb72da938b7cd20078d875cdd2bdfe6907b90457f5b7f46e6f26c46423b73bf07da3891591562b403be429bca65f3a1f69df AstraSDK.zip" \
> AstraSDK.zip.sha
curl -fsS https://dl.orbbec3d.com/dist/astra/v2.1.3/AstraSDK-v2.1.3-Linux-arm.zip -o AstraSDK.zip
sha512sum -c AstraSDK.zip.sha

if [[ $? -ne 0 ]]
then
  cd /
  rm -r /home/AstraSDK/
  exit 1
fi

unzip AstraSDK.zip AstraSDK-v2.1.3-94bca0f52e-20210611T023312Z-Linux-aarch64.tar.gz
tar xzf AstraSDK-v2.1.3-94bca0f52e-20210611T023312Z-Linux-aarch64.tar.gz

mkdir -p /opt/AstraSDK/
cp -r AstraSDK-v2.1.3-94bca0f52e-20210611T023312Z-Linux-aarch64/include/ /opt/AstraSDK/include/
cp -r AstraSDK-v2.1.3-94bca0f52e-20210611T023312Z-Linux-aarch64/lib/ /opt/AstraSDK/lib/

cd /
rm -r /home/AstraSDK/
