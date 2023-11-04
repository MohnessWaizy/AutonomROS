#!/bin/bash

set -e
set -x

sudo rm -f ./build/install.sh
sudo mkdir -p ./build/lib/modules/
sudo cp -r ./requirements/modules/* ./build/lib/modules/
sudo cp -r ./interfaces/* ./build/etc/network/interfaces.d/
sudo cp -r ./reconos/ ./build/opt/
