#!/bin/bash

set -e
set -x

mkdir -p ~/venvs/
sudo apt-get -y install python3-venv python3-psutil
cd ~/venvs/
python3 -m venv ansible
source ~/venvs/ansible/bin/activate
pip install -U pip
pip install ansible
pip install ansible-lint
