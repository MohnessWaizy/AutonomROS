#!/bin/bash

set -e
set -x

sudo mount --bind /dev/ ./build/dev
sudo mount --bind /proc/ ./build/proc
sudo mount --bind /sys/ ./build/sys
