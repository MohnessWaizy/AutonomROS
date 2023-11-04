#!/bin/bash

set -e
set -x

sudo umount ./build/dev
sudo umount ./build/proc
sudo umount ./build/sys
