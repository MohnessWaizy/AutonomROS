#! /bin/bash

#pathToBinFile=$1
#binFile="$(awk -F/ '{print $NF}' <<< $1)"

mkdir -p /lib/firmware
cp $1design_1_wrapper.bin /lib/firmware
echo design_1_wrapper.bin > /sys/class/fpga_manager/fpga0/firmware
