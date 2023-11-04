# Board Installation

This guide describes how to install the operating system on an SD Card for the ZCU104.

This guide is based on: [https://highlevel-synthesis.com/2019/12/15/running-ubuntu-on-ultra96v2-using-petalinux-2019-2-with-networking-and-linux-header/](https://highlevel-synthesis.com/2019/12/15/running-ubuntu-on-ultra96v2-using-petalinux-2019-2-with-networking-and-linux-header/)

Files in [reconos](./reconos) folder are provided by ReconROS/ReconOS.

## Requirements

The following package must be installed:

- `qemu-user-static`

Download the ubuntu-base image (ubuntu-base-20.04.4-base-arm64.tar.gz):

- Link: [https://cdimage.ubuntu.com/ubuntu-base/releases/20.04/release/](https://cdimage.ubuntu.com/ubuntu-base/releases/20.04/release/)
- Verify download (optional): [https://ubuntu.com/tutorials/how-to-verify-ubuntu#1-overview](https://ubuntu.com/tutorials/how-to-verify-ubuntu#1-overview)
- SHA512: `a51fdaf7bcfb5398240c7b6c0653cfa75cbfb675a6dca802fac5b407e08ebf12c0b1295d68e48fcb75ed668c8e92a3cab8017c7896f861d3d61a6fe648f51afc`

Put the file into the `requirements` folder.

Additionally, you need the following BOOT files from petalinux:

- `BOOT.BIN`
- `boot.scr`
- `image.ub`

Put them in some location for later use.

Additionally, you need the following modules:

- `5.4.0-rt1-xilinx-v2020.1`
- `5.4.0-xilinx-v2020.1`

Put both modules into `requriements/modules`.

## Build the rootfs

All commands should be executed from within this folder.

First run the following command:

```bash
./build.sh
```

This creates the `build` folder and installs Ubuntu in it.

Next run:

```bash
./mount.sh
```

This mounts `/dev`, `/proc`, and `/sys` into the newly created `build` folder. Pay attention to the remarks section at the end of this guide if you want to delete the folder later.

Next start the local ubuntu installation with:

```bash
sudo chroot ./build/
```

After that command you are in the local Ubuntu installation, logged in as root.

Now run the following command to install and configure the installation:

```bash
./install.sh
```

After that you need to manually set the password of the newly created `xilinx` user with:

```bash
passwd xilinx
```

Exit the local installation and unmount the mounted folders with:

```bash
exit
./umount.sh
```

Now run the post installation script to copy the required modules and network configuration.

```bash
./post.sh
```

After that the rootfs is ready to be put on an SD card.

## Prepare the SD card

Mount an SD card and create the following partitions:

- Partition 1:
  - Filesystem: FAT32
  - Size: ~100Mb
  - Name: BOOT
  - Type: primary partition
- Partition 2:
  - Filesystem: ext4
  - Size: rest
  - Name: rootfs
  - Type: primary partition

Copy the following files to the BOOT partition:

- `BOOT.BIN`
- `boot.scr`
- `image.ub`

Copy the root file system with the following command:

```bash
sudo cp -a ./build/. /media/<username>/rootfs/
```

Here, `/media/<username>/rootfs/` is the mount point of the SD card. The `-a` option is important to preserve file permissions and ownerships.

Run the following command to synchronize changes and then unmount the SD card:

```bash
sync
```

Now you should be able to boot the board with the SD card.

## Post configuration

Connect the board to a network via Ethernet or WLAN.

Boot the board using the prepared SD card

While the board is booting use `picocom -b 115200 /dev/ttyUSBx` where x is an integer.
(Usually its USB1)

If you did not choose the correct USB device exit picocom with `Ctrl+X`, `Ctrl+A` and try another USB device.
Then log in and run `ip addr` to get the IP address of the board.

You may need to enable the ethernet via `sudo ifup eth0` via picocom first. The wlan interface may have another name, which needs to be
adapted in `/etc/network/interfaces.d/wlan0`. If you want to automatically connect to ethernet or wlan at boot, you need to specify
`auto eth0` in `/etc/network/interfaces.d/eth0`, respectively `auto wlan0` in `/etc/network/interfaces.d/wlan0`. Be aware that with
these options, the OS waits on boot until these interfaces are available.

Create an entry in your ssh config `~/.ssh/config` called `board` for the Zynq UltraScale+ MPSoC ZCU104

```bash
Host board
        User xilinx
        HostName BOARD_IP_ADDRESS
```

Now you can install required software via Ansible.
The ansible playbook is in the `ansible` folder at the root of this repository.
The name is `board`. Run it with the following command (after sourcing the ansible environment and switching to the `ansible` folder):

```bash
ansible-playbook -i hosts.yml board.yml -K
```

## Important Remarks

- If you have mounted `/dev`, `/proc`, and `/sys` into `build`, do **not** delete the `build` folder. It will delete `/dev`, `/proc`, and `/sys` of your **host**! Run `./umount.sh` always before deleting `build` to be sure that the folders are not mounted. Additionally, you can check with `mount` whether the folders are still mounted.
