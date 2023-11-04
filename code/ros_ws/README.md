# ROS 2 Workspace

The ROS 2 workspace is the place to put all plain ROS 2 packages.
The packages are located in `src`.


## Install dependencies
```
make dependencies
```

## Build

Build all packages with
```
make
```

Build all packages for `arm64` with
```
make build-arm64
```


## Clean

Clean the build, installation and log directory with
```
make clean
```

Clean the build and install directory for `arm64` with:
```
make clean-arm64
```


## Board

To use the Makefile with the board, add the following to your `~/.ssh/config`:
```
Host board
    HostName 10.42.0.xxx
    User xilinx
```

Here `xxx` is the IP of the board.

To avoid password prompts run the following:
```
ssh-copy-id -i ~/.ssh/id_ed25519.pub board
```

Here `~/.ssh/id_ed25519.pub` is your public key.


### Copy installation to the board

You can copy the installtion, after building it, to the board with
```
make board-copy
```

### Install dependencies on the board

To install the dependencies for the copied installation on the board, run
```
make board-dependencies
```

### Remove installation from the board

To remove the installation from the board, run
```
make board-clean
```
