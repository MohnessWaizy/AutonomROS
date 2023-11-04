host := board
.PHONY: build build-amd64 build-arm64 clean clean-amd64 clean-arm64 board-copy

build: build-amd64

build-amd64:
	colcon build

build-arm64:
	ros_cross_compile . \
	  --rosdistro galactic \
	  --arch aarch64 \
	  --os ubuntu \
	  --custom-setup-script ./build_helpers/pre-install.sh \
	  --colcon-defaults ./defaults.yaml

dependencies:
	rosdep install -y -i --from-paths src

clean: clean-amd64

clean-amd64:
	rm -rf build/ install/ log/

clean-arm64:
	rm -rf build_aarch64/ install_aarch64/

board-copy:
	ssh $(host) mkdir -p /home/xilinx/ros/
	scp -r install_aarch64/ $(host):/home/xilinx/ros/

board-dependencies:
	ssh $(host) rosdep install --from-paths /home/xilinx/ros/install_aarch64/share --ignore-src --rosdistro galactic -y

board-clean:
	ssh $(host) rm -rf /home/xilinx/ros/
