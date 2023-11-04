host := board
all:
	msg sw hw copy run

build-hw:
	faketime -f '2021-4-6 00:00:00' rdk export_hw
	faketime -f '2021-4-6 00:00:00' rdk build_hw

build-sw:
	rdk export_sw
	rdk build_sw

build-sw-x86:
	rdk export_sw
	cd build.sw/ && make

build-msg: msg
	rdk export_msg
	rdk build_msg

build-msg-x86:
	rdk export_msg
	. /opt/ros/galactic/setup.sh && cd build.msg/ && colcon build && rm build -r -f

copy:
	ssh $(host) mkdir -p carcontrol
	ssh $(host) mkdir -p carcontrol/build.msg
	scp -r build.msg/install $(host):~/carcontrol/build.msg
	scp -r build.msg/log $(host):~/carcontrol/build.msg
	scp build.hw/myReconOS.runs/impl_1/design_1_wrapper.bin $(host):~/carcontrol
	scp build.sw/carcontrol $(host):~/carcontrol
	scp run_software.sh $(host):~/carcontrol
	scp load_fpga.sh $(host):~/carcontrol

run:
	ssh -t $(host) sudo /home/xilinx/carcontrol/load_fpga.sh /home/xilinx/carcontrol/
	ssh -t $(host) sudo /home/xilinx/carcontrol/run_software.sh

run-x86:
	. build.msg/install/local_setup.sh && build.sw/carcontrol
