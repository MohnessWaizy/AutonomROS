host := board
all:
	msg sw hw copy run

hw:
	faketime -f '-2.0y' rdk export_hw
	faketime -f '-2.0y' rdk build_hw

sw:
	rdk export_sw
	rdk build_sw

msg: build.cfg
	rdk export_msg
	rdk build_msg

copy:
	ssh $(host) mkdir -p pwmcontrol
	scp build.hw/myReconOS.runs/impl_1/design_1_wrapper.bin $(host):~/pwmcontrol
	scp build.sw/carcontrol $(host):~/pwmcontrol
	scp run_software.sh $(host):~/pwmcontrol
	scp loadFPGA.sh $(host):~/pwmcontrol

run:
	ssh -t $(host) sudo /home/xilinx/pwmcontrol/loadFPGA.sh /home/xilinx/pwmcontrol/
	ssh -t $(host) sudo /home/xilinx/pwmcontrol/run_software.sh
