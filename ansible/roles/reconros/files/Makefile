build: qemu-aarch64-static
	docker image build -t reconros_arm64_galactic:2.0 .

build-pull: qemu-aarch64-static
	docker image build --pull -t reconros_arm64_galactic:2.0 .

qemu-aarch64-static: /usr/bin/qemu-aarch64-static
	cp /usr/bin/qemu-aarch64-static .
