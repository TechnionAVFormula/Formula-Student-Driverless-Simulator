.PHONY: all help build run

IMAGE_NAME = fsds_image
export USERNAME=fsds
export HOST_UID=$(shell id -u)

all:
	@echo "usage: make [COMMAND]"
	@echo
	@echo "COMMAND options:"
	@echo "    build"
	@echo "        - builds the fsds image"
	@echo "    run xauthority_path=<path/to/Xauthority>"
	@echo "        - runs the simulator"

help: all

build:
	docker build -f Dockerfile.linux -t $(IMAGE_NAME) \
    --network=host \
	--build-arg USERNAME=$(USERNAME) \
	--build-arg HOST_UID=$(HOST_UID) \
    ..

run:
	docker run \
		-it \
		--rm \
		--privileged \
		--net=host \
		--gpus=all \
		--env=DISPLAY=$(DISPLAY) \
		--env=QT_X11_NO_MITSHM=1 \
		--env=SDL_HINT_CUDA_DEVICE='0' \
		--mount type=bind,source=$$(pwd)/../settings.json,destination=/home/$(USERNAME)/Formula-Student-Driverless-Simulator/settings.json \
		--mount type=bind,source=/tmp/.X11-unix,destination=/tmp/.X11-unix \
		--mount type=bind,source=$(xauthority_path),destination=/home/$(USERNAME)/.Xauthority \
		$(IMAGE_NAME) \
		/bin/bash -c /home/$(USERNAME)/FSDS.sh -windowed -ResX=1080 -ResY=720
