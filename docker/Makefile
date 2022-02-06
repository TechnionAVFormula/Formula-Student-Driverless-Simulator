.PHONY: all help build run

IMAGE_NAME = fsds_image

all:
	@echo "hello"

help: all

build:
	docker build -f Dockerfile.linux -t fsds_image \
    --network=host \
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
		--mount type=bind,source=$$(pwd)/../settings.json,destination=/home/fsds/Formula-Student-Driverless-Simulator/settings.json \
		--mount type=bind,source=/tmp/.X11-unix,destination=/tmp/.X11-unix \
		fsds_image \
		/bin/bash -c /home/fsds/FSDS.sh -windowed -ResX=1080 -ResY=720
