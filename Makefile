all: run

enter-build-env:
	docker run -it -v $(shell pwd):/src -w /src vsomeip-ros-bridge:base /bin/bash

compose:
	docker build . -t vsomeip-ros-bridge:base
	docker-compose build

run: compose
	docker-compose up

run-force: compose 
	docker-compose up --force-recreate

clean: 
	rm -rf build install log

colcon-build:
	colcon build

gnss-provider:
	colcon build --packages-select gnss_provider

gnss-bridge:
	colcon build --packages-select gnss_bridge

gnss-someip-lib:
	colcon build --packages-select gnss_someip_lib

gnss-listener:
	colcon build --packages-select gnss_listener

.PHONY: compose run enter-build-env build-gnss-provider
