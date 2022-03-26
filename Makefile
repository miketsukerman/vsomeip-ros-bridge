all: run

enter-build-env:
	docker run -it -v $(shell pwd):/src -w /src vsomeip-ros-bridge:base /bin/bash

compose:
	docker build . -t vsomeip-ros-bridge:base
	docker-compose build

run: compose
	docker-compose up

.PHONY: compose run enter-build-env
