all: run

compose:
	docker build . -t vsomeip-ros-bridge:base
	docker-compose build

run: compose
	docker-compose run simulation

.PHONY: compose 
