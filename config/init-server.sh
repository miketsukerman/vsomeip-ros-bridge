#!/bin/bash

dlt-daemon &
routingmanagerd &

source /src/install/setup.sh

VSOMEIP_APPLICATION_NAME=gnss-server ros2 run gnss_provider gnss-server
