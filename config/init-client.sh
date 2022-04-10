#!/bin/bash

dlt-daemon &
routingmanagerd &

source /src/install/setup.sh

VSOMEIP_APPLICATION_NAME=gnss-client ros2 run gnss_bridge gnss-bridge & 

ros2 run gnss_listener gnss-listener
