#!/bin/bash

dlt-daemon &
routingmanagerd &

gpsfake -c 1 -v /usr/share/nmea/simulation.nmea &

source /src/install/setup.sh

VSOMEIP_APPLICATION_NAME=gnss-server ros2 run gnss_provider gnss-server
