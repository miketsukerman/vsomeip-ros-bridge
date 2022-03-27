# vsomeip <-> ROS2 bridge PoC

Proof of concept project which tries to create a bridge between vsomeip / common-api
and ROS linux.

# Definition of a problem

## Problem

Make SOME/IP data available in ROS2 natively.

## Architecture

![bridge](docs/bridge.png)

* We would use open source SOME/IP stack implementation (COVESA/GENIVI)
* SOME/IP broadcast's published as ROS2 topics.
# How it works

```bash
    export COMMONAPI_CONFIG=/src/install/gnss_someip_lib/etc/commonapi.ini
    export COMMONAPI_DEFAULT_FOLDER=/src/install/gnss_someip_lib/lib/
```

# Cleanup docker bridge networks

```bash
    docker network prune
```

sample output

```bash
    ➜  vsomeip-ros-bridge git:(master) ✗ docker network prune                                                            
    WARNING! This will remove all custom networks not used by at least one container.
    Are you sure you want to continue? [y/N] y
    Deleted Networks:
    vsomeip-ros-bridge_mynet
```

# Links

* https://colcon.readthedocs.io/en/released/user/how-to.html
* https://hub.docker.com/_/ros
* https://docs.ros.org/en/foxy/Tutorials/Composition.html
* https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html
* https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Nodes.html
