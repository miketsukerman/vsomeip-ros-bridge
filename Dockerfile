FROM ros:galactic

ENV COMMONAPI_CONFIG=/src/install/gnss_someip_lib/etc/commonapi.ini
ENV COMMONAPI_DEFAULT_FOLDER=/src/install/gnss_someip_lib/lib/

RUN apt-get update && apt install -y wget unzip git

# Installation of required tools
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends tzdata
RUN apt-get install -y openjdk-13-jdk openjdk-13-jre
RUN apt-get install -y g++ gcc cmake libboost-all-dev make doxygen asciidoc
RUN apt-get install -y net-tools iproute2

# Installation of required libraries
RUN apt-get install -y libboost-all-dev nlohmann-json3-dev graphviz source-highlight

RUN wget https://github.com/COVESA/capicxx-core-tools/releases/download/3.2.0.1/commonapi_core_generator.zip -P /opt && \
    cd /opt && unzip commonapi_core_generator.zip -d commonapi_core_generator && \
    ln -s /opt/commonapi_core_generator/commonapi-core-generator-linux-x86_64 /usr/bin/commonapi-core-generator 

RUN wget https://github.com/COVESA/capicxx-someip-tools/releases/download/3.2.0.1/commonapi_someip_generator.zip -P /opt/ && \
    cd /opt/ && unzip commonapi_someip_generator.zip -d commonapi_someip_generator && \
    ln -s /opt/commonapi_someip_generator/commonapi-someip-generator-linux-x86_64 /usr/bin/commonapi-someip-generator

RUN cd /opt && git clone https://github.com/COVESA/dlt-daemon.git && cd dlt-daemon && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j$(nproc) && make install

RUN cd /opt && git clone https://github.com/COVESA/vsomeip.git && cd vsomeip && \
    mkdir build && cd build && cmake -DENABLE_MULTIPLE_ROUTING_MANAGERS=1 -DVSOMEIP_INSTALL_ROUTINGMANAGERD=1 -DCMAKE_INSTALL_PREFIX=/usr .. && \ 
    make -j$(nproc) && make install

RUN cd /opt && git clone https://github.com/COVESA/capicxx-core-runtime.git && cd capicxx-core-runtime && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j$(nproc) && make install

RUN cd /opt && git clone https://github.com/COVESA/capicxx-someip-runtime.git && cd capicxx-someip-runtime && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j$(nproc) && make install

RUN rm -rf /src
COPY . /src 
RUN rm -rf /src/build /src/install /src/log

RUN cd /src && rm -rf build install log && \ 
    . /opt/ros/galactic/setup.sh && \
    colcon build


