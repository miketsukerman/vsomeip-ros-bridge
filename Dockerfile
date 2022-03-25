FROM ros:noetic-ros-core-focal

RUN apt-get update && apt install -y wget unzip git

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends tzdata
RUN apt-get install -y openjdk-13-jdk openjdk-13-jre
RUN apt-get install -y g++ gcc cmake libboost-all-dev make

RUN wget https://github.com/COVESA/capicxx-core-tools/releases/download/3.2.0.1/commonapi_core_generator.zip -P /opt && \
    cd /opt && unzip commonapi_core_generator.zip -d commonapi_core_generator && \
    ln -s /opt/commonapi_core_generator/commonapi-core-generator-linux-x86_64 /usr/bin/commonapi-core-generator 

RUN wget https://github.com/COVESA/capicxx-someip-tools/releases/download/3.2.0.1/commonapi_someip_generator.zip -P /opt/ && \
    cd /opt/ && unzip commonapi_someip_generator.zip -d commonapi_someip_generator && \
    ln -s /opt/commonapi_someip_generator/commonapi-someip-generator-linux-x86_64 /usr/bin/commonapi-someip-generator

RUN cd /opt && git clone https://github.com/COVESA/dlt-daemon.git && cd dlt-daemon && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j$(nproc) && make install

RUN cd /opt && git clone https://github.com/COVESA/vsomeip.git && cd vsomeip && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j$(nproc) && make install

RUN cd /opt && git clone https://github.com/COVESA/capicxx-core-runtime.git && cd capicxx-core-runtime && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j$(nproc) && make install

RUN cd /opt && git clone https://github.com/COVESA/capicxx-someip-runtime.git && cd capicxx-someip-runtime && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j$(nproc) && make install

COPY . /src
RUN cd /src && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make install && rm -rf /src
