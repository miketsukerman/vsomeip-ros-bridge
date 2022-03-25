cmake_minimum_required(VERSION 3.10)

project(vsomeip-ros-bridge VERSION 0.1)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(GNUInstallDirs)

find_package(CommonAPITools)
find_package(CommonAPI REQUIRED)
find_package(CommonAPI-SomeIP REQUIRED)

add_custom_command(
    DEPENDS 
        ${CMAKE_CURRENT_SOURCE_DIR}/source/idl/gnss.fidl 
        ${CMAKE_CURRENT_SOURCE_DIR}/source/idl/types.fidl 
    OUTPUT
        ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServer.hpp
        ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerProxyBase.hpp
        ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerProxy.hpp
        ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerStub.hpp
    COMMAND
        ${COMMON_API_CORE_GENERATOR} -sk ${CMAKE_CURRENT_SOURCE_DIR}/source/idl/gnss.fidl -d ${CMAKE_BINARY_DIR}/gen
)

add_custom_command(
    DEPENDS 
        ${CMAKE_CURRENT_SOURCE_DIR}/source/idl/gnss.fdepl 
        ${CMAKE_CURRENT_SOURCE_DIR}/source/idl/types.fidl 
    OUTPUT
        ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPProxy.cpp
        ${CMAKE_BINARY_DIR}/gen/v0/gnss/commonSomeIPDeployment.cpp
        ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPDeployment.cpp
        ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPStubAdapter.cpp
    COMMAND
        ${COMMON_API_SOMEIP_GENERATOR} ${CMAKE_CURRENT_SOURCE_DIR}/source/idl/gnss.fdepl -d ${CMAKE_BINARY_DIR}/gen
)

add_executable(client 
    source/client/main.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/commonSomeIPDeployment.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPDeployment.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPProxy.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServer.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerProxyBase.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerProxy.hpp
)

target_include_directories(client PRIVATE 
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/gen>
    ${COMMONAPI_INCLUDE_DIRS}
)

target_link_libraries(client PRIVATE
    CommonAPI
    CommonAPI-SomeIP
)

add_executable(server 
    source/server/main.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/commonSomeIPDeployment.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPStubAdapter.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerStub.hpp
)

target_include_directories(server PRIVATE 
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/gen>
    ${COMMONAPI_INCLUDE_DIRS}
    )

target_link_libraries(server PRIVATE
    CommonAPI
    CommonAPI-SomeIP
)

install(TARGETS client DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS server DESTINATION ${CMAKE_INSTALL_BINDIR})
