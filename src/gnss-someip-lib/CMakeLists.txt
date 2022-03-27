cmake_minimum_required(VERSION 3.10)

project(gnss_someip_lib VERSION 0.0.1)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(ament_cmake REQUIRED)

find_package(CommonAPITools REQUIRED)
find_package(CommonAPI REQUIRED)
find_package(CommonAPI-SomeIP REQUIRED)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(GNSS_FILES 
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/common.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServer.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerProxyBase.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerProxy.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerStub.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPProxy.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/commonSomeIPDeployment.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/commonSomeIPDeployment.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPDeployment.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPDeployment.hpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerSomeIPStubAdapter.cpp
    ${CMAKE_BINARY_DIR}/gen/v0/gnss/TimeServerStubDefault.hpp
)

add_custom_command(
    OUTPUT
        ${GNSS_FILES}
    COMMAND
        ${COMMON_API_CORE_GENERATOR} -sk ${CMAKE_CURRENT_SOURCE_DIR}/src/gnss.fidl -d ${CMAKE_BINARY_DIR}/gen
    COMMAND
        ${COMMON_API_SOMEIP_GENERATOR} ${CMAKE_CURRENT_SOURCE_DIR}/src/gnss.fdepl -d ${CMAKE_BINARY_DIR}/gen
    DEPENDS 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gnss.fidl 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/types.fidl 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gnss.fdepl 
)

set(GNSS_TARGET_NAME gnss)
add_library(${GNSS_TARGET_NAME} SHARED ${GNSS_FILES})
target_link_libraries(${GNSS_TARGET_NAME} PUBLIC 
    CommonAPI
    CommonAPI-SomeIP
)

set(GNSS_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR}/gnss-${PROJECT_VERSION_MAJOR}/)

target_include_directories(${GNSS_TARGET_NAME} 
    PRIVATE 
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/gen>
        ${COMMONAPI_INCLUDE_DIRS}
    PUBLIC 
        $<INSTALL_INTERFACE:${GNSS_INSTALL_INCLUDEDIR}>
)

set_target_properties(${GNSS_TARGET_NAME} PROPERTIES 
                              VERSION ${PROJECT_VERSION}
                            SOVERSION ${PROJECT_VERSION_MAJOR})

install(TARGETS 
        ${GNSS_TARGET_NAME} 
    EXPORT 
        ${GNSS_TARGET_NAME}Config
    DESTINATION 
        ${CMAKE_INSTALL_LIBDIR}
    )

install(DIRECTORY ${CMAKE_BINARY_DIR}/gen/ DESTINATION ${GNSS_INSTALL_INCLUDEDIR})

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${GNSS_TARGET_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

set(GNSS_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${GNSS_TARGET_NAME}-${PROJECT_VERSION_MAJOR}/)

install(FILES 
        ${CMAKE_CURRENT_BINARY_DIR}/${GNSS_TARGET_NAME}ConfigVersion.cmake 
    DESTINATION 
        ${GNSS_INSTALL_CMAKEDIR}
    )

install(EXPORT 
        ${GNSS_TARGET_NAME}Config
    NAMESPACE 
        ${GNSS_TARGET_NAME}::
    DESTINATION 
        ${GNSS_INSTALL_CMAKEDIR}
)

export(TARGETS 
        ${GNSS_TARGET_NAME}
    NAMESPACE 
        ${GNSS_TARGET_NAME}::
    FILE 
        ${GNSS_TARGET_NAME}Config.cmake
)

configure_file(config/commonapi.ini.in ${CMAKE_BINARY_DIR}/commonapi.ini)

install(FILES ${CMAKE_BINARY_DIR}/commonapi.ini DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})

ament_package()
