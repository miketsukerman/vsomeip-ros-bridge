cmake_minimum_required(VERSION 3.10)

project(gnss_provider VERSION 0.0.2)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(GNUInstallDirs)

find_package(PkgConfig REQUIRED)

pkg_check_modules(gps libgps REQUIRED IMPORTED_TARGET)

find_package(CommonAPI REQUIRED)
find_package(CommonAPI-SomeIP REQUIRED)
find_package(gnss REQUIRED)
find_package(gnss_someip_lib REQUIRED)

find_package(concurrentqueue REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(SERVER_TARGET_NAME gnss-server)
add_executable(${SERVER_TARGET_NAME} 
    src/main.cpp
)

target_include_directories(${SERVER_TARGET_NAME} PRIVATE 
    ${COMMONAPI_INCLUDE_DIRS}
    )

target_link_libraries(${SERVER_TARGET_NAME} PRIVATE
    CommonAPI
    CommonAPI-SomeIP
    gnss::gnss
    rclcpp::rclcpp
    concurrentqueue::concurrentqueue
    PkgConfig::gps
    gnss_someip_lib::gnss-msg__rosidl_generator_cpp    # TODO: figure out how to define exported targets
    gnss_someip_lib::gnss-msg__rosidl_typesupport_cpp    # TODO: figure out how to define exported targets
)

target_compile_features(${SERVER_TARGET_NAME} PRIVATE cxx_std_17)
target_compile_definitions(${SERVER_TARGET_NAME} PRIVATE -DGNSS_SERVER_VERSION="${PROJECT_VERSION}")

install(TARGETS ${SERVER_TARGET_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
