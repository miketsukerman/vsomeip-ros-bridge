if(NOT FRANCA_ROS_TOOLS_FOUND)

    if(NOT DEFINED FIDL2ROS_TRANSLATOR)
        find_program(FIDL2ROS_TRANSLATOR NAMES fidl2ros)
        message(STATUS "fidl2ros translator: ${FIDL2ROS_TRANSLATOR}")
    endif()

    find_package(PackageHandleStandardArgs REQUIRED)

    find_package_handle_standard_args(FrancaRosTools DEFAULT_MSG FIDL2ROS_TRANSLATOR)

    mark_as_advanced(FRANCA_ROS_TOOLS_FOUND FIDL2ROS_TRANSLATOR)
endif()
