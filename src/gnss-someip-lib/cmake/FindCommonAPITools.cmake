if(NOT COMMON_API_GENERATORS_FOUND)

    if(NOT DEFINED COMMON_API_CORE_GENERATOR)
        find_program(COMMON_API_CORE_GENERATOR NAMES commonapi-core-generator)
        message(STATUS "Core generator: ${COMMON_API_CORE_GENERATOR}")
    endif()

    if(NOT DEFINED COMMON_API_SOMEIP_GENERATOR)
        find_program(COMMON_API_SOMEIP_GENERATOR NAMES commonapi-someip-generator)
        message(STATUS "SOME/IP generator: ${COMMON_API_SOMEIP_GENERATOR}")
    endif()

    find_package(PackageHandleStandardArgs REQUIRED)

    find_package_handle_standard_args(CommonAPIGenerators DEFAULT_MSG COMMON_API_CORE_GENERATOR COMMON_API_SOMEIP_GENERATOR)

    mark_as_advanced(COMMON_API_GENERATORS_FOUND COMMON_API_CORE_GENERATOR COMMON_API_SOMEIP_GENERATOR)
endif()