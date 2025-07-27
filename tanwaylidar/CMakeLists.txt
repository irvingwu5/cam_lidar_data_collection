cmake_minimum_required(VERSION 3.8)
project(tanwaylidar)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -Wall -pthread")

set(SRC
    demo/Demo_UseSDK.cpp
    lidar/LidarDevice.cpp
    utils/PackageList.cpp
    about/about.cpp
    log/twlog.cpp
)

add_library(tanwaylidar SHARED ${SRC})

target_compile_definitions(tanwaylidar PRIVATE
    DEBUG_STRING=""
    DEBUG_MODE=false
    VERSION_NUMBER="2.0.5"
)
target_include_directories(tanwaylidar PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/about
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/log
)

if(WIN32)
    target_link_libraries(tanwaylidar
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/algo-AMD64.lib
    )
else()
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        target_link_libraries(tanwaylidar
            pthread
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/libalgo-x86_64.a
        )
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_link_libraries(tanwaylidar
            pthread
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/libalgo-aarch64.a
        )
    else()
        message(STATUS "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
endif()
