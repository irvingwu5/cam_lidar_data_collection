# Install script for directory: /home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbenewake_lidar_driver.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbenewake_lidar_driver.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbenewake_lidar_driver.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/wxy/Documents/CppProjects/camera_lidar_driver/cmake-build-debug/benewakeLidar/libbenewake_lidar_driver.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbenewake_lidar_driver.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbenewake_lidar_driver.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbenewake_lidar_driver.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_hornx2_driver.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_lidar_driver.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_common.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_dcsp.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_dsop.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_mdop.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_protocol.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_udp.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/msg_queue.h"
    "/home/wxy/Documents/CppProjects/camera_lidar_driver/benewakeLidar/include/benewake_tables.h"
    )
endif()

