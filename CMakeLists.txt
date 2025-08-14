cmake_minimum_required(VERSION 3.8)
project(CollectionServer)

# --- 添加下面两行 ---
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV 找到，版本: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "未找到 OpenCV，请确保已安装并配置环境变量")
endif()
## 查找 libusb
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
# 主项目头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${LIBUSB_INCLUDE_DIRS})

# 添加benewakeLidar和 tanwaylidar 两个子目录，调用它们的CMakeLists.txt
add_subdirectory(benewakeLidar)
add_subdirectory(tanwaylidar)

# 主项目源文件
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(CollectionServer ${SRC_FILES})

# 链接两个库（假设它们都生成了名为benewake_lidar_driver和 tanwaylidar 的库）
target_link_libraries(CollectionServer
    benewake_lidar_driver
    tanwaylidar
    ${OpenCV_LIBS}
        ${LIBUSB_LIBRARIES}
)
