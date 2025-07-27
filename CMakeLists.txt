cmake_minimum_required(VERSION 3.8)
project(CollectionServer)

set(CMAKE_CXX_STANDARD 11)

# 主项目头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include)

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
)
