# benewakeLidar/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project("benewake_lidar_driver")

# 仅保留源文件（移除头文件，头文件无需编译）
set(SRC 
    src/benewake_hornx2_driver.cpp
    src/benewake_lidar_driver.cpp
    src/benewake_dcsp.cpp
    src/benewake_dsop.cpp
    src/benewake_mdop.cpp
    src/benewake_protocol.cpp
    src/benewake_udp.cpp
)

# 设置 C++ 标准（与主项目一致，避免冲突）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加编译选项（-O3、-pthread 等）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")

# 生成共享库
add_library(benewake_lidar_driver SHARED ${SRC})

# 声明头文件路径（关键！）
# PRIVATE：库内部使用的头文件路径（如实现细节）
# PUBLIC：依赖该库的目标（如主项目）需要的头文件路径
target_include_directories(benewake_lidar_driver
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include  # 子项目内部使用（如源文件包含头文件）
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include  # 主项目链接时需要此路径
)

# 设置公共头文件（安装时使用，可选）
set_target_properties(benewake_lidar_driver PROPERTIES 
    PUBLIC_HEADER 
        "include/benewake_hornx2_driver.h;include/benewake_lidar_driver.h;include/benewake_common.h;include/benewake_dcsp.h;include/benewake_dsop.h;include/benewake_mdop.h;include/benewake_protocol.h;include/benewake_udp.h;include/msg_queue.h;include/benewake_tables.h"
)

# 安装规则（根据系统调整）
if(WIN32)
    install(TARGETS benewake_lidar_driver
        PUBLIC_HEADER DESTINATION include
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
    )
else()
    install(TARGETS benewake_lidar_driver
        PUBLIC_HEADER DESTINATION include
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
endif()