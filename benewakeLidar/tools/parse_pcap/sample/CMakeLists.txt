cmake_minimum_required (VERSION 3.8)
project ("sample")

set(CMAKE_CXX_STANDARD 11)

include_directories(
    ${PROJECT_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/include
)

link_directories(${PROJECT_SOURCE_DIR}/../build/Release)

add_executable(${PROJECT_NAME} main.cpp)

IF (WIN32)
	TARGET_LINK_LIBRARIES (sample 
		${PROJECT_SOURCE_DIR}/../build/Release/parse_pcap.lib
	)
ELSEIF (UNIX)
	TARGET_LINK_LIBRARIES (sample 
		${PROJECT_SOURCE_DIR}/../build/libparse_pcap.so
	)
ENDIF ()