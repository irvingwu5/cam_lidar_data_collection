cmake_minimum_required (VERSION 3.8)

project ("record_pcap")

SET(SRC ${PROJECT_SOURCE_DIR}/record_pcap.cpp
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread")


LINK_DIRECTORIES(
)

INCLUDE_DIRECTORIES (
	${CMAKE_SOURCE_DIR}/include
)

add_executable (${PROJECT_NAME}
	${SRC}
)

TARGET_LINK_LIBRARIES (record_pcap 
	libpcap.so
)