cmake_minimum_required (VERSION 3.8)

project ("parse_pcap")

SET(SRC ${PROJECT_SOURCE_DIR}/src/parse_pcap.cpp
)
        	
SET(INC ${PROJECT_SOURCE_DIR}/include/benewake_common.h	
		${PROJECT_SOURCE_DIR}/include/benewake_tables.h	
		${PROJECT_SOURCE_DIR}/include/parse_pcap.h
)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread")


LINK_DIRECTORIES(
)

ADD_DEFINITIONS (
	-DBENEWAKE_DLL_EXPORT
)

INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/include
)

ADD_LIBRARY (parse_pcap SHARED
	${SRC} ${INC}
)

SET_TARGET_PROPERTIES (parse_pcap PROPERTIES PUBLIC_HEADER 
	"include/benewake_common.h;include/benewake_tables.h;include/parse_pcap.h")

if(WIN32)
INSTALL (TARGETS parse_pcap 
	PUBLIC_HEADER DESTINATION include
	RUNTIME DESTINATION lib
)
else()
INSTALL (TARGETS parse_pcap 
	PUBLIC_HEADER DESTINATION include
	LIBRARY DESTINATION lib
)
endif()
