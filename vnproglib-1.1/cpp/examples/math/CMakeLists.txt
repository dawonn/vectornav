cmake_minimum_required(VERSION 2.8.4)
project(math)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

add_subdirectory(../.. libvncxx)

if(CMAKE_COMPILER_ID MATCHES "GNU|Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(SOURCE_FILES
    main.cpp)

include_directories(../../include)

add_executable(math ${SOURCE_FILES})

target_link_libraries(math LINK_PUBLIC libvncxx)
