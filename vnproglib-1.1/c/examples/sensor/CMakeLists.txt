cmake_minimum_required(VERSION 2.8.4)
project(sensor)

add_subdirectory(../.. libvnc)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
if (CMAKE_COMPILER_IS_GNUCC)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpedantic -std=c90")
endif()

set(SOURCE_FILES
    main.c)

include_directories("../../include")

add_executable(sensor ${SOURCE_FILES})

target_link_libraries(sensor LINK_PUBLIC libvnc)

if (UNIX OR APPLE)
	target_link_libraries(sensor LINK_PUBLIC pthread)
endif()
