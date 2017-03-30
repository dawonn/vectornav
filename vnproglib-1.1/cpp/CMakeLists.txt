cmake_minimum_required(VERSION 2.8.4)
project(vncxx CXX)

set(CMAKE_SUPPRESS_REGENERATION TRUE)
#add_definitions(-DPL150)
#
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#option(BUILD_TESTS "Build tests." OFF)
#option(BUILD_BENCHMARKS "Build benchmarks." OFF)
#option(PYTHON "Build for Python library." OFF)
#option(BUILD_GRAPHICS "Build in the graphics library." OFF)

#set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(SOURCE
        src/attitude.cpp
        src/compositedata.cpp
        src/conversions.cpp
        src/criticalsection.cpp
        src/dllvalidator.cpp
        src/error_detection.cpp
        src/event.cpp
        src/ezasyncdata.cpp
        src/memoryport.cpp
        src/packet.cpp
        src/packetfinder.cpp
        src/port.cpp
        src/position.cpp
        src/searcher.cpp
        src/sensors.cpp
        src/serialport.cpp
        src/thread.cpp
        src/types.cpp
        src/util.cpp
        src/utilities.cpp
        src/vntime.cpp
        include/vn/exceptions.h
        include/vn/matrix.h
        include/vn/attitude.h
        include/vn/boostpython.h
        include/vn/dllvalidator.h
        include/vn/signal.h
        include/vn/error_detection.h
        include/vn/position.h
        include/vn/registers.h
        include/vn/utilities.h
        include/vn/memoryport.h
        include/vn/nocopy.h
        include/vn/compositedata.h
        include/vn/criticalsection.h
        include/vn/compiler.h
        include/vn/sensors.h
        include/vn/searcher.h
        include/vn/event.h
        include/vn/ezasyncdata.h
        include/vn/serialport.h
        include/vn/export.h
        include/vn/vector.h
        include/vn/vntime.h
        include/vn/packetfinder.h
        include/vn/conversions.h
        include/vn/types.h
        include/vn/int.h
        include/vn/thread.h
        include/vn/mock.h
        include/vn/port.h
        include/vn/util.h
        include/vn/consts.h
        include/vn/packet.h)

include_directories(
    include)

add_library(vncxx ${SOURCE})

#add_subdirectory(examples/ez_async_data)
#add_subdirectory(examples/getting_started)
#add_subdirectory(examples/math)
#add_subdirectory(examples/uart_protocol)

#
#if (BUILD_GRAPHICS)
#
#	include_directories(
#		../libs/glew/include
#		../libs/glm)
#
#
#	#target_link_libraries(proglib-cpp-graphics PRIVATE GLEW_shared)
#
##	set(CMAKE_PREFIX_PATH "../libs/glew")
##	set(CMAKE_LIBRARY_PATH "../libs/glew/lib/Release/Win32/")
#
#	#message("DIR: ${GLEW_INCLUDE_DIR}")
##	find_package(GLEW REQUIRED)
#	#add_library(GLEW_shared INTERFACE IMPORTED)
#	#message("DIR: ${GLEW_INCLUDE_DIR}")
#
#	#message("${GLEW_FOUND}")
#
##	include_directories(${GLEW_INCLUDE_DIRS})
##    link_libraries(${GLEW_LIBRARIES})
#
#
##    add_subdirectory(../libs/glew GLEW_shared)
#
#
#	add_library(proglib-cpp-graphics SHARED
#		${graphics_sources}
#		${graphics_math_sources})
#	target_compile_definitions(proglib-cpp-graphics PUBLIC -D_BUILD_DLL)
#	target_compile_definitions(proglib-cpp-graphics PUBLIC -Dproglib_cpp_graphics_EXPORTS)
#	target_link_libraries(proglib-cpp-graphics PRIVATE OpenGL32)
#	target_link_libraries(proglib-cpp-graphics PRIVATE GLEW_shared)
#
#
#	
#
#
#endif()
#
##if (WIN32)
##	target_link_libraries(proglib-cpp PRIVATE Setupapi)
#
##	if (PYTHON)
##		include_directories(../libs/PeLib/include)
##		target_link_libraries(proglib-cpp LINK_PUBLIC PeLib)
##		add_subdirectory(../libs/PeLib PeLib)
##	endif()
##endif()
#
#
## The properties below are mainly for the Python library.
##if (WIN32)
##set_target_properties(
##	proglib-cpp PROPERTIES
##	RUNTIME_OUTPUT_DIRECTORY "../../../vnpy"
##	RUNTIME_OUTPUT_DIRECTORY_RELEASE "../../../vnpy"
##	RUNTIME_OUTPUT_DIRECTORY_DEBUG "../../../vnpy"
##	LIBRARY_OUTPUT_DIRECTORY "../../../vnpy")
##elseif(UNIX)
##set_target_properties(
##	proglib-cpp PROPERTIES
##	RUNTIME_OUTPUT_DIRECTORY "../vnpy"
##	RUNTIME_OUTPUT_DIRECTORY_RELEASE "../vnpy"
##	RUNTIME_OUTPUT_DIRECTORY_DEBUG "../vnpy"
##	LIBRARY_OUTPUT_DIRECTORY "../vnpy")
##endif()
#
#if (BUILD_TESTS)
#
#    file(GLOB_RECURSE TEST_SOURCE_FILES src/vn/**/*.test.cpp)
#    set(TEST_SOURCE_FILES ${TEST_SOURCE_FILES} src/vn/main.cpp)
#
#	set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries (DLLs).")
#	add_subdirectory(../libs/googletest gtest)
#
#	include_directories(../libs/googletest/include)
#
#	add_executable(proglib-cpp-test ${TEST_SOURCE_FILES})
#
#	target_link_libraries(proglib-cpp-test LINK_PUBLIC gtest)
#	target_link_libraries(proglib-cpp-test LINK_PUBLIC proglib-cpp)
#
#endif()
#
#if (BUILD_BENCHMARKS)
#
#    file(GLOB_RECURSE BENCHMARK_SOURCE_FILES src/vn/**/*.benchmark.cpp)
#    set(BENCHMARK_SOURCE_FILES ${BENCHMARK_SOURCE_FILES} src/vn/benchmark.cpp)
#
#	add_subdirectory(../libs/hayai hayai)
#	add_subdirectory(../libs/glew GLEW_shared)
#
#	include_directories(../libs/hayai/src)
#
#	add_executable(proglib-cpp-benchmark ${BENCHMARK_SOURCE_FILES})
#
#	#target_link_libraries(proglib-cpp-benchmark LINK_PUBLIC hayai)
#	target_link_libraries(proglib-cpp-benchmark LINK_PUBLIC proglib-cpp)
#	target_link_libraries(proglib-cpp-benchmark LINK_PUBLIC proglib-cpp-graphics)
#	target_link_libraries(proglib-cpp-graphics PRIVATE OpenGL32)
#	target_link_libraries(proglib-cpp-graphics PRIVATE GLEW_shared)
#
#endif()
#
