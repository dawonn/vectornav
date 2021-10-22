cmake_minimum_required(VERSION 3.16)
project(vncxx VERSION 1.1.5.0 LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # TODO[DERECK] eliminate warnings in vncxx
    # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

add_library(${PROJECT_NAME} SHARED
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
    src/rtcmlistener.cpp
    src/rtcmmessage.cpp
    src/searcher.cpp
    src/sensors.cpp
    src/serialport.cpp
    src/thread.cpp
    src/types.cpp
    src/util.cpp
    src/utilities.cpp
    src/vntime.cpp
)

target_include_directories(
    ${PROJECT_NAME} PUBLIC include
)

install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/vn DESTINATION include)
