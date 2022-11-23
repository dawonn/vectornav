cmake_minimum_required(VERSION 3.5)
project(vectornav_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Grab all message files 
file(GLOB msg_sources RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
file(GLOB act_sources RELATIVE ${PROJECT_SOURCE_DIR} action/*.action)

message("Grabbing messages ${act_sources}")


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_sources}
  ${act_sources}
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()