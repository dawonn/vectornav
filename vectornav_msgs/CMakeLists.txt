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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommonGroup.msg"
  "msg/TimeGroup.msg"
  "msg/ImuGroup.msg"
  "msg/GpsGroup.msg"
  "msg/AttitudeGroup.msg"
  "msg/InsGroup.msg"
  "msg/TimeUTC.msg"
  "msg/DOP.msg"
  "msg/VpeStatus.msg"
  "msg/TimeStatus.msg"
  "msg/InsStatus.msg"
  DEPENDENCIES geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()