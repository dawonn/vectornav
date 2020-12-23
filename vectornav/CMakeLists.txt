cmake_minimum_required(VERSION 3.5)
project(vectornav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# TODO[DERECK] fix library warnings
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# endif()

# find dependencies
# TODO[DERECK] Move to auto-dep macro
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)

# vncxx
add_subdirectory(vnproglib-1.1.5.0/cpp)

# vectornav
add_executable(${PROJECT_NAME} src/vectornav.cc)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs vectornav_msgs)
target_link_libraries(${PROJECT_NAME} vncxx )

# vn_sensor_msgs
add_executable(vn_sensor_msgs src/vn_sensor_msgs.cc)
ament_target_dependencies(vn_sensor_msgs rclcpp sensor_msgs vectornav_msgs)

# install
install(TARGETS
  ${PROJECT_NAME}
  vn_sensor_msgs
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()