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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# vncxx
add_subdirectory(vnproglib-1.2.0.0/cpp)

# vectornav library (with composable node)
add_library(${PROJECT_NAME}_lib SHARED src/vectornav.cc)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp rclcpp_components geometry_msgs vectornav_msgs rclcpp_action)
target_link_libraries(${PROJECT_NAME}_lib vncxx)
rclcpp_components_register_nodes(${PROJECT_NAME}_lib "vectornav::Vectornav")

# vectornav node
add_executable(${PROJECT_NAME} src/vectornav_node.cc)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs vectornav_msgs)
target_link_libraries(${PROJECT_NAME} vncxx ${PROJECT_NAME}_lib)

# vn_sensor_msgs library (with composable node)
add_library(vn_sensor_msgs_lib SHARED src/vn_sensor_msgs.cc)
ament_target_dependencies(vn_sensor_msgs_lib
  rclcpp rclcpp_components sensor_msgs vectornav_msgs tf2_geometry_msgs)
rclcpp_components_register_nodes(vn_sensor_msgs_lib "vectornav::VnSensorMsgs")

# vn_sensor_msgs node
add_executable(vn_sensor_msgs src/vn_sensor_msgs_node.cc)
ament_target_dependencies(vn_sensor_msgs rclcpp geometry_msgs vectornav_msgs)
target_link_libraries(vn_sensor_msgs vn_sensor_msgs_lib)


# install shared libraries with composable nodes into global
# directory so they can be loaded by other packages
install(TARGETS
  ${PROJECT_NAME}_lib
  vn_sensor_msgs_lib
  DESTINATION lib)

# install executables into project specific folder
install(TARGETS
  ${PROJECT_NAME}
  vn_sensor_msgs
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
