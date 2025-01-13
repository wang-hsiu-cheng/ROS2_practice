cmake_minimum_required(VERSION 3.5)
project(basic_function)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(communicate_test REQUIRED)

add_executable(talker src/lambda_publish.cpp)
ament_target_dependencies(talker 
  rclcpp 
  std_msgs
)
add_executable(listener src/lambda_subscribe.cpp)
ament_target_dependencies(listener 
  rclcpp 
  std_msgs
)

add_executable(talker1 src/publish.cpp)
ament_target_dependencies(talker1 
  rclcpp 
  std_msgs
)
add_executable(listener1 src/subscribe.cpp)
ament_target_dependencies(listener1 
  rclcpp 
  std_msgs
)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server 
  rclcpp 
  std_msgs 
  communicate_test
)
add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client 
  rclcpp 
  std_msgs 
  communicate_test
)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server 
  rclcpp 
  rclcpp_action 
  rclcpp_components
  std_msgs 
  communicate_test
)
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client 
  rclcpp 
  rclcpp_action 
  rclcpp_components
  std_msgs 
  communicate_test
)

add_executable(frame_broadcaster src/frame_broadcaster.cpp)
ament_target_dependencies(frame_broadcaster 
  rclcpp  
  std_msgs
  geometry_msgs
  tf2_ros
)
add_executable(frame_listener src/frame_listener.cpp)
ament_target_dependencies(frame_listener 
  rclcpp  
  std_msgs
  geometry_msgs
  tf2_ros
)
add_executable(point_transformer src/point_transformer.cpp)
ament_target_dependencies(point_transformer 
  rclcpp  
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  talker
  listener
  talker1
  listener1

  service_server
  service_client

  action_server
  action_client

  frame_broadcaster
  frame_listener
  point_transformer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch/
  # rviz/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
