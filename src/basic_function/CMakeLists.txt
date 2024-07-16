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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/lambda_publish.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/lambda_subscribe.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
add_executable(talker1 src/publish.cpp)
ament_target_dependencies(talker1 rclcpp std_msgs)
add_executable(listener1 src/subscribe.cpp)
ament_target_dependencies(listener1 rclcpp std_msgs)
# add_executable(talker2 src/callback_publish.cpp)
# ament_target_dependencies(talker2 rclcpp std_msgs)

install(TARGETS
  talker
  listener
  talker1
  listener1
  # talker2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
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
