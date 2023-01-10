cmake_minimum_required(VERSION 3.8)
project(cpp_pkg_template)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(dummy_cpp_node src/cpp_node_template.cpp)
ament_target_dependencies(dummy_cpp_node rclcpp)

add_executable(dummy_publisher_cpp_node src/cpp_publisher_template.cpp)
ament_target_dependencies(dummy_publisher_cpp_node rclcpp example_interfaces)

add_executable(dummy_subscriber_cpp_node src/cpp_subscriber_template.cpp)
ament_target_dependencies(dummy_subscriber_cpp_node rclcpp example_interfaces)

install(TARGETS
  dummy_cpp_node
  dummy_publisher_cpp_node
  dummy_subscriber_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
