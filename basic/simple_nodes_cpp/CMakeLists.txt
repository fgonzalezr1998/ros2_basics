cmake_minimum_required(VERSION 3.8)
project(simple_nodes_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(iterative_node_cpp src/iterative_node_cpp.cpp)
ament_target_dependencies(iterative_node_cpp rclcpp)

add_executable(timer_node_cpp src/timer_node_cpp.cpp)
ament_target_dependencies(timer_node_cpp rclcpp)

add_executable(simple_pub_node_cpp src/simple_pub_cpp.cpp)
ament_target_dependencies(simple_pub_node_cpp rclcpp std_msgs)

add_executable(simple_sub_node_cpp src/simple_sub_cpp.cpp)
ament_target_dependencies(simple_sub_node_cpp rclcpp std_msgs)

add_executable(simple_sub_node_iterative_cpp src/simple_sub_iterative_cpp.cpp)
ament_target_dependencies(simple_sub_node_iterative_cpp rclcpp std_msgs)

install(TARGETS
  iterative_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  timer_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_pub_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_sub_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_sub_node_iterative_cpp
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
