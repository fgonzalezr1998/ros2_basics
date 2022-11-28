cmake_minimum_required(VERSION 3.5)
project(rc_tb2)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
)

include_directories(include)

add_library(RcTb2 SHARED src/rc_tb2/RcTb2.cpp)
add_library(RcActions SHARED src/rc_actions/RcActions.cpp)

target_link_libraries(RcTb2 RcActions)
ament_target_dependencies(RcTb2 ${dependencies})

add_executable(xbox_rc_tb2_node src/xbox_rc_tb2_node.cpp)
target_link_libraries(xbox_rc_tb2_node RcTb2)
ament_target_dependencies(xbox_rc_tb2_node ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS RcTb2 RcActions xbox_rc_tb2_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
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
