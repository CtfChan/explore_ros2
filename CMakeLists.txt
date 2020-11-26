cmake_minimum_required(VERSION 3.5)
project(explore_ros2)

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
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(
  include
)

set(dependencies
  rclcpp
  nav2_costmap_2d
  nav_msgs
  map_msgs
  geometry_msgs
  visualization_msgs
)

# library
add_library(explore_ros2_core SHARED
  src/utils.cpp
  src/costmap.cpp
  src/frontier_search.cpp
)
ament_target_dependencies(explore_ros2_core ${dependencies})
install(TARGETS explore_ros2_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# exec
add_executable(explorer 
  src/explorer.cpp
  src/utils.cpp
  src/costmap.cpp
  src/frontier_search.cpp
)
ament_target_dependencies(explorer ${dependencies})
target_include_directories(explorer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS explorer
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
