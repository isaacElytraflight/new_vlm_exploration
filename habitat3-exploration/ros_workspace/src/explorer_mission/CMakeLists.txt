cmake_minimum_required(VERSION 3.8)
project(explorer_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(explorer_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

set(EXPLORER_MISSION_LIBS
  frontier_detection
  frontier_tree
  discrete_navigator
  nav2_navigator
)

add_library(frontier_detection src/frontier_detection.cpp)
target_include_directories(frontier_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(frontier_detection nav_msgs geometry_msgs)
target_link_libraries(frontier_detection ${OpenCV_LIBS})

add_library(frontier_tree src/frontier_tree.cpp)
target_include_directories(frontier_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(frontier_tree explorer_msgs geometry_msgs)
target_link_libraries(frontier_tree ${OpenCV_LIBS})

add_library(discrete_navigator src/discrete_navigator.cpp)
target_include_directories(discrete_navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(nav2_navigator src/nav2_navigator.cpp)
target_include_directories(nav2_navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_navigator
  rclcpp rclcpp_action nav2_msgs tf2_geometry_msgs)

set(node_targets
  actions_node
  frontier_vlm_client_node
  explore_node
)

add_executable(actions_node src/actions_node.cpp)
ament_target_dependencies(actions_node
  rclcpp rclcpp_action tf2_ros cv_bridge sensor_msgs explorer_msgs)
target_link_libraries(actions_node ${OpenCV_LIBS})

add_executable(frontier_vlm_client_node src/frontier_vlm_client_node.cpp)
ament_target_dependencies(frontier_vlm_client_node
  rclcpp rclcpp_action sensor_msgs std_msgs explorer_msgs)

add_executable(explore_node src/explore_node.cpp)
target_link_libraries(explore_node frontier_detection frontier_tree discrete_navigator nav2_navigator)
ament_target_dependencies(explore_node
  rclcpp rclcpp_action tf2_ros nav_msgs sensor_msgs std_msgs geometry_msgs
  diagnostic_msgs explorer_msgs)

install(TARGETS
  ${EXPLORER_MISSION_LIBS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${node_targets}
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  explorer_mission/maprender_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME maprender_node)

install(PROGRAMS
  explorer_mission/vlm/vlm_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vlm_node)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_harness test/test_harness.cpp)
  target_include_directories(test_harness PUBLIC include)

  ament_add_gtest(test_frontier_detection test/test_frontier_detection.cpp)
  target_link_libraries(test_frontier_detection frontier_detection)
  target_include_directories(test_frontier_detection PUBLIC include)
  ament_target_dependencies(test_frontier_detection nav_msgs geometry_msgs)

  ament_add_gtest(test_frontier_tree test/test_frontier_tree.cpp)
  target_link_libraries(test_frontier_tree frontier_tree)
  target_include_directories(test_frontier_tree PUBLIC include)
  ament_target_dependencies(test_frontier_tree explorer_msgs geometry_msgs)

  ament_add_gtest(test_discrete_navigator test/test_discrete_navigator.cpp)
  target_link_libraries(test_discrete_navigator discrete_navigator)
  target_include_directories(test_discrete_navigator PUBLIC include)
endif()

ament_package()
