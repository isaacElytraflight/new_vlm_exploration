cmake_minimum_required(VERSION 3.8)
project(explorer_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(explorer_msgs REQUIRED)

include_directories(include)

add_library(pc_occupancy
  src/occupancy_map.cpp
  src/pc_occupancy.cpp
)
target_include_directories(pc_occupancy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(known_pose_pc_mapper_node src/known_pose_pc_mapper_node.cpp)
target_link_libraries(known_pose_pc_mapper_node pc_occupancy)
ament_target_dependencies(known_pose_pc_mapper_node
  rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs)

install(TARGETS pc_occupancy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS known_pose_pc_mapper_node
  DESTINATION lib/${PROJECT_NAME})

# Python nodes (ament_cmake_python installs the package; scripts via PROGRAMS).
install(PROGRAMS
  explorer_bridge/explorer_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME explorer_bridge_node)
install(PROGRAMS
  explorer_bridge/habitat_map_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME habitat_map_node)
install(PROGRAMS
  explorer_bridge/depth_camera_info_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME depth_camera_info_node)
install(PROGRAMS
  explorer_bridge/depth_to_laserscan_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME depth_to_laserscan_node)
install(PROGRAMS
  explorer_bridge/known_pose_mapper_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME known_pose_mapper_node)
install(PROGRAMS
  explorer_bridge/cmd_vel_to_discrete_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_to_discrete_node)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pc_occupancy test/test_pc_occupancy.cpp)
  target_link_libraries(test_pc_occupancy pc_occupancy)
  target_include_directories(test_pc_occupancy PUBLIC include)

  ament_add_gtest(test_pose_gate test/test_pose_gate.cpp)
  target_include_directories(test_pose_gate PUBLIC include)
endif()

ament_package()
