cmake_minimum_required(VERSION 3.8)
project(explorer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frontier.msg"
  "msg/FrontierArray.msg"
  "msg/FrontierViews.msg"
  "msg/FrontierBlacklist.msg"
  "msg/Vertex.msg"
  "msg/Edge.msg"
  "msg/Graph.msg"
  "srv/AddEdge.srv"
  "srv/RunDijkstra.srv"
  "action/DiscreteMove.action"
  "action/FrontierViewsProcess.action"
  "action/PerceiveAndCapture.action"
  "action/Rotate360.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

ament_package()
