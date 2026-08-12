cmake_minimum_required(VERSION 3.8)
project(cbf_safety_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(cbf_safety_filter_node src/cbf_safety_filter_node.cpp)

ament_target_dependencies(cbf_safety_filter_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

target_include_directories(cbf_safety_filter_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cbf_safety_filter_node
  ${OpenCV_LIBS}
  OsqpEigen::OsqpEigen
  Eigen3::Eigen
)

install(TARGETS
  cbf_safety_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
