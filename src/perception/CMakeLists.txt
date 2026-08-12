cmake_minimum_required(VERSION 3.8)

project(perception)

# ============================================================================
# C++ Standard
# ============================================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ============================================================================
# ROS2 Packages & Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Tìm thư viện Eigen3 (bắt buộc cho bản ByteTrack này)
find_package(Eigen3 REQUIRED)

# ==== THÊM MỚI: dependencies cho depth_projector_component ====
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
# ================================================================

# ============================================================================
# Custom Message Generation
# ============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObject3D.msg"
  "msg/TrackedObject3DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Lấy target typesupport để link vào các component C++ dùng msg này sau này
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ============================================================================
# Include & Link Directories
# ============================================================================
include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  /usr/lib/aarch64-linux-gnu
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})


# ============================================================================
# YOLOv8 TensorRT Component
# ============================================================================
add_library(yolov8_trt_component SHARED
  src/yolov8_trt_component.cpp
)

target_link_libraries(yolov8_trt_component
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  nvinfer
)

ament_target_dependencies(
  yolov8_trt_component
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  cv_bridge
)

rclcpp_components_register_nodes(
  yolov8_trt_component
  "perception::Yolov8TrtComponent"
)

# ============================================================================
# USB Camera Component
# ============================================================================
add_library(usb_cam_component SHARED
  src/usb_cam_component.cpp
)

target_link_libraries(usb_cam_component
  ${OpenCV_LIBS}
)

ament_target_dependencies(
  usb_cam_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)

rclcpp_components_register_nodes(
  usb_cam_component
  "perception::UsbCamComponent"
)

# ============================================================================
# ByteTrack Component
# ============================================================================

add_library(bytetrack_component SHARED
  src/bytetrack_component.cpp
  src/bytetrack_core/src/BYTETracker.cpp
  src/bytetrack_core/src/KalmanFilter.cpp
  src/bytetrack_core/src/lapjv.cpp
  src/bytetrack_core/src/Object.cpp
  src/bytetrack_core/src/Rect.cpp
  src/bytetrack_core/src/STrack.cpp
)

target_include_directories(bytetrack_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bytetrack_core/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(bytetrack_component PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(bytetrack_component
  rclcpp
  rclcpp_components
  vision_msgs
)

rclcpp_components_register_node(bytetrack_component
  PLUGIN "perception::ByteTrackComponent"
  EXECUTABLE bytetrack_component_node
)

# ============================================================================
# Depth Projector Component  ==== THÊM MỚI ====
# ============================================================================
add_library(depth_projector_component SHARED
  src/depth_projector_component.cpp
)

target_link_libraries(depth_projector_component
  ${PCL_LIBRARIES}
  "${cpp_typesupport_target}"
)

ament_target_dependencies(depth_projector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  geometry_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
)

# depth_projector_component sinh ra sau khi generate message xong,
# nên cần add_dependencies để đảm bảo thứ tự build đúng
add_dependencies(depth_projector_component ${PROJECT_NAME})

rclcpp_components_register_nodes(
  depth_projector_component
  "perception::DepthProjectorComponent"
)

# ============================================================================
# Plugin XML export
# ============================================================================
pluginlib_export_plugin_description_file(
  rclcpp_components
  plugins.xml
)


# ============================================================================
# Install targets
# ============================================================================
install(
  TARGETS
    yolov8_trt_component
    usb_cam_component
    bytetrack_component
    depth_projector_component

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# ============================================================================
# Install python nodes
# ============================================================================
install(
  PROGRAMS
    scripts/visualizer_node.py
    scripts/bev_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================================
# Install launch/config/plugins
# ============================================================================
install(
  DIRECTORY
    launch
    config

  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    plugins.xml

  DESTINATION share/${PROJECT_NAME}
)


# ============================================================================
# Export dependencies
# ============================================================================
ament_export_dependencies(
  rclcpp
  rclcpp_components
  pluginlib
  sensor_msgs
  vision_msgs
  cv_bridge
  geometry_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
)

ament_package()
