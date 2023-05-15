cmake_minimum_required(VERSION 3.8)
project(auto_aim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(message_filters REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(detector_openvino REQUIRED)
find_package(mindvision_ros REQUIRED)
find_package(ma_vision_interfaces REQUIRED)



include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED

  src/auto_aim_node.cpp
)


ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  image_transport
  image_transport_plugins
  sensor_msgs
  camera_info_manager
  cv_bridge
  message_filters
  
  mindvision_ros
  ma_vision_interfaces
  detector_openvino
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  fmt::fmt
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN MA::AutoAimNode
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
