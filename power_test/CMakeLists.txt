cmake_minimum_required(VERSION 3.8)
project(power_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(message_filters REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# find_package(ma_vision_interfaces REQUIRED)



include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED
  src/power_test.cpp
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

  visualization_msgs
  
  tf2
  tf2_ros

  # ma_vision_interfaces
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  fmt::fmt
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN MA::PowerTestNode
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
