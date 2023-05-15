cmake_minimum_required(VERSION 3.8)
project(mindvision_ros)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

# add_definitions(-Wall -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)


add_library(${PROJECT_NAME} SHARED
    src/mindvision_node.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC mvsdk/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC mvsdk/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC mvsdk/lib/arm64)
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME} MVSDK)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  image_transport
  image_transport_plugins
  sensor_msgs
  camera_info_manager
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN mindvision_camera::MVCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_package()

install(DIRECTORY 
  config
DESTINATION share/${PROJECT_NAME}/
) 

install(TARGETS
${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)