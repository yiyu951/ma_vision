cmake_minimum_required(VERSION 3.8)
project(ma_vision_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}

"msg/SerialSendData.msg"

DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_export_include_directories("include")

install(
  DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)


ament_package()
