cmake_minimum_required(VERSION 3.8)
project(ma_vision_bringup)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ma_vision_interfaces REQUIRED)
find_package(auto_aim REQUIRED)


ament_package()


install(DIRECTORY
config launch
DESTINATION share/${PROJECT_NAME}
)