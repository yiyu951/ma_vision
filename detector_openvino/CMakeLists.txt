cmake_minimum_required(VERSION 3.8)
project(detector_openvino)


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(OpenCV REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ma_vision_interfaces REQUIRED)

include_directories(include)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${InferenceEngine_INCLUDE_DIRS})

add_library(detector_openvino SHARED
  detect.cpp
)

target_link_libraries(detector_openvino ${OpenCV_LIBS} ${InferenceEngine_LIBRARIES})


ament_export_include_directories("include")
ament_export_include_directories(
  ${OpenCV_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR} 
  ${InferenceEngine_INCLUDE_DIRS}
)
ament_export_libraries(detector_openvino)

# ament_export_dependencies(
#   ${InferenceEngine_LIBRARIES}
#   ${OpenCV_LIBS}
# )


install(TARGETS
  detector_openvino
  DESTINATION lib
)

install(
  DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
