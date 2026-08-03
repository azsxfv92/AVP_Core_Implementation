cmake_minimum_required(VERSION 3.22)
project(avp_core_implementation VERSION 1.0)

option(BUILD_INFERENCE "Build CUDA/TensorRT inference targets" ON)

if(BUILD_INFERENCE)
  enable_language(CUDA)
endif()

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load dependency package
find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# Packages for creating messages
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/VehicleState.msg"
    "msg/ParkingSlot.msg"
    DEPENDENCIES std_msgs geometry_msgs
)

# common include path
include_directories(include)

# for week01
add_executable(week1_perf_test src/main.cpp)
target_link_libraries(week1_perf_test Threads::Threads)

# for week02 - Define executable first, then its dependencies
add_executable(avp_main_node src/AVP_main.cpp)
target_link_libraries(avp_main_node "${PROJECT_NAME}__rosidl_typesupport_cpp")
ament_target_dependencies(avp_main_node
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
)

# set avp_controller_node
add_executable(avp_controller_node src/AVP_controller.cpp)
target_link_libraries(avp_controller_node "${PROJECT_NAME}__rosidl_typesupport_cpp")
ament_target_dependencies(avp_controller_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
)


add_executable(sync_harness_node src/sync_harness_node.cpp)
target_link_libraries(sync_harness_node "${PROJECT_NAME}__rosidl_typesupport_cpp")
ament_target_dependencies(sync_harness_node
    rclcpp
    message_filters
)


install(TARGETS
    week1_perf_test
    avp_main_node
    avp_controller_node
    sync_harness_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_INFERENCE)

# add trt_infer_node
add_executable(trt_infer_node
  src/trt_infer_node.cpp
  src/video_encoder.cpp
  src/yolo_postprocess.cpp
)

ament_target_dependencies(trt_infer_node
  rclcpp
  sensor_msgs
  cv_bridge
  vision_msgs
)

find_package(CUDAToolkit REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

target_include_directories(trt_infer_node PRIVATE
  include
  ${CUDAToolkit_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
)


target_link_libraries(trt_infer_node
  nvinfer
  CUDA::cudart
  avp_preprocess
  ${OpenCV_LIBS}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)

install(TARGETS
  trt_infer_node
  DESTINATION lib/${PROJECT_NAME}
)

add_library(avp_preprocess
  src/preprocess/preprocess_cpu.cpp
  src/preprocess/preprocess.cu
)

target_include_directories(avp_preprocess PUBLIC include)
target_link_libraries(avp_preprocess PUBLIC 
  ${OpenCV_LIBS}
  CUDA::cudart
)
 
install(TARGETS
  avp_preprocess
  DESTINATION lib
)
endif()

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

add_executable(aeb_node src/aeb_node.cpp)
ament_target_dependencies(aeb_node
  rclcpp
  std_msgs
  geometry_msgs
  autoware_perception_msgs
  tf2_ros
  tf2_geometry_msgs  
)
install(TARGETS aeb_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
