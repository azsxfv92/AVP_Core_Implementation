cmake_minimum_required(VERSION 3.22)
project(avp_core_implementation VERSION 1.0)

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

# add trt_infer_node
add_executable(trt_infer_node
  src/trt_infer_node.cpp
)

ament_target_dependencies(trt_infer_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(trt_infer_node PRIVATE
  include
  ${CUDAToolkit_INCLUDE_DIRS}
)

find_package(CUDAToolkit REQUIRED)

target_link_libraries(trt_infer_node
  nvinfer
  CUDA::cudart
  ${OpenCV_LIBS}
)

install(TARGETS
  trt_infer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()