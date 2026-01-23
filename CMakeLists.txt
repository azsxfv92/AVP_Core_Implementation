cmake_minimum_required(VERSION 3.22)
project(avp_core_implementation VERSION 1.0)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load dependency package
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Packages for creating messages
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

ament_target_dependencies(avp_controller_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
)
target_link_libraries(avp_controller_node "${PROJECT_NAME}__rosidl_typesupport_cpp")

install(TARGETS
    week1_perf_test
    avp_main_node
    avp_controller_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()