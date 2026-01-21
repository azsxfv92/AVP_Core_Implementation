cmake_minimum_required(VERSION 3.22)
project(AVP_Core_Implementation VERSION 1.0)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load dependency package
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# common include path
include_directories(include)

# for week01
add_executable(week1_perf_test src/main.cpp)
target_link_libraries(week1_perf_test Threads::Threads)

# for week02
add_executable(avp_main_node src/AVP_main.cpp)
ament_target_dependencies(avp_main_node rclcpp sensor_msgs)

install(TARGETS
    week1_perf_test
    avp_main_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()