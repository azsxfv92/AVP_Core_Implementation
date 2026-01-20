cmake_minimum_required(VERSION 3.22)
project(AVP_Core_Implementation VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# for week01
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME} src/week01_mordern_cpp/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)