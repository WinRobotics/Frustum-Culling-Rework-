cmake_minimum_required(VERSION 3.16)

project(lidar_camera VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED COMPONENTS common io filters visualization)
find_package(X11 REQUIRED)

add_executable(${PROJECT_NAME} src/lidar_camera.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES} ${X11_LIBRARIES})

target_compile_options(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})