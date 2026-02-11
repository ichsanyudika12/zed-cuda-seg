cmake_minimum_required(VERSION 3.10)

project(zed_segment_CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${ZED_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /usr/local/cuda/include)

add_executable(main src/main.cpp)

target_link_libraries(main ${ZED_LIBRARIES} ${OpenCV_LIBS} cuda)