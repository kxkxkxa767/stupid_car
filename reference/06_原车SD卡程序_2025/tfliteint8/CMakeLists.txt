cmake_minimum_required(VERSION 3.10)
project(tf_segmentation)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ftree-vectorize")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# TensorFlow Lite 头文件和库路径（如有不同请自行调整）
include_directories(/usr/include/tensorflow-lite)
link_directories(/usr/lib)

add_executable(tf src/tf.cpp)

target_link_libraries(tf
    ${OpenCV_LIBS}
    tensorflow-lite
)