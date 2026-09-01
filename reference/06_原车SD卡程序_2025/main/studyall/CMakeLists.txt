cmake_minimum_required(VERSION 3.10)
project(HSV)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -funroll-loops")

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# pigpio 库路径（如有需要可修改）
find_library(PIGPIO_LIB pigpio)

# TensorFlow Lite 路径（请根据实际安装位置修改）
set(TFLITE_LIB_PATH /usr/lib)
set(TFLITE_INCLUDE_PATH /usr/include/tensorflow-lite)

# 查找 TensorFlow Lite 库
find_library(TFLITE_LIB tensorflow-lite HINTS ${TFLITE_LIB_PATH})

# 添加可执行文件
add_executable(main src/main.cpp)

# 包含 TensorFlow Lite 头文件
target_include_directories(main PRIVATE ${TFLITE_INCLUDE_PATH})

# 链接 OpenCV、pigpio、TensorFlow Lite
target_link_libraries(main ${OpenCV_LIBS} ${PIGPIO_LIB} ${TFLITE_LIB})

# 如果 pigpio 没找到，可以手动指定
# target_link_libraries(main ${OpenCV_LIBS} /usr/lib/libpigpio.so ${TFLITE_LIB})