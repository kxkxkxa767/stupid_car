cmake_minimum_required(VERSION 3.10)
project(HSV)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# pigpio 库路径（如有需要可修改）
find_library(PIGPIO_LIB pigpio)

# 添加可执行文件
add_executable(main main.cpp)

# 链接 OpenCV 和 pigpio
target_link_libraries(main ${OpenCV_LIBS} ${PIGPIO_LIB})

# 如果 pigpio 没找到，可以手动指定
# target_link_libraries(main ${OpenCV_LIBS} /usr/lib/libpigpio.so)