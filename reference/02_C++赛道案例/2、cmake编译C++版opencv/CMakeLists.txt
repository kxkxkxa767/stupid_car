cmake_minimum_required(VERSION 2.8)
project(demo)
find_package(OpenCV REQUIRED)
set(PROTO_PB_DIR ${PROJECT_SOURCE_DIR}pio)

include_directories( ${PROTO_PB_DIR} )
add_executable(demo demo.cpp)
target_link_libraries(demo ${OpenCV_LIBS})