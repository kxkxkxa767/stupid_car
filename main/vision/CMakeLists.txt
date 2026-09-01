cmake_minimum_required(VERSION 3.20)

project(xt_netrc_vision VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui videoio imgcodecs)

set(XTNETRC_VISION_SOURCES
    src/main.cpp
    src/lane_detector.cpp
)

function(xtnetrc_configure_vision_target target_name)
    target_include_directories(${target_name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_link_libraries(${target_name} PRIVATE ${OpenCV_LIBS})
    target_compile_options(${target_name} PRIVATE
        $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:-Wall;-Wextra;-Wpedantic>
    )
    # 无论从根工程还是本目录单独编译，可执行文件都放在当前 build 根目录。
    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
endfunction()

add_executable(xt_netrc_vision ${XTNETRC_VISION_SOURCES})
xtnetrc_configure_vision_target(xt_netrc_vision)

# macOS 需要真正的 .app 才能稳定触发并记住摄像头权限。
if(APPLE)
    add_executable(xt_netrc_vision_app MACOSX_BUNDLE ${XTNETRC_VISION_SOURCES})
    xtnetrc_configure_vision_target(xt_netrc_vision_app)
    set_target_properties(xt_netrc_vision_app PROPERTIES
        OUTPUT_NAME "XT-NetRC Vision"
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "@executable_path/../Frameworks"
        MACOSX_BUNDLE_BUNDLE_NAME "XT-NetRC Vision"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.yuhaojin.xtnetrc.vision"
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_LIST_DIR}/../../macos/Info.plist.in"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
        MACOSX_BUNDLE_BUNDLE_VERSION "1"
    )

    # 把 OpenCV 动态库放入 app，避免启动时追溯到 Documents 目录并引发额外权限询问。
    get_filename_component(OPENCV_RUNTIME_DIR "${OpenCV_DIR}/../.." ABSOLUTE)
    file(GLOB OPENCV_RUNTIME_DYLIBS
        "${OPENCV_RUNTIME_DIR}/libopencv_*.${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}.dylib"
    )
    add_custom_command(TARGET xt_netrc_vision_app POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E make_directory
                "$<TARGET_BUNDLE_CONTENT_DIR:xt_netrc_vision_app>/Frameworks"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                ${OPENCV_RUNTIME_DYLIBS}
                "$<TARGET_BUNDLE_CONTENT_DIR:xt_netrc_vision_app>/Frameworks"
        COMMAND /usr/bin/codesign --force --deep --sign -
                "$<TARGET_BUNDLE_DIR:xt_netrc_vision_app>"
        COMMENT "Bundling OpenCV and ad-hoc signing the macOS app"
        VERBATIM
    )
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(xt_netrc_selftest
        tests/lane_detector_test.cpp
        src/lane_detector.cpp
    )
    target_include_directories(xt_netrc_selftest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_link_libraries(xt_netrc_selftest PRIVATE ${OpenCV_LIBS})
    target_compile_options(xt_netrc_selftest PRIVATE
        $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:-Wall;-Wextra;-Wpedantic>
    )
    set_target_properties(xt_netrc_selftest PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    add_test(NAME lane_detector_selftest COMMAND xt_netrc_selftest)
endif()

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
