cmake_minimum_required(VERSION 3.10)
project(video_recorder VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find GStreamer packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

# Link directories
link_directories(
    ${GSTREAMER_LIBRARY_DIRS}
    ${GSTREAMER_VIDEO_LIBRARY_DIRS}
    ${GTK3_LIBRARY_DIRS}
    ${JSONCPP_LIBRARY_DIRS}
)

# Source files
set(SOURCES
    src/main.cpp
    src/video_preview.cpp
    src/video_pipeline.cpp
    src/video_controller.cpp
    src/gtk_stream_buf.cpp
)

# Create video_recorder executable
add_executable(video_recorder ${SOURCES})

# Link libraries for video_recorder
target_link_libraries(video_recorder
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)

# Create frame_extractor executable
add_executable(frame_extractor src/frame_extractor.cpp)

# Link libraries for frame_extractor
target_link_libraries(frame_extractor
    ${GSTREAMER_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)

# Compiler flags
target_compile_options(video_recorder PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
    ${GSTREAMER_VIDEO_CFLAGS_OTHER}
    ${GTK3_CFLAGS_OTHER}
    ${JSONCPP_CFLAGS_OTHER}
)
