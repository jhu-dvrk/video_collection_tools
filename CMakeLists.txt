# Create core library
add_library(video_core STATIC
    src/video_preview.cpp
    src/video_pipeline.cpp
    src/video_controller.cpp
    src/gtk_stream_buf.cpp
)
target_include_directories(video_core PUBLIC include)
target_link_libraries(video_core PUBLIC
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${GSTREAMER_LINK_LIBRARIES}
    ${GSTREAMER_VIDEO_LINK_LIBRARIES}
    ${GTK3_LINK_LIBRARIES}
    ${JSONCPP_LINK_LIBRARIES}
)
target_compile_options(video_core PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
    ${GSTREAMER_VIDEO_CFLAGS_OTHER}
    ${GTK3_CFLAGS_OTHER}
    ${JSONCPP_CFLAGS_OTHER}
)

cmake_minimum_required(VERSION 3.10)
project(jhu_data_collection)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

include_directories(
    include
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

add_executable(video_recorder src/video_recorder.cpp)
target_link_libraries(video_recorder video_core)
target_compile_options(video_recorder PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
    ${GSTREAMER_VIDEO_CFLAGS_OTHER}
    ${GTK3_CFLAGS_OTHER}
    ${JSONCPP_CFLAGS_OTHER}
)

add_executable(frame_extractor src/frame_extractor.cpp)
target_link_libraries(frame_extractor video_core)

# ROS2 node executable
add_executable(video_recorder_node
    src/video_recorder_node.cpp
)
ament_target_dependencies(video_recorder_node rclcpp sensor_msgs std_msgs)
target_link_libraries(video_recorder_node video_core)
target_compile_options(video_recorder_node PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
    ${GSTREAMER_VIDEO_CFLAGS_OTHER}
    ${GTK3_CFLAGS_OTHER}
    ${JSONCPP_CFLAGS_OTHER}
)

install(TARGETS
    video_recorder
    frame_extractor
    video_recorder_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts for ros2 run

install(PROGRAMS
    scripts/set_recording.py
    scripts/unset_recording.py
    scripts/start_recording.py
    scripts/stop_recording.py
    scripts/set_data_directory.py
    DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY scripts/
    DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY include/
    DESTINATION include/
)

ament_package()
