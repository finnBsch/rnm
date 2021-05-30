cmake_minimum_required(VERSION 3.19)
project(frame_reader)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        cv_bridge
        image_transport
        sensor_msgs
        )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( OpenCV REQUIRED )
include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(frame_reader main.cpp)
target_link_libraries( frame_reader ${OpenCV_LIBS} /usr/lib/x86_64-linux-gnu/libopencv_highgui.so)
MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS} )
target_link_libraries( frame_reader ${catkin_LIBRARIES} )