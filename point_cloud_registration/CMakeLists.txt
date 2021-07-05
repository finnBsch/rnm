cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_registration)

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

find_package(catkin REQUIRED COMPONENTS
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        std_msgs
        genmsg
        rospy
        PCL REQUIRED
        forward_kin
        message_generation
)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PCJScombined.msg
  registrationResults.msg
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   alignment_service.srv
         registration_results_service.srv
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        DEPENDS EIGEN3
        CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(VoxelGridFilter src/VoxelGridFilter.cpp)
#add_executable(PCMatrixTransform src/PCMatrixTransform.cpp)
#add_executable(STLconversion_node src/STLconversion_node.cpp)
add_executable(Synchronizer_node src/Synchronizer_node.cpp)
add_executable(PC_stitching_node src/PC_stitching_node.cpp)
#add_executable(skeleton_splitter src/skeleton_splitter.cpp)
add_executable(Scanner_node src/Scanner_node.cpp)
#add_executable(STLRegistration_sac_ia src/STLRegistration_sac_ia.cpp)
add_executable(STLRegistration_node src/STLRegistration_node.cpp)
add_executable(RegistrationServiceTest src/RegistrationServiceTest.cpp)
add_executable(PointCloudRecording src/PointCloudRecording.cpp)
#add_executable(joint_states_publisher src/joint_states_publisher.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(VoxelGridFilter ${catkin_LIBRARIES})
#target_link_libraries(PCMatrixTransform ${catkin_LIBRARIES})
#target_link_libraries (PCMatrixTransform ${PCL_LIBRARIES})
#target_link_libraries (PCMatrixTransform ${EIGEN3_LIBRARIES})
#target_link_libraries(STLconversion_node ${catkin_LIBRARIES})
target_link_libraries(Synchronizer_node ${catkin_LIBRARIES})
target_link_libraries(PC_stitching_node ${catkin_LIBRARIES})
#target_link_libraries(skeleton_splitter ${catkin_LIBRARIES})
target_link_libraries(Scanner_node ${catkin_LIBRARIES})
#target_link_libraries(STLRegistration ${catkin_LIBRARIES})
target_link_libraries(STLRegistration_node ${catkin_LIBRARIES})
target_link_libraries(RegistrationServiceTest ${catkin_LIBRARIES})
target_link_libraries(PointCloudRecording ${catkin_LIBRARIES})
#target_link_libraries(joint_states_publisher ${catkin_LIBRARIES})

#add_dependencies(PCMatrixTransform ${catkin_EXPORTED_TARGETS})
add_dependencies(PC_stitching_node ${catkin_EXPORTED_TARGETS})
add_dependencies(PC_stitching_node point_cloud_registration_generate_messages_cpp)
add_dependencies(Synchronizer_node point_cloud_registration_generate_messages_cpp)
#add_dependencies(STLRegistration point_cloud_registration_generate_messages_cpp)
add_dependencies(STLRegistration_node point_cloud_registration_generate_messages_cpp)
add_dependencies(RegistrationServiceTest point_cloud_registration_generate_messages_cpp)
add_dependencies(Scanner_node point_cloud_registration_generate_messages_cpp)
add_dependencies(PointCloudRecording point_cloud_registration_generate_messages_cpp)
#add_dependencies(joint_states_publisher point_cloud_registration_generate_messages_cpp)

