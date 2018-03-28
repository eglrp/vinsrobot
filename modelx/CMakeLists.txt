cmake_minimum_required(VERSION 2.8.3)
project(modelx)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake_modules)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    #camera_model
    sensor_msgs
    rosbag
    image_transport
    )

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../
${PROJECT_SOURCE_DIR}/../src
${PROJECT_SOURCE_DIR}/../include
${Pangolin_INCLUDE_DIRS}
)

set(LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../lib/libORB_SLAM2.so
)

# Node for monocular camera
#add_executable(ros_mono_vio
#src/ros_mono_vio.cpp
#src/MsgSync/MsgSynchronizer.cpp
#src/MsgSync/MsgSynchronizer.h
#)

#target_link_libraries(ros_mono_vio
#${LIBS}
#)
