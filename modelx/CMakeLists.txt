cmake_minimum_required(VERSION 2.8.3)
project(modelx)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake_modules)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    #geometry_msgs
    nav_msgs
    tf
    cv_bridge
    sensor_msgs
    #camera_model
    sensor_msgs
    rosbag
    image_transport
    )
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Cholmod REQUIRED)
# Include dirs
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(include
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/../
  ${PROJECT_SOURCE_DIR}/../src
  ${PROJECT_SOURCE_DIR}/../src/IMU
  ${PROJECT_SOURCE_DIR}/../include
  ${PROJECT_SOURCE_DIR}/src/MsgSync/
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
)



# Describe catkin Project
catkin_package()

set(LIBS
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/../Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/../Thirdparty/g2o/lib/libg2o.so
    ${PROJECT_SOURCE_DIR}/../lib/libORB_SLAM2.so
    cholmod
    ${CHOLMOD_LIBRARIES}
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
    )

set(SRC_FILES
    ${PROJECT_SOURCE_DIR}/../src/Converter.cc
    ${PROJECT_SOURCE_DIR}/../src/Frame.cc
    ${PROJECT_SOURCE_DIR}/../src/FrameDrawer.cc
    ${PROJECT_SOURCE_DIR}/../src/Initializer.cc
    ${PROJECT_SOURCE_DIR}/../src/KeyFrame.cc
    ${PROJECT_SOURCE_DIR}/../src/KeyFrameDatabase.cc
    ${PROJECT_SOURCE_DIR}/../src/LocalMapping.cc
    ${PROJECT_SOURCE_DIR}/../src/LoopClosing.cc
    ${PROJECT_SOURCE_DIR}/../src/Map.cc
    ${PROJECT_SOURCE_DIR}/../src/MapDrawer.cc
    ${PROJECT_SOURCE_DIR}/../src/MapPoint.cc
    ${PROJECT_SOURCE_DIR}/../src/Optimizer.cc
    ${PROJECT_SOURCE_DIR}/../src/ORBextractor.cc
    ${PROJECT_SOURCE_DIR}/../src/ORBmatcher.cc
    ${PROJECT_SOURCE_DIR}/../src/PnPsolver.cc
    ${PROJECT_SOURCE_DIR}/../src/System.cc
    ${PROJECT_SOURCE_DIR}/../src/Tracking.cc
    ${PROJECT_SOURCE_DIR}/../src/Viewer.cc
    ${PROJECT_SOURCE_DIR}/../src/IMU/configparam.cpp
    ${PROJECT_SOURCE_DIR}/../src/IMU/g2otypes.cpp
    ${PROJECT_SOURCE_DIR}/../src/IMU/imudata.cpp
    ${PROJECT_SOURCE_DIR}/../src/IMU/IMUPreintegrator.cpp
    ${PROJECT_SOURCE_DIR}/../src/IMU/NavState.cpp
    ${PROJECT_SOURCE_DIR}/../src/IMU/so3.cpp
    ${PROJECT_SOURCE_DIR}/../src/IMU/
    ${PROJECT_SOURCE_DIR}/../src/IMU/
    ${PROJECT_SOURCE_DIR}/src/MsgSync/MsgSynchronizer.cpp
)

add_executable(ros_vio
    src/ros_vio.cc
    ${SRC_FILES}
    )
target_link_libraries( ros_vio ${catkin_LIBRARIES} ${LIBS} )

add_executable(ros_stereo
    src/ros_stereo.cc
    ${SRC_FILES}
    )
target_link_libraries(ros_stereo ${catkin_LIBRARIES} ${LIBS} )

#add_executable(mapping
#src/mapping.cpp
#${SRC_FILES}
#)
#target_link_libraries(mapping
#${LIBS}
#${catkin_LIBRARIES}
#)

#Node for monocular camera
#add_executable(ros_mono_vio
#src/ros_mono_vio.cpp
#${SRC_FILES}
#)
#target_link_libraries(ros_mono_vio
#${catkin_LIBRARIES}
##${LIBS}
#)
