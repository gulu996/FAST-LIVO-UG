cmake_minimum_required(VERSION 3.0.2)
project(mvs_ros_driver)
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake) 

add_compile_options(-std=c++14)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  LINK_DIRECTORIES(/opt/MVS/lib/aarch64/)
  message(STATUS "Running on arrch64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  LINK_DIRECTORIES(/opt/MVS/lib/64/)
  message(STATUS "Running on x86_64")
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  diagnostic_msgs
  livox_ros_driver2
  rosbag
)
find_package(OpenCV)
find_package(PCL REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs diagnostic_msgs livox_ros_driver2
)

include_directories(
  /opt/MVS/include
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
)

add_executable(grabImgWithTrigger src/grab_trigger.cpp)
add_dependencies(grabImgWithTrigger ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  grabImgWithTrigger
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  MvCameraControl
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(timestamp_monitor_test
    test/timestamp_monitor_test.cpp)
  if(TARGET timestamp_monitor_test)
    target_include_directories(timestamp_monitor_test PRIVATE
      include
      ${catkin_INCLUDE_DIRS})
    target_link_libraries(timestamp_monitor_test
      ${catkin_LIBRARIES})
  endif()
endif()
