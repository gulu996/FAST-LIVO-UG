cmake_minimum_required(VERSION 3.0.2)
project(mvs_ros_driver)
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake) 

add_compile_options(-std=c++14)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" MVS_TARGET_PROCESSOR)
if(MVS_TARGET_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(MVS_LIBRARY_DIR /opt/MVS/lib/aarch64)
elseif(MVS_TARGET_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(MVS_LIBRARY_DIR /opt/MVS/lib/64)
else()
  message(FATAL_ERROR
    "mvs_ros_driver does not know the MVS SDK library directory for target "
    "architecture '${CMAKE_SYSTEM_PROCESSOR}'")
endif()
link_directories("${MVS_LIBRARY_DIR}")
message(STATUS
  "mvs_ros_driver target architecture: ${CMAKE_SYSTEM_PROCESSOR}; "
  "MVS SDK library directory: ${MVS_LIBRARY_DIR}")

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
