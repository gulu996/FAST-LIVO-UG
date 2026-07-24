cmake_minimum_required(VERSION 3.0.2)
project(sensor_recording_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_time_msgs
  std_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy sensor_time_msgs std_msgs)

catkin_install_python(PROGRAMS
  scripts/check_recording_ready.py
  scripts/session_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_recorder_gate.py)
  add_rostest(test/software_simulation.test)
endif()
