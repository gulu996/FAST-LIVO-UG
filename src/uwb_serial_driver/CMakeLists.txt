cmake_minimum_required(VERSION 3.0.2)
project(uwb_serial_driver)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_time_msgs
  std_msgs
)

catkin_python_setup()

add_message_files(FILES
  UwbRange.msg
  UwbRangeArray.msg
  UwbStatus.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime rospy sensor_time_msgs std_msgs)

catkin_install_python(PROGRAMS scripts/uwb_serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_exclusive_serial.py)
  catkin_add_nosetests(test/test_parser.py)
  add_rostest(test/distance_round5_integration.test)
endif()
