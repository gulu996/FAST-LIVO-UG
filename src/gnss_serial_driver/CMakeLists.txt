cmake_minimum_required(VERSION 3.0.2)
project(gnss_serial_driver)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gnss_comm
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_time_msgs
  std_msgs
)

catkin_python_setup()

add_message_files(FILES
  GnssPvtStamped.msg
  GnssStatus.msg
)
generate_messages(DEPENDENCIES gnss_comm std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gnss_serial_math
  CATKIN_DEPENDS
    geometry_msgs gnss_comm message_runtime nav_msgs roscpp rospy sensor_time_msgs std_msgs
  DEPENDS EIGEN3
)

add_library(gnss_serial_math src/gnss_math.cpp)
target_include_directories(gnss_serial_math PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(gnss_serial_adapter_node src/gnss_adapter_node.cpp)
set_target_properties(gnss_serial_adapter_node PROPERTIES
  OUTPUT_NAME gnss_adapter_node)
add_dependencies(gnss_serial_adapter_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_serial_adapter_node
  gnss_serial_math
  ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/gnss_serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS gnss_serial_math gnss_serial_adapter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
  catkin_add_gtest(gnss_math_test test/gnss_math_self_test.cpp)
  if(TARGET gnss_math_test)
    target_link_libraries(gnss_math_test gnss_serial_math)
  endif()
endif()
