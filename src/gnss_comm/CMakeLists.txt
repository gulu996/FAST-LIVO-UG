cmake_minimum_required(VERSION 2.8.3)
project(gnss_comm)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(FILES
  GnssBestXYZMsg.msg
  GnssEphemMsg.msg
  GnssGloEphemMsg.msg
  GnssMeasMsg.msg
  GnssObsMsg.msg
  GnssPVTSolnMsg.msg
  GnssSvsMsg.msg
  GnssTimeMsg.msg
  GnssTimePulseInfoMsg.msg
  StampedFloat64Array.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
