cmake_minimum_required(VERSION 3.0.2)
project(sensor_time_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(FILES
  McuEvent.msg
  RawSerialFrame.msg
  TimeMapping.msg
  TimeStatus.msg
  UtcObservation.msg
)

add_service_files(FILES
  ConvertTime.srv
  HostMonotonicToLocal.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
