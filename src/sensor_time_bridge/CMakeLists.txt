cmake_minimum_required(VERSION 3.0.2)
project(sensor_time_bridge)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_time_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_time_core
  CATKIN_DEPENDS roscpp sensor_time_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(sensor_time_core
  src/local_time.cpp
  src/time_mapping.cpp
  src/wire_protocol.cpp
)
add_dependencies(sensor_time_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_time_core ${catkin_LIBRARIES})

add_executable(sensor_time_bridge_node src/bridge_node.cpp)
add_dependencies(sensor_time_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_time_bridge_node sensor_time_core ${catkin_LIBRARIES})

add_executable(mcu_event_simulator src/mcu_event_simulator.cpp)
add_dependencies(mcu_event_simulator ${catkin_EXPORTED_TARGETS})
target_link_libraries(mcu_event_simulator sensor_time_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(sensor_time_core_test test/sensor_time_core_test.cpp)
  if(TARGET sensor_time_core_test)
    target_link_libraries(sensor_time_core_test sensor_time_core ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS sensor_time_core sensor_time_bridge_node mcu_event_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
