cmake_minimum_required(VERSION 3.0.2)
project(zed_time_bridge)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/zed_time_audit.py scripts/correlate_imus.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  set(SENSOR_TIME_BRIDGE_PYTHON_ROOT "$ENV{SENSOR_TIME_BRIDGE_PYTHON_ROOT}"
      CACHE PATH "Existing read-only sensor_time_bridge Python package parent")
  if(NOT EXISTS "${SENSOR_TIME_BRIDGE_PYTHON_ROOT}/sensor_time_bridge/imu_anchor.py")
    message(FATAL_ERROR "Set SENSOR_TIME_BRIDGE_PYTHON_ROOT to the existing sensor_time_bridge/src directory")
  endif()
  add_test(NAME zed_time_bridge_checks COMMAND ${PYTHON_EXECUTABLE}
    -B -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/test -v)
  set_tests_properties(zed_time_bridge_checks PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src:${SENSOR_TIME_BRIDGE_PYTHON_ROOT}:$ENV{PYTHONPATH};PYTHONDONTWRITEBYTECODE=1")
endif()
