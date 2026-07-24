SET(PROJECT_NAME vikit_ros)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Release, RelWithDebInfo or Debug")
ENDIF()
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# Use the compiler's target defaults instead of forcing an x86 ISA on Jetson.
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)
ADD_DEFINITIONS(-D_LINUX -D_REENTRANT)
IF(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  ADD_COMPILE_OPTIONS(
    -Wall
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-unknown-pragmas)
ENDIF()
MESSAGE(STATUS "vikit_ros target architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp vikit_common visualization_msgs tf cmake_modules)

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Describe catkin Project
catkin_package(
    DEPENDS EIGEN3 OpenCV Sophus vikit_common
    CATKIN_DEPENDS roscpp visualization_msgs tf 
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

# Set Sourcefiles
LIST(APPEND SOURCEFILES 
  src/output_helper.cpp
)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${catkin_LIBRARIES})
