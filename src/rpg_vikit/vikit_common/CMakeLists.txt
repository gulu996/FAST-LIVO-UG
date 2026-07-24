SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Release, RelWithDebInfo or Debug")
ENDIF()
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(USE_ROS TRUE) # Set False if you want to build this package without Catkin

# Use the compiler's target defaults on every architecture. In particular, do not
# inject x86 SIMD flags into native Jetson builds.
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
  # Keep the existing deterministic floating-point policy without selecting an ISA.
  SET(CMAKE_CXX_FLAGS_RELEASE
      "${CMAKE_CXX_FLAGS_RELEASE} -O2 -ffp-contract=off -frounding-math")
ENDIF()
MESSAGE(STATUS "vikit_common target architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS Eigen OpenCV Sophus
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Set Sourcefiles
LIST(APPEND SOURCEFILES src/atan_camera.cpp
                        src/omni_camera.cpp
                        src/math_utils.cpp
                        src/vision.cpp
                        src/performance_monitor.cpp
                        src/robust_cost.cpp
                        src/user_input_thread.cpp
                        src/pinhole_camera.cpp
                        src/equidistant_camera.cpp
                        src/polynomial_camera.cpp
                        src/homography.cpp
                        src/img_align.cpp)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${catkin_LIBRARIES})

IF(USE_ROS)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES})
ENDIF()

# Tests
ADD_EXECUTABLE(test_vk_common_camera test/test_camera.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_camera ${PROJECT_NAME})

ADD_EXECUTABLE(test_vk_common_triangulation test/test_triangulation.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_triangulation ${PROJECT_NAME})

ADD_EXECUTABLE(test_vk_common_patch_score test/test_patch_score.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_patch_score ${PROJECT_NAME})


################################################################################
# Create the vikit_commonConfig.cmake file for other cmake projects.
IF(NOT USE_ROS)
  GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
  SET(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
  SET(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
  SET(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE )
  export( PACKAGE vikit_common )

  INSTALL(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ENDIF()
