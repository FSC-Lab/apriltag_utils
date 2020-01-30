cmake_minimum_required(VERSION 2.8)
project(track_april_tag)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
sensor_msgs
cv_bridge
image_transport
geometry_msgs
nav_msgs
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
endif()

find_package(apriltag REQUIRED)


include_directories(
${PROJECT_SOURCE_DIR}
${apriltag_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
/usr/local/include/apriltag
${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
roscpp 
sensor_msgs 
std_msgs
geometry_msgs
cv_bridge
image_transport
nav_msgs
)

file(GLOB_RECURSE PYTHON_SCRIPTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS scripts/*.py)

catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(${PROJECT_NAME} SHARED
${apriltag_LIBRARIES}
src/apriltag_detector.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
/usr/local/lib/libapriltag.so
/usr/local/lib/libapriltag.so.3
/usr/local/lib/libapriltag.so.3.1.0
)

##############################################################################
# Sources for apriltag emulator
##############################################################################
file(GLOB_RECURSE EMULATOR_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*emulator.cpp)
add_executable(apriltag_emulator ${EMULATOR_SOURCES})
target_link_libraries(apriltag_emulator
${PROJECT_NAME}
${catkin_LIBRARIES})

##############################################################################
# Sources for apriltag detector
##############################################################################
file(GLOB_RECURSE DETECTOR_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*detector.cpp)

add_executable(apriltag_detector ${DETECTOR_SOURCES})
target_link_libraries(apriltag_detector 
${PROJECT_NAME}
${catkin_LIBRARIES})

##############################################################################
# Sources for Kalman Filter
##############################################################################
file(GLOB_RECURSE FILTER_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*filter.cpp)

add_executable(pose_filter ${FILTER_SOURCES})

target_link_libraries(pose_filter
${PROJECT_NAME}
${catkin_LIBRARIES}
${apriltag_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
)

