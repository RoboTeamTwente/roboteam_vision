cmake_minimum_required(VERSION 2.8.3)
project(roboteam_vision)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roboteam_msgs
  roboteam_utils
)

## System dependencies are found with CMake's conventions

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        roboteam_vision_convert
    CATKIN_DEPENDS
        message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(roboteam_vision_convert
    src/convert/convert_detection.cpp
    src/convert/convert_geometry_current.cpp
    src/convert/convert_geometry_legacy.cpp
    src/convert/convert_referee.cpp
    src/convert/convert_units.cpp
    )
add_dependencies(roboteam_vision_convert ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(roboteam_vision
    # Main executable program
    src/roboteam_vision.cpp

    src/transform.cpp

    # Net code from ssl vision
    src/net/netraw.cpp
    src/net/netraw.h
    src/net/robocup_ssl_client.cpp
    src/net/robocup_ssl_client.h
    src/net/util.h
    )
target_link_libraries(roboteam_vision
    roboteam_vision_convert
    ${catkin_LIBRARIES}
    )
add_dependencies(roboteam_vision
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

add_executable(crashtest
    src/crashtest.cpp
    )
target_link_libraries(crashtest
    ${catkin_LIBRARIES}
    )
add_dependencies(roboteam_vision
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

#############
## Testing ##
#############

catkin_add_gtest(vision_test
    test/main.cpp
	test/SimpleTest.cpp
)
target_link_libraries(vision_test ${catkin_LIBRARIES})
add_dependencies(roboteam_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
