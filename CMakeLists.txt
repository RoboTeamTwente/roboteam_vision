cmake_minimum_required(VERSION 2.8.3)
project(roboteam_vision)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

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
#  LIBRARIES roboteam_vision
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
)

# Get all the cpp files to compile.
file(GLOB roboteam_vision_SRC
    "src/*.cpp"
    "src/convert/*.cpp"
    "src/net/*.cpp"
    "src/proto/cpp/*.cc"
)

add_executable(roboteam_vision
    ${roboteam_vision_SRC}
    )
target_link_libraries(roboteam_vision ${catkin_LIBRARIES})
add_dependencies(roboteam_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Testing ##
#############

catkin_add_gtest(vision_test
    test/main.cpp
	test/SimpleTest.cpp
)
target_link_libraries(vision_test ${catkin_LIBRARIES})
add_dependencies(roboteam_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
