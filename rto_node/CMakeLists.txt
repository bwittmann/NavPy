cmake_minimum_required(VERSION 3.0.2)
project(rto_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  nav_msgs
  rto_msgs
  tf
)

list(APPEND CMAKE_MODULE_PATH "/usr/local/robotino/api2/cmake" "/opt/robotino/cmake")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(RobotinoAPI2)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   rto_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rto_node
  CATKIN_DEPENDS image_transport nav_msgs rto_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/robotino/api2
)

## Declare a cpp library
# add_library(rto_node
#   src/${PROJECT_NAME}/rto_node.cpp
# )

## Declare a cpp executable
# add_executable(rto_node_node src/rto_node_node.cpp)

add_executable(
  rto_node
  src/rto_node.cpp
  src/AnalogInputArrayROS.cpp
  src/BumperROS.cpp
  src/CameraROS.cpp
  src/CompactBHAROS.cpp
  src/ComROS.cpp
  src/DigitalInputArrayROS.cpp
  src/DigitalOutputArrayROS.cpp
  src/DistanceSensorArrayROS.cpp
  src/ElectricalGripperROS.cpp
  src/EncoderInputROS.cpp
  src/GrapplerROS.cpp
  src/LaserRangeFinderROS.cpp
  src/MotorArrayROS.cpp
  src/NorthStarROS.cpp
  src/OdometryROS.cpp
  src/OmniDriveROS.cpp
  src/PowerManagementROS.cpp
  src/RTONode.cpp
)

target_link_libraries(
  rto_node
  ${REC_ROBOTINO_API2_LIBRARY}
  ${catkin_LIBRARIES}
)


add_executable(
  rto_odometry_node
  src/rto_odometry_node.cpp
  src/ComROS.cpp
  src/OdometryROS.cpp
  src/RTOOdometryNode.cpp
)

target_link_libraries(
  rto_odometry_node
  ${REC_ROBOTINO_API2_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(
  rto_laserrangefinder_node
  src/rto_laserrangefinder_node.cpp
  src/ComROS.cpp
  src/LaserRangeFinderROS.cpp
  src/RTOLaserRangeFinderNode.cpp
)

target_link_libraries(
  rto_laserrangefinder_node
  ${REC_ROBOTINO_API2_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(
  rto_camera_node
  src/rto_camera_node.cpp
  src/ComROS.cpp
  src/CameraROS.cpp
  src/RTOCameraNode.cpp
)

target_link_libraries(
  rto_camera_node
  ${REC_ROBOTINO_API2_LIBRARY}
  ${catkin_LIBRARIES}
)

#add_executable(
#  rto_mapping_node
#  src/rto_mapping_node.cpp
#  src/MappingROS.cpp
#  src/RTOMappingNode.cpp
#  src/InitialPoseROS.cpp
#  src/NavGoalROS.cpp
#  src/ComROS.cpp
#)

#target_link_libraries(
#  rto_mapping_node
#  ${REC_ROBOTINO_API2_LIBRARY}
#  ${catkin_LIBRARIES}
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rto_node_node rto_node_generate_messages_cpp)
add_dependencies(rto_node rto_msgs_generate_messages_cpp)
add_dependencies(rto_odometry_node rto_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(rto_node_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rto_node rto_node_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rto_node.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
