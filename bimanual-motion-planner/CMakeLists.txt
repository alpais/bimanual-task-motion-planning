cmake_minimum_required(VERSION 2.8.3)
project(bimanual_motion_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
##find_package(catkin REQUIRED mathlib std_tools robotlib cds_execution tf tf_conversions cmake_modules)
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs mathlib std_tools robotlib cds_execution tf cmake_modules visualization_msgs tf_conversions bimanual_ds_execution netft_rdt_driver)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
#   std_msgs  # Or other packages containing msgs
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
  INCLUDE_DIRS include
#  LIBRARIES state_transformers
   CATKIN_DEPENDS actionlib_msgs roscpp geometry_msgs sensor_msgs mathlib robotlib std_tools tf robotlib cds_execution tf visualization_msgs tf_conversions bimanual_ds_execution netft_rdt_driver
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
find_package(Eigen REQUIRED)
find_package(bimanual_action_planners)
include_directories(
    include/bimanual_action_server
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${bimanual_action_planners_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(bimanual_action_server
    src/bimanual_action_server/bimanual_action_server.cpp
    src/bimanual_action_server/base_action_functions.cpp
    src/bimanual_action_server/force_ctrl_functions.cpp
    src/bimanual_action_server/uncplCDS_action_functions.cpp
    src/bimanual_action_server/cplCDS_action_functions.cpp
    src/bimanual_action_server/voDS_action_functions.cpp
    src/bimanual_action_server/bimanual_gotocart_functions.cpp)
target_link_libraries(bimanual_action_server
   ${catkin_LIBRARIES})

## Declare a C++ Executable
add_executable(bimanual_action_server_node src/bimanual_action_server_node.cpp)
target_link_libraries(bimanual_action_server_node
  bimanual_action_server
  ${catkin_LIBRARIES}
)

#add_dependencies(
#   bimanual_action_server
#  ${bimanual_action_planners_EXPORTED_TARGETS}
#)

## Declare a cpp executable
#add_executable(bimanual_motion_planner  src/tests/bimanual_motion_planner.cpp)
#add_executable(decoupled_motion_planner  src/tests/decoupled_motion_planner.cpp)
#add_executable(coordinated_motion_planner  src/tests/coordinated_motion_planner.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(bimanual_motion_planner   ${catkin_LIBRARIES} )
#target_link_libraries(decoupled_motion_planner   ${catkin_LIBRARIES} )
#target_link_libraries(coordinated_motion_planner   ${catkin_LIBRARIES} )

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
# install(TARGETS state_transformers state_transformers_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_state_transformers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
