# ============================================================
#
# This file is a part of CoSiMA (CogIMon) project
#
# Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
#
# This file may be licensed under the terms of the
# GNU Lesser General Public License Version 3 (the ``LGPL''),
# or (at your option) any later version.
#
# Software distributed under the License is distributed
# on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
# express or implied. See the LGPL for the specific language
# governing rights and limitations.
#
# You should have received a copy of the LGPL along with this
# program. If not, go to http://www.gnu.org/licenses/lgpl.html
# or write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# The development of this software was supported by:
#   CoR-Lab, Research Institute for Cognition and Robotics
#     Bielefeld University
#
# ============================================================

cmake_minimum_required(VERSION 2.8.3)
project(cosima-robot-sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # rtt_ros
  roscpp
  std_msgs
  # geometry_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OROCOS-RTT REQUIRED)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)


### PROBLEM WITH ROS (KDL VERSION) INCONSISTENCY TODO SOLVE!

### find_package(orocos_kdl REQUIRED)
SET(orocos_kdl_LIBRARIES /home/dwigand/citk/systems/cogimon-minimal-nightly/lib/liborocos-kdl.so.1.4)
include_directories(BEFORE SYSTEM /home/dwigand/citk/systems/cogimon-minimal-nightly/include/kdl)
link_directories(/home/dwigand/citk/systems/cogimon-minimal-nightly/lib/)

SET(kdl_parser_LIBRARIES /home/dwigand/code/cogimon/CoSimA/ros-cosima-ws/devel/.private/kdl_parser/lib/libkdl_parser.so)
include_directories(BEFORE SYSTEM /home/dwigand/code/cogimon/CoSimA/ros-cosima-ws/src/kdl_parser/kdl_parser/include/kdl_parser)
link_directories(/home/dwigand/code/cogimon/CoSimA/ros-cosima-ws/devel/.private/kdl_parser/lib/)


find_package(gazebo REQUIRED)
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})


find_package(Bullet7 REQUIRED)
# find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE)
# find_package(Bullet MODULE)
if (NOT BULLET_FOUND)
  message (FATAL_ERROR "\nCould not find Bullet.")
endif()
# set(BULLET_INCLUDE_DIR "/home/dwigand/citk/systems/cogimon-minimal-nightly/include/bullet")
# set(BULLET_LIBRARY_DIRS "/home/dwigand/citk/systems/cogimon-minimal-nightly/lib")
# set(BULLET_LIBRARIES "BulletSoftBody;BulletDynamics;BulletCollision;LinearMath;/home/dwigand/citk/systems/cogimon-minimal-nightly/lib/libBulletDynamics.so;/home/dwigand/citk/systems/cogimon-minimal-nightly/lib/libBulletCollision.so;/home/dwigand/citk/systems/cogimon-minimal-nightly/lib/libLinearMath.so;/home/dwigand/citk/systems/cogimon-minimal-nightly/lib/libBulletSoftBody.so")
# add_definitions     ( ${BULLET_DEFINITIONS} )
include_directories ( ${BULLET_INCLUDE_DIRS} )
include_directories ( "/home/dwigand/citk/systems/cogimon-minimal-nightly/include/bullet" )
include_directories ( "/home/dwigand/citk/systems/cogimon-minimal-nightly/include" )
link_directories    ( "${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS}" )

# include(FindPackageHandleStandardArgs)
# find_package(PkgConfig)
# if(PKGCONFIG_FOUND)
#   pkg_check_modules(BULLET bullet>=2.8)
# endif()
# find_package_handle_standard_args(Bullet DEFAULT_MSG BULLET_LIBRARIES BULLET_INCLUDE_DIRS)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rtt-bullet-embedded
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${OROCOS-RTT_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rtt-bullet-embedded.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rtt-bullet-embedded_node.cpp)

orocos_component(${CMAKE_PROJECT_NAME}
  # Bullet Interface
  ${PROJECT_SOURCE_DIR}/src/simulator_interface/bullet/b3_capi_wrapper_no_gui.cpp
  ${PROJECT_SOURCE_DIR}/src/simulator_interface/bullet/bullet_if.cpp
  # Gazebo Interface
  ${PROJECT_SOURCE_DIR}/src/simulator_interface/gazebo/gazebo_if.cpp
  # Generic Simulator Interface
  ${PROJECT_SOURCE_DIR}/src/simulator_interface/sim_if.cpp
  # Generic Robot Interface
  ${PROJECT_SOURCE_DIR}/src/robots/robot_manipulator_if.cpp
  # Bullet Robot Interface
  ${PROJECT_SOURCE_DIR}/src/robots/robot_manipulator_bullet.cpp
  # Gazebo Robot Interface
  ${PROJECT_SOURCE_DIR}/src/robots/robot_manipulator_gazebo.cpp
  # Generic OROCOS RTT Component
  ${PROJECT_SOURCE_DIR}/src/rtt_robot_manipulator_sim.cpp
)

# set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(${CMAKE_PROJECT_NAME} ${EIGEN3_LIBRARIES} ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${BULLET_LIBRARIES}
${GAZEBO_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${catkin_LIBRARIES}
# Bullet2FileLoader
# Bullet3Collision
# Bullet3Common
# Bullet3Dynamics
# Bullet3Geometry
# Bullet3OpenCL_clew
# BulletCollision
# BulletDynamics
# BulletExampleBrowserLib
# BulletFileLoader
# BulletInverseDynamics
# BulletInverseDynamicsUtils
# BulletRoboticsGUI
# BulletRobotics
# BulletSoftBody
# BulletWorldImporter
# BulletXmlWorldImporter
)

orocos_generate_package(
  #INCLUDE_DIRS include
  DEPENDS rtt_ros
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rtt-bullet-embedded.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
