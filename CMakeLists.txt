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

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
# SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  roscpp
  std_msgs
  sensor_msgs
  kdl_parser
)

if (NOT OROCOS-RTT_FOUND)
  message (STATUS "OROCOS RTT not found through catkin. Check find_package alternative")
  find_package(OROCOS-RTT REQUIRED)
  if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
  endif()
  
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
endif()

find_package(orocos_kdl REQUIRED)
link_directories(${orocos_kdl_LIBRARY_DIRS})

find_package(gazebo QUIET)
if (NOT GAZEBO_FOUND)
  message (STATUS "\nCould not find Gazebo.")
  add_definitions(-DDISABLE_GAZEBO)
endif()
if (GAZEBO_FOUND)
  if(GAZEBO_VERSION)
    if(${GAZEBO_VERSION} GREATER 6)
        add_definitions(-DGAZEBO_GREATER_6)
    endif()
  endif()
  include_directories(${GAZEBO_INCLUDE_DIRS})
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(Bullet REQUIRED)
if (NOT BULLET_FOUND)
  message (FATAL_ERROR "\nCould not find Bullet.")
  #SET(DISABLE_BULLET TRUE)
else()
  message(STATUS "Version of Bullet too old or not available: disabling Bullet collision detection plugin. Try using Ubuntu 18.04 or later.")
endif()
include_directories (${BULLET_INCLUDE_DIRS})
link_directories    (${BULLET_LIBRARY_DIRS})



#message(ERROR "BULLET_LIBRARIES = ${BULLET_LIBRARIES}")
#message(ERROR "BULLET_INCLUDE_DIRS = ${BULLET_INCLUDE_DIRS}")
#message(ERROR "BULLET_LIBRARY_DIRS = ${BULLET_LIBRARY_DIRS}")
#BulletSoftBody;BulletDynamics;BulletCollision;LinearMath


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

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
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
#  LIBRARIES rtt-bullet-embedded
#  CATKIN_DEPENDS roscpp std_msgs
   DEPENDS EIGEN3 orocos_kdl
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

  ${PROJECT_SOURCE_DIR}/src/test/rtt_interface_test.cpp
  ${PROJECT_SOURCE_DIR}/src/test/interface_test.cpp
  ${PROJECT_SOURCE_DIR}/src/test/b3RobotSimulatorClientAPI_NoDirect.cpp
  ${PROJECT_SOURCE_DIR}/src/test/b3RobotSimulatorClientAPI_NoGUI.cpp
  ${PROJECT_SOURCE_DIR}/src/test/b3RobotSimulatorClientAPI.cpp
)

# set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(${CMAKE_PROJECT_NAME} ${EIGEN3_LIBRARIES} ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${BULLET_LIBRARIES} ${GAZEBO_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${catkin_LIBRARIES}
LinearMath
Bullet3Common
BulletInverseDynamics
BulletCollision
BulletDynamics
BulletSoftBody
BulletRobotics
BulletExampleBrowserLib
BulletRoboticsGUI
gwen
)


orocos_generate_package(
  #INCLUDE_DIRS include
  DEPENDS rtt_ros orocos_kdl
  #DEPENDS EIGEN3 orocos_kdl
)


# TEST interface
add_executable (interface_test ${PROJECT_SOURCE_DIR}/src/test/interface_test.cpp ${PROJECT_SOURCE_DIR}/src/simulator_interface/bullet/b3_capi_wrapper_no_gui.cpp ${PROJECT_SOURCE_DIR}/src/test/b3RobotSimulatorClientAPI_NoDirect.cpp ${PROJECT_SOURCE_DIR}/src/test/b3RobotSimulatorClientAPI_NoGUI.cpp ${PROJECT_SOURCE_DIR}/src/test/b3RobotSimulatorClientAPI.cpp)
target_link_libraries(interface_test ${EIGEN3_LIBRARIES} ${BULLET_LIBRARIES}
LinearMath
Bullet3Common
BulletInverseDynamics
BulletCollision
BulletDynamics
BulletSoftBody
BulletRobotics
BulletExampleBrowserLib
BulletRoboticsGUI
gwen
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
