# ============================================================
#
# This file is a part of CoSiMA (CogIMon) project
#
# Copyright (C) 2018 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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

project(rtt_bullet_embedded)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14  -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS} -g")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OROCOS-RTT REQUIRED)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

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

include_directories(src
  src/api
  ${USE_OROCOS_INCLUDE_DIRS}
  ${OROCOS-RTT_INCLUDE_DIRS}
)

orocos_component(${CMAKE_PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/src/rtt_bullet_embedded.cpp
  ${PROJECT_SOURCE_DIR}/src/api/b3_capi_wrapper_no_gui.cpp
) # ...you may add multiple source files

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${CMAKE_PROJECT_NAME} ${EIGEN3_LIBRARIES} ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${BULLET_LIBRARIES}
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

orocos_generate_package()
