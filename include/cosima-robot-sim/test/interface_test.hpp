/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <iostream>

#include <memory>

// For nanosleep
#include <time.h>

#include <bullet/LinearMath/btAlignedObjectArray.h>
#include <bullet/LinearMath/btVector3.h>
#include <bullet/LinearMath/btQuaternion.h>

// Bullet-Specific includes
#include "../simulator_interface/bullet/b3_capi_wrapper_no_gui.hpp"

// #include "b3RobotSimulatorClientAPI.hpp"

#include <chrono>

class TestInterface
{
public:
  TestInterface();
  void loop();

  std::shared_ptr<b3CApiWrapperNoGui> sim;
  // std::shared_ptr<b3RobotSimulatorClientAPI> sim;

private:
  int num_joints;
  double last_time;
  std::vector<int> vec_joint_indices;
  int model_id;

  int *joint_indices;
  double *zero_forces;
  double *zero_accelerations;
  double *max_forces;
  double *target_positions;

  // Initialize sensing variables
  double *q;
  double *qd;
  double *gc;
  double *M;

  // Initialize acting variables
  double *cmd_trq;
  double *cmd_pos;

  double *_use_test_output;

  std::chrono::milliseconds lastms;

  Eigen::VectorXd fdb_position;
  Eigen::VectorXd fdb_velocity;
  Eigen::VectorXd fdb_gc;
  Eigen::MatrixXd fdb_inertia;
};
