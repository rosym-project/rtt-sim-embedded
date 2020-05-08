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

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <Eigen/Dense>
#include <iostream>

#include <bullet/LinearMath/btAlignedObjectArray.h>
#include <bullet/LinearMath/btVector3.h>
#include <bullet/LinearMath/btQuaternion.h>

#include "api/b3_capi_wrapper_no_gui.hpp"

namespace cosima
{

class RTTBulletEmbedded : public RTT::TaskContext
{
public:
  RTTBulletEmbedded(std::string const &name);

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();

  int spawnModelAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXf &t, const Eigen::VectorXf &r);
  int spawnModel(const std::string &modelName, const std::string &modelURDF);
  void disconnect();
  bool connect();
  bool setGravityVector(const double x, const double y, const double z);
  void resetSimulation();
  int getIdFromModelName(const std::string &modelName);

private:
  std::shared_ptr<b3CApiWrapperNoGui> sim;
  std::map<std::string, int> map_modelName_to_id;
  std::vector<int> vec_model_ids; // Not sure if we really need this!
};

} // namespace cosima