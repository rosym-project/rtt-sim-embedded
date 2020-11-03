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
#include <iostream>

#include <Eigen/Dense>

// ROS Data Type includes
#include <sensor_msgs/JointState.h>

// Generic Simulator Interface includes
#include "../sim_if.hpp"

// Bullet-Specific Interface includes
#include "../bullet/b3_capi_wrapper_no_gui.hpp"

// ROS include for parameter server
#include <ros/ros.h>

namespace cosima
{

    class BulletInterface : public SimulatorInterface
    {
    public:
        BulletInterface();

        int spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &t, const Eigen::VectorXd &r);
        int spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const double &x, const double &y, const double &z);
        int spawnRobot(const std::string &modelName, const std::string &modelURDF);
        void disconnect();
        bool connect();
        bool connectToExternallySpawnedRobot(const std::string &modelName, const unsigned int &modelId);
        bool isConnected();

        void stepSimulation();

        std::shared_ptr<b3CApiWrapperNoGui> sim;
    };

} // namespace cosima