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

// ROS
#include <sensor_msgs/JointState.h>

namespace cosima
{

    class SimulatorInterface
    {
    public:
        SimulatorInterface();

        virtual int spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &t, const Eigen::VectorXd &r) = 0;
        virtual int spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const double &x, const double &y, const double &z) = 0;
        virtual int spawnRobot(const std::string &modelName, const std::string &modelURDF) = 0;
        virtual void disconnect() = 0;
        virtual bool connect() = 0;
        virtual bool connectToExternallySpawnedRobot(const std::string &modelName, const unsigned int &modelId) = 0;
        virtual bool isConnected() = 0;

    protected:
        bool is_connect_called;
    };

} // namespace cosima