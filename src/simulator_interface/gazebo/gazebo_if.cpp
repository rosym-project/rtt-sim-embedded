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
 
#ifndef DISABLE_GAZEBO

#include "../include/cosima-robot-sim/simulator_interface/gazebo/gazebo_if.hpp"
#include <rtt/Logger.hpp>

#include <unistd.h>

#define PRELOG(X) (RTT::log(RTT::X) << "[GazeboInterface] ")

using namespace cosima;
using namespace RTT;

GazeboInterface::GazeboInterface() : SimulatorInterface()
{
    this->is_connect_called = false;
}

void GazeboInterface::stepSimulation()
{
}

bool GazeboInterface::isConnected()
{
    if (!this->is_connect_called)
    {
        return false;
    }
    // return bool(gazebo::physics::get_world());
    return true;
}

int GazeboInterface::spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &t, const Eigen::VectorXd &r)
{
    return -1; // TODO
}

int GazeboInterface::spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const double &x, const double &y, const double &z)
{
    // Eigen::VectorXd basePosition = Eigen::VectorXd::Zero(3);
    // basePosition(0) = x;
    // basePosition(1) = y;
    // basePosition(2) = z;
    // Eigen::Vector4d baseOrientation = Eigen::Vector4d::Zero();
    // baseOrientation(0) = 1;
    // return spawnRobotAtPose(modelName, modelURDF, basePosition, baseOrientation);
    return -1; // TODO
}

int GazeboInterface::spawnRobot(const std::string &modelName, const std::string &modelURDF)
{
    // Eigen::Vector3d t = Eigen::Vector3d::Zero();
    // Eigen::Vector4d q = Eigen::Vector4d::Zero();
    // q(0) = 1;
    // return spawnRobotAtPose(modelName, modelURDF, t, q);
    return -1; // TODO
}

void GazeboInterface::disconnect()
{
    this->is_connect_called = false;
}

bool GazeboInterface::connect()
{
    try
    {
        gazebo::printVersion();
        if (gazebo::physics::get_world())
        {
            this->is_connect_called = true;
            PRELOG(Info) << "Connected to Gazebo" << RTT::endlog();
            return true;
        }
    }
    catch (const std::exception &e)
    {
        PRELOG(Error) << "Could not connect, since the world pointer could not be retrived!" << RTT::endlog();
        return false;
    }
    PRELOG(Error) << "Could not connect, since the world pointer could not be retrived!" << RTT::endlog();
    return false;
}

bool GazeboInterface::connectToExternallySpawnedRobot(const std::string &modelName, const unsigned int &modelId)
{
    if (!gazebo::physics::get_world())
    {
        PRELOG(Error) << "Could not connect, since the world pointer could not be retrived!" << RTT::endlog();
        return false;
    }
    if (gazebo::physics::get_world()->GetModel(modelName))
    {
        PRELOG(Info) << "Model [" << modelName << "] successfully found!" << RTT::endlog();
        return true;
    }
    PRELOG(Error) << "Model [" << modelName << "] NOT found!" << RTT::endlog();
    return false;
}

#endif
