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

#include "../include/cosima-robot-sim/rtt_robot_manipulator_sim.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#include <unistd.h>

#ifndef DISABLE_BULLET
#include "../include/cosima-robot-sim/robots/robot_manipulator_bullet.hpp"
#endif

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace RTT;

RTTRobotManipulatorSim::RTTRobotManipulatorSim(std::string const &name) : RTT::TaskContext(name)
{
    bool _at_least_one_simulator = false;
#ifndef DISABLE_BULLET
    this->bullet_interface = std::shared_ptr<BulletInterface>(new BulletInterface());
    addOperation("connectBullet", &RTTRobotManipulatorSim::connectBullet, this);
    addOperation("disconnectBullet", &RTTRobotManipulatorSim::disconnectBullet, this);
    _at_least_one_simulator = true;
#endif

    if (_at_least_one_simulator)
    {
        addOperation("spawnRobot", &RTTRobotManipulatorSim::spawnRobot, this).doc("Returns the model_id to reference the loaded object.");
        addOperation("spawnRobotAtPose", &RTTRobotManipulatorSim::spawnRobotAtPose, this).doc("Returns the model_id to reference the loaded object.");

        addOperation("setControlMode", &RTTRobotManipulatorSim::setControlMode, this, RTT::OwnThread);

        // addOperation("setActiveKinematicChain", &RTTRobotManipulatorSim::setActiveKinematicChain, this, RTT::OwnThread); // TODO

        addOperation("connectToExternallySpawnedRobot", &RTTRobotManipulatorSim::connectToExternallySpawnedRobot, this, RTT::OwnThread);

        addOperation("spawnRobotAtPos", &RTTRobotManipulatorSim::spawnRobotAtPos, this, RTT::OwnThread);

        addProperty("step", step);
        this->step = true;
    }
}

bool RTTRobotManipulatorSim::setActiveKinematicChain(const std::vector<std::string> &jointNames)
{
    return true;
}

bool RTTRobotManipulatorSim::setControlMode(const std::string &modelName, const std::string &controlMode)
{
    if (map_robot_manipulators.count(modelName))
    {
        return map_robot_manipulators[modelName]->setControlMode(controlMode);
    }
    else
    {
        PRELOG(Error) << "Robot " << modelName << " cannot be found!" << RTT::endlog();
        return false;
    }
}

#ifndef DISABLE_BULLET
bool RTTRobotManipulatorSim::connectBullet()
{
    return this->bullet_interface->connect();
}

void RTTRobotManipulatorSim::disconnectBullet()
{
    this->bullet_interface->disconnect();
}
#endif

bool RTTRobotManipulatorSim::configureHook()
{
    for (auto const &e : map_robot_manipulators)
    {
        if (!e.second->configure())
        {
            return false;
        }
    }
    return true;
}

bool RTTRobotManipulatorSim::startHook()
{
    bool ret = false;
#ifndef DISABLE_BULLET
    bool ret_bullet = this->bullet_interface->isConnected();
    ret = ret || ret_bullet;
#endif
    // bool ret_gazebo = gazebo_interface->isConnected();

    return ret;
}

void RTTRobotManipulatorSim::updateHook()
{
    for (auto const &e : map_robot_manipulators)
    {
        e.second->sense();
        e.second->writeToOrocos();
    }

    for (auto const &e : map_robot_manipulators)
    {
        e.second->readFromOrocos();
        e.second->act();
    }

#ifndef DISABLE_BULLET
    this->bullet_interface->stepSimulation();
#endif
}

void RTTRobotManipulatorSim::stopHook()
{
}

void RTTRobotManipulatorSim::cleanupHook()
{
#ifndef DISABLE_BULLET
    this->bullet_interface->disconnect();
#endif
}

bool RTTRobotManipulatorSim::connectToExternallySpawnedRobot(const std::string &modelName, const unsigned int &modelId, const std::string &simulator)
{
#ifndef DISABLE_BULLET
    if (simulator.compare("bullet") == 0)
    {
        if (this->bullet_interface->connectToExternallySpawnedRobot(modelName, modelId))
        {
            // Create Robot
            std::shared_ptr<RobotManipulatorIF> robot = std::shared_ptr<RobotManipulatorIF>(new RobotManipulatorBullet(modelName, modelId, this->bullet_interface->sim, this));
            map_robot_manipulators[modelName] = robot;
        }
    }
#endif
    return false;
}

int RTTRobotManipulatorSim::spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &t, const Eigen::VectorXd &r, const std::string &simulator)
{
#ifndef DISABLE_BULLET
    if (simulator.compare("bullet") == 0)
    {
        int ret_id = this->bullet_interface->spawnRobotAtPose(modelName, modelURDF, t, r);
        if (ret_id >= 0)
        {
            // Create Robot
            map_robot_manipulators[modelName] = std::shared_ptr<RobotManipulatorIF>(new RobotManipulatorBullet(modelName, ret_id, this->bullet_interface->sim, this));
        }
        else
        {
            return false;
        }
    }
#endif
    return false;
}

int RTTRobotManipulatorSim::spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &pos, const std::string &simulator)
{
    Eigen::Vector4d baseOrientation = Eigen::Vector4d::Zero();
    baseOrientation(0) = 1;
    return spawnRobotAtPose(modelName, modelURDF, pos, baseOrientation, simulator);
}

int RTTRobotManipulatorSim::spawnRobot(const std::string &modelName, const std::string &modelURDF, const std::string &simulator)
{
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Vector4d q = Eigen::Vector4d::Zero();
    q(0) = 1;
    return spawnRobotAtPose(modelName, modelURDF, t, q, simulator);
}

// This macro should appear only once per library
ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::RTTRobotManipulatorSim)
