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

#include "../include/rtt-bullet-embedded/robots/rtt_bullet_robot_manipulator_sim.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#include <unistd.h>

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace RTT;

RTTBulletRobotManipulatorSim::RTTBulletRobotManipulatorSim(std::string const &name) : RTT::TaskContext(name)
{
    addOperation("connect", &RTTBulletRobotManipulatorSim::connect, this);
    addOperation("disconnect", &RTTBulletRobotManipulatorSim::disconnect, this);

    addOperation("spawnRobot", &RTTBulletRobotManipulatorSim::spawnRobot, this).doc("Returns the model_id to reference the loaded object.");
    addOperation("spawnRobotAtPose", &RTTBulletRobotManipulatorSim::spawnRobotAtPose, this).doc("Returns the model_id to reference the loaded object.");

    addOperation("setControlMode", &RTTBulletRobotManipulatorSim::setControlMode, this, RTT::OwnThread);

    addOperation("setActiveKinematicChain", &RTTBulletRobotManipulatorSim::setActiveKinematicChain, this, RTT::OwnThread);

    addOperation("connectToExternallySpawnedRobot", &RTTBulletRobotManipulatorSim::connectToExternallySpawnedRobot, this, RTT::OwnThread);

    addOperation("spawnRobotAtPos", &RTTBulletRobotManipulatorSim::spawnRobotAtPos, this, RTT::OwnThread);

    addProperty("step", step);
    this->step = true;

    sim = std::shared_ptr<b3CApiWrapperNoGui>(new b3CApiWrapperNoGui());
}

bool RTTBulletRobotManipulatorSim::setActiveKinematicChain(const std::vector<std::string> &jointNames)
{
    return true;
}

bool RTTBulletRobotManipulatorSim::setControlMode(const std::string &modelName, const std::string &controlMode)
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

void RTTBulletRobotManipulatorSim::disconnect()
{
    sim->disconnect();
}

bool RTTBulletRobotManipulatorSim::connect()
{
    if (!sim->isConnected())
    {
        bool isConnected = sim->connect(eCONNECT_SHARED_MEMORY);
        // bool isConnected = sim->connect(eCONNECT_DIRECT);

        if (isConnected)
        {
            PRELOG(Error) << "Using shared memory" << RTT::endlog();
            return true;
        }
        else
        {
            PRELOG(Error) << "NOT connected via shared memory" << RTT::endlog();
            return false;
        }
    }
    else
    {
        PRELOG(Error) << "Seems to be already connected" << RTT::endlog();
    }

    return false;
}

bool RTTBulletRobotManipulatorSim::configureHook()
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

bool RTTBulletRobotManipulatorSim::startHook()
{
    if (!sim->isConnected())
    {
        return false;
    }

    return true;
}

void RTTBulletRobotManipulatorSim::updateHook()
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

    this->sim->stepSimulation();
}

void RTTBulletRobotManipulatorSim::stopHook()
{
}

void RTTBulletRobotManipulatorSim::cleanupHook()
{
    sim->disconnect();
}

bool RTTBulletRobotManipulatorSim::connectToExternallySpawnedRobot(const std::string &modelName, const unsigned int &modelId)
{
    btVector3 basePosition = btVector3(0, 0, 0);
    btQuaternion baseOrientation = btQuaternion(0, 0, 0, 1);
    bool ret = sim->getBasePositionAndOrientation(modelId, basePosition, baseOrientation);
    if (ret)
    {
        // Create Robot
        std::shared_ptr<RobotManipulator> robot = std::shared_ptr<RobotManipulator>(new RobotManipulator(modelName, modelId, this->sim, this));
        map_robot_manipulators[modelName] = robot;
        return true;
    }
    return false;
}

int RTTBulletRobotManipulatorSim::spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXf &t, const Eigen::VectorXf &r)
{
    if (sim->isConnected())
    {
        b3RobotSimulatorLoadUrdfFileArgs _args;
        // _args.m_flags = URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION;
        _args.m_flags = URDF_USE_INERTIA_FROM_FILE;

        int model_id = sim->loadURDF(modelURDF, _args);
        if (model_id >= 0)
        {
            PRELOG(Error) << "Loaded Urdf. Received model_id = " << model_id << RTT::endlog();

            btVector3 basePosition = btVector3(t(0), t(1), t(2));
            btQuaternion baseOrientation = btQuaternion(r(1), r(2), r(3), r(0));

            bool ret = sim->resetBasePositionAndOrientation(model_id, basePosition, baseOrientation);
            if (ret)
            {
                // Create Robot
                map_robot_manipulators[modelName] = std::shared_ptr<RobotManipulator>(new RobotManipulator(modelName, model_id, this->sim, this));
            }
            return model_id;
        }
    }
    PRELOG(Error) << "Could NOT load Urdf: " << modelURDF << RTT::endlog();
    return -1;
}

int RTTBulletRobotManipulatorSim::spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const double &x, const double &y, const double &z)
{
    Eigen::VectorXf basePosition = Eigen::VectorXf::Zero(3);
    basePosition(0) = x;
    basePosition(1) = y;
    basePosition(2) = z;
    Eigen::Vector4f baseOrientation = Eigen::Vector4f::Zero();
    baseOrientation(0) = 1;
    return spawnRobotAtPose(modelName, modelURDF, basePosition, baseOrientation);
}

int RTTBulletRobotManipulatorSim::spawnRobot(const std::string &modelName, const std::string &modelURDF)
{
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    Eigen::Vector4f q = Eigen::Vector4f::Zero();
    q(0) = 1;
    return spawnRobotAtPose(modelName, modelURDF, t, q);
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::RTTBulletRobotManipulatorSim)
