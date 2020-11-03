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

#include "../include/cosima-robot-sim/simulator_interface/bullet/bullet_if.hpp"
#include <rtt/Logger.hpp>

#include <bullet/LinearMath/btAlignedObjectArray.h>
#include <bullet/LinearMath/btVector3.h>
#include <bullet/LinearMath/btQuaternion.h>

#include <unistd.h>

#define PRELOG(X) (RTT::log(RTT::X) << "[BulletInterface] ")

using namespace cosima;
using namespace RTT;

BulletInterface::BulletInterface() : SimulatorInterface()
{
    this->is_connect_called = false;
    sim = std::shared_ptr<b3CApiWrapperNoGui>(new b3CApiWrapperNoGui());
}

void BulletInterface::stepSimulation()
{
    sim->stepSimulation();
}

bool BulletInterface::isConnected()
{
    if (!this->is_connect_called)
    {
        return false;
    }
    return sim->isConnected();
}

int BulletInterface::spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXd &t, const Eigen::VectorXd &r)
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
                // Upload id to ROS parameter service
                if (ros::param::has("robot_map"))
                {
                    std::map<std::string, int> map_s;
                    if (ros::param::get("robot_map", map_s))
                    {
                        map_s[modelName] = model_id;
                        ros::param::set("robot_map", map_s);
                        // ROSCPP_DECL void set(const std::string& key, const std::map<std::string, int>& map);
                        PRELOG(Error) << "Uploaded (update?) model_id = " << model_id << RTT::endlog();
                    }
                    else
                    {
                        PRELOG(Error) << "robot_map could not be read from server!" << RTT::endlog();
                        return -1;
                    }
                }
                else
                {
                    std::map<std::string, int> map_s;
                    map_s[modelName] = model_id;
                    ros::param::set("robot_map", map_s);
                    PRELOG(Error) << "Uploaded model_id = " << model_id << RTT::endlog();
                }

                return model_id;
            }
            else
            {
                PRELOG(Error) << "Could NOT reposition loaded URDF: " << modelURDF << RTT::endlog();
                return -1;
            }
        }
    }
    PRELOG(Error) << "Could NOT load Urdf: " << modelURDF << RTT::endlog();
    return -1;
}

int BulletInterface::spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const double &x, const double &y, const double &z)
{
    Eigen::VectorXd basePosition = Eigen::VectorXd::Zero(3);
    basePosition(0) = x;
    basePosition(1) = y;
    basePosition(2) = z;
    Eigen::Vector4d baseOrientation = Eigen::Vector4d::Zero();
    baseOrientation(0) = 1;
    return spawnRobotAtPose(modelName, modelURDF, basePosition, baseOrientation);
}

int BulletInterface::spawnRobot(const std::string &modelName, const std::string &modelURDF)
{
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Vector4d q = Eigen::Vector4d::Zero();
    q(0) = 1;
    return spawnRobotAtPose(modelName, modelURDF, t, q);
}

void BulletInterface::disconnect()
{
    this->is_connect_called = false;
    sim->disconnect();
}

bool BulletInterface::connect()
{
    if (!sim->isConnected())
    {
        bool isConnected = sim->connect(eCONNECT_SHARED_MEMORY);
        // bool isConnected = sim->connect(eCONNECT_DIRECT);

        if (isConnected)
        {
            PRELOG(Error) << "Using shared memory" << RTT::endlog();
            this->is_connect_called = true;
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

bool BulletInterface::connectToExternallySpawnedRobot(const std::string &modelName, const unsigned int &modelId)
{
    // TODO DLW also modify the collision parameters and take care of the F/T sensors that are present if we can find that out?
    btVector3 basePosition = btVector3(0, 0, 0);
    btQuaternion baseOrientation = btQuaternion(0, 0, 0, 1);
    bool ret = sim->getBasePositionAndOrientation(modelId, basePosition, baseOrientation);
    if (ret)
    {
        return true;
    }
    return false;
}
