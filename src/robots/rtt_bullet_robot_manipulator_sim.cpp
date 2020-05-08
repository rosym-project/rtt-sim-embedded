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

#include "rtt_bullet_robot_manipulator_sim.hpp"
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

    this->robot_id = -1;
    this->robot_name = "";

    sim = std::shared_ptr<b3CApiWrapperNoGui>(new b3CApiWrapperNoGui());
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
    if (sim->isConnected())
    {
        // Check if a model is loaded, which needs to be the first step!
        if (this->robot_id < 0)
        {
            PRELOG(Error) << "No robot associated, please spawn or connect a robot first!" << RTT::endlog();
            return false;
        }
        
        // Get number of joints
        int _num_joints = sim->getNumJoints(this->robot_id);
        if (_num_joints <= 0)
        {
            PRELOG(Error) << "The associated object is not a robot, since it has " << _num_joints << " joints!" << RTT::endlog();
            this->num_joints = -1;
            return false;
        }

        // Get motor indices (filter fixed joint types)
        map_joint_names_2_indices.clear();
        vec_joint_indices.clear();
        for (unsigned int i = 0; i < _num_joints; i++)
        {
            b3JointInfo jointInfo;
            sim->getJointInfo(this->robot_id, i, &jointInfo);
            int qIndex = jointInfo.m_jointIndex;
            if ((qIndex > -1) && (jointInfo.m_jointType != eFixedType))
            {
                PRELOG(Error) << "Motorname " << jointInfo.m_jointName << ", index " << jointInfo.m_jointIndex << RTT::endlog();
                map_joint_names_2_indices[jointInfo.m_jointName] = qIndex;
                vec_joint_indices.push_back(qIndex);
            }
        }

        this->num_joints = vec_joint_indices.size();
        PRELOG(Error) << "this->num_joints " << this->num_joints << RTT::endlog();

        // Here I should probably also check the order of the joints for the command order TODO

        // Initialize varibales
        this->joint_indices = new int[this->num_joints];
        this->zero_forces = new double[this->num_joints];
        this->max_forces = new double[this->num_joints];
        this->target_positions = new double[this->num_joints];

        for (unsigned int i = 0; i < this->num_joints; i++)
        {
            this->joint_indices[i] = vec_joint_indices[i];
            this->zero_forces[i] = 0.0;
            this->max_forces[i] = 200.0; // TODO magic number
            this->target_positions[i] = 0.0; // TODO magic number (initial config)

            sim->resetJointState(this->robot_id, this->joint_indices[i], 0.0);
        }

        // handle control modes (initial control mode = PD Position)
        // b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_PD, this->num_joints);
        b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_POSITION_VELOCITY_PD, this->num_joints);
        mode_params.m_jointIndices = this->joint_indices;
        mode_params.m_forces = this->max_forces;
        mode_params.m_targetPositions = this->target_positions;
        PRELOG(Error) << "Switching to CONTROL_MODE_POSITION_VELOCITY_PD" << RTT::endlog();
        sim->setJointMotorControlArray(this->robot_id, mode_params);

        // UNLOCKING THE BREAKS FOR TORQUE CONTROL
        // b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_VELOCITY, 7);
        // mode_params.m_jointIndices = tmp_jointIndices;
        // mode_params.m_forces = tmp_forces;

    }
    else
    {
        return false;
    }
    return true;
}

bool RTTBulletRobotManipulatorSim::startHook()
{
    // TODO
    if (!sim->isConnected())
    {
        return false;
    }

    return true;
}

void RTTBulletRobotManipulatorSim::updateHook()
{    

}

void RTTBulletRobotManipulatorSim::stopHook()
{
}

void RTTBulletRobotManipulatorSim::cleanupHook()
{
    sim->disconnect();
}

bool RTTBulletRobotManipulatorSim::connectToExternallySpawnedRobot(const std::string &modelName, int modelId)
{
    this->robot_id = modelId;
    this->robot_name = modelName;
}

int RTTBulletRobotManipulatorSim::spawnRobotAtPose(const std::string &modelName, const std::string &modelURDF, const Eigen::VectorXf &t, const Eigen::VectorXf &r)
{
    if (sim->isConnected()) {
        int model_id = sim->loadURDF(modelURDF);
        if (model_id >= 0)
        {
            this->robot_id = model_id;

            PRELOG(Error) << "Loaded Urdf. Received model_id = " << model_id << RTT::endlog();

            btVector3 basePosition = btVector3(t(0), t(1), t(2));
            btQuaternion baseOrientation = btQuaternion(r(1), r(2), r(3), r(0));

            sim->resetBasePositionAndOrientation(model_id, basePosition, baseOrientation);
            return model_id;
        }
    }
    PRELOG(Error) << "Could NOT load Urdf: " << modelURDF << RTT::endlog();
    return -1;
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
