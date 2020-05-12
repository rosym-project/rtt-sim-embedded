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

    addOperation("setControlMode", &RTTBulletRobotManipulatorSim::setControlMode, this, RTT::OwnThread);

    addOperation("setActiveKinematicChain", &RTTBulletRobotManipulatorSim::setActiveKinematicChain, this, RTT::OwnThread);

    addOperation("connectToExternallySpawnedRobot", &RTTBulletRobotManipulatorSim::connectToExternallySpawnedRobot, this, RTT::OwnThread);

    addOperation("spawnRobotAtPos", &RTTBulletRobotManipulatorSim::spawnRobotAtPos, this, RTT::OwnThread);

    addProperty("step", step);

    this->step = true;

    this->robot_id = -1;
    this->robot_name = "";
    this->active_control_mode = 0;

    sim = std::shared_ptr<b3CApiWrapperNoGui>(new b3CApiWrapperNoGui());
}

bool RTTBulletRobotManipulatorSim::setActiveKinematicChain(const std::vector<std::string> &jointNames)
{
    if (jointNames.size() != this->vec_joint_indices.size())
    {
        return false;
    }
    // check for inconsistent names
    for (unsigned int i = 0; i < jointNames.size(); i++)
    {  
        if (map_joint_names_2_indices.count(jointNames[i]))
        {
        }
        else
        {
            return false;
        }
    }

    for (unsigned int i = 0; i < jointNames.size(); i++)
    {  
        this->vec_joint_indices[i] = this->map_joint_names_2_indices[jointNames[i]];
        this->joint_indices[i] = vec_joint_indices[i];
    }
    return true;
}

bool RTTBulletRobotManipulatorSim::setControlMode(std::string controlMode)
{
    if (controlMode.compare("JointPositionCtrl") == 0)
    {
        // handle control modes (initial control mode = PD Position)
        // b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_PD, this->num_joints);
        b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_POSITION_VELOCITY_PD, this->num_joints);
        mode_params.m_jointIndices = this->joint_indices;
        mode_params.m_forces = this->max_forces;
        mode_params.m_targetPositions = this->target_positions;
        PRELOG(Error) << "Switching to CONTROL_MODE_POSITION_VELOCITY_PD" << RTT::endlog();
        sim->setJointMotorControlArray(this->robot_id, mode_params);

        active_control_mode = 0;
    }
    else if (controlMode.compare("JointTorqueCtrl") == 0)
    {
        // UNLOCKING THE BREAKS FOR TORQUE CONTROL
        b3RobotSimulatorJointMotorArrayArgs mode_params(CONTROL_MODE_VELOCITY, this->num_joints);
        mode_params.m_jointIndices = this->joint_indices;
        mode_params.m_forces = this->zero_forces;
        PRELOG(Error) << "Releasing Breaks" << RTT::endlog();
        sim->setJointMotorControlArray(this->robot_id, mode_params);

        b3RobotSimulatorJointMotorArrayArgs mode_params_trq(CONTROL_MODE_TORQUE, this->num_joints);
        mode_params_trq.m_jointIndices = this->joint_indices;
        mode_params_trq.m_forces = this->zero_forces;
        PRELOG(Error) << "Switching to CONTROL_MODE_TORQUE" << RTT::endlog();
        sim->setJointMotorControlArray(this->robot_id, mode_params_trq);
        
        active_control_mode = 1;
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
    if (sim->isConnected())
    {
        // Check if a model is loaded, which needs to be the first step!
        if (this->robot_id < 0)
        {
            PRELOG(Error) << "No robot associated, please spawn or connect a robot first!" << RTT::endlog();
            return false;
        }

        sim->syncBodies();
        
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

        // sim->setGravity(btVector3(0, 0, -9.81));

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

            // sim->resetJointState(this->robot_id, this->joint_indices[i], 0.0);
            PRELOG(Error) << "joint_indices[" << i << "] = " << joint_indices[i] << RTT::endlog();
        }

        

        // this->setControlMode("JointPositionCtrl");

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
    // b3JointStates2 _joint_states;
    // sim->getJointStates(this->robot_id, _joint_states);

    // PRELOG(Error) << "vq = " << _joint_states.m_actualStateQ[1] << RTT::endlog();

    Eigen::VectorXf vq(this->num_joints);
    Eigen::VectorXf vqd(this->num_joints);
    Eigen::VectorXf vgc(this->num_joints);
    // Eigen::VectorXf vqdd = Eigen::VectorXf::Zero(this->num_joints);
    double q[this->num_joints];
    double qd[this->num_joints];
    double zqdd[this->num_joints];
    double out_gc[this->num_joints];

    for (unsigned int j = 0; j < this->num_joints; j++)
    {
        b3JointSensorState state;
        sim->getJointState(this->robot_id, joint_indices[j], &state);
        q[j] = state.m_jointPosition;
        qd[j] = state.m_jointVelocity;
        zqdd[j] = 0.0;

        vq(j) = q[j];
        vqd(j) = qd[j];
    }
    // PRELOG(Error) << "vq = " << vq << RTT::endlog();
    // PRELOG(Error) << "vqd = " << vqd << RTT::endlog();

    sim->calculateInverseDynamics(this->robot_id, q, qd, zqdd, out_gc);
    for (unsigned int j = 0; j < this->num_joints; j++)
    {
        vgc(j) = out_gc[j];
    }
    // PRELOG(Error) << "POS = " << vq << RTT::endlog();
    // PRELOG(Error) << "GRA = " << vgc << RTT::endlog();

    // double* massMatrix = NULL;
    int byteSizeDofCount = sizeof(double) * this->num_joints;
    double* massMatrix = (double*)malloc(this->num_joints * byteSizeDofCount);
    sim->calculateMassMatrix(this->robot_id, q, this->num_joints, massMatrix, 0);

    Eigen::MatrixXf m = Eigen::MatrixXf::Zero(this->num_joints, this->num_joints);
    for (unsigned int u = 0; u < this->num_joints; u++)
    {
        for (unsigned int v = 0; v < this->num_joints; v++)
        {
            m(u,v) = massMatrix[u * this->num_joints + v];
        }
    }

    // PRELOG(Error) << m << RTT::endlog();

    // if (this->active_control_mode == 1)
    // {
        b3RobotSimulatorJointMotorArrayArgs mode_params_trq(CONTROL_MODE_TORQUE, this->num_joints);
        mode_params_trq.m_jointIndices = this->joint_indices;
        mode_params_trq.m_forces = out_gc;
        sim->setJointMotorControlArray(this->robot_id, mode_params_trq);



        // for (unsigned int j = 0; j < this->num_joints; j++)
        // {
        //     b3RobotSimulatorJointMotorArgs args(CONTROL_MODE_TORQUE);
        //     args.m_maxTorqueValue = out_gc[j];
        //     sim->setJointMotorControl(this->robot_id, joint_indices[j], args);

        //     // b3GetJointInfo(kPhysClient, twojoint, jointNameToId["joint_2"], &jointInfo);
        //     // command = b3JointControlCommandInit2(kPhysClient, twojoint, CONTROL_MODE_TORQUE);
        //     // b3JointControlSetDesiredForceTorque(command, jointInfo.m_uIndex, 0.5 * sin(10 * simTimeS));
        //     // statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
        // }

    // }

    if (this->step)
        sim->stepSimulation();
    // this->trigger();
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

int RTTBulletRobotManipulatorSim::spawnRobotAtPos(const std::string &modelName, const std::string &modelURDF, const double &x, const double &y, const double &z)
{
    if (sim->isConnected()) {
        int model_id = sim->loadURDF(modelURDF);
        if (model_id >= 0)
        {
            this->robot_id = model_id;

            PRELOG(Error) << "Loaded Urdf. Received model_id = " << model_id << RTT::endlog();

            btVector3 basePosition = btVector3(x, y, z);
            btQuaternion baseOrientation = btQuaternion(0, 0, 0, 1);

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
